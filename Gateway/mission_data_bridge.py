from __future__ import annotations

import copy
import math
import time

from map_coordinates import NORMALIZED_TOP_LEFT


AI_DETECTION_TOPIC = "ai-detection"
OBSERVATION_ANNOTATION_TOPIC = "observation-annotation"
TELEMETRY_STRIKE_TOPIC = "telemetry-strike"
TARGET_SELECT_TOPIC = "target-select"
TARGET_LOCK_TOPIC = "target-lock"
TARGET_UNLOCK_TOPIC = "target-unlock"
MISSION_INPUT_TOPICS = frozenset(
    {
        AI_DETECTION_TOPIC,
        OBSERVATION_ANNOTATION_TOPIC,
        TELEMETRY_STRIKE_TOPIC,
        TARGET_SELECT_TOPIC,
        TARGET_LOCK_TOPIC,
        TARGET_UNLOCK_TOPIC,
    }
)

SCHEMA_VERSION = 1
DETECTION_TTL_MS = 750
FLIGHT_TELEMETRY_TTL_MS = 3000
MAX_DETECTION_BOXES = 100
MAX_DETECTION_HISTORY = 32
MIN_TARGET_TRACKING_IOU = 0.05

_TARGET_MESSAGE_TYPES = {
    TARGET_SELECT_TOPIC: {"target.select", "target.release"},
    TARGET_LOCK_TOPIC: {"target.lock"},
    TARGET_UNLOCK_TOPIC: {"target.unlock"},
}


class MissionDataBridge:
    """Mission Platform 메시지를 StrikerPC 내부 v1 규약으로 정규화한다."""

    def __init__(
        self,
        *,
        mission_id: str,
        session_id: str,
        command_center_identity: str,
        drone_identity: str,
        video_track: str,
    ) -> None:
        self._mission_id = str(mission_id)
        self._session_id = str(session_id)
        self._command_center_identity = str(command_center_identity)
        self._drone_identity = str(drone_identity)
        self._video_track = str(video_track)
        self._last_detection_rejection_reason: str | None = None
        self._detection_history: list[dict] = []
        self._target_states: dict[str, dict] = {}
        self._target_sequence = 0
        self._flight_telemetry: dict | None = None
        self._flight_safety_state: dict | None = None
        self._safety_sequence = -1
        self._safety_session_id = ""

    @property
    def target_states(self) -> list[dict]:
        return copy.deepcopy(list(self._target_states.values()))

    @property
    def flight_telemetry(self) -> dict | None:
        return copy.deepcopy(self._flight_telemetry)

    @property
    def flight_safety_state(self) -> dict | None:
        return copy.deepcopy(self._flight_safety_state)

    @property
    def last_detection_rejection_reason(self) -> str | None:
        return self._last_detection_rejection_reason

    # 발신자 이름과 무관하게 LiveKit 탐지 결과를 Unity 표시 이벤트로 만든다.
    def normalize_detection(
        self,
        message: dict,
        participant_identity: str,
        *,
        participant_sid: str | None = None,
        received_at_ms: int | None = None,
    ) -> dict | None:
        self._last_detection_rejection_reason = None
        if message.get("type") != "detection.frame":
            self._last_detection_rejection_reason = "message-type"
            return None

        frame_sequence = _non_negative_int(message.get("frame_id"))
        source_timestamp_ms = _non_negative_int(message.get("captured_at"))
        boxes = _normalize_boxes(message.get("boxes"))
        if frame_sequence is None:
            self._last_detection_rejection_reason = "invalid-frame"
            return None
        if source_timestamp_ms is None:
            self._last_detection_rejection_reason = "invalid-timestamp"
            return None
        if boxes is None:
            self._last_detection_rejection_reason = "invalid-boxes"
            return None

        now_ms = _now_ms(received_at_ms)
        event = {
            **self._envelope(
                "mission-detection-frame",
                participant_identity,
                frame_sequence,
                now_ms,
            ),
            "sourceTransport": "livekit",
            "sourceTimestampMs": source_timestamp_ms,
            "ttlMs": DETECTION_TTL_MS,
            "coordinateSpace": NORMALIZED_TOP_LEFT,
            "frameCorrelation": "c2-receive-order",
            "boxes": boxes,
        }
        self._prune_detection_history(now_ms)
        self._detection_history.append(copy.deepcopy(event))
        self._detection_history = self._detection_history[-MAX_DETECTION_HISTORY:]
        return event

    # Strike telemetry를 지도 pose와 분리된 비행 상태 규약으로 만든다.
    def normalize_flight_telemetry(
        self,
        message: dict,
        participant_identity: str,
        *,
        received_at_ms: int | None = None,
    ) -> dict | None:
        if participant_identity != self._drone_identity:
            return None
        if message.get("type") != "telemetry.drone" or not self._matches_source(message):
            return None
        if not self._matches_session(message):
            return None

        source_timestamp_ms = _non_negative_int(message.get("timestamp"))
        connected = message.get("connected")
        if source_timestamp_ms is None or not isinstance(connected, bool):
            return None

        telemetry: dict[str, int | float] = {}
        for source_key, target_key, minimum, maximum in (
            ("battery", "batteryPercent", 0.0, 100.0),
            ("temperature", "temperatureC", -50.0, 150.0),
        ):
            value = _bounded_number(message.get(source_key), minimum, maximum)
            if message.get(source_key) is not None and value is None:
                return None
            if value is not None:
                telemetry[target_key] = value
        height_centimeters = _bounded_number(message.get("height"), 0.0, None)
        if message.get("height") is not None and height_centimeters is None:
            return None
        if height_centimeters is not None:
            telemetry["heightMeters"] = float(height_centimeters) / 100.0
            telemetry["heightCentimeters"] = height_centimeters

        now_ms = _now_ms(received_at_ms)
        event = {
            **self._envelope(
                "flight-telemetry",
                participant_identity,
                source_timestamp_ms,
                now_ms,
            ),
            "sourceTimestampMs": source_timestamp_ms,
            "ttlMs": FLIGHT_TELEMETRY_TTL_MS,
            "connected": connected,
            **telemetry,
        }
        self._flight_telemetry = event
        return event

    # 관리자 target 명령을 선택/잠금/해제 상태로 보존한다.
    def normalize_target_state(
        self,
        topic: str,
        message: dict,
        participant_identity: str,
        *,
        received_at_ms: int | None = None,
    ) -> dict | None:
        expected_types = _TARGET_MESSAGE_TYPES.get(topic)
        message_type = str(message.get("type") or "")
        if expected_types is None or message_type not in expected_types:
            return None
        if not self._matches_target_source(message) or not self._matches_session(message):
            return None

        supplied_target_id = message.get("target_id", message.get("targetId"))
        if topic != TARGET_SELECT_TOPIC and (
            not isinstance(supplied_target_id, str) or not supplied_target_id.strip()
        ):
            return None
        target_id = str(supplied_target_id or "target-001").strip()
        if not target_id or len(target_id) > 128:
            return None

        now_ms = _now_ms(received_at_ms)
        if message_type == "target.select":
            box_source = message.get("box")
            if not isinstance(box_source, dict):
                box_source = message
            requested_box = _normalize_box(box_source, 0)
            source_frame_sequence = _non_negative_int(message.get("frame_id"))
            if requested_box is None or source_frame_sequence is None:
                return None
            detection = self._find_recent_detection(source_frame_sequence, now_ms)
            if detection is None:
                return None
            box = _find_detection_box(detection["boxes"], requested_box)
            if box is None:
                return None
        else:
            previous_state = self._target_states.get(target_id)
            if previous_state is None:
                return None
            source_frame_sequence = previous_state["sourceFrameSequence"]
            box = copy.deepcopy(previous_state["box"])

        active = message_type not in {"target.release", "target.unlock"}
        locked = message_type == "target.lock"
        if self._target_state_matches(
            target_id,
            source_frame_sequence,
            box,
            active,
            locked,
        ):
            return None
        return self._store_target_state(
            participant_identity,
            target_id,
            source_frame_sequence,
            box,
            active,
            locked,
            now_ms,
        )

    # 퍼블리셔 receipt를 최근 C2 탐지와 대조해 선택/해제 상태로 만든다.
    def normalize_annotation_receipt(
        self,
        message: dict,
        participant_identity: str,
        *,
        received_at_ms: int | None = None,
    ) -> dict | None:
        if participant_identity != self._drone_identity:
            return None
        if message.get("type") != "observation.annotation.received":
            return None
        if not self._matches_source(message) or not self._matches_session(message):
            return None

        supplied_target_id = message.get("target_id", message.get("targetId"))
        if not isinstance(supplied_target_id, str):
            return None
        target_id = supplied_target_id.strip()
        active = message.get("active")
        if not target_id or len(target_id) > 128 or not isinstance(active, bool):
            return None

        now_ms = _now_ms(received_at_ms)
        previous_state = self._target_states.get(target_id)
        if active:
            source_frame_sequence = _non_negative_int(message.get("frame_id"))
            requested_box = _normalize_receipt_box(message)
            if source_frame_sequence is None or requested_box is None:
                return None
            detection = self._find_recent_detection(source_frame_sequence, now_ms)
            if detection is None:
                return None
            box = _find_detection_box_by_geometry(detection["boxes"], requested_box)
            if box is None:
                return None
            if previous_state is not None and (
                previous_state["sourceFrameSequence"] == source_frame_sequence
                and previous_state["box"] == box
            ):
                return None
        else:
            if previous_state is None or previous_state["active"] is False:
                return None
            source_frame_sequence = previous_state["sourceFrameSequence"]
            box = copy.deepcopy(previous_state["box"])

        return self._store_target_state(
            participant_identity,
            target_id,
            source_frame_sequence,
            box,
            active,
            False,
            now_ms,
        )

    # 새 C2 탐지와 매칭되면 좌표를 갱신하고, 재탐지 실패 시 마지막 선택을 유지한다.
    def track_active_targets(self, detection_event: dict) -> list[dict]:
        if (
            detection_event.get("type") != "mission-detection-frame"
            or detection_event.get("sourceTrack") != self._video_track
        ):
            return []

        frame_sequence = _non_negative_int(detection_event.get("sequence"))
        received_at_ms = _non_negative_int(detection_event.get("receivedAtMs"))
        boxes = detection_event.get("boxes")
        if (
            frame_sequence is None
            or received_at_ms is None
            or not isinstance(boxes, list)
        ):
            return []

        source_identity = str(
            detection_event.get("sourceIdentity") or self._command_center_identity
        )
        events: list[dict] = []
        for target_id, previous_state in list(self._target_states.items()):
            if previous_state.get("active") is not True:
                continue
            previous_frame_sequence = _non_negative_int(
                previous_state.get("sourceFrameSequence")
            )
            if (
                previous_frame_sequence is not None
                and frame_sequence <= previous_frame_sequence
            ):
                continue

            previous_box = previous_state.get("box")
            if not boxes or not isinstance(previous_box, dict):
                continue
            match = max(boxes, key=lambda box: _box_iou(previous_box, box))
            if _box_iou(previous_box, match) < MIN_TARGET_TRACKING_IOU:
                continue

            events.append(
                self._store_target_state(
                    source_identity,
                    target_id,
                    frame_sequence,
                    copy.deepcopy(match),
                    True,
                    previous_state.get("locked") is True,
                    received_at_ms,
                )
            )
        return events

    # 유효 시간 안의 동일 프레임 탐지를 찾는다.
    def _find_recent_detection(self, frame_sequence: int, now_ms: int) -> dict | None:
        self._prune_detection_history(now_ms)
        for detection in reversed(self._detection_history):
            if detection["sequence"] == frame_sequence:
                return detection
        return None

    # 만료되거나 미래 시각인 탐지 프레임을 제거한다.
    def _prune_detection_history(self, now_ms: int) -> None:
        self._detection_history = [
            detection
            for detection in self._detection_history
            if 0 <= now_ms - int(detection["receivedAtMs"]) <= DETECTION_TTL_MS
        ]

    # 동일한 표적 상태의 중복 전송을 막는다.
    def _target_state_matches(
        self,
        target_id: str,
        source_frame_sequence: int,
        box: dict,
        active: bool,
        locked: bool,
    ) -> bool:
        previous_state = self._target_states.get(target_id)
        return previous_state is not None and all(
            (
                previous_state["sourceFrameSequence"] == source_frame_sequence,
                previous_state["box"] == box,
                previous_state["active"] is active,
                previous_state["locked"] is locked,
            )
        )

    # 정규화된 표적 상태를 현재 스냅샷에 보존한다.
    def _store_target_state(
        self,
        participant_identity: str,
        target_id: str,
        source_frame_sequence: int,
        box: dict,
        active: bool,
        locked: bool,
        now_ms: int,
    ) -> dict:
        self._target_sequence += 1
        event = {
            **self._envelope(
                "mission-target-state",
                participant_identity,
                self._target_sequence,
                now_ms,
            ),
            "sourceTransport": "livekit",
            "sourceFrameSequence": source_frame_sequence,
            "targetId": target_id,
            "active": active,
            "locked": locked,
            "box": copy.deepcopy(box) if active else None,
        }
        if active:
            self._target_states[target_id] = event
        else:
            self._target_states.pop(target_id, None)
        return event

    # localhost 안전 상태를 정규화하고 마지막 상태를 재접속 snapshot에 보존한다.
    def normalize_flight_safety_state(
        self,
        message: dict,
        *,
        received_at_ms: int | None = None,
    ) -> dict | None:
        if message.get("type") != "flight-safety-state":
            return None
        if message.get("schemaVersion", SCHEMA_VERSION) != SCHEMA_VERSION:
            return None
        supplied_mission_id = message.get("missionId", message.get("mission_id"))
        if supplied_mission_id is not None and str(supplied_mission_id) != self._mission_id:
            return None
        supplied_drone_id = message.get("droneId", message.get("drone_id"))
        if supplied_drone_id is not None and str(supplied_drone_id) != self._drone_identity:
            return None

        state = message.get("state", message.get("status", message.get("phase")))
        if not isinstance(state, str) or not state.strip():
            return None
        source_timestamp_ms = _non_negative_int(
            message.get("timestamp", message.get("sourceTimestampMs"))
        )
        if source_timestamp_ms is None:
            return None
        supplied_sequence = _non_negative_int(message.get("sequence"))
        if message.get("sequence") is not None and supplied_sequence is None:
            return None

        safety_session_id = str(
            message.get("sessionId", message.get("session_id", ""))
        )
        if safety_session_id and safety_session_id != self._safety_session_id:
            self._safety_session_id = safety_session_id
            self._safety_sequence = -1

        next_sequence = supplied_sequence
        if next_sequence is None:
            next_sequence = self._safety_sequence + 1
        if next_sequence <= self._safety_sequence:
            return None
        self._safety_sequence = next_sequence

        event = {
            **self._envelope(
                "flight-safety-state",
                "flight-safety-gateway",
                next_sequence,
                _now_ms(received_at_ms),
            ),
            "sourceTimestampMs": source_timestamp_ms,
            "state": state.strip(),
        }
        if safety_session_id:
            event["safetySessionId"] = safety_session_id
        for key in (
            "airborne",
            "controlConnected",
            "heartbeatRequired",
            "heartbeatFresh",
            "heartbeatAgeMs",
            "stateAgeMs",
            "forceNeutral",
            "neutralForced",
            "landAttempts",
            "landingAttempts",
            "reason",
            "lastError",
        ):
            if key in message and _is_json_scalar(message[key]):
                event[key] = message[key]
        self._flight_safety_state = event
        return event

    def _matches_source(self, message: dict) -> bool:
        return (
            str(message.get("source", message.get("droneId", "")))
            == self._drone_identity
            and str(message.get("source_track", message.get("sourceTrack", "")))
            == self._video_track
        )

    # 관리자 명령은 참가자 권한으로 인증하므로 웹의 CP source를 허용한다.
    def _matches_target_source(self, message: dict) -> bool:
        source = str(message.get("source") or "").strip()
        source_track = str(
            message.get("source_track", message.get("sourceTrack", ""))
        )
        return bool(source) and source_track == self._video_track

    def _matches_session(self, message: dict) -> bool:
        supplied_session = message.get("sessionId", message.get("session_id"))
        return supplied_session is None or str(supplied_session) == self._session_id

    def _envelope(
        self,
        event_type: str,
        source_identity: str,
        sequence: int,
        received_at_ms: int,
    ) -> dict:
        return {
            "type": event_type,
            "schemaVersion": SCHEMA_VERSION,
            "missionId": self._mission_id,
            "sessionId": self._session_id,
            "droneId": self._drone_identity,
            "sourceIdentity": source_identity,
            "sourceTrack": self._video_track,
            "sequence": sequence,
            "receivedAtMs": received_at_ms,
        }


def _normalize_boxes(value) -> list[dict] | None:
    if not isinstance(value, list) or len(value) > MAX_DETECTION_BOXES:
        return None
    boxes: list[dict] = []
    for index, raw_box in enumerate(value):
        box = _normalize_box(raw_box, index)
        if box is None:
            return None
        boxes.append(box)
    return boxes


def _find_detection_box(boxes: list[dict], requested_box: dict) -> dict | None:
    comparable_keys = (
        "classId",
        "confidence",
        "x",
        "y",
        "width",
        "height",
    )
    for box in boxes:
        if all(box.get(key) == requested_box.get(key) for key in comparable_keys):
            return copy.deepcopy(box)
    return None


def _find_detection_box_by_geometry(
    boxes: list[dict],
    requested_box: dict,
) -> dict | None:
    comparable_keys = ("x", "y", "width", "height")
    for box in boxes:
        if all(box.get(key) == requested_box.get(key) for key in comparable_keys):
            return copy.deepcopy(box)
    return None


def _box_iou(left: dict, right: dict) -> float:
    try:
        left_x1 = float(left["x"])
        left_y1 = float(left["y"])
        left_x2 = left_x1 + float(left["width"])
        left_y2 = left_y1 + float(left["height"])
        right_x1 = float(right["x"])
        right_y1 = float(right["y"])
        right_x2 = right_x1 + float(right["width"])
        right_y2 = right_y1 + float(right["height"])
    except (KeyError, TypeError, ValueError):
        return 0.0

    intersection = max(0.0, min(left_x2, right_x2) - max(left_x1, right_x1)) * max(
        0.0,
        min(left_y2, right_y2) - max(left_y1, right_y1),
    )
    left_area = max(0.0, left_x2 - left_x1) * max(0.0, left_y2 - left_y1)
    right_area = max(0.0, right_x2 - right_x1) * max(0.0, right_y2 - right_y1)
    union = left_area + right_area - intersection
    return intersection / union if union > 0.0 else 0.0


def _normalize_receipt_box(value: dict) -> dict | None:
    x = _bounded_number(value.get("x"), 0.0, 1.0)
    y = _bounded_number(value.get("y"), 0.0, 1.0)
    width = _bounded_number(value.get("width"), 0.0, 1.0)
    height = _bounded_number(value.get("height"), 0.0, 1.0)
    if None in {x, y, width, height}:
        return None
    if width <= 0.0 or height <= 0.0 or x + width > 1.0 or y + height > 1.0:
        return None
    return {"x": x, "y": y, "width": width, "height": height}


def _normalize_box(value, index: int) -> dict | None:
    if not isinstance(value, dict):
        return None
    x = _bounded_number(value.get("x"), 0.0, 1.0)
    y = _bounded_number(value.get("y"), 0.0, 1.0)
    width = _bounded_number(value.get("width"), 0.0, 1.0)
    height = _bounded_number(value.get("height"), 0.0, 1.0)
    confidence = _bounded_number(value.get("confidence"), 0.0, 1.0)
    class_id = _non_negative_int(value.get("class_id", value.get("classId")))
    if None in {x, y, width, height, confidence, class_id}:
        return None
    if width <= 0.0 or height <= 0.0 or x + width > 1.0 or y + height > 1.0:
        return None

    class_name = str(value.get("class_name", value.get("className", "")))
    if len(class_name) > 128:
        return None
    return {
        "index": index,
        "classId": class_id,
        "className": class_name,
        "confidence": confidence,
        "x": x,
        "y": y,
        "width": width,
        "height": height,
    }


def _non_negative_int(value) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int) and value >= 0:
        return value
    return None


def _bounded_number(value, minimum: float, maximum: float | None) -> int | float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    if not math.isfinite(number) or number < minimum:
        return None
    if maximum is not None and number > maximum:
        return None
    return value


def _now_ms(value: int | None) -> int:
    return int(time.time() * 1000) if value is None else int(value)


def _is_json_scalar(value) -> bool:
    return value is None or isinstance(value, (str, int, float, bool))
