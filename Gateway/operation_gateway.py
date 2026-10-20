from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import logging
import math
import os
import socket
import time
import uuid
from pathlib import Path

from dotenv import load_dotenv
from livekit import api, rtc

from map_coordinates import (
    VENUE_XZ_METERS,
    VenueBounds,
    normalized_to_venue,
    venue_to_normalized,
)
from mission_data_bridge import (
    AI_DETECTION_TOPIC,
    MISSION_INPUT_TOPICS,
    OBSERVATION_ANNOTATION_TOPIC,
    TARGET_LOCK_TOPIC,
    TARGET_SELECT_TOPIC,
    TARGET_UNLOCK_TOPIC,
    TELEMETRY_STRIKE_TOPIC,
    MissionDataBridge,
)


logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

COMMAND_TOPIC = "operation.commands"
EVENT_TOPIC = "operation.events"
ANNOTATION_TOPIC = "annotations"
SYNC_TOPIC = "sync"
TARGET_POSITION_TOPIC = "target-position"
DRONE_POSITION_TOPIC = "drone-position"
MAX_STATE_ITEMS = 250
SNAPSHOT_CHUNK_SIZE = 40
PING_RESPONSE_TIMEOUT_SECONDS = 3.0
MAX_DRAWING_POINTS = 128
MAX_DRAWING_INPUT_POINTS = 4096
FADE_DRAWING_SECONDS = 3.0
DRAWING_SURFACES = {"recon", "strike", "map"}
MANUAL_TARGET_SURFACES = {"recon", "strike"}
AI_DETECTION_LOG_INTERVAL_SECONDS = 1.0


class UnityEventProtocol(asyncio.DatagramProtocol):
    def __init__(self, gateway: "OperationGateway") -> None:
        self._gateway = gateway

    # Unity UDP 이벤트를 비동기 처리 큐에 전달한다.
    def datagram_received(self, data: bytes, address) -> None:
        self._gateway.enqueue_unity_event(data)


class OperationGateway:
    def __init__(self, args: argparse.Namespace) -> None:
        self._args = args
        self._loop = asyncio.get_running_loop()
        self._unity_transport: asyncio.DatagramTransport | None = None
        self._unity_target = ("127.0.0.1", args.unity_command_port)
        self._unity_events: asyncio.Queue[bytes] = asyncio.Queue(maxsize=256)
        self._annotations: dict[str, dict] = {}
        self._pings: dict[str, dict] = {}
        self._pending_ping_requests: dict[str, dict] = {}
        self._cp_target_positions: dict[str, dict] = {}
        self._last_drone_telemetry: dict | None = None
        self._last_drone_heading: dict | None = None
        self._ai_detection_ingress_log_last_at = 0.0
        self._ai_detection_ingress_log_pending_count = 0
        self._ai_detection_log_last_at = 0.0
        self._ai_detection_log_pending_count = 0
        self._ai_detection_rejection_log_last_at = 0.0
        self._ai_detection_rejection_log_pending_count = 0
        self._drone_target_id = (
            "recon" if str(args.video_track) == "video-recon" else "strike"
        )
        self._mission_bridge = MissionDataBridge(
            mission_id=args.room,
            session_id=args.mission_session_id,
            command_center_identity=args.command_center_identity,
            drone_identity=args.drone_identity,
            video_track=args.video_track,
        )
        self._revision = 0
        self._map_path = Path(args.map_path).resolve()
        self._map = self._load_map(self._map_path)
        self._map_state = self._build_map_state(
            self._map_path,
            self._map,
            args.map_public_base_url,
        )
        self._room = self._create_room()

    # 재접속마다 새 Room을 만들고 동일한 데이터 이벤트를 다시 연결한다.
    def _create_room(self) -> rtc.Room:
        room = rtc.Room()

        @room.on("data_received")
        def on_data_received(packet: rtc.DataPacket) -> None:
            self._loop.create_task(self._handle_room_packet(packet))

        @room.on("participant_connected")
        def on_participant_connected(participant: rtc.RemoteParticipant) -> None:
            self._loop.create_task(self._sync_participant(participant.identity))

        return room

    # 로컬 Unity 브리지는 즉시 준비하고 LiveKit은 백오프로 계속 재접속한다.
    async def run(self) -> None:
        await self._connect_unity_bridge()
        logging.info("Operation Gateway local bridge ready: %s", self._args.room)
        print("STRIKERPC_READY OPERATION_GATEWAY", flush=True)
        retry_seconds = 1.0
        try:
            while True:
                if not self._room.isconnected():
                    try:
                        await self._connect_livekit()
                        retry_seconds = 1.0
                    except asyncio.CancelledError:
                        raise
                    except Exception as error:
                        logging.warning(
                            "Operation Gateway LiveKit unavailable; local bridge "
                            "remains active: %s",
                            error,
                        )
                        self._room = self._create_room()
                        await asyncio.sleep(retry_seconds)
                        retry_seconds = min(10.0, retry_seconds * 2.0)
                        continue
                try:
                    payload = await asyncio.wait_for(
                        self._unity_events.get(),
                        timeout=0.25,
                    )
                    await self._handle_unity_event(payload)
                except asyncio.TimeoutError:
                    pass
                await self._expire_ping_requests()
        finally:
            if self._unity_transport is not None:
                self._unity_transport.close()
            if self._room.isconnected():
                await self._room.disconnect()

    # LiveKit 연결 성공 후 현재 참가자에게 최신 스냅샷을 보낸다.
    async def _connect_livekit(self) -> None:
        token = self._create_token()
        await self._room.connect(
            self._args.livekit_url,
            token,
            rtc.RoomOptions(auto_subscribe=False),
        )
        logging.info("Operation Gateway connected: %s", self._args.room)
        for identity in self._room.remote_participants:
            await self._sync_participant(identity)

    # Unity 이벤트를 제한된 큐에 넣는다.
    def enqueue_unity_event(self, payload: bytes) -> None:
        if self._unity_events.full():
            try:
                self._unity_events.get_nowait()
            except asyncio.QueueEmpty:
                pass
        try:
            self._unity_events.put_nowait(payload)
        except asyncio.QueueFull:
            pass

    # LiveKit AI 검출 패킷 유입을 검증 전 단계에서 1초 단위로 기록한다.
    def _log_ai_detection_ingress(
        self,
        message: dict,
        participant_identity: str,
        participant_sid: str,
        now: float,
    ) -> None:
        self._ai_detection_ingress_log_pending_count += 1
        if (
            self._ai_detection_ingress_log_last_at > 0.0
            and now - self._ai_detection_ingress_log_last_at
            < AI_DETECTION_LOG_INTERVAL_SECONDS
        ):
            return

        boxes = message.get("boxes")
        box_count = len(boxes) if isinstance(boxes, list) else -1
        logging.info(
            "LiveKit ai-detection IN participant=%s sid=%s type=%s frame=%s "
            "boxes=%d source=%s track=%s packets=%d",
            participant_identity,
            participant_sid,
            message.get("type"),
            message.get("frame_id"),
            box_count,
            message.get("source", message.get("droneId")),
            message.get("source_track", message.get("sourceTrack")),
            self._ai_detection_ingress_log_pending_count,
        )
        self._ai_detection_ingress_log_last_at = now
        self._ai_detection_ingress_log_pending_count = 0

    # 거부된 LiveKit AI 검출과 정확한 검증 사유를 1초 단위로 기록한다.
    def _log_ai_detection_rejected(
        self,
        message: dict | None,
        participant_identity: str,
        participant_sid: str,
        reason: str,
        now: float,
    ) -> None:
        self._ai_detection_rejection_log_pending_count += 1
        if (
            self._ai_detection_rejection_log_last_at > 0.0
            and now - self._ai_detection_rejection_log_last_at
            < AI_DETECTION_LOG_INTERVAL_SECONDS
        ):
            return

        message = message or {}
        logging.warning(
            "LiveKit ai-detection REJECT participant=%s sid=%s reason=%s "
            "type=%s frame=%s source=%s track=%s packets=%d",
            participant_identity,
            participant_sid,
            reason,
            message.get("type"),
            message.get("frame_id"),
            message.get("source", message.get("droneId")),
            message.get("source_track", message.get("sourceTrack")),
            self._ai_detection_rejection_log_pending_count,
        )
        self._ai_detection_rejection_log_last_at = now
        self._ai_detection_rejection_log_pending_count = 0

    # 정상 수신한 LiveKit AI 검출을 1초 단위 요약 로그로 남긴다.
    def _log_ai_detection_received(
        self,
        event: dict,
        participant_identity: str,
        participant_sid: str,
        now: float,
    ) -> None:
        self._ai_detection_log_pending_count += 1
        if (
            self._ai_detection_log_last_at > 0.0
            and now - self._ai_detection_log_last_at
            < AI_DETECTION_LOG_INTERVAL_SECONDS
        ):
            return

        logging.info(
            "LiveKit ai-detection RX participant=%s sid=%s frame=%s boxes=%d "
            "source=%s track=%s frames=%d",
            participant_identity,
            participant_sid,
            event.get("sequence"),
            len(event.get("boxes") or ()),
            event.get("droneId"),
            event.get("sourceTrack"),
            self._ai_detection_log_pending_count,
        )
        self._ai_detection_log_last_at = now
        self._ai_detection_log_pending_count = 0

    # LiveKit 관리자 명령을 검증하고 처리한다.
    async def _handle_room_packet(self, packet: rtc.DataPacket) -> None:
        if packet.participant is None or packet.topic not in {
            COMMAND_TOPIC,
            ANNOTATION_TOPIC,
            SYNC_TOPIC,
            TARGET_POSITION_TOPIC,
            *MISSION_INPUT_TOPICS,
        }:
            return

        participant_identity = str(packet.participant.identity)
        participant_sid = str(
            getattr(packet.participant, "sid", "") or participant_identity
        )
        detection_log_now = (
            time.monotonic() if packet.topic == AI_DETECTION_TOPIC else 0.0
        )
        try:
            message = json.loads(packet.data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            if packet.topic == AI_DETECTION_TOPIC:
                self._log_ai_detection_rejected(
                    None,
                    participant_identity,
                    participant_sid,
                    "invalid-json",
                    detection_log_now,
                )
            return

        if not isinstance(message, dict):
            if packet.topic == AI_DETECTION_TOPIC:
                self._log_ai_detection_rejected(
                    None,
                    participant_identity,
                    participant_sid,
                    "invalid-message",
                    detection_log_now,
                )
            return

        message_type = message.get("type")
        if packet.topic == AI_DETECTION_TOPIC:
            self._log_ai_detection_ingress(
                message,
                participant_identity,
                participant_sid,
                detection_log_now,
            )
            event = self._mission_bridge.normalize_detection(
                message,
                participant_identity,
                participant_sid=participant_sid,
            )
            if event is not None:
                self._log_ai_detection_received(
                    event,
                    participant_identity,
                    participant_sid,
                    detection_log_now,
                )
                target_events = self._mission_bridge.track_active_targets(event)
                self._send_unity_command(event)
                await self._publish_event(event, reliable=False)
                for target_event in target_events:
                    self._revision += 1
                    target_event["revision"] = self._revision
                    self._send_unity_command(target_event)
                    await self._publish_event(
                        target_event,
                        reliable=target_event.get("active") is False,
                    )
            else:
                self._log_ai_detection_rejected(
                    message,
                    participant_identity,
                    participant_sid,
                    self._mission_bridge.last_detection_rejection_reason
                    or "unknown",
                    detection_log_now,
                )
            return
        if packet.topic == TELEMETRY_STRIKE_TOPIC:
            event = self._mission_bridge.normalize_flight_telemetry(
                message,
                participant_identity,
            )
            if event is not None:
                self._revision += 1
                event["revision"] = self._revision
                await self._publish_event(event, reliable=False)
            return
        if packet.topic == OBSERVATION_ANNOTATION_TOPIC:
            event = self._mission_bridge.normalize_annotation_receipt(
                message,
                participant_identity,
            )
            if event is not None:
                self._revision += 1
                event["revision"] = self._revision
                self._send_unity_command(event)
                await self._publish_event(event)
            return

        if message_type in {"state-request", "state-response"}:
            if message_type == "state-request":
                await self._publish_original_snapshot(participant_identity, packet.topic)
            return

        participant_role = self._participant_role(packet.participant)
        if packet.topic == TARGET_POSITION_TOPIC:
            if participant_role != "admin":
                logging.warning("권한 없는 CP 지도 핑 거부: %s", participant_identity)
                return
            await self._handle_cp_target_position(message, participant_identity)
            return

        if packet.topic == ANNOTATION_TOPIC:
            if message_type == "drawing":
                await self._add_drawing(message, participant_identity)
            elif message_type == "drawing-clear":
                await self._clear_drawing_surface(message, participant_identity)
            elif message_type == "drawing.add":
                drawing_id = str(message.get("id") or "").strip()
                if drawing_id:
                    drawing = dict(message)
                    drawing["id"] = self._deployed_drawing_id(
                        participant_identity,
                        drawing_id,
                    )
                    await self._add_drawing(drawing, participant_identity)
            elif message_type == "drawing.clear":
                await self._clear_drawing_surface(message, participant_identity)
            elif message_type == "state.request":
                await self._publish_deployed_annotation_state(
                    participant_identity,
                )
            elif message_type == "state.response":
                await self._restore_deployed_drawings(
                    message.get("state"),
                    participant_identity,
                )
                await self._restore_deployed_manual_targets(
                    message.get("state"),
                    participant_identity,
                )
            elif message_type == "target.add":
                target = message.get("target")
                if isinstance(target, dict):
                    target_id = str(target.get("id") or message.get("id") or "").strip()
                    message_id = str(message.get("id") or target_id).strip()
                    if target_id and message_id == target_id:
                        await self._add_manual_target(
                            {
                                "type": "manual-target",
                                "id": self._deployed_target_item_id(
                                    participant_identity,
                                    target_id,
                                ),
                                "targetId": target_id,
                                "source": message.get("source"),
                                "surface": target.get("surface"),
                                "x": target.get("x"),
                                "y": target.get("y"),
                            },
                            participant_identity,
                        )
            elif message_type == "target.remove":
                target_id = str(message.get("id") or "").strip()
                if target_id:
                    await self._delete_manual_target(
                        {
                            "type": "manual-target-delete",
                            "id": self._deployed_target_item_id(
                                participant_identity,
                                target_id,
                            ),
                        },
                        participant_identity,
                        participant_role == "admin",
                    )
            elif message_type == "target.clear":
                await self._clear_manual_targets(
                    participant_identity,
                    False,
                )
            elif message_type == "manual-target":
                await self._add_manual_target(message, participant_identity)
            elif message_type == "manual-target-delete":
                await self._delete_manual_target(
                    message,
                    participant_identity,
                    participant_role == "admin",
                )
            elif message_type == "stroke":
                await self._add_annotation(
                    {"type": "annotation-add", "annotation": dict(message)},
                    participant_identity,
                )
            # 배포된 HAMMER viewer target은 소유권이 있는 strike 수동 표적으로 제한 변환한다.
            elif (
                participant_role == "viewer"
                and participant_identity.startswith("viewer-hammer-")
                and message_type == "target"
            ):
                await self._add_manual_target(
                    {
                        "type": "manual-target",
                        "id": message.get("id"),
                        "targetId": message.get("id"),
                        "source": "HAMMER",
                        "surface": "strike",
                        "x": message.get("x"),
                        "y": message.get("y"),
                    },
                    participant_identity,
                )
            elif participant_role == "admin" and message_type == "target":
                await self._add_annotation(
                    {"type": "annotation-add", "annotation": dict(message)},
                    participant_identity,
                )
            elif participant_role == "admin" and message_type == "clear":
                await self._clear_annotations()
            else:
                logging.warning(
                    "권한 없거나 지원하지 않는 주석 명령 거부: %s",
                    participant_identity,
                )
            return

        if participant_role != "admin":
            logging.warning("권한 없는 명령 거부: %s", packet.participant.identity)
            return

        if packet.topic in {
            TARGET_SELECT_TOPIC,
            TARGET_LOCK_TOPIC,
            TARGET_UNLOCK_TOPIC,
        }:
            event = self._mission_bridge.normalize_target_state(
                packet.topic,
                message,
                participant_identity,
            )
            if event is not None:
                self._revision += 1
                event["revision"] = self._revision
                self._send_unity_command(event)
                await self._publish_event(event)
            return

        if packet.topic != COMMAND_TOPIC:
            return

        if message_type == "annotation-add":
            await self._add_annotation(message, packet.participant.identity)
        elif message_type == "annotation-clear":
            await self._clear_annotations()
        elif message_type == "ping-clear":
            self._send_unity_command({"type": "ping-clear"})
        elif message_type in {"ping-delete", "ping-delete-request"}:
            await self._forward_ping_delete(
                message,
                packet.participant.identity,
                "admin",
            )
        elif message_type == "map-ping-request":
            await self._forward_ping_request(
                message,
                packet.participant.identity,
                "admin",
            )

    # 관제 초기화를 중앙 상태와 Unity에 함께 반영한다.
    async def _clear_annotations(self) -> None:
        self._annotations.clear()
        self._revision += 1
        event = {"type": "annotation-clear", "revision": self._revision}
        self._send_unity_command(event)
        await self._publish_event(event)

    # 새 SAM3 자유 드로잉을 Unity가 처리하는 선분 주석으로 변환한다.
    async def _add_drawing(self, message: dict, identity: str) -> None:
        drawing_id = str(message.get("id") or "").strip()
        surface = str(message.get("surface") or "")
        mode = "fade" if message.get("mode") == "fade" else "keep"
        points = self._normalize_drawing_points(message.get("points"))
        if not drawing_id or len(drawing_id) > 100 or surface not in DRAWING_SURFACES:
            return
        if points is None:
            return
        if any(item.get("drawingId") == drawing_id for item in self._annotations.values()):
            return

        for index in range(1, len(points)):
            await self._add_annotation(
                {
                    "type": "annotation-add",
                    "annotation": {
                        "id": f"{drawing_id}-{index - 1:03d}",
                        "type": "stroke",
                        "start": points[index - 1],
                        "end": points[index],
                        "surface": surface,
                        "drawingId": drawing_id,
                        "mode": mode,
                    },
                },
                identity,
            )

        if mode == "fade":
            loop = asyncio.get_running_loop()
            loop.call_later(
                FADE_DRAWING_SECONDS,
                lambda: loop.create_task(
                    self._remove_drawing_annotations(drawing_id, identity, False)
                ),
            )

    # 현재 화면에서 요청자가 만든 드로잉만 제거한다.
    async def _clear_drawing_surface(self, message: dict, identity: str) -> None:
        surface = str(message.get("surface") or "")
        if surface not in DRAWING_SURFACES:
            return
        item_ids = [
            item_id
            for item_id, item in self._annotations.items()
            if item.get("surface") == surface
            and item.get("creator") == identity
            and item.get("type") == "stroke"
            and item.get("drawingId")
        ]
        await self._remove_annotations(item_ids)

    # 배포 웹의 화면용 드로잉 ID를 참가자별 중앙 상태 ID로 분리한다.
    def _deployed_drawing_id(self, identity: str, drawing_id: str) -> str:
        prefix = f"{identity}:"
        return drawing_id if drawing_id.startswith(prefix) else f"{prefix}{drawing_id}"

    # 배포 웹의 상태 응답에서 해당 참가자가 직접 만든 드로잉만 복원한다.
    async def _restore_deployed_drawings(self, state, identity: str) -> None:
        if not isinstance(state, dict):
            return
        drawings = state.get("drawings")
        if not isinstance(drawings, dict):
            return
        for surface, items in drawings.items():
            if surface not in DRAWING_SURFACES or not isinstance(items, list):
                continue
            for drawing in items:
                if not isinstance(drawing, dict):
                    continue
                drawing_id = str(drawing.get("id") or "").strip()
                if not drawing_id or ":" in drawing_id:
                    continue
                await self._add_drawing(
                    {
                        "id": self._deployed_drawing_id(identity, drawing_id),
                        "surface": surface,
                        "points": drawing.get("points"),
                        "mode": drawing.get("mode"),
                    },
                    identity,
                )

    # SAM3 수동 표적을 소유자 정보가 있는 target 주석으로 변환한다.
    async def _add_manual_target(self, message: dict, identity: str) -> None:
        item_id = str(message.get("id") or "").strip()
        target_id = str(message.get("targetId") or "").strip()
        source = str(message.get("source") or "").strip()
        surface = str(message.get("surface") or "")
        try:
            x = float(message["x"])
            y = float(message["y"])
        except (KeyError, TypeError, ValueError):
            return
        if (
            not item_id
            or len(item_id) > 100
            or not target_id
            or len(target_id) > 100
            or len(source) > 32
            or surface not in MANUAL_TARGET_SURFACES
            or not math.isfinite(x)
            or not math.isfinite(y)
        ):
            return
        existing = self._annotations.get(item_id)
        if existing is not None and existing.get("creator") != identity:
            return
        await self._add_annotation(
            {
                "type": "annotation-add",
                "annotation": {
                    "id": item_id,
                    "type": "target",
                    "targetId": target_id,
                    "source": source,
                    "surface": surface,
                    "manual": True,
                    "x": x,
                    "y": y,
                },
            },
            identity,
        )

    # 수동 표적은 작성자 또는 관리자만 삭제한다.
    async def _delete_manual_target(
        self,
        message: dict,
        identity: str,
        allow_any_creator: bool,
    ) -> None:
        item_id = str(message.get("id") or "").strip()
        item = self._annotations.get(item_id)
        if (
            not item_id
            or item is None
            or item.get("manual") is not True
            or item.get("type") != "target"
            or (not allow_any_creator and item.get("creator") != identity)
        ):
            return
        await self._remove_annotations([item_id])

    # 배포 웹의 화면용 표적 ID를 참가자별 중앙 상태 ID로 분리한다.
    def _deployed_target_item_id(self, identity: str, target_id: str) -> str:
        prefix = f"{identity}:"
        return target_id if target_id.startswith(prefix) else f"{prefix}{target_id}"

    # 배포 웹의 전체 삭제는 수동 표적에만 적용하고 작성자 권한을 유지한다.
    async def _clear_manual_targets(
        self,
        identity: str,
        allow_any_creator: bool,
    ) -> None:
        item_ids = [
            item_id
            for item_id, item in self._annotations.items()
            if item.get("manual") is True
            and item.get("type") == "target"
            and (allow_any_creator or item.get("creator") == identity)
        ]
        await self._remove_annotations(item_ids)

    # 배포 웹의 상태 응답에서 해당 참가자가 직접 만든 수동 표적만 복원한다.
    async def _restore_deployed_manual_targets(
        self,
        state,
        identity: str,
    ) -> None:
        if not isinstance(state, dict):
            return
        targets = state.get("manualTargets")
        if not isinstance(targets, list):
            return
        for target in targets:
            if not isinstance(target, dict):
                continue
            target_id = str(target.get("id") or "").strip()
            if not target_id or ":" in target_id:
                continue
            await self._add_manual_target(
                {
                    "type": "manual-target",
                    "id": self._deployed_target_item_id(identity, target_id),
                    "targetId": target_id,
                    "source": identity,
                    "surface": target.get("surface"),
                    "x": target.get("x"),
                    "y": target.get("y"),
                },
                identity,
            )

    # 중앙 상태와 Unity에서 단일 드로잉의 선분을 함께 제거한다.
    async def _remove_drawing_annotations(
        self,
        drawing_id: str,
        identity: str,
        allow_any_creator: bool,
    ) -> None:
        item_ids = [
            item_id
            for item_id, item in self._annotations.items()
            if item.get("drawingId") == drawing_id
            and (allow_any_creator or item.get("creator") == identity)
        ]
        await self._remove_annotations(item_ids)

    # 여러 주석을 개별 삭제 이벤트로 변환해 중앙 상태와 Unity에 반영한다.
    async def _remove_annotations(self, item_ids: list[str]) -> None:
        for item_id in item_ids:
            item = self._annotations.pop(item_id, None)
            if item is None:
                continue
            self._revision += 1
            event = {
                "type": "annotation-delete",
                "id": item_id,
                "revision": self._revision,
            }
            if self._annotation_for_unity(item):
                self._send_unity_command(event)
            await self._publish_event(event)

    # Unity가 확정한 핑 또는 텔레메트리를 LiveKit에 전달한다.
    async def _handle_unity_event(self, payload: bytes) -> None:
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return

        message_type = message.get("type")
        if message_type == "annotation-sync-request":
            self._send_unity_annotation_snapshot()
            self._send_unity_mission_snapshot()
        elif message_type == "mission-sync-request":
            self._send_unity_mission_snapshot()
        elif message_type == "flight-safety-state":
            event = self._mission_bridge.normalize_flight_safety_state(message)
            if event is not None:
                self._revision += 1
                event["revision"] = self._revision
                self._send_unity_command(event)
                await self._publish_event(event)
        elif message_type == "ping-sync-request":
            self._send_unity_command(self._map_state)
            self._send_unity_ping_snapshot()
        elif message_type == "map-ping-request":
            await self._forward_ping_request(
                message,
                str(message.get("requesterIdentity") or "quest-pilot"),
                "pilot",
            )
        elif message_type == "ping-delete-request":
            await self._forward_ping_delete(
                message,
                str(message.get("requesterIdentity") or "quest-pilot"),
                "pilot",
            )
        elif message_type == "ping-event":
            if not self._matches_map(message):
                return
            ping_id = str(message.get("id", ""))
            if not ping_id:
                return
            request_id = str(message.get("requestId", ""))
            if request_id:
                self._pending_ping_requests.pop(request_id, None)
            self._revision += 1
            message["revision"] = self._revision
            self._pings[ping_id] = message
            removed_ping_ids: list[str] = []
            while len(self._pings) > MAX_STATE_ITEMS:
                oldest_id = next(iter(self._pings))
                self._pings.pop(oldest_id)
                removed_ping_ids.append(oldest_id)
            await self._publish_event(message)
            cp_target = self._cp_target_positions.get(ping_id)
            if cp_target is not None:
                await self._publish_topic(
                    {
                        "type": "target.position.received",
                        "target_id": cp_target["target_id"],
                        "source": "operation-gateway",
                        "timestamp": int(time.time() * 1000),
                    },
                    TARGET_POSITION_TOPIC,
                    [cp_target["identity"]],
                    True,
                )
            for removed_id in removed_ping_ids:
                self._send_unity_command({"type": "ping-delete", "id": removed_id})
                self._revision += 1
                await self._publish_event(
                    {
                        "type": "ping-deleted",
                        "id": removed_id,
                        "revision": self._revision,
                    }
                )
        elif message_type == "ping-rejected":
            request_id = str(message.get("requestId", ""))
            pending = self._pending_ping_requests.pop(request_id, None)
            destination = message.get("requesterIdentity")
            requester_role = str(message.get("requesterRole") or "")
            if pending is not None:
                destination = pending["identity"]
                requester_role = pending["role"]
            await self._send_ping_rejection(message, destination, requester_role)
        elif message_type == "ping-deleted":
            request_id = str(message.get("requestId", ""))
            if request_id:
                self._pending_ping_requests.pop(request_id, None)
            ping_id = str(message.get("id", ""))
            if self._pings.pop(ping_id, None) is not None:
                self._revision += 1
                message["revision"] = self._revision
                await self._publish_event(message)
            cp_target = self._cp_target_positions.pop(ping_id, None)
            if cp_target is not None:
                await self._publish_topic(
                    {
                        "type": "target.position.cleared",
                        "target_id": cp_target["target_id"],
                        "source": "operation-gateway",
                        "timestamp": int(time.time() * 1000),
                    },
                    TARGET_POSITION_TOPIC,
                    [cp_target["identity"]],
                    True,
                )
        elif message_type == "ping-cleared":
            self._pings.clear()
            self._revision += 1
            await self._publish_event({"type": "ping-clear", "revision": self._revision})
        elif message_type == "drone-telemetry":
            if not self._matches_map(message):
                return
            self._last_drone_telemetry = message
            await self._publish_event(message, reliable=False)
            deployed_position = self._deployed_drone_position(message)
            if deployed_position is not None:
                await self._publish_topic(
                    deployed_position,
                    DRONE_POSITION_TOPIC,
                    reliable=False,
                )
        elif message_type == "drone-heading":
            if not self._matches_map(message):
                return
            raw_heading = message.get("headingDegrees")
            if isinstance(raw_heading, bool):
                return
            try:
                numeric_heading = float(raw_heading)
            except (TypeError, ValueError):
                return
            if not math.isfinite(numeric_heading):
                return
            heading_degrees = int(numeric_heading)
            if (
                numeric_heading != heading_degrees
                or not 0 <= heading_degrees < 360
                or message.get("headingReference") != "apriltag-board-forward"
            ):
                return
            if (
                self._last_drone_heading is not None
                and self._last_drone_heading.get("headingDegrees") == heading_degrees
                and self._last_drone_heading.get("mapId") == message.get("mapId")
                and self._last_drone_heading.get("mapVersion") == message.get("mapVersion")
                and self._last_drone_heading.get("scanId") == message.get("scanId")
            ):
                return
            message["headingDegrees"] = heading_degrees
            self._last_drone_heading = message
            await self._publish_event(message)

    # 비디오 주석을 중앙 상태에 추가한다.
    async def _add_annotation(self, message: dict, identity: str) -> None:
        item = message.get("annotation")
        if not isinstance(item, dict) or item.get("type") not in {"target", "stroke"}:
            return
        item_id = str(item.get("id") or uuid.uuid4())
        item["id"] = item_id
        item["creator"] = identity
        item["timestamp"] = int(time.time() * 1000)

        if not self._validate_annotation(item):
            return

        self._revision += 1
        event = {"type": "annotation-event", "revision": self._revision, "annotation": item}
        self._annotations[item_id] = item
        self._trim_state(self._annotations)
        if self._annotation_for_unity(item):
            self._send_unity_command(event)
        await self._publish_event(event, reliable=item.get("type") != "stroke-preview")

    # 지도 핑 요청을 Unity로 전달한다.
    async def _forward_ping_request(
        self,
        message: dict,
        identity: str,
        requester_role: str,
        replace_existing: bool = False,
    ) -> None:
        request_id = str(message.get("requestId") or uuid.uuid4())
        if request_id in self._pings and not replace_existing:
            if requester_role == "pilot":
                self._send_unity_command(self._pings[request_id])
            else:
                await self._publish_event(self._pings[request_id], [identity])
            return
        if request_id in self._pending_ping_requests:
            return

        if not self._matches_map(message):
            await self._send_ping_rejection(
                {
                    "type": "ping-rejected",
                    "requestId": request_id,
                    "requesterIdentity": identity,
                    "requesterRole": requester_role,
                    "reason": "지도 버전이 일치하지 않습니다.",
                },
                identity,
                requester_role,
            )
            return

        try:
            map_x = float(message["mapX"])
            map_z = float(message["mapZ"])
        except (KeyError, TypeError, ValueError):
            await self._send_ping_rejection(
                {
                    "type": "ping-rejected",
                    "requestId": request_id,
                    "requesterIdentity": identity,
                    "requesterRole": requester_role,
                    "reason": "핑 좌표가 올바르지 않습니다.",
                },
                identity,
                requester_role,
            )
            return

        if not VenueBounds.from_mapping(self._map).contains(map_x, map_z):
            await self._send_ping_rejection(
                {
                    "type": "ping-rejected",
                    "requestId": request_id,
                    "requesterIdentity": identity,
                    "requesterRole": requester_role,
                    "reason": "지도 경계를 벗어난 좌표입니다.",
                },
                identity,
                requester_role,
            )
            return

        command = {
            "type": "map-ping-request",
            "requestId": request_id,
            "mapId": self._map["mapId"],
            "mapVersion": self._map["version"],
            "scanId": self._map["scanId"],
            "mapX": map_x,
            "mapZ": map_z,
            "requesterIdentity": identity,
            "requesterRole": requester_role,
            "timestamp": int(time.time() * 1000),
        }
        if self._unity_transport is not None:
            self._track_ping_request(request_id, identity, requester_role, "create")
            self._send_unity_command(command)
        else:
            await self._send_ping_rejection(
                {
                    "type": "ping-rejected",
                    "requestId": request_id,
                    "requesterIdentity": identity,
                    "requesterRole": requester_role,
                    "reason": "Unity 핑 브리지가 연결되지 않았습니다.",
                },
                identity,
                requester_role,
            )

    # CP의 정규화 지도 좌표를 현재 venue 지도 핑 명령으로 변환한다.
    async def _handle_cp_target_position(self, message: dict, identity: str) -> None:
        message_type = str(message.get("type") or "")
        target_id = str(message.get("target_id") or "").strip()
        if not target_id or len(target_id) > 100:
            return

        ping_id = self._cp_target_ping_id(target_id)
        if message_type == "target.position.clear":
            self._cp_target_positions[ping_id] = {
                "identity": identity,
                "target_id": target_id,
            }
            await self._forward_ping_delete(
                {
                    "requestId": f"{ping_id}-delete-{uuid.uuid4().hex[:8]}",
                    "id": ping_id,
                },
                identity,
                "admin",
            )
            return
        if message_type != "target.position":
            logging.warning(
                "지원하지 않는 CP 지도 핑 명령 거부: %s (%s)",
                identity,
                message_type,
            )
            return

        position = message.get("position")
        if not isinstance(position, dict):
            return
        try:
            normalized_x = float(position["x"])
            normalized_y = float(position["y"])
        except (KeyError, TypeError, ValueError):
            return
        if (
            not math.isfinite(normalized_x)
            or not math.isfinite(normalized_y)
            or not 0.0 <= normalized_x <= 1.0
            or not 0.0 <= normalized_y <= 1.0
        ):
            return

        # CP 미니맵은 canonical venue-map.png를 좌우 반전해 표시한다.
        # 화면 X를 원본 지도 X로 되돌린 뒤 Venue 미터 좌표로 변환한다.
        venue_normalized_x = 1.0 - normalized_x
        map_x, map_z = normalized_to_venue(
            VenueBounds.from_mapping(self._map),
            venue_normalized_x,
            normalized_y,
        )
        self._cp_target_positions[ping_id] = {
            "identity": identity,
            "target_id": target_id,
        }
        await self._forward_ping_request(
            {
                "type": "map-ping-request",
                "requestId": ping_id,
                "mapId": self._map["mapId"],
                "mapVersion": self._map["version"],
                "scanId": self._map["scanId"],
                "mapX": map_x,
                "mapZ": map_z,
            },
            identity,
            "admin",
            replace_existing=True,
        )

    # CP target ID를 재접속 후에도 유지되는 고정 길이 Unity 핑 ID로 만든다.
    @staticmethod
    def _cp_target_ping_id(target_id: str) -> str:
        digest = hashlib.sha256(target_id.encode("utf-8")).hexdigest()[:24]
        return f"cp-target-{digest}"

    # 파일럿 소유권 또는 관리자 권한을 확인한 뒤 Unity 삭제 명령을 보낸다.
    async def _forward_ping_delete(
        self,
        message: dict,
        identity: str,
        requester_role: str,
    ) -> None:
        request_id = str(message.get("requestId") or uuid.uuid4())
        ping_id = str(message.get("id", ""))
        ping = self._pings.get(ping_id)
        if ping is None:
            await self._send_ping_rejection(
                {
                    "type": "ping-rejected",
                    "requestId": request_id,
                    "requesterIdentity": identity,
                    "requesterRole": requester_role,
                    "reason": "삭제할 핑이 없습니다.",
                },
                identity,
                requester_role,
            )
            return
        if requester_role != "admin" and ping.get("requesterIdentity") != identity:
            await self._send_ping_rejection(
                {
                    "type": "ping-rejected",
                    "requestId": request_id,
                    "requesterIdentity": identity,
                    "requesterRole": requester_role,
                    "reason": "자신이 만든 핑만 삭제할 수 있습니다.",
                },
                identity,
                requester_role,
            )
            return

        self._track_ping_request(request_id, identity, requester_role, "delete")
        self._send_unity_command(
            {
                "type": "ping-delete",
                "requestId": request_id,
                "id": ping_id,
                "requesterIdentity": identity,
                "requesterRole": requester_role,
            }
        )

    # Unity 응답을 기다리는 핑 명령을 세션 메모리에 기록한다.
    def _track_ping_request(
        self,
        request_id: str,
        identity: str,
        requester_role: str,
        operation: str,
    ) -> None:
        self._pending_ping_requests[request_id] = {
            "identity": identity,
            "role": requester_role,
            "operation": operation,
            "deadline": time.monotonic() + PING_RESPONSE_TIMEOUT_SECONDS,
        }

    # 제한 시간 안에 Unity 응답이 없으면 요청자에게 실패를 알린다.
    async def _expire_ping_requests(self) -> None:
        now = time.monotonic()
        expired = [
            request_id
            for request_id, pending in self._pending_ping_requests.items()
            if pending["deadline"] <= now
        ]
        for request_id in expired:
            pending = self._pending_ping_requests.pop(request_id)
            await self._send_ping_rejection(
                {
                    "type": "ping-rejected",
                    "requestId": request_id,
                    "requesterIdentity": pending["identity"],
                    "requesterRole": pending["role"],
                    "reason": "Unity가 핑 요청에 응답하지 않았습니다.",
                },
                pending["identity"],
                pending["role"],
            )

    # 파일럿 실패는 localhost Unity로, 관리자 실패는 LiveKit으로 보낸다.
    async def _send_ping_rejection(
        self,
        message: dict,
        identity: str | None,
        requester_role: str,
    ) -> None:
        if requester_role == "pilot":
            self._send_unity_command(message)
            return
        await self._publish_event(message, [identity] if identity else None)

    # 메시지가 현재 스캔 ID와 지도 버전에 속하는지 확인한다.
    def _matches_map(self, message: dict) -> bool:
        return (
            message.get("mapId") == self._map["mapId"]
            and message.get("scanId") == self._map["scanId"]
            and int(message.get("mapVersion", -1)) == int(self._map["version"])
        )

    # JSON 명령을 Unity localhost 브리지로 보낸다.
    def _send_unity_command(self, command: dict) -> None:
        if self._unity_transport is None:
            return
        self._unity_transport.sendto(
            json.dumps(command, separators=(",", ":")).encode("utf-8"),
            self._unity_target,
        )

    # Unity 파일럿에게 현재 영상 주석을 청크 단위로 동기화한다.
    def _send_unity_annotation_snapshot(self) -> None:
        annotations = [
            item for item in self._annotations.values() if self._annotation_for_unity(item)
        ]
        chunks = [
            annotations[index:index + SNAPSHOT_CHUNK_SIZE]
            for index in range(0, len(annotations), SNAPSHOT_CHUNK_SIZE)
        ] or [[]]
        snapshot_id = str(uuid.uuid4())
        for index, chunk in enumerate(chunks):
            self._send_unity_command(
                {
                    "type": "annotation-snapshot",
                    "snapshotId": snapshot_id,
                    "chunkIndex": index,
                    "chunkCount": len(chunks),
                    "revision": self._revision,
                    "annotations": chunk,
                }
            )

    # 현재 핑을 UDP 크기를 제한한 청크로 Unity에 동기화한다.
    def _send_unity_ping_snapshot(self) -> None:
        pings = list(self._pings.values())
        chunks = [
            pings[index:index + SNAPSHOT_CHUNK_SIZE]
            for index in range(0, len(pings), SNAPSHOT_CHUNK_SIZE)
        ] or [[]]
        snapshot_id = str(uuid.uuid4())
        for index, chunk in enumerate(chunks):
            self._send_unity_command(
                {
                    "type": "ping-snapshot",
                    "snapshotId": snapshot_id,
                    "chunkIndex": index,
                    "chunkCount": len(chunks),
                    "revision": self._revision,
                    "pings": chunk,
                }
            )

    # 선택 target과 안전 상태를 Unity 재접속 시 다시 전달한다.
    def _send_unity_mission_snapshot(self) -> None:
        for target_state in self._mission_bridge.target_states:
            self._send_unity_command(target_state)
        safety_state = self._mission_bridge.flight_safety_state
        if safety_state is not None:
            self._send_unity_command(safety_state)

    # 현재 작전 상태를 참가자에게 분할 전송한다.
    async def _publish_snapshot(self, identity: str) -> None:
        items = [
            {"kind": "map", "value": self._map_state},
            *({"kind": "annotation", "value": value} for value in self._annotations.values()),
            *({"kind": "ping", "value": value} for value in self._pings.values()),
        ]
        if self._last_drone_telemetry is not None:
            items.append({"kind": "drone", "value": self._last_drone_telemetry})
        if self._last_drone_heading is not None:
            items.append({"kind": "heading", "value": self._last_drone_heading})
        items.extend(
            {"kind": "mission-target", "value": value}
            for value in self._mission_bridge.target_states
        )
        flight_telemetry = self._mission_bridge.flight_telemetry
        if flight_telemetry is not None:
            items.append({"kind": "flight-telemetry", "value": flight_telemetry})
        flight_safety_state = self._mission_bridge.flight_safety_state
        if flight_safety_state is not None:
            items.append({"kind": "flight-safety", "value": flight_safety_state})
        chunks = [items[index:index + SNAPSHOT_CHUNK_SIZE] for index in range(0, len(items), SNAPSHOT_CHUNK_SIZE)] or [[]]
        snapshot_id = str(uuid.uuid4())
        for index, chunk in enumerate(chunks):
            await self._publish_event(
                {
                    "type": "state-snapshot",
                    "snapshotId": snapshot_id,
                    "chunkIndex": index,
                    "chunkCount": len(chunks),
                    "revision": self._revision,
                    "items": chunk,
                },
                [identity],
            )

    # 기존 상태 스냅샷과 배포 웹의 공유 주석 요청을 함께 보낸다.
    async def _sync_participant(self, identity: str) -> None:
        await self._publish_snapshot(identity)
        if self._last_drone_telemetry is not None:
            deployed_position = self._deployed_drone_position(
                self._last_drone_telemetry
            )
            if deployed_position is not None:
                await self._publish_topic(
                    deployed_position,
                    DRONE_POSITION_TOPIC,
                    [identity],
                    False,
                )
        await self._publish_topic(
            {
                "type": "state.request",
                "source": "operation-gateway",
                "timestamp": int(time.time() * 1000),
            },
            ANNOTATION_TOPIC,
            [identity],
            True,
        )

    # Unity Venue X/Z 위치를 배포 웹의 좌상단 정규화 좌표로 변환한다.
    def _deployed_drone_position(self, message: dict) -> dict | None:
        try:
            map_x = float(message["mapX"])
            map_z = float(message["mapZ"])
        except (KeyError, TypeError, ValueError):
            return None
        bounds = VenueBounds.from_mapping(self._map)
        if not bounds.contains(map_x, map_z):
            return None
        normalized_x, normalized_y = venue_to_normalized(bounds, map_x, map_z)
        try:
            timestamp = int(message.get("timestamp"))
        except (TypeError, ValueError, OverflowError):
            timestamp = int(time.time() * 1000)
        return {
            "type": "drone.position",
            "target_id": self._drone_target_id,
            "source": "operation-gateway",
            "source_track": f"video-{self._drone_target_id}",
            "position": {"x": normalized_x, "y": normalized_y},
            "timestamp": timestamp,
        }

    # 배포 웹이 이해하는 작성자별 drawings/manualTargets 상태로 응답한다.
    async def _publish_deployed_annotation_state(self, identity: str) -> None:
        states = self._deployed_annotation_states()
        if not states:
            states = {"operation-gateway": {"drawings": {}, "manualTargets": []}}
        for source, state in states.items():
            await self._publish_topic(
                {
                    "type": "state.response",
                    "source": source,
                    "state": state,
                    "timestamp": int(time.time() * 1000),
                },
                ANNOTATION_TOPIC,
                [identity],
                True,
            )

    # 중앙 선분·표적 상태를 배포 웹의 공유 상태 구조로 재구성한다.
    def _deployed_annotation_states(self) -> dict[str, dict]:
        states: dict[str, dict] = {}
        drawing_groups: dict[tuple[str, str, str, str], list[dict]] = {}
        for item in self._annotations.values():
            creator = str(item.get("creator") or "operation-gateway")
            if item.get("type") == "target" and item.get("manual") is True:
                state = states.setdefault(
                    creator,
                    {"drawings": {}, "manualTargets": []},
                )
                state["manualTargets"].append(
                    {
                        "id": str(item.get("targetId") or item.get("id") or ""),
                        "surface": item.get("surface"),
                        "x": item.get("x"),
                        "y": item.get("y"),
                    }
                )
                continue
            drawing_id = str(item.get("drawingId") or "")
            surface = str(item.get("surface") or "")
            if (
                item.get("type") == "stroke"
                and drawing_id
                and surface in DRAWING_SURFACES
            ):
                mode = "fade" if item.get("mode") == "fade" else "keep"
                drawing_groups.setdefault(
                    (creator, drawing_id, surface, mode),
                    [],
                ).append(item)

        for (creator, drawing_id, surface, mode), segments in drawing_groups.items():
            segments.sort(key=lambda item: str(item.get("id") or ""))
            points: list[dict] = []
            for segment in segments:
                start = segment.get("start")
                end = segment.get("end")
                if not isinstance(start, dict) or not isinstance(end, dict):
                    continue
                if not points or points[-1] != start:
                    points.append(dict(start))
                points.append(dict(end))
            if len(points) < 2:
                continue
            prefix = f"{creator}:"
            local_drawing_id = (
                drawing_id[len(prefix):]
                if drawing_id.startswith(prefix)
                else drawing_id
            )
            state = states.setdefault(
                creator,
                {"drawings": {}, "manualTargets": []},
            )
            drawings = state["drawings"]
            drawings.setdefault(surface, []).append(
                {
                    "id": local_drawing_id,
                    "points": points,
                    "mode": mode,
                }
            )
        return states

    # 팀 Viewer 상태 요청에 중앙 마킹 상태를 응답한다.
    async def _publish_original_snapshot(self, identity: str, topic: str) -> None:
        await self._publish_topic(
            {
                "type": "state-response",
                "annotations": list(self._annotations.values()),
                "missionTargets": self._mission_bridge.target_states,
                "flightTelemetry": self._mission_bridge.flight_telemetry,
                "flightSafetyState": self._mission_bridge.flight_safety_state,
                "subtitles": [],
                "revision": self._revision,
                "timestamp": int(time.time() * 1000),
            },
            topic,
            [identity],
            True,
        )

    # LiveKit Data Packet으로 이벤트를 발행한다.
    async def _publish_event(
        self,
        message: dict,
        destination_identities: list[str] | None = None,
        reliable: bool = True,
    ) -> None:
        await self._publish_topic(
            message,
            EVENT_TOPIC,
            destination_identities,
            reliable,
        )

    # 지정 LiveKit 토픽으로 JSON 데이터를 발행한다.
    async def _publish_topic(
        self,
        message: dict,
        topic: str,
        destination_identities: list[str] | None = None,
        reliable: bool = True,
    ) -> None:
        if not self._room.isconnected():
            logging.debug("LiveKit disconnected; skipped topic: %s", topic)
            return
        await self._room.local_participant.publish_data(
            json.dumps(message, separators=(",", ":"), ensure_ascii=False),
            reliable=reliable,
            destination_identities=destination_identities or [],
            topic=topic,
        )

    # Unity 양방향 UDP 채널을 연다.
    async def _connect_unity_bridge(self) -> None:
        transport, _ = await self._loop.create_datagram_endpoint(
            lambda: UnityEventProtocol(self),
            local_addr=("127.0.0.1", self._args.unity_event_port),
        )
        self._unity_transport = transport

    # Gateway용 LiveKit JWT를 생성한다.
    def _create_token(self) -> str:
        grants = api.VideoGrants(
            room_join=True,
            room=self._args.room,
            can_publish=False,
            can_subscribe=False,
            can_publish_data=True,
        )
        return (
            api.AccessToken(self._args.api_key, self._args.api_secret)
            .with_identity("operation-gateway")
            .with_name("Operation Gateway")
            .with_metadata(json.dumps({"role": "gateway"}))
            .with_grants(grants)
            .to_jwt()
        )

    # LiveKit 참가자의 서명된 역할 메타데이터를 읽는다.
    @staticmethod
    def _participant_role(participant: rtc.RemoteParticipant) -> str:
        identity_role = (
            "admin"
            if str(participant.identity).startswith("admin-")
            else "viewer"
        )
        metadata = participant.metadata or ""
        if not metadata:
            return identity_role

        try:
            role = str(json.loads(metadata).get("role", ""))
        except json.JSONDecodeError:
            return "viewer"
        if role in {"admin", "viewer"}:
            return role
        return identity_role

    # 주석 좌표 범위를 검사한다.
    @staticmethod
    def _validate_annotation(item: dict) -> bool:
        def point_valid(point: dict) -> bool:
            try:
                return 0.0 <= float(point["x"]) <= 1.0 and 0.0 <= float(point["y"]) <= 1.0
            except (KeyError, TypeError, ValueError):
                return False

        if item["type"] == "target":
            return point_valid(item)
        return point_valid(item.get("start", {})) and point_valid(item.get("end", {}))

    # 자유 드로잉 점을 유한한 정규화 좌표로 검증하고 float로 고정한다.
    @staticmethod
    def _normalize_drawing_points(points) -> list[dict] | None:
        if (
            not isinstance(points, list)
            or not 2 <= len(points) <= MAX_DRAWING_INPUT_POINTS
        ):
            return None
        normalized = []
        for point in points:
            if not isinstance(point, dict):
                return None
            try:
                x = float(point["x"])
                y = float(point["y"])
            except (KeyError, TypeError, ValueError):
                return None
            if not math.isfinite(x) or not math.isfinite(y):
                return None
            if not 0.0 <= x <= 1.0 or not 0.0 <= y <= 1.0:
                return None
            normalized.append({"x": x, "y": y})
        if len(normalized) > MAX_DRAWING_POINTS:
            last_index = len(normalized) - 1
            normalized = [
                normalized[round(index * last_index / (MAX_DRAWING_POINTS - 1))]
                for index in range(MAX_DRAWING_POINTS)
            ]
        return normalized

    # Unity FPV에는 strike 또는 기존 surface 미지정 주석만 전달한다.
    @staticmethod
    def _annotation_for_unity(item: dict) -> bool:
        return str(item.get("surface") or "strike") == "strike"

    # 상태 항목 수를 제한한다.
    @staticmethod
    def _trim_state(state: dict[str, dict]) -> None:
        while len(state) > MAX_STATE_ITEMS:
            state.pop(next(iter(state)))

    # 지도 파일을 읽고 필수 범위를 검사한다.
    @staticmethod
    def _load_map(path: Path) -> dict:
        data = json.loads(path.read_text(encoding="utf-8"))
        required = {"mapId", "scanId", "version", "minX", "maxX", "minZ", "maxZ"}
        missing = required.difference(data)
        if missing:
            raise RuntimeError(f"지도 설정 누락: {', '.join(sorted(missing))}")
        try:
            VenueBounds.from_mapping(data)
        except ValueError as error:
            raise RuntimeError(str(error)) from error
        return data

    # 지도 manifest와 이미지의 공개 위치 및 SHA-256을 공유 상태로 만든다.
    @staticmethod
    def _build_map_state(path: Path, data: dict, public_base_url: str) -> dict:
        image_path = path.parent / Path(str(data.get("imagePath", "venue-map.png"))).name
        if not image_path.is_file():
            raise RuntimeError(f"지도 이미지를 찾을 수 없습니다: {image_path}")

        bounds = VenueBounds.from_mapping(data)
        min_x = bounds.min_x
        max_x = bounds.max_x
        min_z = bounds.min_z
        max_z = bounds.max_z
        map_width_meters = bounds.width
        map_height_meters = bounds.depth
        image_width_pixels = int(data.get("imageWidth", 0))
        image_height_pixels = int(data.get("imageHeight", 0))
        if (
            not all(math.isfinite(value) for value in (min_x, max_x, min_z, max_z))
            or map_width_meters <= 0
            or map_height_meters <= 0
            or image_width_pixels <= 0
            or image_height_pixels <= 0
        ):
            raise RuntimeError("지도 경계 또는 이미지 크기가 올바르지 않습니다")

        base_url = public_base_url.rstrip("/")
        manifest_name = path.name
        image_name = image_path.name
        manifest_url = f"{base_url}/{manifest_name}" if base_url else f"/{manifest_name}"
        image_url = f"{base_url}/{image_name}" if base_url else f"/{image_name}"
        return {
            "type": "map-state",
            "mapId": data["mapId"],
            "mapVersion": int(data["version"]),
            "scanId": data["scanId"],
            "name": str(data.get("name", "")),
            "coordinateSpace": VENUE_XZ_METERS,
            "minX": min_x,
            "maxX": max_x,
            "minZ": min_z,
            "maxZ": max_z,
            "mapWidthMeters": map_width_meters,
            "mapHeightMeters": map_height_meters,
            "imageWidthPixels": image_width_pixels,
            "imageHeightPixels": image_height_pixels,
            "metersPerPixelX": map_width_meters / image_width_pixels,
            "metersPerPixelZ": map_height_meters / image_height_pixels,
            "imageOriginMapX": min_x,
            "imageOriginMapZ": max_z,
            "manifestUrl": manifest_url,
            "imageUrl": image_url,
            "manifestSha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "imageSha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
        }


# 환경변수와 명령행 설정을 읽는다.
def parse_args() -> argparse.Namespace:
    load_dotenv()
    default_map = Path(__file__).resolve().parent.parent / "Assets" / "Maps" / "current" / "venue-map.json"
    parser = argparse.ArgumentParser(description="LiveKit 작전 데이터 Gateway")
    parser.add_argument("--livekit-url", default=os.getenv("LIVEKIT_URL"))
    parser.add_argument("--api-key", default=os.getenv("LIVEKIT_API_KEY"))
    parser.add_argument("--api-secret", default=os.getenv("LIVEKIT_API_SECRET"))
    parser.add_argument("--room", default=os.getenv("LIVEKIT_ROOM", "mission-alpha"))
    parser.add_argument("--map-path", default=str(default_map))
    parser.add_argument(
        "--map-public-base-url",
        default=os.getenv("MAP_PUBLIC_BASE_URL", ""),
    )
    parser.add_argument("--unity-command-port", type=int, default=49300)
    parser.add_argument("--unity-event-port", type=int, default=49301)
    parser.add_argument(
        "--command-center-identity",
        default=os.getenv("MISSION_COMMAND_CENTER_IDENTITY", "c2-01"),
    )
    parser.add_argument(
        "--drone-identity",
        default=os.getenv("MISSION_DRONE_IDENTITY", "tello-publisher"),
    )
    parser.add_argument(
        "--video-track",
        default=os.getenv("MISSION_VIDEO_TRACK", "video-strike"),
    )
    parser.add_argument(
        "--mission-session-id",
        default=(
            os.getenv("STRIKERPC_MISSION_SESSION_ID")
            or os.getenv("DRONEOPS_MISSION_SESSION_ID")
            or str(uuid.uuid4())
        ),
    )
    args = parser.parse_args()
    if not args.livekit_url or not args.api_key or not args.api_secret:
        parser.error("LIVEKIT_URL, LIVEKIT_API_KEY, LIVEKIT_API_SECRET이 필요합니다.")
    return args


# 실행 중인 이벤트 루프 안에서 Gateway를 생성하고 시작한다.
async def _run_gateway(args: argparse.Namespace) -> None:
    gateway = OperationGateway(args)
    await gateway.run()


# Operation Gateway를 실행한다.
def main() -> int:
    try:
        asyncio.run(_run_gateway(parse_args()))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
