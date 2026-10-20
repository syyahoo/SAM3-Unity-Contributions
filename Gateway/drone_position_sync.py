from __future__ import annotations

import argparse
import json
import locale
import math
import re
import select
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field

from pose_protocol import (
    LOCAL_MAP_DEGRADED_FLAG,
    LOCAL_MAP_READY_FLAG,
    RELOCALIZING_FLAG,
    SESSION_MAP_FLAG,
    PosePacketV2,
    TrackingMode,
)


WIFI_SIGNAL_REFRESH_SECONDS = 2.0
FRAME_MOTION_FRESH_SECONDS = 0.35
FRAME_MOTION_MIN_TRACKED_POINTS = 60
FRAME_MOTION_MIN_INLIER_RATIO = 0.7
FRAME_MOTION_MIN_CONFIDENCE = 0.8
FRAME_MOTION_MAX_STATIONARY_SPEED_MPS = 0.03
FRAME_MOTION_MIN_STATIONARY_STREAK = 3
FRAME_MOTION_MAX_ATTITUDE_DELTA_DEGREES = 2.0
FRAME_MOTION_MAX_HEIGHT_DELTA_METERS = 0.08
FRAME_MOTION_MAX_SOURCE_AGE_SECONDS = 0.25
FRAME_MOTION_MAX_FUTURE_LEAD_SECONDS = 0.05
FRAME_MOTION_MAX_SESSION_GAP_SECONDS = 0.6
FRAME_MOTION_STATIONARY_MAX_PIXELS = 0.6
FRAME_MOTION_STATIONARY_MAX_ROTATION_DEGREES = 0.35
FRAME_MOTION_STATIONARY_MAX_SCALE_DELTA = 0.006
FRAME_MOTION_INLIER_RATIO_TOLERANCE = 0.02
CONTROL_MOTION_FRESH_SECONDS = 0.25
CONTROL_MOTION_MAX_SOURCE_AGE_SECONDS = 0.25
CONTROL_MOTION_MAX_FUTURE_LEAD_SECONDS = 0.05
CONTROL_MOTION_MAX_SPEED_MPS = 1.0
GROUNDED_MAX_HEIGHT_METERS = 0.05


def parse_windows_wifi_signal_percent(output: str) -> int | None:
    for line in output.splitlines():
        label, separator, raw_value = line.partition(":")
        if not separator or label.strip().casefold() not in {"signal", "신호"}:
            continue
        match = re.fullmatch(r"\s*(\d{1,3})%\s*", raw_value)
        if match is None:
            continue
        value = int(match.group(1))
        return value if 0 <= value <= 100 else None
    return None


def read_windows_wifi_signal_percent() -> int | None:
    if sys.platform != "win32":
        return None
    creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    try:
        result = subprocess.run(
            ["netsh", "wlan", "show", "interfaces"],
            check=False,
            capture_output=True,
            timeout=1.0,
            creationflags=creation_flags,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    output = result.stdout.decode(
        locale.getpreferredencoding(False),
        errors="replace",
    )
    return parse_windows_wifi_signal_percent(output)


def append_wifi_signal(payload: bytes, signal_percent: int | None) -> bytes:
    if signal_percent is None:
        return payload
    value = max(0, min(100, int(signal_percent)))
    separator = b"" if payload.endswith(b";") else b";"
    return payload + separator + f"wifi:{value};".encode("ascii")


class WifiSignalMonitor:
    def __init__(self, refresh_seconds: float = WIFI_SIGNAL_REFRESH_SECONDS) -> None:
        self.refresh_seconds = max(0.5, refresh_seconds)
        self.signal_percent: int | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="WifiSignalMonitor",
        )
        self._thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.2)
        self._thread = None

    def _run(self) -> None:
        while not self._stop_event.is_set():
            self.signal_percent = read_windows_wifi_signal_percent()
            self._stop_event.wait(self.refresh_seconds)


@dataclass
class TelloState:
    received_at: float = 0.0
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    height_m: float = 0.0
    tof_m: float = 0.0
    barometer_m: float = 0.0
    acceleration: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class FrameMotionState:
    received_at: float = 0.0
    sequence: int = 0
    source_timestamp_ns: int = 0
    stationary: bool = False
    confidence: float = 0.0
    tracked_points: int = 0
    inlier_count: int = 0
    inlier_ratio: float = 0.0
    median_flow_pixels: float = 0.0
    sensor_stable: bool = False
    stationary_streak: int = 0


@dataclass
class ControlMotionState:
    received_at: float = 0.0
    sequence: int = 0
    source_timestamp_ns: int = 0
    axes: tuple[int, int, int, int] = (0, 0, 0, 0)


@dataclass
class FusedPose:
    initialized: bool = False
    sequence: int = 0
    source_timestamp_ns: int = 0
    position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rotation: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    velocity: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    stddev: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    yaw_stddev: float = 45.0
    confidence: float = 0.0
    anchor_count: int = 0
    tag_count: int = 0
    visual_source: str = ""
    last_visual_at: float = 0.0
    last_visual_sequence: int = 0
    flags: int = 0


class PositionFusion:
    COLD_START_STREAK = 5
    COLD_START_MIN_ANCHORS = 50
    COLD_START_MIN_CONFIDENCE = 0.75
    COLD_START_MAX_REPROJECTION_ERROR = 1.5

    def __init__(self, correction_seconds: float = 0.25) -> None:
        self.pose = FusedPose()
        self.state = TelloState()
        self.frame_motion = FrameMotionState()
        self.control_motion = ControlMotionState()
        self.correction_seconds = correction_seconds
        self._last_update = time.monotonic()
        self._last_height_m: float | None = None
        self._last_tof_m: float | None = None
        self._last_barometer_m: float | None = None
        self._last_attitude: tuple[float, float, float] | None = None
        self._grounded_attitude_reference: tuple[float, float, float] | None = None
        self._grounded_rotation_reference: list[float] | None = None
        self._correction_from_position: list[float] | None = None
        self._correction_from_rotation: list[float] | None = None
        self._correction_target_position: list[float] | None = None
        self._correction_target_rotation: list[float] | None = None
        self._correction_started_at = 0.0
        self._telemetry_yaw_offset_degrees: float | None = None
        self._pending_venue_yaw_degrees: float | None = None
        self._cold_start_streak = 0
        self._cold_start_last_sequence = 0
        self._cold_start_position: list[float] | None = None
        self._cold_start_rotation: list[float] | None = None
        self._visual_session_sequence = 0
        self._visual_session_source_timestamp_ns = 0
        self._frame_motion_reference_attitude: tuple[float, float, float] | None = None
        self._frame_motion_reference_heights: tuple[float, float] | None = None
        self._frame_motion_hold_active = False

    def update_state(self, state: TelloState) -> None:
        self.state = state
        if self._pending_venue_yaw_degrees is not None and state.received_at > 0.0:
            self._telemetry_yaw_offset_degrees = _delta_degrees(
                self._pending_venue_yaw_degrees,
                state.yaw,
            )
            self._pending_venue_yaw_degrees = None
        if (
            self.pose.initialized
            and self._grounded_attitude_reference is None
            and state.received_at > 0.0
        ):
            self._grounded_attitude_reference = (
                state.pitch,
                state.roll,
                state.yaw,
            )
            self._grounded_rotation_reference = list(self.pose.rotation)

    def update_frame_motion(
        self,
        message: dict,
        now: float | None = None,
    ) -> bool:
        now = time.monotonic() if now is None else now
        if not self.pose.initialized:
            return False
        try:
            if (
                int(message.get("version", 0)) != 1
                or message.get("kind") != "frame-motion"
            ):
                return False
            raw_sequence = message["sequence"]
            raw_source_timestamp_ns = message["sourceTimestampNs"]
            if isinstance(raw_sequence, bool) or isinstance(
                raw_source_timestamp_ns,
                bool,
            ):
                return False
            sequence = int(raw_sequence)
            source_timestamp_ns = int(raw_source_timestamp_ns)
            interval_seconds = float(message["intervalSeconds"])
            tracked_points = int(message["trackedPoints"])
            inlier_count = int(message["inlierCount"])
            inlier_ratio = float(message["inlierRatio"])
            median_flow_pixels = float(message["medianFlowPixels"])
            forward_backward_error = float(
                message["medianForwardBackwardErrorPixels"]
            )
            affine_rotation = float(message["affineRotationDegrees"])
            affine_scale = float(message["affineScale"])
            stationary = message["stationary"]
            confidence = float(message["confidence"])
        except (KeyError, OverflowError, TypeError, ValueError):
            return False

        numeric_values = (
            interval_seconds,
            inlier_ratio,
            median_flow_pixels,
            forward_backward_error,
            affine_rotation,
            affine_scale,
            confidence,
        )
        received_timestamp_ns = time.perf_counter_ns()
        minimum_source_timestamp_ns = received_timestamp_ns - int(
            FRAME_MOTION_MAX_SOURCE_AGE_SECONDS * 1_000_000_000
        )
        maximum_source_timestamp_ns = received_timestamp_ns + int(
            FRAME_MOTION_MAX_FUTURE_LEAD_SECONDS * 1_000_000_000
        )
        new_motion_session = (
            self.frame_motion.sequence > 0
            and sequence <= self.frame_motion.sequence
            and source_timestamp_ns > self.frame_motion.source_timestamp_ns
        )
        if (
            (
                sequence <= self.frame_motion.sequence
                and not new_motion_session
            )
            or (
                self.frame_motion.source_timestamp_ns > 0
                and source_timestamp_ns <= self.frame_motion.source_timestamp_ns
            )
            or sequence <= 0
            or source_timestamp_ns <= 0
            or not minimum_source_timestamp_ns
            <= source_timestamp_ns
            <= maximum_source_timestamp_ns
            or not isinstance(stationary, bool)
            or not all(math.isfinite(value) for value in numeric_values)
            or not 0.05 <= interval_seconds <= 0.6
            or tracked_points < FRAME_MOTION_MIN_TRACKED_POINTS
            or not FRAME_MOTION_MIN_TRACKED_POINTS <= inlier_count <= tracked_points
            or not FRAME_MOTION_MIN_INLIER_RATIO <= inlier_ratio <= 1.0
            or abs(inlier_ratio - inlier_count / tracked_points)
            > FRAME_MOTION_INLIER_RATIO_TOLERANCE
            or median_flow_pixels < 0.0
            or not 0.0 <= forward_backward_error <= 1.5
            or not 0.8 <= affine_scale <= 1.2
            or not 0.0 <= confidence <= 1.0
            or (
                stationary
                and (
                    median_flow_pixels > FRAME_MOTION_STATIONARY_MAX_PIXELS
                    or abs(affine_rotation)
                    > FRAME_MOTION_STATIONARY_MAX_ROTATION_DEGREES
                    or abs(affine_scale - 1.0)
                    > FRAME_MOTION_STATIONARY_MAX_SCALE_DELTA
                )
            )
        ):
            return False

        source_gap_seconds = (
            (source_timestamp_ns - self.frame_motion.source_timestamp_ns)
            / 1_000_000_000.0
            if self.frame_motion.source_timestamp_ns > 0
            else 0.0
        )
        receive_gap_seconds = (
            now - self.frame_motion.received_at
            if self.frame_motion.received_at > 0.0
            else 0.0
        )
        reset_motion_baseline = (
            new_motion_session
            or source_gap_seconds > FRAME_MOTION_MAX_SESSION_GAP_SECONDS
            or receive_gap_seconds > FRAME_MOTION_MAX_SESSION_GAP_SECONDS
        )
        if reset_motion_baseline:
            self._reset_frame_motion_tracking()

        current_attitude = (self.state.pitch, self.state.roll, self.state.yaw)
        current_heights = (self.state.height_m, self.state.tof_m)
        attitude_stable = False
        height_stable = False
        state_fresh = (
            self.state.received_at > 0.0
            and 0.0 <= now - self.state.received_at <= 0.2
        )
        if self._frame_motion_reference_attitude is not None:
            attitude_stable = max(
                abs(
                    _delta_degrees(
                        current_attitude[index],
                        self._frame_motion_reference_attitude[index],
                    )
                )
                for index in range(3)
            ) <= FRAME_MOTION_MAX_ATTITUDE_DELTA_DEGREES
        if self._frame_motion_reference_heights is not None:
            height_deltas = [
                abs(current - previous)
                for current, previous in zip(
                    current_heights,
                    self._frame_motion_reference_heights,
                )
                if current > 0.05 and previous > 0.05
            ]
            height_stable = bool(height_deltas) and max(height_deltas) <= (
                FRAME_MOTION_MAX_HEIGHT_DELTA_METERS
            )
        self._frame_motion_reference_attitude = current_attitude
        self._frame_motion_reference_heights = current_heights
        sensor_stable = state_fresh and attitude_stable and height_stable
        stationary_streak = (
            0
            if reset_motion_baseline
            else self.frame_motion.stationary_streak
        )
        stationary_streak = (
            stationary_streak + 1
            if stationary
            and sensor_stable
            and confidence >= FRAME_MOTION_MIN_CONFIDENCE
            else 0
        )
        self.frame_motion = FrameMotionState(
            received_at=now,
            sequence=sequence,
            source_timestamp_ns=source_timestamp_ns,
            stationary=stationary,
            confidence=confidence,
            tracked_points=tracked_points,
            inlier_count=inlier_count,
            inlier_ratio=inlier_ratio,
            median_flow_pixels=median_flow_pixels,
            sensor_stable=sensor_stable,
            stationary_streak=stationary_streak,
        )
        return True

    # Pilot이 실제 전송한 최신 RC만 비추적 위치 예측의 짧은 보조 입력으로 보관한다.
    def update_control_motion(
        self,
        message: dict,
        now: float | None = None,
    ) -> bool:
        now = time.monotonic() if now is None else now
        try:
            if (
                int(message.get("version", 0)) != 1
                or message.get("kind") != "control-motion"
            ):
                return False
            raw_sequence = message["sequence"]
            raw_source_timestamp_ns = message["sourceTimestampNs"]
            raw_axes = message["axes"]
            if (
                isinstance(raw_sequence, bool)
                or isinstance(raw_source_timestamp_ns, bool)
                or not isinstance(raw_axes, list)
                or len(raw_axes) != 4
                or any(isinstance(value, bool) for value in raw_axes)
            ):
                return False
            sequence = int(raw_sequence)
            source_timestamp_ns = int(raw_source_timestamp_ns)
            axes = tuple(int(value) for value in raw_axes)
        except (KeyError, OverflowError, TypeError, ValueError):
            return False

        received_timestamp_ns = time.perf_counter_ns()
        minimum_source_timestamp_ns = received_timestamp_ns - int(
            CONTROL_MOTION_MAX_SOURCE_AGE_SECONDS * 1_000_000_000
        )
        maximum_source_timestamp_ns = received_timestamp_ns + int(
            CONTROL_MOTION_MAX_FUTURE_LEAD_SECONDS * 1_000_000_000
        )
        new_control_session = (
            self.control_motion.sequence > 0
            and sequence <= self.control_motion.sequence
            and source_timestamp_ns > self.control_motion.source_timestamp_ns
        )
        if (
            (sequence <= self.control_motion.sequence and not new_control_session)
            or sequence <= 0
            or source_timestamp_ns <= self.control_motion.source_timestamp_ns
            or not minimum_source_timestamp_ns
            <= source_timestamp_ns
            <= maximum_source_timestamp_ns
            or any(value < -100 or value > 100 for value in axes)
        ):
            return False

        self.control_motion = ControlMotionState(
            received_at=now,
            sequence=sequence,
            source_timestamp_ns=source_timestamp_ns,
            axes=axes,
        )
        return True

    def update_visual(self, message: dict, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        try:
            if int(message.get("version", 1)) != 1:
                return False
            raw_sequence = message.get("sequence", 0)
            raw_source_timestamp_ns = message["sourceTimestampNs"]
            if isinstance(raw_sequence, bool) or isinstance(
                raw_source_timestamp_ns,
                bool,
            ):
                return False
            sequence = int(raw_sequence)
            source_timestamp_ns = int(raw_source_timestamp_ns)
            position = [float(value) for value in message["position"]]
            raw_rotation = [float(value) for value in message["rotation"]]
            source = str(message["source"])
            confidence = float(message.get("confidence", 0.0))
            anchor_count = int(message.get("anchorCount", 0))
            tag_count = int(message.get("tagCount", 0))
            reprojection_error = float(message.get("reprojectionError", 3.0))
            flags = int(message.get("flags", 0))
            if source not in {"apriltag", "anchor"}:
                return False
            if (
                sequence <= 0
                or len(position) != 3
                or len(raw_rotation) != 4
                or source_timestamp_ns <= 0
            ):
                return False
            if not all(math.isfinite(value) for value in (*position, *raw_rotation)):
                return False
            if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
                return False
            if (
                anchor_count < 0
                or tag_count < 0
                or not math.isfinite(reprojection_error)
                or reprojection_error < 0.0
                or flags < 0
                or flags > 255
            ):
                return False
            if sum(value * value for value in raw_rotation) <= 1e-12:
                return False
            rotation = _normalize_quaternion(raw_rotation)
        except (KeyError, OverflowError, TypeError, ValueError):
            return False

        new_visual_session = (
            self._visual_session_sequence > 0
            and sequence <= self._visual_session_sequence
            and source_timestamp_ns
            > self._visual_session_source_timestamp_ns
        )
        if (
            (
                sequence <= self._visual_session_sequence
                and not new_visual_session
            )
            or (
                self._visual_session_source_timestamp_ns > 0
                and source_timestamp_ns
                <= self._visual_session_source_timestamp_ns
            )
        ):
            return False
        if new_visual_session:
            self._reset_visual_session()
        self._visual_session_sequence = sequence
        self._visual_session_source_timestamp_ns = source_timestamp_ns

        should_align_yaw = (
            source == "apriltag"
            and (
                new_visual_session
                or self._telemetry_yaw_offset_degrees is None
                or now - self.pose.last_visual_at > 0.35
            )
        )
        if should_align_yaw:
            venue_yaw = _quaternion_yaw_degrees(rotation)
            if self.state.received_at > 0.0 and now - self.state.received_at <= 0.2:
                self._telemetry_yaw_offset_degrees = _delta_degrees(
                    venue_yaw,
                    self.state.yaw,
                )
                self._pending_venue_yaw_degrees = None
            else:
                self._pending_venue_yaw_degrees = venue_yaw

        cold_start_ready = False
        if not self.pose.initialized and source == "anchor":
            cold_start_ready = self._accept_cold_start_candidate(
                sequence,
                position,
                rotation,
                confidence,
                anchor_count,
                reprojection_error,
                flags,
            )
        uses_visual_rotation = (
            not self.pose.initialized
            or new_visual_session
            or now - self.pose.last_visual_at > 0.35
        )
        if not self.pose.initialized:
            if source != "apriltag" and not cold_start_ready:
                return False
            self.pose.position = position
            self.pose.rotation = rotation
            self.pose.initialized = True
            if cold_start_ready:
                venue_yaw = _quaternion_yaw_degrees(rotation)
                if self.state.received_at > 0.0 and now - self.state.received_at <= 0.2:
                    self._telemetry_yaw_offset_degrees = _delta_degrees(
                        venue_yaw,
                        self.state.yaw,
                    )
                else:
                    self._pending_venue_yaw_degrees = venue_yaw
            has_state_baseline = self.state.received_at > 0.0
            self._last_height_m = self.state.height_m if has_state_baseline else None
            self._last_tof_m = self.state.tof_m if has_state_baseline else None
            self._last_barometer_m = (
                self.state.barometer_m if has_state_baseline else None
            )
            self._last_attitude = (
                (self.state.pitch, self.state.roll, self.state.yaw)
                if has_state_baseline
                else None
            )
        else:
            self._correction_from_position = list(self.pose.position)
            self._correction_from_rotation = list(self.pose.rotation)
            self._correction_target_position = position
            # 연속 visual 프레임은 위치만 보정하고 회전은 Tello 자세 변화에 맡긴다.
            self._correction_target_rotation = (
                rotation
                if uses_visual_rotation
                else list(self.pose.rotation)
            )
            self._correction_started_at = now

        if uses_visual_rotation and self.state.received_at > 0.0:
            self._grounded_attitude_reference = (
                self.state.pitch,
                self.state.roll,
                self.state.yaw,
            )
            self._grounded_rotation_reference = list(rotation)

        self.pose.source_timestamp_ns = source_timestamp_ns
        self.pose.last_visual_sequence = sequence
        self.pose.confidence = confidence
        self.pose.anchor_count = anchor_count
        self.pose.tag_count = tag_count
        self.pose.visual_source = source
        self.pose.last_visual_at = now
        self.pose.flags = flags
        error = max(0.01, reprojection_error)
        base_stddev = 0.03 if source == "apriltag" else 0.08
        self.pose.stddev = [base_stddev + error * 0.01] * 3
        self.pose.yaw_stddev = 2.0 + error
        return True

    def _accept_cold_start_candidate(
        self,
        sequence: int,
        position: list[float],
        rotation: list[float],
        confidence: float,
        anchor_count: int,
        reprojection_error: float,
        flags: int,
    ) -> bool:
        required_flags = SESSION_MAP_FLAG | LOCAL_MAP_READY_FLAG
        invalid_flags = LOCAL_MAP_DEGRADED_FLAG | RELOCALIZING_FLAG
        valid = (
            flags & required_flags == required_flags
            and flags & invalid_flags == 0
            and anchor_count >= self.COLD_START_MIN_ANCHORS
            and confidence >= self.COLD_START_MIN_CONFIDENCE
            and reprojection_error <= self.COLD_START_MAX_REPROJECTION_ERROR
            and sequence > self._cold_start_last_sequence
        )
        if not valid:
            self._reset_cold_start_candidate()
            return False

        stable = True
        if self._cold_start_position is not None and self._cold_start_rotation is not None:
            position_delta = math.sqrt(
                sum(
                    (position[index] - self._cold_start_position[index]) ** 2
                    for index in range(3)
                )
            )
            rotation_dot = abs(
                sum(
                    rotation[index] * self._cold_start_rotation[index]
                    for index in range(4)
                )
            )
            rotation_delta = math.degrees(
                2.0 * math.acos(max(-1.0, min(1.0, rotation_dot)))
            )
            stable = position_delta <= 0.25 and rotation_delta <= 10.0

        self._cold_start_streak = self._cold_start_streak + 1 if stable else 1
        self._cold_start_last_sequence = sequence
        self._cold_start_position = list(position)
        self._cold_start_rotation = list(rotation)
        return self._cold_start_streak >= self.COLD_START_STREAK

    def _reset_cold_start_candidate(self) -> None:
        self._cold_start_streak = 0
        self._cold_start_last_sequence = 0
        self._cold_start_position = None
        self._cold_start_rotation = None

    def _reset_frame_motion_tracking(self) -> None:
        self.frame_motion = FrameMotionState()
        self._frame_motion_reference_attitude = None
        self._frame_motion_reference_heights = None

    def _reset_visual_session(self) -> None:
        self.pose.source_timestamp_ns = 0
        self.pose.last_visual_sequence = 0
        self._visual_session_sequence = 0
        self._visual_session_source_timestamp_ns = 0
        self._correction_from_position = None
        self._correction_from_rotation = None
        self._correction_target_position = None
        self._correction_target_rotation = None
        self._correction_started_at = 0.0
        self._last_height_m = None
        self._last_tof_m = None
        self._last_barometer_m = None
        self._last_attitude = None
        self._grounded_attitude_reference = None
        self._grounded_rotation_reference = None
        self._reset_cold_start_candidate()
        self._reset_frame_motion_tracking()

    def predict(self, now: float | None = None) -> None:
        now = now or time.monotonic()
        dt = min(0.1, max(0.0, now - self._last_update))
        self._last_update = now
        if not self.pose.initialized:
            return
        self._apply_visual_correction(now)

        if self._telemetry_yaw_offset_degrees is None:
            reported_velocity = (0.0, -self.state.velocity[2], 0.0)
            acceleration = (0.0, -self.state.acceleration[2], 0.0)
        else:
            reported_velocity = _state_velocity_to_venue(
                self.state,
                self._telemetry_yaw_offset_degrees,
            )
            acceleration = _state_acceleration_to_venue(
                self.state,
                self._telemetry_yaw_offset_degrees,
            )
        reported_velocity = self._velocity_with_control_fallback(
            now,
            reported_velocity,
        )
        predicted_velocity = [
            self.pose.velocity[index] + acceleration[index] * dt
            for index in range(3)
        ]
        state_fresh = now - self.state.received_at <= 0.2
        velocity = (
            tuple(
                reported_velocity[index] * 0.85
                + predicted_velocity[index] * 0.15
                for index in range(3)
            )
            if state_fresh
            else tuple(value * 0.98 for value in predicted_velocity)
        )
        grounded = self._is_grounded(state_fresh)
        if grounded:
            velocity = (0.0, 0.0, 0.0)
        frame_motion_hold = self._should_hold_horizontal_position(
            now,
            reported_velocity,
        )
        if frame_motion_hold:
            velocity = (0.0, velocity[1], 0.0)
        if frame_motion_hold != self._frame_motion_hold_active:
            print(
                "POSITION_STATE FRAME_MOTION_HOLD "
                f"active={'true' if frame_motion_hold else 'false'} "
                f"points={self.frame_motion.inlier_count} "
                f"flowPx={self.frame_motion.median_flow_pixels:.3f}",
                flush=True,
            )
            self._frame_motion_hold_active = frame_motion_hold
        self.pose.velocity = list(velocity)
        if not grounded:
            for index in range(3):
                self.pose.position[index] += velocity[index] * dt
        altitude_deltas = []
        for current, previous in (
            (self.state.height_m, self._last_height_m),
            (self.state.tof_m, self._last_tof_m),
            (self.state.barometer_m, self._last_barometer_m),
        ):
            if previous is not None and math.isfinite(current) and abs(current - previous) < 0.5:
                altitude_deltas.append(current - previous)
        if not grounded and altitude_deltas and self.state.received_at > 0:
            self.pose.position[1] += sum(altitude_deltas) / len(altitude_deltas)
        self._last_height_m = self.state.height_m
        self._last_tof_m = self.state.tof_m
        self._last_barometer_m = self.state.barometer_m
        current_attitude = (self.state.pitch, self.state.roll, self.state.yaw)
        grounded_attitude_applied = (
            grounded
            and self._apply_grounded_attitude(current_attitude)
        )
        if (
            not grounded_attitude_applied
            and self._last_attitude is not None
            and state_fresh
        ):
            pitch_delta = _delta_degrees(current_attitude[0], self._last_attitude[0])
            roll_delta = _delta_degrees(current_attitude[1], self._last_attitude[1])
            yaw_delta = _delta_degrees(current_attitude[2], self._last_attitude[2])
            if max(abs(pitch_delta), abs(roll_delta), abs(yaw_delta)) < 45.0:
                delta_rotation = _euler_delta_quaternion(
                    pitch_delta,
                    yaw_delta,
                    -roll_delta,
                )
                self._apply_attitude_delta(delta_rotation)
        self._last_attitude = current_attitude

        visual_age = now - self.pose.last_visual_at
        growth = 0.02 if visual_age < 0.5 else 0.08
        self.pose.stddev = [min(25.0, value + growth * dt) for value in self.pose.stddev]
        self.pose.yaw_stddev = min(180.0, self.pose.yaw_stddev + growth * 10.0 * dt)
        if visual_age > 0.35:
            self.pose.confidence = max(0.05, self.pose.confidence - dt * 0.08)

    # 신선한 Tello h 값이 착륙 상태를 나타내면 위치 적분을 정지한다.
    def _is_grounded(self, state_fresh: bool) -> bool:
        return (
            state_fresh
            and self.state.received_at > 0.0
            and math.isfinite(self.state.height_m)
            and 0.0 <= self.state.height_m <= GROUNDED_MAX_HEIGHT_METERS
        )

    # 착륙 중 큰 자세 변화도 마지막 시각 포즈 기준의 절대 자세로 복원한다.
    def _apply_grounded_attitude(
        self,
        current_attitude: tuple[float, float, float],
    ) -> bool:
        if (
            self._grounded_attitude_reference is None
            or self._grounded_rotation_reference is None
        ):
            return False
        reference_pitch, reference_roll, reference_yaw = (
            self._grounded_attitude_reference
        )
        current_pitch, current_roll, current_yaw = current_attitude
        reference_rotation = _euler_delta_quaternion(
            reference_pitch,
            reference_yaw,
            -reference_roll,
        )
        current_rotation = _euler_delta_quaternion(
            current_pitch,
            current_yaw,
            -current_roll,
        )
        relative_rotation = _quaternion_multiply(
            _quaternion_inverse(reference_rotation),
            current_rotation,
        )
        self.pose.rotation = _normalize_quaternion(
            _quaternion_multiply(
                self._grounded_rotation_reference,
                relative_rotation,
            )
        )
        return True

    # 진행 중인 시각 보정 기준에도 기체의 실제 자세 변화를 반영해 정상 회전을 보존한다.
    def _apply_attitude_delta(self, delta_rotation: list[float]) -> None:
        self.pose.rotation = _normalize_quaternion(
            _quaternion_multiply(self.pose.rotation, delta_rotation)
        )
        if self._correction_from_rotation is not None:
            self._correction_from_rotation = _normalize_quaternion(
                _quaternion_multiply(
                    self._correction_from_rotation,
                    delta_rotation,
                )
            )
        if self._correction_target_rotation is not None:
            self._correction_target_rotation = _normalize_quaternion(
                _quaternion_multiply(
                    self._correction_target_rotation,
                    delta_rotation,
                )
            )

    # 비추적 중 Tello 평면 속도가 0일 때만 fresh RC를 보수적인 보조 속도로 사용한다.
    def _velocity_with_control_fallback(
        self,
        now: float,
        reported_velocity: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        if (
            now - self.pose.last_visual_at <= 0.35
            or math.hypot(reported_velocity[0], reported_velocity[2])
            > FRAME_MOTION_MAX_STATIONARY_SPEED_MPS
            or self._telemetry_yaw_offset_degrees is None
            or self.state.received_at <= 0.0
            or not 0.0 <= now - self.state.received_at <= 0.2
            or not 0.0 <= now - self.control_motion.received_at
            <= CONTROL_MOTION_FRESH_SECONDS
        ):
            return reported_velocity

        left_right, forward_back, _, _ = self.control_motion.axes
        right = left_right / 100.0 * CONTROL_MOTION_MAX_SPEED_MPS
        forward = forward_back / 100.0 * CONTROL_MOTION_MAX_SPEED_MPS
        if right == 0.0 and forward == 0.0:
            return reported_velocity
        yaw = math.radians(self.state.yaw + self._telemetry_yaw_offset_degrees)
        return (
            right * math.cos(yaw) + forward * math.sin(yaw),
            reported_velocity[1],
            -right * math.sin(yaw) + forward * math.cos(yaw),
        )

    def _should_hold_horizontal_position(
        self,
        now: float,
        reported_velocity: tuple[float, float, float],
    ) -> bool:
        horizontal_speed = math.hypot(
            reported_velocity[0],
            reported_velocity[2],
        )
        return (
            self.pose.initialized
            and now - self.pose.last_visual_at > 0.35
            and 0.0 <= now - self.frame_motion.received_at
            <= FRAME_MOTION_FRESH_SECONDS
            and self.frame_motion.stationary
            and self.frame_motion.sensor_stable
            and self.frame_motion.confidence >= FRAME_MOTION_MIN_CONFIDENCE
            and self.frame_motion.stationary_streak
            >= FRAME_MOTION_MIN_STATIONARY_STREAK
            and horizontal_speed <= FRAME_MOTION_MAX_STATIONARY_SPEED_MPS
        )

    def _apply_visual_correction(self, now: float) -> None:
        if (
            self._correction_from_position is None
            or self._correction_from_rotation is None
            or self._correction_target_position is None
            or self._correction_target_rotation is None
        ):
            return
        amount = min(
            1.0,
            max(0.0, now - self._correction_started_at)
            / max(0.001, self.correction_seconds),
        )
        self.pose.position = [
            start + (target - start) * amount
            for start, target in zip(
                self._correction_from_position,
                self._correction_target_position,
            )
        ]
        self.pose.rotation = _slerp(
            self._correction_from_rotation,
            self._correction_target_rotation,
            amount,
        )
        if amount >= 1.0:
            self._correction_from_position = None
            self._correction_from_rotation = None
            self._correction_target_position = None
            self._correction_target_rotation = None

    def packet(self, now: float | None = None) -> PosePacketV2:
        now = now or time.monotonic()
        self.pose.sequence += 1
        age = now - self.pose.last_visual_at
        state_age = now - self.state.received_at
        state_fresh = self.state.received_at > 0.0 and 0.0 <= state_age <= 0.2
        if not self.pose.initialized:
            mode = TrackingMode.INITIALIZING
        elif age <= 0.35 and self.pose.visual_source == "apriltag":
            mode = TrackingMode.APRILTAG_LOCKED
        elif age <= 0.35:
            mode = TrackingMode.VISUAL_TRACKED
        elif state_fresh:
            mode = TrackingMode.SENSOR_PREDICTED
        else:
            mode = TrackingMode.UNCERTAIN
        timestamp = time.perf_counter_ns()
        flags = self.pose.flags
        if self.pose.initialized and age > 0.5 and flags & LOCAL_MAP_READY_FLAG:
            flags |= LOCAL_MAP_DEGRADED_FLAG | RELOCALIZING_FLAG
        return PosePacketV2(
            sequence=self.pose.sequence,
            source_timestamp_ns=timestamp,
            position=tuple(self.pose.position),
            rotation=tuple(self.pose.rotation),
            velocity=tuple(self.pose.velocity),
            position_stddev=tuple(self.pose.stddev),
            yaw_stddev=self.pose.yaw_stddev,
            confidence=self.pose.confidence,
            visual_anchor_count=self.pose.anchor_count,
            detected_tag_count=self.pose.tag_count,
            mode=mode,
            flags=flags,
        )


class DronePositionSync:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.measurement_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.state_socket.bind(("0.0.0.0", args.state_port))
            self.measurement_socket.bind(("127.0.0.1", args.measurement_port))
        except Exception:
            self.state_socket.close()
            self.measurement_socket.close()
            raise
        self.state_socket.setblocking(False)
        self.measurement_socket.setblocking(False)
        self.state_relay = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pose_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.visual_pose_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fusion = PositionFusion(args.correction_seconds)
        self.state_target = ("127.0.0.1", args.state_relay_port)
        self.pilot_state_target = ("127.0.0.1", args.pilot_state_relay_port)
        self.desktop_state_target = ("127.0.0.1", args.desktop_state_relay_port)
        self.livekit_state_target = ("127.0.0.1", args.livekit_state_relay_port)
        self.pose_target = ("127.0.0.1", args.pose_port)
        pilot_pose_relay_port = getattr(args, "pilot_pose_relay_port", None)
        self.pilot_pose_target = (
            None
            if pilot_pose_relay_port is None
            else ("127.0.0.1", pilot_pose_relay_port)
        )
        self.visual_pose_target = ("127.0.0.1", args.visual_pose_port)
        self.wifi_signal_monitor = WifiSignalMonitor()
        self.wifi_signal_monitor.start()

    def run(self) -> None:
        interval = 1.0 / self.args.pose_rate
        next_pose = time.monotonic()
        print("STRIKERPC_READY POSITION_SYNC", flush=True)
        while True:
            timeout = max(0.0, min(0.02, next_pose - time.monotonic()))
            readable, _, _ = select.select(
                [self.state_socket, self.measurement_socket],
                [],
                [],
                timeout,
            )
            for current in readable:
                payload, _ = current.recvfrom(65535)
                if current is self.state_socket:
                    relay_payload = append_wifi_signal(
                        payload,
                        self.wifi_signal_monitor.signal_percent,
                    )
                    self.state_relay.sendto(relay_payload, self.state_target)
                    self.state_relay.sendto(relay_payload, self.pilot_state_target)
                    self.state_relay.sendto(relay_payload, self.desktop_state_target)
                    self.state_relay.sendto(relay_payload, self.livekit_state_target)
                    self.fusion.update_state(parse_tello_state(payload))
                else:
                    accepted_payload = accept_visual_measurement(payload, self.fusion)
                    if accepted_payload is not None:
                        self.visual_pose_sender.sendto(
                            accepted_payload,
                            self.visual_pose_target,
                        )
            now = time.monotonic()
            if now >= next_pose:
                self.fusion.predict(now)
                pose_payload = self.fusion.packet(now).encode()
                self.pose_sender.sendto(pose_payload, self.pose_target)
                if self.pilot_pose_target is not None:
                    self.pose_sender.sendto(pose_payload, self.pilot_pose_target)
                next_pose = now + interval

    def close(self) -> None:
        self.wifi_signal_monitor.close()
        self.state_socket.close()
        self.measurement_socket.close()
        self.state_relay.close()
        self.pose_sender.close()
        self.visual_pose_sender.close()


# 검증된 visual pose를 PositionFusion 보정 좌표로 바꿔 FPV 투영 경로로 전달한다.
def accept_visual_measurement(
    payload: bytes,
    fusion: PositionFusion,
    now: float | None = None,
) -> bytes | None:
    try:
        message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(message, dict):
        return None
    if message.get("kind") == "frame-motion":
        fusion.update_frame_motion(message, now)
        return None
    if message.get("kind") == "control-motion":
        fusion.update_control_motion(message, now)
        return None
    if not fusion.update_visual(message, now):
        return None
    smoothed = dict(message)
    smoothed["position"] = list(fusion.pose.position)
    smoothed["rotation"] = list(fusion.pose.rotation)
    smoothed["smoothed"] = True
    return json.dumps(smoothed, separators=(",", ":")).encode("utf-8")


def parse_tello_state(payload: bytes) -> TelloState:
    try:
        text = payload.decode("ascii")
    except UnicodeDecodeError:
        return TelloState()
    values: dict[str, float] = {}
    for item in text.split(";"):
        if ":" not in item:
            continue
        key, value = item.split(":", 1)
        try:
            values[key] = float(value)
        except ValueError:
            continue
    return TelloState(
        received_at=time.monotonic(),
        velocity=(
            values.get("vgx", 0.0) / 100.0,
            values.get("vgy", 0.0) / 100.0,
            values.get("vgz", 0.0) / 100.0,
        ),
        pitch=values.get("pitch", 0.0),
        roll=values.get("roll", 0.0),
        yaw=values.get("yaw", 0.0),
        height_m=values.get("h", 0.0) / 100.0,
        tof_m=values.get("tof", 0.0) / 100.0,
        barometer_m=values.get("baro", 0.0),
        acceleration=(
            values.get("agx", 0.0) * 0.00980665,
            values.get("agy", 0.0) * 0.00980665,
            values.get("agz", 0.0) * 0.00980665,
        ),
    )


def _state_velocity_to_venue(
    state: TelloState,
    yaw_offset_degrees: float = 0.0,
) -> tuple[float, float, float]:
    right, forward, vertical = state.velocity
    yaw = math.radians(state.yaw + yaw_offset_degrees)
    return (
        right * math.cos(yaw) + forward * math.sin(yaw),
        -vertical,
        -right * math.sin(yaw) + forward * math.cos(yaw),
    )


def _state_acceleration_to_venue(
    state: TelloState,
    yaw_offset_degrees: float = 0.0,
) -> tuple[float, float, float]:
    right, forward, vertical = state.acceleration
    yaw = math.radians(state.yaw + yaw_offset_degrees)
    return (
        right * math.cos(yaw) + forward * math.sin(yaw),
        -vertical,
        -right * math.sin(yaw) + forward * math.cos(yaw),
    )


def _normalize_quaternion(value: list[float]) -> list[float]:
    length = math.sqrt(sum(component * component for component in value))
    if length <= 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [component / length for component in value]


# Unity 좌표계 quaternion의 전방 벡터로 Venue yaw를 계산한다.
def _quaternion_yaw_degrees(value: list[float]) -> float:
    x, y, z, w = _normalize_quaternion(value)
    forward_x = 2.0 * (x * z + w * y)
    forward_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.atan2(forward_x, forward_z))


def _slerp(first: list[float], second: list[float], amount: float) -> list[float]:
    first = _normalize_quaternion(first)
    second = _normalize_quaternion(second)
    dot = sum(left * right for left, right in zip(first, second))
    if dot < 0.0:
        second = [-value for value in second]
        dot = -dot
    if dot > 0.9995:
        return _normalize_quaternion(
            [left + amount * (right - left) for left, right in zip(first, second)]
        )
    angle = math.acos(max(-1.0, min(1.0, dot)))
    scale = math.sin(angle)
    return [
        (
            math.sin((1.0 - amount) * angle) * left
            + math.sin(amount * angle) * right
        )
        / scale
        for left, right in zip(first, second)
    ]


def _delta_degrees(current: float, previous: float) -> float:
    return (current - previous + 180.0) % 360.0 - 180.0


def _euler_delta_quaternion(
    pitch_degrees: float,
    yaw_degrees: float,
    roll_degrees: float,
) -> list[float]:
    pitch = math.radians(pitch_degrees) * 0.5
    yaw = math.radians(yaw_degrees) * 0.5
    roll = math.radians(roll_degrees) * 0.5
    qx = [math.sin(pitch), 0.0, 0.0, math.cos(pitch)]
    qy = [0.0, math.sin(yaw), 0.0, math.cos(yaw)]
    qz = [0.0, 0.0, math.sin(roll), math.cos(roll)]
    return _quaternion_multiply(_quaternion_multiply(qy, qx), qz)


def _quaternion_multiply(first: list[float], second: list[float]) -> list[float]:
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def _quaternion_inverse(value: list[float]) -> list[float]:
    x, y, z, w = _normalize_quaternion(value)
    return [-x, -y, -z, w]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="StrikerPC continuous position sync")
    parser.add_argument("--state-port", type=int, default=8890)
    parser.add_argument("--state-relay-port", type=int, default=8891)
    parser.add_argument("--pilot-state-relay-port", type=int, default=8892)
    parser.add_argument("--desktop-state-relay-port", type=int, default=8893)
    parser.add_argument("--livekit-state-relay-port", type=int, default=8894)
    parser.add_argument("--measurement-port", type=int, default=49003)
    parser.add_argument("--pose-port", type=int, default=49001)
    parser.add_argument("--pilot-pose-relay-port", type=int, default=49006)
    parser.add_argument("--visual-pose-port", type=int, default=49004)
    parser.add_argument("--pose-rate", type=float, default=30.0)
    parser.add_argument("--correction-seconds", type=float, default=0.25)
    return parser.parse_args()


def main() -> int:
    sync = DronePositionSync(parse_args())
    try:
        sync.run()
    except KeyboardInterrupt:
        return 0
    finally:
        sync.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
