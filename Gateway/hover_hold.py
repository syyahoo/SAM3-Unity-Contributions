from __future__ import annotations

import json
import logging
import math
import socket
import time
from dataclasses import dataclass
from typing import Callable

from pose_protocol import (
    LOCAL_MAP_DEGRADED_FLAG,
    RELOCALIZING_FLAG,
    PosePacketV2,
    TrackingMode,
)


log = logging.getLogger(__name__)


@dataclass(frozen=True)
class HoverHoldPolicy:
    """보수적인 위치·고도·헤딩 유지 한계와 게인을 보관한다."""

    minimum_confidence: float = 0.65
    maximum_horizontal_stddev_meters: float = 0.25
    maximum_vertical_stddev_meters: float = 0.30
    maximum_yaw_stddev_degrees: float = 10.0
    horizontal_deadband_meters: float = 0.05
    vertical_deadband_meters: float = 0.04
    velocity_deadband_mps: float = 0.03
    yaw_deadband_degrees: float = 3.0
    horizontal_position_gain: float = 18.0
    horizontal_velocity_gain: float = 10.0
    vertical_position_gain: float = 20.0
    vertical_velocity_gain: float = 8.0
    yaw_gain: float = 0.35
    maximum_horizontal_axis: int = 15
    maximum_vertical_axis: int = 12
    maximum_yaw_axis: int = 10
    maximum_axis_slew_per_second: float = 120.0

    def __post_init__(self) -> None:
        positive_values = (
            self.maximum_horizontal_stddev_meters,
            self.maximum_vertical_stddev_meters,
            self.maximum_yaw_stddev_degrees,
            self.horizontal_position_gain,
            self.horizontal_velocity_gain,
            self.vertical_position_gain,
            self.vertical_velocity_gain,
            self.yaw_gain,
            self.maximum_axis_slew_per_second,
        )
        if any(value <= 0 for value in positive_values):
            raise ValueError("Hover Hold 정책 값은 양수여야 합니다.")
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError("Hover Hold 최소 신뢰도는 0~1이어야 합니다.")
        if any(
            value < 0
            for value in (
                self.horizontal_deadband_meters,
                self.vertical_deadband_meters,
                self.velocity_deadband_mps,
                self.yaw_deadband_degrees,
            )
        ):
            raise ValueError("Hover Hold deadband는 음수일 수 없습니다.")
        if any(
            value <= 0 or value > 100
            for value in (
                self.maximum_horizontal_axis,
                self.maximum_vertical_axis,
                self.maximum_yaw_axis,
            )
        ):
            raise ValueError("Hover Hold 축 한계는 1~100이어야 합니다.")


class HoverPoseReceiver:
    """Position Sync의 최신 TVP2 pose 한 건만 비차단으로 보관한다."""

    def __init__(
        self,
        port: int = 49006,
        *,
        clock: Callable[[], float] = time.monotonic,
        counter_ns: Callable[[], int] = time.perf_counter_ns,
        maximum_source_age_seconds: float = 0.2,
    ) -> None:
        if not 0 <= port <= 65535:
            raise ValueError("Hover pose 포트가 올바르지 않습니다.")
        if maximum_source_age_seconds <= 0:
            raise ValueError("Hover pose source age는 양수여야 합니다.")
        self._clock = clock
        self._counter_ns = counter_ns
        self._maximum_source_age_ns = int(maximum_source_age_seconds * 1_000_000_000)
        self._maximum_receive_age_seconds = maximum_source_age_seconds
        self._maximum_future_lead_ns = 50_000_000
        self._latest: PosePacketV2 | None = None
        self._latest_received_at = 0.0
        self._last_source_timestamp_ns = 0
        self._closed = False
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind(("127.0.0.1", port))
            self._socket.setblocking(False)
        except Exception:
            self._socket.close()
            raise

    @property
    def address(self) -> tuple[str, int]:
        return self._socket.getsockname()

    # 수신 큐를 비우고 freshness를 만족하는 가장 최신 pose를 반환한다.
    def latest(self, now: float | None = None) -> PosePacketV2 | None:
        if self._closed:
            return None
        while True:
            try:
                payload, _ = self._socket.recvfrom(256)
            except BlockingIOError:
                break
            except OSError:
                return None
            self._accept(payload)

        current = self._clock() if now is None else now
        if (
            self._latest is None
            or current < self._latest_received_at
            or current - self._latest_received_at
            > self._maximum_receive_age_seconds
        ):
            return None
        return self._latest

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._socket.close()

    def _accept(self, payload: bytes) -> bool:
        packet = PosePacketV2.decode(payload)
        if packet is None:
            return False
        counter_ns = self._counter_ns()
        source_age_ns = counter_ns - packet.source_timestamp_ns
        if (
            packet.source_timestamp_ns <= self._last_source_timestamp_ns
            or source_age_ns < -self._maximum_future_lead_ns
            or source_age_ns > self._maximum_source_age_ns
        ):
            return False
        self._latest = packet
        self._latest_received_at = self._clock()
        self._last_source_timestamp_ns = packet.source_timestamp_ns
        return True


@dataclass(frozen=True)
class ImageHoverMotion:
    sequence: int
    source_timestamp_ns: int
    interval_seconds: float
    tracked_points: int
    inlier_count: int
    inlier_ratio: float
    flow_x_pixels: float
    flow_y_pixels: float
    flow_pixels: float
    forward_backward_error_pixels: float
    rotation_degrees: float
    scale: float
    confidence: float


@dataclass(frozen=True)
class ImageHoverPolicy:
    """영상 상대 이동을 낮은 출력의 수평 RC로 바꾸는 보수적 한계를 보관한다."""

    minimum_tracked_points: int = 60
    minimum_inlier_ratio: float = 0.70
    minimum_confidence: float = 0.65
    maximum_forward_backward_error_pixels: float = 1.5
    maximum_flow_pixels: float = 60.0
    maximum_rotation_degrees: float = 1.0
    maximum_attitude_delta_degrees: float = 0.75
    lateral_deadband_pixels: float = 1.5
    scale_deadband: float = 0.004
    lateral_gain: float = 0.20
    forward_scale_gain: float = 250.0
    maximum_lateral_axis: int = 12
    maximum_forward_axis: int = 10
    maximum_axis_slew_per_second: float = 80.0

    def __post_init__(self) -> None:
        if self.minimum_tracked_points <= 0:
            raise ValueError("Image Hover 최소 특징점 수는 양수여야 합니다.")
        if not 0.0 <= self.minimum_inlier_ratio <= 1.0:
            raise ValueError("Image Hover inlier 비율은 0~1이어야 합니다.")
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError("Image Hover 신뢰도는 0~1이어야 합니다.")
        if any(
            value <= 0.0
            for value in (
                self.maximum_forward_backward_error_pixels,
                self.maximum_flow_pixels,
                self.maximum_rotation_degrees,
                self.maximum_attitude_delta_degrees,
                self.lateral_gain,
                self.forward_scale_gain,
                self.maximum_axis_slew_per_second,
            )
        ):
            raise ValueError("Image Hover 정책 한계와 게인은 양수여야 합니다.")
        if self.lateral_deadband_pixels < 0.0 or self.scale_deadband < 0.0:
            raise ValueError("Image Hover deadband는 음수일 수 없습니다.")
        if not 1 <= self.maximum_lateral_axis <= 100 or not 1 <= self.maximum_forward_axis <= 100:
            raise ValueError("Image Hover 축 한계는 1~100이어야 합니다.")


class HoverMotionReceiver:
    """Vision의 최신 optical-flow 측정 한 건만 비차단으로 보관한다."""

    def __init__(
        self,
        port: int = 49006,
        *,
        clock: Callable[[], float] = time.monotonic,
        counter_ns: Callable[[], int] = time.perf_counter_ns,
        maximum_age_seconds: float = 0.35,
    ) -> None:
        if not 0 <= port <= 65535:
            raise ValueError("Hover motion 포트가 올바르지 않습니다.")
        if maximum_age_seconds <= 0.0:
            raise ValueError("Hover motion age는 양수여야 합니다.")
        self._clock = clock
        self._counter_ns = counter_ns
        self._maximum_source_age_ns = int(maximum_age_seconds * 1_000_000_000)
        self._maximum_receive_age_seconds = maximum_age_seconds
        self._maximum_future_lead_ns = 50_000_000
        self._latest: ImageHoverMotion | None = None
        self._latest_received_at = 0.0
        self._last_source_timestamp_ns = 0
        self._last_sequence = 0
        self._closed = False
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind(("127.0.0.1", port))
            self._socket.setblocking(False)
        except Exception:
            self._socket.close()
            raise

    @property
    def address(self) -> tuple[str, int]:
        return self._socket.getsockname()

    # 수신 큐를 비우고 freshness를 만족하는 최신 영상 이동만 반환한다.
    def latest(self, now: float | None = None) -> ImageHoverMotion | None:
        if self._closed:
            return None
        while True:
            try:
                payload, _ = self._socket.recvfrom(2048)
            except BlockingIOError:
                break
            except OSError:
                return None
            self._accept(payload)

        current = self._clock() if now is None else now
        if (
            self._latest is None
            or current < self._latest_received_at
            or current - self._latest_received_at > self._maximum_receive_age_seconds
        ):
            return None
        return self._latest

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._socket.close()

    def _accept(self, payload: bytes) -> bool:
        try:
            message = json.loads(payload.decode("utf-8"))
            if (
                not isinstance(message, dict)
                or int(message.get("version", 0)) != 1
                or message.get("kind") != "frame-motion"
            ):
                return False
            sequence = int(message["sequence"])
            source_timestamp_ns = int(message["sourceTimestampNs"])
            motion = ImageHoverMotion(
                sequence=sequence,
                source_timestamp_ns=source_timestamp_ns,
                interval_seconds=float(message["intervalSeconds"]),
                tracked_points=int(message["trackedPoints"]),
                inlier_count=int(message["inlierCount"]),
                inlier_ratio=float(message["inlierRatio"]),
                flow_x_pixels=float(message["medianFlowXPixels"]),
                flow_y_pixels=float(message["medianFlowYPixels"]),
                flow_pixels=float(message["medianFlowPixels"]),
                forward_backward_error_pixels=float(
                    message["medianForwardBackwardErrorPixels"]
                ),
                rotation_degrees=float(message["affineRotationDegrees"]),
                scale=float(message["affineScale"]),
                confidence=float(message["confidence"]),
            )
        except (
            KeyError,
            OverflowError,
            TypeError,
            UnicodeDecodeError,
            ValueError,
            json.JSONDecodeError,
        ):
            return False
        numeric_values = (
            motion.interval_seconds,
            motion.inlier_ratio,
            motion.flow_x_pixels,
            motion.flow_y_pixels,
            motion.flow_pixels,
            motion.forward_backward_error_pixels,
            motion.rotation_degrees,
            motion.scale,
            motion.confidence,
        )
        counter_ns = self._counter_ns()
        source_age_ns = counter_ns - source_timestamp_ns
        new_session = (
            self._last_sequence > 0
            and sequence <= self._last_sequence
            and source_timestamp_ns > self._last_source_timestamp_ns
        )
        if (
            sequence <= 0
            or source_timestamp_ns <= 0
            or (sequence <= self._last_sequence and not new_session)
            or (
                self._last_source_timestamp_ns > 0
                and source_timestamp_ns <= self._last_source_timestamp_ns
            )
            or source_age_ns < -self._maximum_future_lead_ns
            or source_age_ns > self._maximum_source_age_ns
            or not all(math.isfinite(value) for value in numeric_values)
            or not 0.05 <= motion.interval_seconds <= 0.6
            or motion.tracked_points <= 0
            or not 0 <= motion.inlier_count <= motion.tracked_points
            or not 0.0 <= motion.inlier_ratio <= 1.0
            or motion.flow_pixels < 0.0
            or motion.forward_backward_error_pixels < 0.0
            or not 0.5 <= motion.scale <= 1.5
            or not 0.0 <= motion.confidence <= 1.0
        ):
            return False
        self._latest = motion
        self._latest_received_at = self._clock()
        self._last_sequence = sequence
        self._last_source_timestamp_ns = source_timestamp_ns
        return True


class ImageHoverHoldController:
    """절대 pose 없이 누적 영상 이동을 되돌리는 제한된 수평 RC를 만든다."""

    def __init__(
        self,
        motion_source: HoverMotionReceiver,
        *,
        attitude_source: Callable[[], tuple[float, float, float, float] | None],
        policy: ImageHoverPolicy | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._motion_source = motion_source
        self._attitude_source = attitude_source
        self._policy = policy or ImageHoverPolicy()
        self._clock = clock
        self._last_sequence = 0
        self._last_attitude: tuple[float, float, float] | None = None
        self._offset_x_pixels = 0.0
        self._scale_error = 0.0
        self._baseline_pending = True
        self._last_output = (0.0, 0.0, 0.0, 0.0)
        self._last_apply_at = clock()
        self._state = "idle"

    @property
    def state(self) -> str:
        return self._state

    # 데드맨 해제 후 새 영상 기준을 잡고 좌우·전후 드리프트만 보정한다.
    def apply(
        self,
        axes: tuple[int, int, int, int],
        *,
        manual_control_active: bool,
        control_stream_fresh: bool,
        airborne: bool,
        force_neutral: bool,
        now: float | None = None,
    ) -> tuple[int, int, int, int]:
        current = self._clock() if now is None else now
        manual_axes = tuple(int(value) for value in axes)
        if force_neutral:
            self._reset("safety-neutral", current)
            return (0, 0, 0, 0)
        if not airborne:
            self._reset("grounded", current)
            return manual_axes
        if manual_control_active:
            self._reset("manual", current)
            return manual_axes
        if not control_stream_fresh:
            self._reset("control-unavailable", current)
            return (0, 0, 0, 0)

        motion = self._motion_source.latest(current)
        attitude_sample = self._attitude_source()
        if motion is None:
            self._reset("waiting-image", current)
            return (0, 0, 0, 0)
        attitude = self._fresh_attitude(attitude_sample, current)
        if attitude is None:
            self._reset("waiting-sensors", current)
            return (0, 0, 0, 0)
        if not self._motion_is_trusted(motion):
            self._reset("waiting-quality", current)
            return (0, 0, 0, 0)

        if motion.sequence != self._last_sequence:
            if self._baseline_pending:
                self._baseline_pending = False
                self._last_sequence = motion.sequence
                self._last_attitude = attitude
                self._set_state("acquiring")
                return (0, 0, 0, 0)
            attitude_delta = max(
                abs(_delta_degrees(current_value, previous_value))
                for current_value, previous_value in zip(attitude, self._last_attitude or attitude)
            )
            self._last_sequence = motion.sequence
            self._last_attitude = attitude
            if attitude_delta > self._policy.maximum_attitude_delta_degrees:
                self._offset_x_pixels = 0.0
                self._scale_error = 0.0
                self._last_output = (0.0, 0.0, 0.0, 0.0)
                self._set_state("waiting-stable")
                return (0, 0, 0, 0)
            self._offset_x_pixels += motion.flow_x_pixels
            self._scale_error += math.log(motion.scale)

        self._set_state("holding-image")
        lateral_error = _outside_deadband(
            self._offset_x_pixels,
            self._policy.lateral_deadband_pixels,
        )
        scale_error = _outside_deadband(
            self._scale_error,
            self._policy.scale_deadband,
        )
        requested = (
            _clamp(
                lateral_error * self._policy.lateral_gain,
                self._policy.maximum_lateral_axis,
            ),
            _clamp(
                -scale_error * self._policy.forward_scale_gain,
                self._policy.maximum_forward_axis,
            ),
            0.0,
            0.0,
        )
        return self._slew_limited(requested, current)

    def close(self) -> None:
        self._motion_source.close()

    def _motion_is_trusted(self, motion: ImageHoverMotion) -> bool:
        return (
            motion.tracked_points >= self._policy.minimum_tracked_points
            and motion.inlier_count >= self._policy.minimum_tracked_points
            and motion.inlier_ratio >= self._policy.minimum_inlier_ratio
            and motion.confidence >= self._policy.minimum_confidence
            and motion.forward_backward_error_pixels
            <= self._policy.maximum_forward_backward_error_pixels
            and motion.flow_pixels <= self._policy.maximum_flow_pixels
            and abs(motion.rotation_degrees)
            <= self._policy.maximum_rotation_degrees
            and 0.85 <= motion.scale <= 1.15
        )

    @staticmethod
    def _fresh_attitude(
        sample: tuple[float, float, float, float] | None,
        now: float,
    ) -> tuple[float, float, float] | None:
        if sample is None or len(sample) != 4:
            return None
        received_at, pitch, roll, yaw = sample
        if (
            not all(math.isfinite(value) for value in sample)
            or not 0.0 <= now - received_at <= 0.25
        ):
            return None
        return (pitch, roll, yaw)

    def _slew_limited(
        self,
        requested: tuple[float, float, float, float],
        now: float,
    ) -> tuple[int, int, int, int]:
        elapsed = max(0.0, min(0.1, now - self._last_apply_at))
        maximum_delta = self._policy.maximum_axis_slew_per_second * elapsed
        output = tuple(
            previous + max(-maximum_delta, min(maximum_delta, target - previous))
            for previous, target in zip(self._last_output, requested)
        )
        self._last_output = output
        self._last_apply_at = now
        return tuple(int(round(value)) for value in output)

    def _reset(self, state: str, now: float) -> None:
        self._last_sequence = 0
        self._last_attitude = None
        self._offset_x_pixels = 0.0
        self._scale_error = 0.0
        self._baseline_pending = True
        self._last_output = (0.0, 0.0, 0.0, 0.0)
        self._last_apply_at = now
        self._set_state(state)

    def _set_state(self, state: str) -> None:
        if state == self._state:
            return
        log.info("Image Hover state %s -> %s", self._state, state)
        self._state = state


class HoverHoldController:
    """수동 조종이 없을 때만 검증된 pose 기준의 제한된 RC 보정을 만든다."""

    def __init__(
        self,
        pose_source: HoverPoseReceiver,
        *,
        policy: HoverHoldPolicy | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._pose_source = pose_source
        self._policy = policy or HoverHoldPolicy()
        self._clock = clock
        self._target: tuple[float, float, float, float] | None = None
        self._last_output = (0.0, 0.0, 0.0, 0.0)
        self._last_apply_at = clock()
        self._state = "idle"

    @property
    def state(self) -> str:
        return self._state

    @property
    def target(self) -> tuple[float, float, float, float] | None:
        return self._target

    # 데드맨 기반 수동 조종 상태를 우선하고, 비활성일 때만 자동 보정을 계산한다.
    def apply(
        self,
        axes: tuple[int, int, int, int],
        *,
        manual_control_active: bool,
        control_stream_fresh: bool,
        airborne: bool,
        force_neutral: bool,
        now: float | None = None,
    ) -> tuple[int, int, int, int]:
        current = self._clock() if now is None else now
        manual_axes = tuple(int(value) for value in axes)
        if force_neutral:
            self._reset("safety-neutral", current)
            return (0, 0, 0, 0)
        if not airborne:
            self._reset("grounded", current)
            return manual_axes
        if manual_control_active:
            self._reset("manual", current)
            return manual_axes
        if not control_stream_fresh:
            self._reset("control-unavailable", current)
            return (0, 0, 0, 0)

        pose = self._pose_source.latest(current)
        if pose is None or not self._pose_is_trusted(pose):
            self._reset("waiting-pose", current)
            return (0, 0, 0, 0)

        yaw_degrees = _quaternion_yaw_degrees(pose.rotation)
        if self._target is None:
            self._target = (
                pose.position[0],
                pose.position[1],
                pose.position[2],
                yaw_degrees,
            )
            self._last_output = (0.0, 0.0, 0.0, 0.0)
            self._last_apply_at = current
        self._set_state("holding")

        target_x, target_y, target_z, target_yaw = self._target
        error_x = _outside_deadband(
            target_x - pose.position[0],
            self._policy.horizontal_deadband_meters,
        )
        error_y = _outside_deadband(
            target_y - pose.position[1],
            self._policy.vertical_deadband_meters,
        )
        error_z = _outside_deadband(
            target_z - pose.position[2],
            self._policy.horizontal_deadband_meters,
        )
        velocity_x = _outside_deadband(
            pose.velocity[0],
            self._policy.velocity_deadband_mps,
        )
        velocity_y = _outside_deadband(
            pose.velocity[1],
            self._policy.velocity_deadband_mps,
        )
        velocity_z = _outside_deadband(
            pose.velocity[2],
            self._policy.velocity_deadband_mps,
        )

        world_x = (
            self._policy.horizontal_position_gain * error_x
            - self._policy.horizontal_velocity_gain * velocity_x
        )
        world_z = (
            self._policy.horizontal_position_gain * error_z
            - self._policy.horizontal_velocity_gain * velocity_z
        )
        yaw_radians = math.radians(yaw_degrees)
        right = world_x * math.cos(yaw_radians) - world_z * math.sin(yaw_radians)
        forward = world_x * math.sin(yaw_radians) + world_z * math.cos(yaw_radians)
        up = (
            self._policy.vertical_position_gain * error_y
            - self._policy.vertical_velocity_gain * velocity_y
        )
        yaw_error = _delta_degrees(target_yaw, yaw_degrees)
        yaw = (
            0.0
            if abs(yaw_error) <= self._policy.yaw_deadband_degrees
            else self._policy.yaw_gain * yaw_error
        )
        requested = (
            _clamp(right, self._policy.maximum_horizontal_axis),
            _clamp(forward, self._policy.maximum_horizontal_axis),
            _clamp(up, self._policy.maximum_vertical_axis),
            _clamp(yaw, self._policy.maximum_yaw_axis),
        )
        return self._slew_limited(requested, current)

    def close(self) -> None:
        self._pose_source.close()

    def _pose_is_trusted(self, pose: PosePacketV2) -> bool:
        values = (
            *pose.position,
            *pose.rotation,
            *pose.velocity,
            *pose.position_stddev,
            pose.yaw_stddev,
            pose.confidence,
        )
        rotation_length_squared = sum(value * value for value in pose.rotation)
        return (
            pose.mode in {TrackingMode.APRILTAG_LOCKED, TrackingMode.VISUAL_TRACKED}
            and not pose.flags & (LOCAL_MAP_DEGRADED_FLAG | RELOCALIZING_FLAG)
            and all(math.isfinite(value) for value in values)
            and 0.5 <= rotation_length_squared <= 1.5
            and all(value >= 0.0 for value in pose.position_stddev)
            and pose.confidence >= self._policy.minimum_confidence
            and max(pose.position_stddev[0], pose.position_stddev[2])
            <= self._policy.maximum_horizontal_stddev_meters
            and pose.position_stddev[1]
            <= self._policy.maximum_vertical_stddev_meters
            and 0.0 <= pose.yaw_stddev
            <= self._policy.maximum_yaw_stddev_degrees
        )

    def _slew_limited(
        self,
        requested: tuple[float, float, float, float],
        now: float,
    ) -> tuple[int, int, int, int]:
        elapsed = max(0.0, min(0.1, now - self._last_apply_at))
        maximum_delta = self._policy.maximum_axis_slew_per_second * elapsed
        output = tuple(
            previous + max(-maximum_delta, min(maximum_delta, target - previous))
            for previous, target in zip(self._last_output, requested)
        )
        self._last_output = output
        self._last_apply_at = now
        return tuple(int(round(value)) for value in output)

    def _reset(self, state: str, now: float) -> None:
        self._target = None
        self._last_output = (0.0, 0.0, 0.0, 0.0)
        self._last_apply_at = now
        self._set_state(state)

    def _set_state(self, state: str) -> None:
        if state == self._state:
            return
        log.info("Hover Hold state %s -> %s", self._state, state)
        self._state = state


def _outside_deadband(value: float, deadband: float) -> float:
    return 0.0 if abs(value) <= deadband else value


def _clamp(value: float, maximum: int) -> float:
    return max(-float(maximum), min(float(maximum), value))


def _delta_degrees(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


# Unity 좌표계 quaternion의 전방 벡터로 Venue yaw를 계산한다.
def _quaternion_yaw_degrees(value: tuple[float, float, float, float]) -> float:
    length = math.sqrt(sum(component * component for component in value))
    if length <= 1e-12:
        return 0.0
    x, y, z, w = (component / length for component in value)
    forward_x = 2.0 * (x * z + w * y)
    forward_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.atan2(forward_x, forward_z))
