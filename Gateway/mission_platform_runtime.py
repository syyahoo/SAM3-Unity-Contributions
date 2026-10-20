from __future__ import annotations

import argparse
import asyncio
import importlib
import json
import logging
import os
import queue
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from dotenv import load_dotenv

from control_protocol import (
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
    COMMAND_LAND,
    COMMAND_NEUTRAL,
    COMMAND_RC,
    COMMAND_TAKEOFF,
    INTERNAL_CONTROL_FLAG,
    ControlPacket,
    ControlMotionPublisher,
    encode_ack,
)
from pilot_runtime import (
    EMERGENCY_RECV_SLICE_SECONDS,
    CapturingVideoProxy,
    EmergencyCommandCoordinator,
    EmergencyCommandPreempted,
    LocalVideoSession,
    PilotReadyLogHandler,
    create_resilient_video_publisher,
)
from pilot_shared_control import PilotSharedControlInput
from pilot_shared_frame import PilotSharedFramePublisher
from vision_frame_tcp import LatestFrameTcpPublisher
from xr_control_adapter import query_performance_counter
from desktop_control_input import DesktopControlInput
from flight_safety_gateway import FlightSafetyGateway
from hover_hold import ImageHoverHoldController, HoverMotionReceiver
from xr_control_adapter import query_performance_frequency


SUPPORTED_COMMANDS = {
    COMMAND_RC,
    COMMAND_NEUTRAL,
    COMMAND_TAKEOFF,
    COMMAND_LAND,
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
}
CONFIRMED_COMMANDS = {
    COMMAND_TAKEOFF: "takeoff",
    COMMAND_LAND: "land",
    COMMAND_EMERGENCY: "emergency",
}

CONTROL_RATE_HZ = 60
CONTROL_INPUT_HOLD_SECONDS = 0.1
CONTROL_INPUT_NEUTRAL_SECONDS = 0.3
CONTROL_RECOVERY_BLEND_SECONDS = 0.1
CONTROL_RECOVERY_FRESH_FRAMES = 3
CONTROL_LINK_LOSS_GRACE_SECONDS = 1.0
CONTROL_STATUS_INTERVAL_SECONDS = 1.0
CONTROL_COMMAND_QUEUE_SIZE = 8
CONTROL_RECONNECT_BACKOFF_SECONDS = (0.25, 0.5, 1.0, 2.0)
VIDEO_COMMAND_TIMEOUT_SECONDS = 0.75
VIDEO_CAPTURE_FFMPEG_OPTIONS = "fflags;nobuffer|flags;low_delay|max_delay;0"


# H.264 프레임 스레드 누적으로 생기는 지연을 막고 Tello 영상을 연다.
def open_low_latency_tello_capture(strike_main, url: str, retries: int = 8):
    cv2 = strike_main.cv2
    os.environ.setdefault(
        "OPENCV_FFMPEG_CAPTURE_OPTIONS",
        VIDEO_CAPTURE_FFMPEG_OPTIONS,
    )
    for attempt in range(1, retries + 1):
        capture = cv2.VideoCapture(
            url,
            cv2.CAP_FFMPEG,
            [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
                5000,
                cv2.CAP_PROP_READ_TIMEOUT_MSEC,
                1000,
                cv2.CAP_PROP_N_THREADS,
                1,
            ],
        )
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if capture.isOpened():
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                ok, frame = capture.read()
                if ok and frame is not None and frame.size:
                    return capture
                time.sleep(0.1)
        capture.release()
        time.sleep(min(0.5 * attempt, 2.0))
    raise RuntimeError(f"Tello H.264 stream did not produce a frame: {url}")


@dataclass(frozen=True)
class ControlRuntimePolicy:
    """60Hz 제어 루프와 짧은 입력 단절 완화 정책을 보관한다."""

    rate_hz: int = CONTROL_RATE_HZ
    hold_seconds: float = CONTROL_INPUT_HOLD_SECONDS
    neutral_seconds: float = CONTROL_INPUT_NEUTRAL_SECONDS
    recovery_blend_seconds: float = CONTROL_RECOVERY_BLEND_SECONDS
    recovery_fresh_frames: int = CONTROL_RECOVERY_FRESH_FRAMES
    link_loss_grace_seconds: float = CONTROL_LINK_LOSS_GRACE_SECONDS
    reconnect_backoff_seconds: tuple[float, ...] = CONTROL_RECONNECT_BACKOFF_SECONDS

    def __post_init__(self) -> None:
        if self.rate_hz <= 0:
            raise ValueError("제어 주기는 양수여야 합니다.")
        if self.hold_seconds < 0 or self.neutral_seconds <= self.hold_seconds:
            raise ValueError("입력 중립 시간은 유지 시간보다 길어야 합니다.")
        if self.recovery_blend_seconds < 0 or self.recovery_fresh_frames <= 0:
            raise ValueError("제어 복구 정책 값이 올바르지 않습니다.")
        if self.link_loss_grace_seconds <= 0:
            raise ValueError("Tello 링크 손실 유예 시간은 양수여야 합니다.")
        if not self.reconnect_backoff_seconds or any(
            delay <= 0 for delay in self.reconnect_backoff_seconds
        ):
            raise ValueError("재연결 백오프 값은 모두 양수여야 합니다.")

    # 입력 갱신이 멈춘 뒤 유지 구간을 지나 축을 중립까지 선형 감쇠한다.
    def decayed_axes(
        self,
        axes: tuple[int, int, int, int],
        age_seconds: float,
    ) -> tuple[int, int, int, int]:
        if age_seconds <= self.hold_seconds:
            return axes
        if age_seconds >= self.neutral_seconds:
            return (0, 0, 0, 0)
        scale = (
            (self.neutral_seconds - age_seconds)
            / (self.neutral_seconds - self.hold_seconds)
        )
        return tuple(int(round(value * scale)) for value in axes)


@dataclass(order=True, frozen=True)
class _ControlWork:
    priority: int
    order: int
    action: str = field(compare=False)


class HighRateControlEngine:
    """입력 폴링과 RC 송신을 고정 주기로 실행하고 확인 명령을 분리한다."""

    _ACTION_PRIORITIES = {
        "land": 1,
        "reconnect": 2,
        "takeoff": 3,
    }

    def __init__(
        self,
        control_input,
        adapter,
        rc_factory: Callable,
        *,
        policy: ControlRuntimePolicy | None = None,
        hover_hold: ImageHoverHoldController | None = None,
        flight_safety: FlightSafetyGateway | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._control_input = control_input
        self._adapter = adapter
        self._rc_factory = rc_factory
        self._policy = policy or ControlRuntimePolicy()
        self._hover_hold = hover_hold
        self._flight_safety = flight_safety
        self._clock = clock
        self._stop = threading.Event()
        self._control_thread: threading.Thread | None = None
        self._command_thread: threading.Thread | None = None
        self._work_queue: queue.PriorityQueue[_ControlWork] = queue.PriorityQueue(
            CONTROL_COMMAND_QUEUE_SIZE,
        )
        self._state_lock = threading.Lock()
        self._work_order = 0
        self._last_axes = (0, 0, 0, 0)
        self._last_input_at = clock()
        self._recovery_required = True
        self._recovery_release_seen = False
        self._recovery_fresh_count = 0
        self._recovery_started_at: float | None = None
        self._reconnect_pending = False
        self._link_unhealthy_since: float | None = None
        self._started_at = clock()
        self._last_link_check_at = 0.0
        self._last_status_at = 0.0
        self._last_exception_log_at = 0.0
        self.sent_ticks = 0
        self.failed_ticks = 0
        self.missed_deadlines = 0

    @property
    def is_running(self) -> bool:
        thread = self._control_thread
        return thread is not None and thread.is_alive() and not self._stop.is_set()

    # 명령 작업자와 60Hz 제어 쓰레드를 순서대로 시작한다.
    def start(self) -> None:
        if self.is_running:
            return
        self._stop.clear()
        self._started_at = self._clock()
        self._command_thread = threading.Thread(
            target=self._command_loop,
            name="StrikerPCMissionCommand",
            daemon=True,
        )
        self._control_thread = threading.Thread(
            target=self._control_loop,
            name="StrikerPCMissionControl60Hz",
            daemon=True,
        )
        self._command_thread.start()
        self._control_thread.start()

    # 중립을 먼저 전송하고 두 쓰레드를 제한 시간 안에 종료한다.
    def stop(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        try:
            self._adapter.send_rc_control(self._rc_factory())
        except Exception as error:
            logging.warning("Final neutral RC failed: %s", error)
        for thread in (self._control_thread, self._command_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logging.error("Control worker did not stop in time: %s", thread.name)
        self._fail_queued_actions()

    # 재연결 직후 신선한 프레임 세 개가 들어올 때까지 중립을 유지한다.
    def require_recovery(self) -> None:
        with self._state_lock:
            self._last_axes = (0, 0, 0, 0)
            self._last_input_at = self._clock()
            self._recovery_required = True
            self._recovery_release_seen = False
            self._recovery_fresh_count = 0
            self._recovery_started_at = None

    # 전송 실패 또는 상태 패킷 단절을 단일 재연결 작업으로 합친다.
    def request_reconnect(self) -> None:
        with self._state_lock:
            if self._reconnect_pending or self._stop.is_set():
                return
            self._reconnect_pending = True
        self.require_recovery()
        self._control_input.set_reconnecting()
        self._queue_work("reconnect")

    # 단조 증가 시계를 기준으로 누락 슬롯을 건너뛰며 고정 주기를 유지한다.
    def _control_loop(self) -> None:
        interval = 1.0 / self._policy.rate_hz
        next_tick = self._clock()
        while not self._stop.is_set():
            now = self._clock()
            remaining = next_tick - now
            if remaining > 0:
                self._stop.wait(min(remaining, interval))
                continue

            overdue = now - next_tick
            if overdue >= interval:
                missed = int(overdue / interval)
                self.missed_deadlines += missed
                next_tick += missed * interval
            next_tick += interval

            try:
                self._run_control_tick(now)
            except Exception:
                self.failed_ticks += 1
                self._log_loop_exception(now)
                try:
                    self.request_reconnect()
                except Exception:
                    logging.exception("Failed to enter reconnect state")
            self._log_status_if_due(now)

    # 입력 한 건을 반영하고 현재 정책에 맞는 RC를 한 번 전송한다.
    def _run_control_tick(self, now: float) -> None:
        value = self._control_input.poll_for_engine()
        if value is not None:
            action = getattr(value, "action", None)
            if action is not None:
                with self._state_lock:
                    self._last_axes = (0, 0, 0, 0)
                    self._last_input_at = now
                self._dispatch_action(action)
            else:
                axes = (
                    int(value.left_right),
                    int(value.forward_back),
                    int(value.up_down),
                    int(value.yaw),
                )
                with self._state_lock:
                    self._last_axes = axes
                    self._last_input_at = now
                    if self._recovery_required and self._control_input.active_source in {
                        "desktop",
                        "shared",
                        "unity",
                    }:
                        if not self._recovery_release_seen:
                            if axes == (0, 0, 0, 0):
                                self._recovery_release_seen = True
                                self._recovery_fresh_count = 1
                        else:
                            self._recovery_fresh_count += 1
                        if self._recovery_release_seen and (
                            self._recovery_fresh_count
                            >= self._policy.recovery_fresh_frames
                        ):
                            self._recovery_required = False
                            self._recovery_started_at = now

        with self._state_lock:
            axes = self._policy.decayed_axes(
                self._last_axes,
                max(0.0, now - self._last_input_at),
            )
            if self._recovery_required:
                axes = (0, 0, 0, 0)
            elif self._recovery_started_at is not None:
                blend_age = now - self._recovery_started_at
                blend_seconds = self._policy.recovery_blend_seconds
                if blend_seconds > 0 and blend_age < blend_seconds:
                    blend = max(0.0, min(1.0, blend_age / blend_seconds))
                    axes = tuple(int(round(value * blend)) for value in axes)
                else:
                    self._recovery_started_at = None
            recovery_required = self._recovery_required

        if self._hover_hold is not None:
            safety_controller = (
                None if self._flight_safety is None else self._flight_safety.controller
            )
            axes = self._hover_hold.apply(
                axes,
                manual_control_active=self._control_input.manual_control_active,
                control_stream_fresh=(
                    self._control_input.control_stream_fresh
                    and not recovery_required
                ),
                airborne=bool(
                    safety_controller is not None and safety_controller.airborne
                ),
                force_neutral=bool(
                    safety_controller is not None and safety_controller.force_neutral
                ),
                now=now,
            )

        sent = bool(self._adapter.send_rc_control(self._rc_factory(*axes)))
        if sent:
            self.sent_ticks += 1
        else:
            self.failed_ticks += 1
            if not self._control_input.emergency_latched:
                self.request_reconnect()
        self._check_link_health(now)

    # Emergency는 기존 선점 채널로 보내고 확인 명령만 작업 큐에 넣는다.
    def _dispatch_action(self, action: str) -> None:
        if action == "emergency":
            self._control_input.request_emergency()
            return
        if action not in self._ACTION_PRIORITIES:
            self._control_input.complete_action(action, False)
            return
        self._queue_work(action)

    def _queue_work(self, action: str) -> None:
        with self._state_lock:
            self._work_order += 1
            work = _ControlWork(
                self._ACTION_PRIORITIES[action],
                self._work_order,
                action,
            )
        try:
            self._work_queue.put_nowait(work)
        except queue.Full:
            if action != "reconnect":
                self._control_input.complete_action(action, False)
            else:
                self._fail_queued_actions()
                try:
                    self._work_queue.put_nowait(work)
                except queue.Full:
                    with self._state_lock:
                        self._reconnect_pending = False
                    logging.error("Reconnect request queue is full")

    # 확인 명령의 응답 대기를 제어 쓰레드에서 완전히 분리한다.
    def _command_loop(self) -> None:
        while not self._stop.is_set():
            try:
                work = self._work_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if work.action == "reconnect":
                    self._run_reconnect()
                    continue
                action = getattr(self._adapter, work.action, None)
                if action is None:
                    self._control_input.complete_action(work.action, False)
                    continue
                if work.action == "takeoff":
                    arm = getattr(self._adapter, "arm", None)
                    if arm is not None and not arm():
                        logging.warning("Tello SDK arm command was not acknowledged")
                action()
            except Exception as error:
                logging.exception("Tello control action failed (%s): %s", work.action, error)
            finally:
                self._work_queue.task_done()

    # 프로세스를 유지한 채 Tello 명령 소켓만 백오프로 다시 연결한다.
    def _run_reconnect(self) -> None:
        self._fail_queued_actions()
        delays = self._policy.reconnect_backoff_seconds
        attempt = 0
        try:
            while not self._stop.is_set():
                attempt += 1
                if self._adapter.strikerpc_reconnect_once():
                    self.require_recovery()
                    self._link_unhealthy_since = None
                    self._control_input.finish_reconnect()
                    logging.info("Tello transport reconnected after %d attempt(s)", attempt)
                    return
                delay = delays[min(attempt - 1, len(delays) - 1)]
                logging.warning(
                    "Tello transport reconnect failed attempt=%d retryIn=%.2fs",
                    attempt,
                    delay,
                )
                if self._stop.wait(delay):
                    return
        finally:
            with self._state_lock:
                self._reconnect_pending = False

    def _fail_queued_actions(self) -> None:
        while True:
            try:
                work = self._work_queue.get_nowait()
            except queue.Empty:
                return
            try:
                if work.action != "reconnect":
                    self._control_input.complete_action(work.action, False)
            finally:
                self._work_queue.task_done()

    # Tello 상태 패킷이 유예 시간 동안 끊긴 경우에만 재연결을 요청한다.
    def _check_link_health(self, now: float) -> None:
        if now - self._last_link_check_at < 0.25:
            return
        self._last_link_check_at = now
        if self._adapter.strikerpc_link_is_healthy():
            self._link_unhealthy_since = None
            return
        if now - self._started_at < self._policy.link_loss_grace_seconds:
            return
        if self._link_unhealthy_since is None:
            self._link_unhealthy_since = now
            return
        if now - self._link_unhealthy_since >= self._policy.link_loss_grace_seconds:
            self.request_reconnect()

    def _log_loop_exception(self, now: float) -> None:
        if now - self._last_exception_log_at < CONTROL_STATUS_INTERVAL_SECONDS:
            return
        self._last_exception_log_at = now
        logging.exception("Unexpected exception in 60Hz control loop")

    def _log_status_if_due(self, now: float) -> None:
        if now - self._last_status_at < CONTROL_STATUS_INTERVAL_SECONDS:
            return
        self._last_status_at = now
        logging.info(
            "Control 60Hz status sent=%d failed=%d missed=%d source=%s inputAgeMs=%d",
            self.sent_ticks,
            self.failed_ticks,
            self.missed_deadlines,
            self._control_input.active_source,
            max(0, round((now - self._last_input_at) * 1000)),
        )


class MissionPlatformControlInput:
    """StrikerPC XR 고정 패킷을 mission-platform 입력 객체로 변환한다."""

    def __init__(
        self,
        input_factory: Callable,
        *,
        pilot_port: int = 49210,
        watchdog_seconds: float = CONTROL_INPUT_NEUTRAL_SECONDS,
        clock: Callable[[], float] = time.monotonic,
        counter: Callable[[], int] = query_performance_counter,
        counter_frequency: int | None = None,
        input_freshness_seconds: float | None = None,
        shared_input: PilotSharedControlInput | None = None,
        desktop_input: DesktopControlInput | None = None,
        emergency_coordinator: EmergencyCommandCoordinator | None = None,
        control_policy: ControlRuntimePolicy | None = None,
        hover_hold: ImageHoverHoldController | None = None,
        flight_safety: FlightSafetyGateway | None = None,
    ) -> None:
        self._input_factory = input_factory
        self._watchdog_seconds = watchdog_seconds
        self._clock = clock
        self._counter = counter
        self._counter_frequency = counter_frequency or query_performance_frequency()
        self._input_freshness_seconds = input_freshness_seconds
        self._input_tick_offset: int | None = None
        self._shared_input = shared_input
        self._desktop_input = desktop_input
        self._emergency_coordinator = emergency_coordinator
        self._control_policy = control_policy or ControlRuntimePolicy()
        self._hover_hold = hover_hold
        self._flight_safety = flight_safety
        self._control_engine: HighRateControlEngine | None = None
        self._adapter = None
        self._rc_factory: Callable | None = None
        self._last_sequence = -1
        self._last_packet_at = clock()
        self._watchdog_sent = True
        self._adapter_sender: tuple[str, int] | None = None
        self._pending_command: tuple[int, tuple[str, int], str] | None = None
        self._higher_priority_active = False
        self._udp_neutral_handoff = False
        self._active_source = "unknown"
        self._manual_control_active = False
        self._last_control_frame_at: float | None = None
        self._udp_manual_control_active = False
        self._udp_control_frame_at: float | None = None
        self._reconnecting = False
        self._connected = False
        self._closed = False
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("127.0.0.1", pilot_port))
        self._socket.setblocking(False)
        if self._desktop_input is not None and self._emergency_coordinator is not None:
            handler_setter = getattr(self._desktop_input, "set_emergency_handler", None)
            if handler_setter is not None:
                handler_setter(self._emergency_coordinator.request_emergency)

    @property
    def address(self) -> tuple[str, int]:
        return self._socket.getsockname()

    @property
    def active_source(self) -> str:
        return self._active_source

    @property
    def manual_control_active(self) -> bool:
        return self._manual_control_active

    @property
    def control_stream_fresh(self) -> bool:
        return bool(
            self._last_control_frame_at is not None
            and 0.0 <= self._clock() - self._last_control_frame_at
            <= self._watchdog_seconds
        )

    @property
    def emergency_latched(self) -> bool:
        return bool(
            self._emergency_coordinator is not None
            and self._emergency_coordinator.is_latched
        )

    # Tello 연결 후 생성된 로컬 어댑터를 60Hz 제어 엔진에 연결한다.
    def attach_adapter(self, adapter, rc_factory: Callable) -> None:
        if self._closed:
            raise RuntimeError("닫힌 제어 입력에는 Tello 어댑터를 연결할 수 없습니다.")
        self._adapter = adapter
        self._rc_factory = rc_factory
        self._control_engine = HighRateControlEngine(
            self,
            adapter,
            rc_factory,
            policy=self._control_policy,
            hover_hold=self._hover_hold,
            flight_safety=self._flight_safety,
            clock=self._clock,
        )

    def connect(self) -> bool:
        if self._closed:
            return False
        self._connected = True
        self._set_control_ready(True)
        if self._control_engine is not None:
            self._control_engine.start()
        print("STRIKERPC_STAGE PILOT_IPC_READY", flush=True)
        return True

    # 최신 패킷만 적용하고 입력이 끊기면 중립 입력을 한 번 반환한다.
    def get_input(self):
        if self._control_engine is not None and self._control_engine.is_running:
            return None
        return self.poll_for_engine()

    # 60Hz 제어 쓰레드 또는 기존 upstream 루프가 최신 입력을 한 번 폴링한다.
    def poll_for_engine(self):
        if not self._connected or self._closed:
            return None
        if self._reconnecting:
            self._manual_control_active = False
            self._last_control_frame_at = None
            desktop_input = (
                self._desktop_input.poll_action_nowait()
                if self._desktop_input is not None
                else None
            )
            shared_input = (
                self._shared_input.poll_action_nowait()
                if self._shared_input is not None
                else None
            )
            self._poll_udp(accept_external=False)
            for value in (desktop_input, shared_input):
                command = getattr(value, "action", None)
                if command is not None:
                    self.complete_action(command, False)
            return None
        if (
            self._emergency_coordinator is not None
            and self._emergency_coordinator.is_latched
        ):
            self._manual_control_active = False
            self._last_control_frame_at = None
            return None

        desktop_input = (
            self._desktop_input.poll_action_nowait()
            if self._desktop_input is not None
            else None
        )
        shared_input = (
            self._shared_input.poll_action_nowait()
            if self._shared_input is not None
            else None
        )

        desktop_active = self._desktop_input is not None and (
            self._desktop_input.is_active or desktop_input is not None
        )
        shared_active = self._shared_input is not None and (
            self._shared_input.is_active or shared_input is not None
        )
        higher_priority_active = desktop_active or shared_active
        if not higher_priority_active and self._higher_priority_active:
            self._higher_priority_active = False
            self._udp_neutral_handoff = True
        internal_neutral = self._poll_udp(accept_external=not higher_priority_active)
        if internal_neutral is not None and higher_priority_active:
            self._manual_control_active = False
            self._last_control_frame_at = None
            return internal_neutral
        if higher_priority_active:
            self._higher_priority_active = True
            source = self._desktop_input if desktop_active else self._shared_input
            value = desktop_input if desktop_active else shared_input
            self._active_source = "desktop" if desktop_active else "shared"
            self._manual_control_active = bool(
                getattr(source, "control_active", source.is_active)
            )
            if value is not None:
                self._last_control_frame_at = self._clock()
            return value
        if internal_neutral is not None:
            self._apply_udp_control_state()
            return internal_neutral

        if (
            not self._watchdog_sent
            and self._clock() - self._last_packet_at > self._watchdog_seconds
        ):
            self._watchdog_sent = True
            self._manual_control_active = False
            return self._create_input((0, 0, 0, 0))
        return None

    # 연결 복구 중 신규 동작 명령을 차단하고 기존 입력을 즉시 중립화한다.
    def set_reconnecting(self) -> None:
        if self._closed:
            return
        pending = self._pending_command
        if pending is not None:
            self._send_ack(pending[0], pending[1], success=False)
            self._pending_command = None
        self._set_control_ready(False)
        self._reconnecting = True
        self._active_source = "reconnecting"
        self._manual_control_active = False
        self._last_control_frame_at = None
        self._higher_priority_active = False
        self._udp_neutral_handoff = True

    # 새 Tello 세션이 준비되면 중립 상태에서 입력 수신을 다시 허용한다.
    def finish_reconnect(self) -> None:
        if self._closed:
            return
        self._last_packet_at = self._clock()
        self._watchdog_sent = True
        self._adapter_sender = None
        self._last_sequence = -1
        self._input_tick_offset = None
        self._reconnecting = False
        self._active_source = "reconnecting"
        self._manual_control_active = False
        self._last_control_frame_at = None
        self._udp_manual_control_active = False
        self._udp_control_frame_at = None
        self._set_control_ready(True)

    # 영상 또는 RC 경로에서도 같은 재연결 상태기로 진입한다.
    def request_reconnect(self) -> None:
        if self._control_engine is not None:
            self._control_engine.request_reconnect()

    # 모든 입력 소스의 emergency 요청을 기존 단일 선점 채널로 전달한다.
    def request_emergency(self) -> None:
        if self._emergency_coordinator is None:
            self.complete_action("emergency", False)
            return
        self._emergency_coordinator.request_emergency()

    # 상위 입력 중에도 UDP queue를 비우고, 전환 후 fresh frame만 적용한다.
    def _poll_udp(self, *, accept_external: bool):
        latest_input = None
        while True:
            try:
                payload, sender = self._socket.recvfrom(256)
            except BlockingIOError:
                break

            packet = ControlPacket.decode(payload)
            if packet is None or packet.command not in SUPPORTED_COMMANDS:
                continue

            if packet.flags == INTERNAL_CONTROL_FLAG:
                if packet.command == COMMAND_NEUTRAL:
                    self._udp_manual_control_active = False
                    self._udp_control_frame_at = None
                    latest_input = self._create_input((0, 0, 0, 0))
                continue
            if packet.flags != 0:
                continue

            if self._reconnecting:
                if packet.command in {COMMAND_HELLO, *CONFIRMED_COMMANDS}:
                    self._send_ack(packet.sequence, sender, success=False)
                continue

            if packet.command == COMMAND_HELLO:
                existing_unity_session = (
                    self._adapter_sender == sender
                    and self._active_source == "unity"
                )
                self._input_tick_offset = self._counter() - packet.input_ticks
                self._adapter_sender = sender
                self._last_sequence = -1
                self._send_ack(packet.sequence, sender, success=True)
                self._udp_neutral_handoff = False
                self._udp_manual_control_active = False
                self._udp_control_frame_at = None
                if accept_external and not existing_unity_session:
                    return self._create_input((0, 0, 0, 0))
                continue
            if not self._input_is_fresh(packet.input_ticks):
                continue
            if not accept_external:
                continue
            if self._adapter_sender is None or sender != self._adapter_sender:
                continue
            if packet.sequence <= self._last_sequence:
                continue

            self._last_sequence = packet.sequence
            packet_received_at = self._clock()
            self._last_packet_at = packet_received_at
            self._watchdog_sent = False
            self._active_source = "unity"
            self._udp_manual_control_active = packet.command == COMMAND_RC
            self._udp_control_frame_at = packet_received_at

            command = CONFIRMED_COMMANDS.get(packet.command)
            if command is not None:
                if self._pending_command is not None:
                    self._send_ack(packet.sequence, sender, success=False)
                    continue
                self._pending_command = (packet.sequence, sender, command)
                return self._create_input((0, 0, 0, 0), command)

            if self._udp_neutral_handoff:
                self._udp_neutral_handoff = False
                return self._create_input((0, 0, 0, 0))

            axes = (
                (0, 0, 0, 0)
                if packet.command == COMMAND_NEUTRAL
                else tuple(max(-100, min(100, value)) for value in packet.axes)
            )
            latest_input = self._create_input(axes)
        return latest_input

    # UDP 프레임의 Neutral/Rc 의미를 축 값과 분리해 Hover Hold에 전달한다.
    def _apply_udp_control_state(self) -> None:
        self._manual_control_active = self._udp_manual_control_active
        self._last_control_frame_at = self._udp_control_frame_at

    # HELLO에서 맞춘 프로세스별 Stopwatch 기준점으로 입력 freshness를 계산한다.
    def _input_is_fresh(self, input_ticks: int) -> bool:
        maximum_age = self._input_freshness_seconds
        if maximum_age is None:
            return True
        normalized_input_ticks = input_ticks
        if self._input_tick_offset is not None:
            normalized_input_ticks += self._input_tick_offset
        age_ticks = self._counter() - normalized_input_ticks
        return (
            -0.05 * self._counter_frequency
            <= age_ticks
            <= maximum_age * self._counter_frequency
        )

    # 실제 Tello 명령 결과를 원래 XR 요청에 회신한다.
    def complete_action(
        self,
        command: str,
        success: bool,
        *,
        dispatched: bool = False,
    ) -> None:
        if self._desktop_input is not None:
            if dispatched:
                self._desktop_input.complete_command(
                    command,
                    success,
                    dispatched=True,
                )
            else:
                self._desktop_input.complete_command(command, success)
        if self._shared_input is not None:
            self._shared_input.complete_command(command, success)
        pending = self._pending_command
        if pending is None or pending[2] != command:
            return
        self._pending_command = None
        self._send_ack(pending[0], pending[1], success=success)

    # 비상 mailbox 결과는 원 요청자인 Desktop 입력에만 회신한다.
    def complete_desktop_emergency(
        self,
        success: bool,
        *,
        dispatched: bool,
    ) -> None:
        if self._desktop_input is not None:
            self._desktop_input.complete_command(
                "emergency",
                success,
                dispatched=dispatched,
            )

    def disconnect(self) -> bool:
        if self._closed:
            return True
        self._set_control_ready(False)
        if self._control_engine is not None:
            self._control_engine.stop()
        pending = self._pending_command
        if pending is not None:
            self._send_ack(pending[0], pending[1], success=False)
            self._pending_command = None
        self._connected = False
        self._closed = True
        self._manual_control_active = False
        self._last_control_frame_at = None
        self._socket.close()
        if self._shared_input is not None:
            self._shared_input.close_nowait()
        if self._desktop_input is not None:
            self._desktop_input.close_nowait()
        return True

    # Tello 준비 상태를 Desktop receiver와 비상 mailbox에 함께 전달한다.
    def _set_control_ready(self, ready: bool) -> None:
        if self._emergency_coordinator is not None:
            self._emergency_coordinator.set_ready(ready)
        if self._desktop_input is not None:
            ready_setter = getattr(self._desktop_input, "set_ready", None)
            if ready_setter is not None:
                ready_setter(ready)

    def _create_input(self, axes: tuple[int, int, int, int], action: str | None = None):
        return self._input_factory(*axes, action)

    def _send_ack(
        self,
        sequence: int,
        destination: tuple[str, int],
        *,
        success: bool,
    ) -> None:
        try:
            self._socket.sendto(
                encode_ack(
                    sequence,
                    query_performance_counter(),
                    status=0 if success else 1,
                ),
                destination,
            )
        except OSError as error:
            logging.warning("Control ACK send failed: %s", error)


def create_managed_tello_adapter(
    base_class,
    control_input,
    rc_factory,
    emergency_coordinator: EmergencyCommandCoordinator | None = None,
    flight_safety: FlightSafetyGateway | None = None,
    motion_publisher: ControlMotionPublisher | None = None,
):
    """명령 ACK와 종료 중립 RC를 보장하는 Tello adapter를 만든다."""

    if emergency_coordinator is not None:
        def complete_emergency(success: bool, dispatched: bool) -> None:
            control_input.complete_action(
                "emergency",
                success,
                dispatched=dispatched,
            )
            if flight_safety is not None and dispatched:
                flight_safety.record_action("emergency", True)

        emergency_coordinator.set_completion_handler(
            complete_emergency,
        )

    class ManagedTelloAdapter(base_class):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._strikerpc_closed = False
            self._strikerpc_transport_reset = False
            self._strikerpc_send_lock = threading.Lock()
            self._strikerpc_worker_stop = threading.Event()
            self._strikerpc_emergency_worker: threading.Thread | None = None
            if flight_safety is not None:
                flight_safety.attach_adapter(self)
            self._start_emergency_worker()
            attach_adapter = getattr(control_input, "attach_adapter", None)
            if attach_adapter is not None:
                attach_adapter(self, rc_factory)

        # 초기 연결 실패가 영구 종료 경로를 호출하지 않도록 전송 소켓만 정리한다.
        def connect(self) -> bool:
            self._strikerpc_transport_reset = True
            try:
                return bool(super().connect())
            finally:
                self._strikerpc_transport_reset = False

        # Tello 상태 포트는 기존 FlightSafetyGateway 한 곳에서만 소유한다.
        def _start_state_receiver(self) -> None:
            if flight_safety is not None:
                return
            super()._start_state_receiver()

        def takeoff(self) -> bool:
            return self._complete("takeoff", super().takeoff)

        def land(self) -> bool:
            return self._complete("land", super().land)

        def emergency(self) -> bool:
            return self._complete("emergency", super().emergency)

        def telemetry(self) -> dict[str, int | float | bool]:
            if flight_safety is None:
                return super().telemetry()
            return flight_safety.telemetry()

        # 영상 명령의 긴 ACK 대기가 land와 제어를 막지 않도록 제한 시간을 줄인다.
        def start_video(self) -> bool:
            success = self._video_command("streamon")
            self._video = success
            if success or (
                emergency_coordinator is not None
                and emergency_coordinator.is_latched
            ):
                return success
            if not self.strikerpc_link_is_healthy():
                control_input.request_reconnect()
            logging.warning("Video streamon will be retried without stopping RC control")
            return True

        def stop_video(self) -> bool:
            success = self._video_command("streamoff")
            if success:
                self._video = False
            return success

        def restart_video(self) -> bool:
            self.stop_video()
            if emergency_coordinator is not None and emergency_coordinator.is_latched:
                return False
            time.sleep(0.3)
            return self.start_video()

        def _video_command(self, command: str) -> bool:
            try:
                return (
                    self.send_command(command, VIDEO_COMMAND_TIMEOUT_SECONDS)
                    .strip()
                    .lower()
                    == "ok"
                )
            except (OSError, RuntimeError, TimeoutError) as error:
                logging.warning("Tello video command failed (%s): %s", command, error)
                return False

        def send_command(self, command: str, timeout: float | None = None) -> str:
            if not self._supports_priority_channel():
                return super().send_command(command, timeout)
            try:
                return self._send_command_with_priority(command, timeout)
            except EmergencyCommandPreempted as error:
                emergency_coordinator.complete_request(
                    error.request_id,
                    error.dispatched,
                )
                raise

        def send_rc_control(self, control) -> bool:
            self._sync_flight_safety_authority()
            if flight_safety is not None and flight_safety.controller.force_neutral:
                control = rc_factory()
            if not self._supports_priority_channel():
                sent = super().send_rc_control(control)
                if sent and motion_publisher is not None:
                    motion_publisher.publish(control)
                return sent
            command = control.to_command()
            if (
                emergency_coordinator.is_latched
                and command.strip().lower() != "rc 0 0 0 0"
            ):
                return False
            try:
                with self._strikerpc_send_lock:
                    command_socket = self._sock
                    if command_socket is None or not self._connected:
                        return False
                    command_socket.sendto(
                        command.encode(),
                        (self.host, self.port),
                    )
                if motion_publisher is not None:
                    motion_publisher.publish(control)
                return True
            except OSError as error:
                logging.warning("Tello RC send failed: %s", error)
                return False

        def close(self) -> None:
            if self._strikerpc_transport_reset:
                super().close()
                return
            if self._strikerpc_closed:
                return
            self._strikerpc_closed = True
            if (
                flight_safety is not None
                and getattr(self, "connected", False)
                and flight_safety.controller.airborne
                and not (
                    emergency_coordinator is not None
                    and emergency_coordinator.is_latched
                )
            ):
                self.send_rc_control(rc_factory())
                flight_safety.controlled_land_now()
            if emergency_coordinator is not None:
                emergency_coordinator.set_ready(False)
            self._strikerpc_worker_stop.set()
            if emergency_coordinator is not None:
                emergency_coordinator.wake()
            worker = self._strikerpc_emergency_worker
            if worker is not None and worker is not threading.current_thread():
                worker.join(timeout=1.0)
            try:
                if getattr(self, "connected", False):
                    self.send_rc_control(rc_factory())
                    time.sleep(0.1)
            finally:
                super().close()

        def _complete(self, command: str, action: Callable[[], bool]) -> bool:
            self._sync_flight_safety_authority()
            try:
                success = bool(action())
            except Exception:
                control_input.complete_action(command, False)
                if flight_safety is not None:
                    flight_safety.record_action(command, False)
                raise
            control_input.complete_action(command, success)
            if flight_safety is not None:
                flight_safety.record_action(command, success)
            return success

        # 실제 활성 입력이 Unity 계열일 때만 Unity heartbeat 착륙 감시를 요구한다.
        def _sync_flight_safety_authority(self) -> None:
            if flight_safety is None:
                return
            source = getattr(control_input, "active_source", "unknown")
            if source == "desktop":
                required = False
            elif source in {"unity", "shared"}:
                required = True
            else:
                return
            setter = getattr(flight_safety, "set_unity_heartbeat_required", None)
            if setter is not None:
                setter(required)

        # 자동 착륙에서만 짧은 idempotent land ACK timeout을 사용한다.
        def strikerpc_safety_land(self, *, timeout_seconds: float = 3.0) -> bool:
            try:
                return self.send_command("land", timeout_seconds).strip().lower() == "ok"
            except (OSError, RuntimeError, TimeoutError) as error:
                logging.warning("Tello safety land failed: %s", error)
                return False

        # Heartbeat watchdog가 상위 입력 루프와 무관하게 즉시 중립 RC를 보낸다.
        def strikerpc_safety_neutral(self) -> bool:
            if not getattr(self, "_sock", None) or not getattr(self, "_connected", False):
                return False
            try:
                neutral = rc_factory()
                payload = neutral.to_command().encode()
                with self._strikerpc_send_lock:
                    command_socket = self._sock
                    if command_socket is None or not self._connected:
                        return False
                    sent = (
                        command_socket.sendto(payload, (self.host, self.port))
                        == len(payload)
                    )
                if sent and motion_publisher is not None:
                    motion_publisher.publish(neutral)
                return sent
            except (AttributeError, OSError) as error:
                logging.warning("Tello safety neutral failed: %s", error)
                return False

        # Tello 상태 UDP와 명령 소켓 상태를 함께 사용해 링크 건강도를 판단한다.
        def strikerpc_link_is_healthy(self) -> bool:
            if not getattr(self, "_connected", False) or not getattr(self, "_sock", None):
                return False
            if flight_safety is None:
                return True
            try:
                return bool(flight_safety.telemetry().get("connected", False))
            except Exception as error:
                logging.warning("Tello link health check failed: %s", error)
                return False

        # 전체 프로세스를 내리지 않고 명령 소켓만 새 세대로 교체한다.
        def strikerpc_reconnect_once(self) -> bool:
            if self._strikerpc_closed or not self._supports_priority_channel():
                return False
            if emergency_coordinator is not None and emergency_coordinator.is_latched:
                return False
            local_port = getattr(self, "local_port", None)
            if not isinstance(local_port, int):
                logging.error("Tello reconnect unavailable: local command port is missing")
                return False

            command_socket = None
            try:
                with self._lock:
                    with self._strikerpc_send_lock:
                        old_socket = self._sock
                        self._sock = None
                        self._connected = False
                        if old_socket is not None:
                            old_socket.close()

                        command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                        command_socket.bind(("0.0.0.0", local_port))
                        command_socket.settimeout(VIDEO_COMMAND_TIMEOUT_SECONDS)
                        command_socket.sendto(b"command", (self.host, self.port))
                        response, _ = command_socket.recvfrom(1024)
                        answer = response.decode(errors="replace").strip().lower()
                        if answer != "ok":
                            raise RuntimeError(f"Tello reconnect rejected: {answer}")
                        command_socket.settimeout(self.timeout)
                        self._sock = command_socket
                        self._connected = True
                        self._video = False
                        command_socket = None
                record_recovery = getattr(flight_safety, "record_link_recovered", None)
                if record_recovery is not None:
                    record_recovery()
                return True
            except (OSError, RuntimeError, TimeoutError) as error:
                logging.warning("Tello command socket reconnect failed: %s", error)
                if command_socket is not None:
                    command_socket.close()
                with self._strikerpc_send_lock:
                    self._sock = None
                    self._connected = False
                return False

        def _supports_priority_channel(self) -> bool:
            return (
                emergency_coordinator is not None
                and hasattr(self, "_sock")
                and hasattr(self, "_lock")
                and hasattr(self, "host")
                and hasattr(self, "port")
            )

        # 기존 command lock 안에서 응답을 50ms 단위로 기다리며 비상을 확인한다.
        def _send_command_with_priority(
            self,
            command: str,
            timeout: float | None,
        ) -> str:
            if not self._sock or not self._connected:
                raise RuntimeError("Tello is not connected")
            with self._lock:
                request_id = emergency_coordinator.claim_request()
                if request_id is not None:
                    raise EmergencyCommandPreempted(
                        request_id,
                        self._dispatch_emergency_locked(request_id),
                    )
                if emergency_coordinator.is_latched:
                    raise RuntimeError("비상 정지 latch 이후 Tello 명령은 거부됩니다.")

                logging.debug(
                    "Tello command send: %s -> %s:%s",
                    command,
                    self.host,
                    self.port,
                )
                with self._strikerpc_send_lock:
                    command_socket = self._sock
                    if command_socket is None or not self._connected:
                        raise RuntimeError("Tello is not connected")
                    command_socket.sendto(command.encode(), (self.host, self.port))
                original_timeout = command_socket.gettimeout()
                response_timeout = timeout if timeout is not None else self.timeout
                deadline = time.monotonic() + response_timeout
                try:
                    while True:
                        request_id = emergency_coordinator.claim_request()
                        if request_id is not None:
                            raise EmergencyCommandPreempted(
                                request_id,
                                self._dispatch_emergency_locked(request_id),
                            )
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise TimeoutError(
                                f"Tello response timed out: {command}",
                            )
                        command_socket.settimeout(
                            min(EMERGENCY_RECV_SLICE_SECONDS, remaining),
                        )
                        try:
                            response, _ = command_socket.recvfrom(1024)
                        except socket.timeout:
                            continue

                        request_id = emergency_coordinator.claim_request()
                        if request_id is not None:
                            raise EmergencyCommandPreempted(
                                request_id,
                                self._dispatch_emergency_locked(request_id),
                            )
                        answer = response.decode(errors="replace").strip()
                        logging.debug(
                            "Tello command response: %s <- %s",
                            command,
                            answer,
                        )
                        if answer.lower() == "error":
                            raise RuntimeError(f"Tello rejected command: {command}")
                        return answer
                finally:
                    try:
                        command_socket.settimeout(original_timeout)
                    except OSError:
                        logging.debug("Tello command socket closed while waiting for ACK")

        # 현재 command lock 소유자만 중립과 emergency를 같은 UDP 소켓에 쓴다.
        def _dispatch_emergency_locked(self, request_id: int) -> bool:
            if not emergency_coordinator.begin_dispatch(request_id):
                return False
            if not self._sock or not self._connected:
                return False
            try:
                with self._strikerpc_send_lock:
                    command_socket = self._sock
                    if command_socket is None or not self._connected:
                        return False
                    command_socket.sendto(b"rc 0 0 0 0", (self.host, self.port))
            except OSError as error:
                logging.warning("비상 정지 전 중립 RC 전송 실패: %s", error)
            if motion_publisher is not None:
                motion_publisher.publish(rc_factory())
            try:
                payload = b"emergency"
                with self._strikerpc_send_lock:
                    command_socket = self._sock
                    if command_socket is None or not self._connected:
                        return False
                    sent = command_socket.sendto(payload, (self.host, self.port))
                logging.warning("Tello emergency 명령을 우선 전송했습니다.")
                return sent == len(payload)
            except OSError as error:
                logging.error("Tello emergency 우선 전송 실패: %s", error)
                return False

        # confirmed 명령이 없을 때도 mailbox를 같은 command lock에서 처리한다.
        def _run_emergency_worker(self) -> None:
            while True:
                signaled = emergency_coordinator.wait_for_request(
                    EMERGENCY_RECV_SLICE_SECONDS,
                )
                if not signaled:
                    if self._strikerpc_worker_stop.is_set():
                        return
                    continue
                request_id = None
                dispatched = False
                with self._lock:
                    request_id = emergency_coordinator.claim_request()
                    if request_id is not None:
                        dispatched = self._dispatch_emergency_locked(request_id)
                if request_id is not None:
                    emergency_coordinator.complete_request(request_id, dispatched)
                if self._strikerpc_worker_stop.is_set():
                    return

        def _start_emergency_worker(self) -> None:
            if not self._supports_priority_channel():
                return
            self._strikerpc_emergency_worker = threading.Thread(
                target=self._run_emergency_worker,
                name="StrikerPCMissionEmergency",
                daemon=True,
            )
            self._strikerpc_emergency_worker.start()

    return ManagedTelloAdapter


def load_mission_source(manifest_path: Path) -> tuple[Path, dict, dict]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    item = manifest["repositories"]["missionPlatform"]
    return (manifest_path.parent / item["path"]).resolve(), item, manifest


# 신규 팀 저장소의 origin, 고정 커밋, tracked 무변경 상태를 확인한다.
def verify_team_source(path: Path, expected: dict) -> None:
    command = ["git", "-c", f"safe.directory={path.as_posix()}", "-C", str(path)]
    commit = subprocess.run(
        [*command, "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=False,
    )
    if commit.returncode != 0 or commit.stdout.strip() != expected["commit"]:
        raise RuntimeError(f"drone-mission-platform 고정 커밋이 아닙니다: {commit.stdout.strip()}")
    origin = subprocess.run(
        [*command, "remote", "get-url", "origin"],
        capture_output=True,
        text=True,
        check=False,
    )
    if origin.returncode != 0 or origin.stdout.strip() != expected["origin"]:
        raise RuntimeError(f"drone-mission-platform origin이 다릅니다: {origin.stdout.strip()}")
    worktree_changed = subprocess.run(
        [*command, "diff", "--quiet", "HEAD", "--"],
        check=False,
    ).returncode
    staged_changed = subprocess.run(
        [*command, "diff", "--cached", "--quiet", "HEAD", "--"],
        check=False,
    ).returncode
    if worktree_changed or staged_changed:
        raise RuntimeError("drone-mission-platform tracked 파일이 수정되어 있습니다.")


def load_livekit_environment(env_path: Path, identity: str, room: str | None) -> None:
    if not env_path.is_file():
        raise RuntimeError(f"LiveKit 환경 파일이 없습니다: {env_path}")
    load_dotenv(env_path, override=False)
    os.environ["LIVEKIT_IDENTITY"] = identity
    if room:
        os.environ["LIVEKIT_ROOM"] = room


def _verify_import_path(module, mission_root: Path) -> None:
    imported_path = Path(module.__file__).resolve()
    if mission_root not in imported_path.parents:
        raise RuntimeError(f"drone-mission-platform 모듈이 아닙니다: {imported_path}")


# 영상 이동과 같은 시점의 Tello 자세가 신선할 때만 호버 보정을 허용한다.
def _hover_attitude_sample(
    flight_safety: FlightSafetyGateway,
) -> tuple[float, float, float, float] | None:
    state = flight_safety.controller.last_state
    if state is None or any(
        value is None
        for value in (
            state.pitch_degrees,
            state.roll_degrees,
            state.yaw_degrees,
        )
    ):
        return None
    return (
        state.received_at,
        float(state.pitch_degrees),
        float(state.roll_degrees),
        float(state.yaw_degrees),
    )


# 새 strike_gateway에 기존 XR 입력과 영상 소비자를 주입해 실행한다.
def run_mission_platform(args: argparse.Namespace) -> int:
    manifest_path = Path(args.manifest).resolve()
    mission_root, expected, manifest = load_mission_source(manifest_path)
    verify_team_source(mission_root, expected)

    pilot_item = manifest["repositories"]["pilot"]
    pilot_root = (manifest_path.parent / pilot_item["path"]).resolve()
    env_path = Path(args.env_file).resolve() if args.env_file else pilot_root / ".env"
    local_only = bool(args.local_only)
    if not local_only:
        load_livekit_environment(env_path, args.identity, args.room)

    sys.path.insert(0, str(mission_root))
    try:
        strike_main = importlib.import_module("apps.strike_gateway.main")
        gamepad_module = importlib.import_module("adapters.gamepad_adapter")
        tello_module = importlib.import_module("adapters.tello_adapter")
        for module in (strike_main, gamepad_module, tello_module):
            _verify_import_path(module, mission_root)

        def create_shared_input(_, *, rc=None, command=None, stop_session=False):
            del stop_session
            if rc is None:
                return gamepad_module.GamepadInput(action=command)
            return gamepad_module.GamepadInput(
                rc.left_right,
                rc.forward_back,
                rc.up_down,
                rc.yaw,
                command,
            )

        mission_id = args.room or (
            "mission-alpha"
            if local_only
            else os.getenv("LIVEKIT_ROOM", "mission-alpha")
        )
        shared_input = PilotSharedControlInput(
            create_shared_input,
            tello_module.RcControl,
            name=manifest["sharedMemory"]["control"],
            watchdog_seconds=args.watchdog_seconds,
            input_freshness_seconds=args.input_freshness_seconds,
        )
        desktop_input = DesktopControlInput(
            create_shared_input,
            tello_module.RcControl,
            port=args.desktop_control_port,
            watchdog_seconds=args.watchdog_seconds,
        )
        emergency_coordinator = EmergencyCommandCoordinator()
        motion_publisher = ControlMotionPublisher(args.measurement_port)
        flight_safety = FlightSafetyGateway(
            state_port=args.state_port,
            heartbeat_port=args.heartbeat_port,
            operation_event_port=args.operation_event_port,
            desktop_state_port=args.desktop_state_port,
            additional_status_ports=(args.dashboard_state_port,),
            drone_id=args.identity,
            mission_id=mission_id,
        )
        hover_hold = (
            ImageHoverHoldController(
                HoverMotionReceiver(args.hover_motion_port),
                attitude_source=lambda: _hover_attitude_sample(flight_safety),
            )
            if args.hover_hold
            else None
        )
        control_input = MissionPlatformControlInput(
            gamepad_module.GamepadInput,
            pilot_port=args.pilot_port,
            watchdog_seconds=args.watchdog_seconds,
            shared_input=shared_input,
            desktop_input=desktop_input,
            emergency_coordinator=emergency_coordinator,
            input_freshness_seconds=args.input_freshness_seconds,
            hover_hold=hover_hold,
            flight_safety=flight_safety,
        )
        managed_adapter = create_managed_tello_adapter(
            tello_module.TelloAdapter,
            control_input,
            tello_module.RcControl,
            emergency_coordinator,
            flight_safety,
            motion_publisher,
        )
        frame_publisher = LatestFrameTcpPublisher(args.vision_tcp_port)
        shared_frame_publisher = PilotSharedFramePublisher(
            manifest["sharedMemory"]["video"],
        )
        original_adapter = strike_main.TelloAdapter
        original_open_capture = strike_main.open_tello_capture
        original_video_publisher = strike_main.VideoPublisher

        def open_capture_proxy(video_url: str):
            return CapturingVideoProxy(
                open_low_latency_tello_capture(strike_main, video_url),
                frame_publisher,
                shared_frame_publisher,
            )

        strike_main.TelloAdapter = managed_adapter
        strike_main.open_tello_capture = open_capture_proxy
        strike_main.VideoPublisher = (
            LocalVideoSession
            if local_only
            else create_resilient_video_publisher(original_video_publisher)
        )
        runtime_args = argparse.Namespace(
            drone_id=args.identity,
            mission=mission_id,
            video_url=f"udp://0.0.0.0:{args.video_port}",
            tello_host=args.tello_ip,
            tello_port=args.command_port,
            local_command_port=args.local_command_port,
            local_state_port=args.state_port,
            land_on_exit=False,
            preview=False,
            demo_position=False,
        )

        ready_handler = PilotReadyLogHandler()
        logging.getLogger().addHandler(ready_handler)
        original_cwd = Path.cwd()
        flight_safety.start()
        try:
            os.chdir(mission_root)
            asyncio.run(strike_main.run(runtime_args, input_source=control_input))
        finally:
            strike_main.TelloAdapter = original_adapter
            strike_main.open_tello_capture = original_open_capture
            strike_main.VideoPublisher = original_video_publisher
            logging.getLogger().removeHandler(ready_handler)
            control_input.disconnect()
            frame_publisher.close()
            shared_frame_publisher.close()
            flight_safety.close()
            if hover_hold is not None:
                hover_hold.close()
            motion_publisher.close()
            os.chdir(original_cwd)
    finally:
        sys.path.remove(str(mission_root))
    return 0


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="drone-mission-platform Strike Gateway 통합 호스트")
    parser.add_argument("--manifest", default=str(root / "team-sources.json"))
    parser.add_argument("--env-file")
    parser.add_argument("--identity", default="strike-gateway-01")
    parser.add_argument("--room")
    parser.add_argument("--tello-ip", default="192.168.10.1")
    parser.add_argument("--command-port", type=int, default=8889)
    parser.add_argument("--local-command-port", type=int, default=9000)
    parser.add_argument("--video-port", type=int, default=11111)
    parser.add_argument("--pilot-port", type=int, default=49210)
    parser.add_argument("--desktop-control-port", type=int, default=49211)
    parser.add_argument("--state-port", type=int, default=8892)
    parser.add_argument("--desktop-state-port", type=int, default=8893)
    parser.add_argument("--dashboard-state-port", type=int, default=8895)
    parser.add_argument("--heartbeat-port", type=int, default=49304)
    parser.add_argument("--operation-event-port", type=int, default=49301)
    parser.add_argument("--watchdog-seconds", type=float, default=0.3)
    parser.add_argument("--input-freshness-seconds", type=float, default=0.25)
    parser.add_argument("--vision-tcp-port", type=int, default=49000)
    parser.add_argument("--measurement-port", type=int, default=49003)
    parser.add_argument("--hover-motion-port", type=int, default=49006)
    parser.add_argument(
        "--hover-pose-port",
        dest="hover_motion_port",
        type=int,
        default=argparse.SUPPRESS,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--hover-hold", action="store_true")
    parser.add_argument(
        "--local-only",
        action="store_true",
        help="LiveKit 없이 SharedMemory와 Vision TCP만 게시합니다.",
    )
    parser.add_argument("--land-on-exit", action="store_true")
    return parser.parse_args()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    try:
        return run_mission_platform(parse_args())
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
