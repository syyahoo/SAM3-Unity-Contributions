from __future__ import annotations

import argparse
import asyncio
import importlib
import json
import logging
import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Callable

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
from xr_control_adapter import query_performance_counter
from desktop_control_input import DesktopControlInput
from pilot_shared_frame import PilotSharedFramePublisher
from pilot_shared_control import PilotSharedControlInput
from vision_frame_tcp import LatestFrameTcpPublisher


SUPPORTED_COMMANDS = {
    COMMAND_RC,
    COMMAND_NEUTRAL,
    COMMAND_TAKEOFF,
    COMMAND_LAND,
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
}

EMERGENCY_RECV_SLICE_SECONDS = 0.05


class EmergencyCommandPreempted(RuntimeError):
    def __init__(self, request_id: int, dispatched: bool) -> None:
        super().__init__("비상 정지가 진행 중인 Tello 명령을 선점했습니다.")
        self.request_id = request_id
        self.dispatched = dispatched


class EmergencyCommandCoordinator:
    """데스크톱 비상 요청을 Tello 소켓 소유자 한 곳에 전달한다."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._ready = False
        self._latched = False
        self._next_request_id = 0
        self._pending_request_id: int | None = None
        self._claimed_request_id: int | None = None
        self._dispatching_request_id: int | None = None
        self._completion_handler: Callable[[bool, bool], None] | None = None

    @property
    def is_latched(self) -> bool:
        with self._lock:
            return self._latched

    # 실제 ACK 전달 함수를 런타임 입력 합성기가 준비된 뒤 연결한다.
    def set_completion_handler(
        self,
        handler: Callable[[bool, bool], None],
    ) -> None:
        with self._lock:
            self._completion_handler = handler

    # Tello 연결 준비 상태만 갱신하고 비상 latch는 런타임 동안 유지한다.
    def set_ready(self, ready: bool) -> None:
        completion: Callable[[bool, bool], None] | None = None
        with self._lock:
            self._ready = bool(ready)
            if not ready:
                self._event.clear()
                if (
                    self._pending_request_id is not None
                    or self._claimed_request_id is not None
                ):
                    self._pending_request_id = None
                    self._claimed_request_id = None
                    completion = self._completion_handler
        if completion is not None:
            completion(False, False)

    # Desktop receiver에서는 소켓을 만지지 않고 요청만 게시한다.
    def request_emergency(self) -> None:
        completion: Callable[[bool, bool], None] | None = None
        with self._lock:
            if (
                not self._ready
                or self._pending_request_id is not None
                or self._claimed_request_id is not None
                or self._dispatching_request_id is not None
            ):
                completion = self._completion_handler
            else:
                self._next_request_id += 1
                self._pending_request_id = self._next_request_id
                self._latched = True
                self._event.set()
        if completion is not None:
            completion(False, False)

    # 현재 Tello lock 소유자가 요청 하나를 원자적으로 가져간다.
    def claim_request(self) -> int | None:
        with self._lock:
            request_id = self._pending_request_id
            if request_id is None:
                return None
            self._pending_request_id = None
            self._claimed_request_id = request_id
            self._event.clear()
            return request_id

    # close와 경쟁하지 않도록 실제 sendto 시작을 원자적으로 확정한다.
    def begin_dispatch(self, request_id: int) -> bool:
        with self._lock:
            if not self._ready or request_id != self._claimed_request_id:
                return False
            self._claimed_request_id = None
            self._dispatching_request_id = request_id
            return True

    # 실제 emergency sendto 결과를 Desktop ACK 경로로 전달한다.
    def complete_request(self, request_id: int, dispatched: bool) -> None:
        completion: Callable[[bool, bool], None] | None = None
        with self._lock:
            if request_id != self._dispatching_request_id:
                return
            self._dispatching_request_id = None
            completion = self._completion_handler
        if completion is not None:
            completion(dispatched, dispatched)

    def wait_for_request(self, timeout: float) -> bool:
        return self._event.wait(timeout)

    def wake(self) -> None:
        self._event.set()


def _is_neutral_rc_command(command: str) -> bool:
    return command.strip().lower() == "rc 0 0 0 0"


class PilotRelayControlInput:
    def __init__(
        self,
        action_factory: Callable,
        rc_factory: Callable,
        *,
        pilot_port: int = 49210,
        watchdog_seconds: float = 0.25,
    ) -> None:
        self._action_factory = action_factory
        self._rc_factory = rc_factory
        self._watchdog_seconds = watchdog_seconds
        self._last_sequence = -1
        self._last_packet_at = time.monotonic()
        self._watchdog_sent = True
        self._pending_command: tuple[int, tuple[str, int], str] | None = None
        self._adapter_sender: tuple[str, int] | None = None
        self._closed = False
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("127.0.0.1", pilot_port))
        self._socket.setblocking(False)
        print("STRIKERPC_STAGE PILOT_IPC_READY", flush=True)

    @property
    def address(self) -> tuple[str, int]:
        return self._socket.getsockname()

    @property
    def instructions(self) -> str:
        return "[Quest3] XR Adapter 입력을 Pilot-PC 제어 루프로 전달합니다."

    # 최신 XR 입력을 원본 ControlAction으로 변환한다.
    async def poll_action(self):
        await asyncio.sleep(0.005)
        latest_rc = None
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
                    latest_rc = self._action_factory(
                        "xr-internal-neutral",
                        rc=self._rc_factory(),
                    )
                continue
            if packet.flags != 0:
                continue

            if packet.command == COMMAND_HELLO:
                self._adapter_sender = sender
                self._last_sequence = -1
                self._socket.sendto(
                    encode_ack(packet.sequence, query_performance_counter()),
                    sender,
                )
                continue
            if self._adapter_sender is None or sender != self._adapter_sender:
                continue
            if packet.sequence <= self._last_sequence:
                continue

            self._last_sequence = packet.sequence
            self._last_packet_at = time.monotonic()
            self._watchdog_sent = False

            if packet.command in {
                COMMAND_TAKEOFF,
                COMMAND_LAND,
                COMMAND_EMERGENCY,
            }:
                if self._pending_command is not None:
                    self._socket.sendto(
                        encode_ack(
                            packet.sequence,
                            query_performance_counter(),
                            status=1,
                        ),
                        sender,
                    )
                    continue
                command = {
                    COMMAND_TAKEOFF: "takeoff",
                    COMMAND_LAND: "land",
                    COMMAND_EMERGENCY: "emergency",
                }[packet.command]
                self._pending_command = (packet.sequence, sender, command)
                return self._action_factory(
                    command,
                    command=command,
                    stop_session=packet.command == COMMAND_EMERGENCY,
                )

            axes = (
                (0, 0, 0, 0)
                if packet.command == COMMAND_NEUTRAL
                else tuple(max(-100, min(100, value)) for value in packet.axes)
            )
            latest_rc = self._action_factory(
                "quest3-rc",
                rc=self._rc_factory(
                    left_right=axes[0],
                    forward_back=axes[1],
                    up_down=axes[2],
                    yaw=axes[3],
                ),
            )

        if latest_rc is not None:
            return latest_rc
        if (
            not self._watchdog_sent
            and time.monotonic() - self._last_packet_at > self._watchdog_seconds
        ):
            self._watchdog_sent = True
            return self._action_factory("quest3-watchdog-neutral", rc=self._rc_factory())
        return None

    # 실제 Tello 명령 결과를 XR Adapter에 회신한다.
    def complete_command(self, command: str, success: bool) -> None:
        pending = self._pending_command
        if pending is None or pending[2] != command:
            return
        self._pending_command = None
        self._socket.sendto(
            encode_ack(
                pending[0],
                query_performance_counter(),
                status=0 if success else 1,
            ),
            pending[1],
        )

    # IPC 소켓을 닫는다.
    async def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._socket.close()


class KeyboardAndRelayControlInput:
    def __init__(
        self,
        relay_input: PilotRelayControlInput,
        keyboard_input,
        shared_input: PilotSharedControlInput | None = None,
        desktop_input: DesktopControlInput | None = None,
        emergency_coordinator: EmergencyCommandCoordinator | None = None,
    ) -> None:
        self._relay_input = relay_input
        self._keyboard_input = keyboard_input
        self._shared_input = shared_input
        self._desktop_input = desktop_input
        self._emergency_coordinator = emergency_coordinator
        self._tracking_enabled = False
        if self._desktop_input is not None and self._emergency_coordinator is not None:
            handler_setter = getattr(self._desktop_input, "set_emergency_handler", None)
            if handler_setter is not None:
                handler_setter(self._emergency_coordinator.request_emergency)

    @property
    def instructions(self) -> str:
        instructions = f"{self._keyboard_input.instructions}\n{self._relay_input.instructions}"
        if self._shared_input is not None:
            instructions += f"\n{self._shared_input.instructions}"
        if self._desktop_input is not None:
            instructions += "\n[Desktop] GUI localhost 제어 입력을 사용합니다."
        return instructions

    # 추적 상태를 두 수동 입력에 동일하게 적용한다.
    def set_tracking_enabled(self, enabled: bool) -> None:
        self._tracking_enabled = enabled
        self._keyboard_input.set_tracking_enabled(enabled)

    # 비상·명령·XR RC·키보드 RC 순서로 입력 우선순위를 적용한다.
    async def poll_action(self):
        if (
            self._emergency_coordinator is not None
            and self._emergency_coordinator.is_latched
        ):
            return None
        inputs = [
            self._relay_input.poll_action(),
            self._keyboard_input.poll_action(),
        ]
        if self._shared_input is not None:
            inputs.append(self._shared_input.poll_action())
        if self._desktop_input is not None:
            inputs.append(self._desktop_input.poll_action())
        actions = await asyncio.gather(*inputs)
        relay_action, keyboard_action = actions[:2]
        offset = 2
        shared_action = actions[offset] if self._shared_input is not None else None
        if self._shared_input is not None:
            offset += 1
        desktop_action = (
            actions[offset] if self._desktop_input is not None else None
        )
        if self._is_emergency(desktop_action):
            return desktop_action
        if self._is_emergency(shared_action):
            return shared_action
        if self._is_emergency(relay_action):
            return relay_action
        if self._is_emergency(keyboard_action):
            if relay_action is not None and relay_action.command is not None:
                self._relay_input.complete_command(relay_action.command, False)
            if shared_action is not None and shared_action.command is not None:
                self._shared_input.complete_command(shared_action.command, False)
            return keyboard_action
        if desktop_action is not None and desktop_action.command is not None:
            return desktop_action
        if shared_action is not None and shared_action.command is not None:
            return shared_action
        if relay_action is not None and relay_action.command is not None:
            return relay_action
        if keyboard_action is not None and keyboard_action.command is not None:
            return keyboard_action
        if self._tracking_enabled:
            return None
        if self._desktop_input is not None and (
            self._desktop_input.is_active or desktop_action is not None
        ):
            return desktop_action
        if self._shared_input is not None and self._shared_input.is_active:
            return shared_action
        if self._has_active_rc(relay_action):
            return relay_action
        if self._has_active_rc(keyboard_action):
            return keyboard_action
        return relay_action or keyboard_action

    # 입력 소스를 함께 닫는다.
    async def close(self) -> None:
        self.set_ready(False)
        close_tasks = [self._relay_input.close(), self._keyboard_input.close()]
        if self._shared_input is not None:
            close_tasks.append(self._shared_input.close())
        if self._desktop_input is not None:
            close_tasks.append(self._desktop_input.close())
        await asyncio.gather(*close_tasks)

    # 실제 명령 결과를 명령을 생성한 IPC 입력에 회신한다.
    def complete_command(
        self,
        command: str,
        success: bool,
        *,
        dispatched: bool = False,
    ) -> None:
        if dispatched:
            if self._desktop_input is not None:
                self._desktop_input.complete_command(
                    command,
                    success,
                    dispatched=True,
                )
            return
        self._relay_input.complete_command(command, success)
        if self._shared_input is not None:
            self._shared_input.complete_command(command, success)
        if self._desktop_input is not None:
            self._desktop_input.complete_command(command, success)

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

    # Tello 준비 상태를 Desktop receiver와 비상 mailbox에 함께 전달한다.
    def set_ready(self, ready: bool) -> None:
        if self._emergency_coordinator is not None:
            self._emergency_coordinator.set_ready(ready)
        if self._desktop_input is not None:
            ready_setter = getattr(self._desktop_input, "set_ready", None)
            if ready_setter is not None:
                ready_setter(ready)

    @staticmethod
    def _is_emergency(action) -> bool:
        return action is not None and action.command == "emergency"

    @staticmethod
    def _has_active_rc(action) -> bool:
        if action is None or action.rc is None:
            return False
        return any(
            getattr(action.rc, name, 0) != 0
            for name in ("left_right", "forward_back", "up_down", "yaw")
        )


class PilotReadyLogHandler(logging.Handler):
    def __init__(self) -> None:
        super().__init__()
        self._ready_sent = False

    # 로컬 첫 프레임 또는 LiveKit 게시 로그를 준비 신호로 변환한다.
    def emit(self, record: logging.LogRecord) -> None:
        if self._ready_sent:
            return
        message = record.getMessage()
        if message.startswith((
            "Local video frame ready:",
            "LiveKit video track published:",
        )):
            self._ready_sent = True
            print("STRIKERPC_READY PILOT_PC", flush=True)


class CapturingVideoProxy:
    """LiveKit 상태와 무관하게 원본 영상을 읽고 최신 BGR 프레임을 공유한다."""

    _MAX_CONSECUTIVE_FAILURES = 300
    _FAILURE_RETRY_SECONDS = 0.005

    def __init__(
        self,
        capture,
        publisher: LatestFrameTcpPublisher,
        shared_publisher: PilotSharedFramePublisher,
    ) -> None:
        self._capture = capture
        self._publisher = publisher
        self._shared_publisher = shared_publisher
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._latest_frame = None
        self._latest_sequence = 0
        self._delivered_sequence = 0
        self._reader_error: BaseException | None = None
        self._released = False
        self._announced = False
        self._reader_thread = threading.Thread(
            target=self._read_frames,
            name="StrikerPCLocalVideoReader",
            daemon=True,
        )
        self._reader_thread.start()

    @property
    def has_failed(self) -> bool:
        with self._lock:
            return self._reader_error is not None

    @property
    def reader_error(self) -> BaseException | None:
        with self._lock:
            return self._reader_error

    # LiveKit 연결 전에 캡처를 시작해 로컬 SharedMemory와 Vision TCP를 유지한다.
    def _read_frames(self) -> None:
        consecutive_failures = 0
        try:
            while not self._stop_event.is_set():
                success, frame = self._capture.read()
                if not success or frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= self._MAX_CONSECUTIVE_FAILURES:
                        raise RuntimeError(
                            "Tello video stream stopped delivering local frames"
                        )
                    self._stop_event.wait(self._FAILURE_RETRY_SECONDS)
                    continue

                consecutive_failures = 0
                source_timestamp_ns = time.perf_counter_ns()
                self._publisher.update_frame(frame, source_timestamp_ns)
                self._shared_publisher.update_frame(frame, source_timestamp_ns)
                with self._lock:
                    self._latest_frame = frame
                    self._latest_sequence += 1
                if not self._announced:
                    self._announced = True
                    print("STRIKERPC_STAGE VISION_FRAME_READY", flush=True)
                    logging.getLogger(__name__).info(
                        "Local video frame ready: %sx%s",
                        frame.shape[1],
                        frame.shape[0],
                    )
        except Exception as error:
            if not self._stop_event.is_set():
                with self._lock:
                    self._reader_error = error

    def read(self):
        with self._lock:
            if self._latest_sequence <= self._delivered_sequence:
                return False, None
            self._delivered_sequence = self._latest_sequence
            return True, self._latest_frame

    # 백그라운드 읽기를 먼저 중단하고 원본 VideoCapture를 한 번만 해제한다.
    def release(self):
        with self._lock:
            if self._released:
                return None
            self._released = True
        self._stop_event.set()
        result = self._capture.release()
        if threading.current_thread() is not self._reader_thread:
            self._reader_thread.join(timeout=2.0)
        return result

    def __getattr__(self, name):
        return getattr(self._capture, name)


# LiveKit 없이 로컬 프레임 공급과 Tello 영상 세션만 유지한다.
class LocalVideoSession:
    def __init__(self, *args, **kwargs) -> None:
        del args, kwargs

    async def run(self, capture, stop_event) -> None:
        while not stop_event.is_set():
            if getattr(capture, "has_failed", False):
                error = getattr(capture, "reader_error", None)
                raise RuntimeError("Local Tello video capture failed") from error
            await asyncio.sleep(0.05)


# LiveKit 재접속은 같은 로컬 캡처 위에서 수행해 SharedMemory를 끊지 않는다.
def create_resilient_video_publisher(
    base_class,
    *,
    initial_retry_seconds: float = 1.0,
    maximum_retry_seconds: float = 10.0,
):
    class ResilientVideoPublisher(base_class):
        async def run(self, capture, stop_event) -> None:
            retry_seconds = max(0.0, initial_retry_seconds)
            maximum_seconds = max(retry_seconds, maximum_retry_seconds)

            while not stop_event.is_set():
                self._raise_if_local_capture_failed(capture)
                sequence_before = getattr(self, "_sequence", 0)
                try:
                    await super().run(capture, stop_event)
                except asyncio.CancelledError:
                    raise
                except Exception as error:
                    self._raise_if_local_capture_failed(capture)
                    logging.getLogger(__name__).warning(
                        "LiveKit video unavailable; local video remains active: %s",
                        error,
                    )
                else:
                    if not stop_event.is_set():
                        logging.getLogger(__name__).warning(
                            "LiveKit video session ended; reconnecting while local "
                            "video remains active"
                        )

                if stop_event.is_set():
                    return
                self._raise_if_local_capture_failed(capture)
                if getattr(self, "_sequence", 0) > sequence_before:
                    retry_seconds = max(0.0, initial_retry_seconds)
                if retry_seconds > 0.0:
                    await asyncio.sleep(retry_seconds)
                retry_seconds = min(
                    maximum_seconds,
                    max(initial_retry_seconds, retry_seconds * 2.0),
                )

        @staticmethod
        def _raise_if_local_capture_failed(capture) -> None:
            if not getattr(capture, "has_failed", False):
                return
            error = getattr(capture, "reader_error", None)
            raise RuntimeError("Local Tello video capture failed") from error

    ResilientVideoPublisher.__name__ = f"Resilient{base_class.__name__}"
    return ResilientVideoPublisher


# 실제 명령 결과 ACK와 종료 중립 RC를 보장하는 클라이언트를 만든다.
def create_managed_tello_client(
    base_class,
    control_input,
    rc_factory,
    emergency_coordinator: EmergencyCommandCoordinator | None = None,
    motion_publisher: ControlMotionPublisher | None = None,
):
    if emergency_coordinator is not None:
        complete_emergency = getattr(
            control_input,
            "complete_desktop_emergency",
            None,
        )
        if complete_emergency is None:
            complete_emergency = lambda success, *, dispatched: (
                control_input.complete_command(
                    "emergency",
                    success,
                    dispatched=dispatched,
                )
            )
        emergency_coordinator.set_completion_handler(
            lambda success, dispatched: complete_emergency(
                success,
                dispatched=dispatched,
            ),
        )

    class ManagedTelloCommandClient(base_class):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._strikerpc_closed = False
            self._strikerpc_worker_stop = threading.Event()
            self._strikerpc_emergency_worker: threading.Thread | None = None
            try:
                self.enter_sdk_mode()
                self._start_emergency_worker()
                self._set_control_ready(True)
                print("STRIKERPC_STAGE TELLO_READY", flush=True)
            except Exception:
                self.close()
                raise

        def send_command(self, command: str, **kwargs):
            try:
                if self._supports_priority_channel():
                    response = self._send_command_with_priority(command, **kwargs)
                else:
                    response = super().send_command(command, **kwargs)
            except EmergencyCommandPreempted as error:
                emergency_coordinator.complete_request(
                    error.request_id,
                    error.dispatched,
                )
                control_input.complete_command(command, False)
                raise
            except Exception:
                control_input.complete_command(command, False)
                raise
            success = response is None or str(response).strip().lower() == "ok"
            control_input.complete_command(command, success)
            return response

        # Tello 전송에 성공한 RC만 짧은 비추적 위치 예측 입력으로 공유한다.
        def send_rc_control(self, control):
            result = super().send_rc_control(control)
            if motion_publisher is not None:
                motion_publisher.publish(control)
            return result

        def close(self) -> None:
            if self._strikerpc_closed:
                return
            self._strikerpc_closed = True
            self._set_control_ready(False)
            self._strikerpc_worker_stop.set()
            if emergency_coordinator is not None:
                emergency_coordinator.wake()
            worker = self._strikerpc_emergency_worker
            if worker is not None and worker is not threading.current_thread():
                worker.join(timeout=1.0)
            try:
                if not getattr(self, "_closed", False):
                    self.send_rc_control(rc_factory())
            except Exception:
                pass
            super().close()

        def _supports_priority_channel(self) -> bool:
            return (
                emergency_coordinator is not None
                and hasattr(self, "socket")
                and hasattr(self, "command_address")
                and hasattr(self, "_lock")
                and hasattr(self, "config")
            )

        # 기존 signature와 lock 획득 정책을 보존하면서 응답을 짧게 나눠 기다린다.
        def _send_command_with_priority(
            self,
            command: str,
            *,
            wait_response: bool = True,
            timeout: float | None = None,
            acquire_timeout: float | None = None,
        ):
            if acquire_timeout is None:
                acquired = self._lock.acquire()
            elif acquire_timeout <= 0:
                acquired = self._lock.acquire(blocking=False)
            else:
                acquired = self._lock.acquire(timeout=acquire_timeout)
            if not acquired:
                module = sys.modules.get(base_class.__module__)
                error_type = getattr(module, "CommandChannelBusyError", RuntimeError)
                raise error_type(f"Tello command channel busy: {command}")

            try:
                request_id = emergency_coordinator.claim_request()
                if request_id is not None:
                    raise EmergencyCommandPreempted(
                        request_id,
                        self._dispatch_emergency_locked(request_id),
                    )
                if emergency_coordinator.is_latched and not (
                    not wait_response and _is_neutral_rc_command(command)
                ):
                    raise RuntimeError("비상 정지 latch 이후 Tello 명령은 거부됩니다.")
                if getattr(self, "_closed", False):
                    raise OSError("Tello command socket is closed")

                log_method = logging.debug if command.startswith("rc ") else logging.info
                if wait_response:
                    self._drain_socket()
                log_method("Tello command sent: %s", command)
                self.socket.sendto(command.encode("utf-8"), self.command_address)
                if not wait_response:
                    return None

                original_timeout = self.socket.gettimeout()
                response_timeout = (
                    timeout
                    if timeout is not None
                    else float(self.config.response_timeout)
                )
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
                        self.socket.settimeout(
                            min(EMERGENCY_RECV_SLICE_SECONDS, remaining),
                        )
                        try:
                            response, _ = self.socket.recvfrom(1024)
                        except socket.timeout:
                            continue

                        request_id = emergency_coordinator.claim_request()
                        if request_id is not None:
                            raise EmergencyCommandPreempted(
                                request_id,
                                self._dispatch_emergency_locked(request_id),
                            )
                        decoded = response.decode(
                            "utf-8",
                            errors="replace",
                        ).strip()
                        logging.info("Tello response: %s -> %s", command, decoded)
                        return decoded
                finally:
                    self.socket.settimeout(original_timeout)
            finally:
                self._lock.release()

        # 현재 command lock 소유자만 중립과 emergency를 같은 UDP 소켓에 쓴다.
        def _dispatch_emergency_locked(self, request_id: int) -> bool:
            if not emergency_coordinator.begin_dispatch(request_id):
                return False
            if getattr(self, "_closed", False):
                return False
            try:
                self.socket.sendto(b"rc 0 0 0 0", self.command_address)
            except OSError as error:
                logging.warning("비상 정지 전 중립 RC 전송 실패: %s", error)
            if motion_publisher is not None:
                motion_publisher.publish(rc_factory())
            try:
                payload = b"emergency"
                sent = self.socket.sendto(payload, self.command_address)
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
                name="StrikerPCTelloEmergency",
                daemon=True,
            )
            self._strikerpc_emergency_worker.start()

        def _set_control_ready(self, ready: bool) -> None:
            if emergency_coordinator is not None:
                emergency_coordinator.set_ready(ready)
            setter = getattr(control_input, "set_ready", None)
            if setter is not None:
                setter(ready)

    return ManagedTelloCommandClient


# manifest에서 팀 Pilot-PC 경로와 고정 커밋을 읽는다.
def load_team_source(manifest_path: Path) -> tuple[Path, dict]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    item = manifest["repositories"]["pilot"]
    return (manifest_path.parent / item["path"]).resolve(), item


# 팀 원본 커밋과 tracked 상태를 확인한다.
def verify_team_source(path: Path, expected: dict) -> None:
    command = ["git", "-c", f"safe.directory={path.as_posix()}", "-C", str(path)]
    commit = subprocess.run(
        [*command, "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=False,
    )
    if commit.returncode != 0 or commit.stdout.strip() != expected["commit"]:
        raise RuntimeError(f"Pilot-PC 고정 커밋이 아닙니다: {commit.stdout.strip()}")
    tracked_diff = subprocess.run([*command, "diff", "--quiet", "HEAD"], check=False)
    if tracked_diff.returncode != 0:
        raise RuntimeError("Pilot-PC tracked 파일이 수정되어 있습니다.")


# 팀 Pilot-PC 원본 앱에 IPC 입력만 주입해 실행한다.
def run_team_pilot(args: argparse.Namespace) -> int:
    manifest_path = Path(args.manifest).resolve()
    team_root, expected = load_team_source(manifest_path)
    verify_team_source(team_root, expected)
    sys.path.insert(0, str(team_root))
    app = importlib.import_module("tello_app.app")
    control = importlib.import_module("tello_app.control")
    config = importlib.import_module("tello_app.config")
    tello = importlib.import_module("tello_app.tello")

    imported_path = Path(app.__file__).resolve()
    if team_root not in imported_path.parents:
        raise RuntimeError(f"Pilot-PC 원본이 아닌 모듈이 로드되었습니다: {imported_path}")

    relay_input = PilotRelayControlInput(
        control.ControlAction,
        tello.RcControl,
        pilot_port=args.pilot_port,
        watchdog_seconds=args.watchdog_seconds,
    )
    shared_input = PilotSharedControlInput(
        control.ControlAction,
        tello.RcControl,
        name=args.shared_control_name,
        watchdog_seconds=args.watchdog_seconds,
    )
    desktop_input = DesktopControlInput(
        control.ControlAction,
        tello.RcControl,
        port=args.desktop_control_port,
        watchdog_seconds=args.watchdog_seconds,
    )
    emergency_coordinator = EmergencyCommandCoordinator()
    motion_publisher = ControlMotionPublisher(args.measurement_port)
    combined_input = KeyboardAndRelayControlInput(
        relay_input,
        control.KeyboardControlInput(),
        shared_input,
        desktop_input,
        emergency_coordinator,
    )
    original_tello_config = config.TelloConfig
    app.TelloConfig = lambda: original_tello_config(
        host=args.tello_ip,
        command_port=args.command_port,
        local_port=args.local_command_port,
        video_url=f"udp://0.0.0.0:{args.video_port}",
        response_timeout=args.response_timeout,
    )
    app.create_control_source = lambda name: (
        combined_input
        if name.lower() in {"keyboard", "quest3"}
        else control.create_control_source(name)
    )
    app.TelloCommandClient = create_managed_tello_client(
        tello.TelloCommandClient,
        combined_input,
        tello.RcControl,
        emergency_coordinator,
        motion_publisher,
    )
    frame_publisher = LatestFrameTcpPublisher(args.vision_tcp_port)
    shared_frame_publisher = PilotSharedFramePublisher()
    original_open_tello_video = app.open_tello_video
    original_tracking_pipeline = app.TrackingPipeline

    def open_video_proxy(video_url: str):
        return CapturingVideoProxy(
            original_open_tello_video(video_url),
            frame_publisher,
            shared_frame_publisher,
        )

    app.open_tello_video = open_video_proxy

    class TrackingPipelineProxy(original_tracking_pipeline):
        def process_frame(self, frame):
            processed = super().process_frame(frame)
            target = getattr(self, "latest_target", None)
            boxes = (
                [tuple(int(value) for value in target.bbox)]
                if target is not None and getattr(target, "disappeared", 1) == 0
                else []
            )
            frame_publisher.update_dynamic_boxes(boxes)
            return processed

    app.TrackingPipeline = TrackingPipelineProxy

    pilot_args = list(args.pilot_args)
    if pilot_args[:1] == ["--"]:
        pilot_args = pilot_args[1:]
    if "--control-source" not in pilot_args:
        pilot_args.extend(("--control-source", "keyboard"))

    ready_handler = PilotReadyLogHandler()
    logging.getLogger().addHandler(ready_handler)
    original_argv = sys.argv
    original_cwd = Path.cwd()
    try:
        sys.argv = [str(imported_path), *pilot_args]
        os.chdir(team_root)
        app.main()
    finally:
        frame_publisher.close()
        shared_frame_publisher.close()
        motion_publisher.close()
        logging.getLogger().removeHandler(ready_handler)
        sys.argv = original_argv
        os.chdir(original_cwd)
    return 0


# 실행 옵션을 읽는다.
def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="Pilot-PC 원본 통합 실행 호스트")
    parser.add_argument("--manifest", default=str(root / "team-sources.json"))
    parser.add_argument("--tello-ip", default="192.168.10.1")
    parser.add_argument("--command-port", type=int, default=8889)
    parser.add_argument("--local-command-port", type=int, default=9000)
    parser.add_argument("--video-port", type=int, default=11111)
    parser.add_argument("--pilot-port", type=int, default=49210)
    parser.add_argument("--desktop-control-port", type=int, default=49211)
    parser.add_argument("--watchdog-seconds", type=float, default=0.25)
    parser.add_argument("--response-timeout", type=float, default=7.0)
    parser.add_argument("--vision-tcp-port", type=int, default=49000)
    parser.add_argument("--measurement-port", type=int, default=49003)
    parser.add_argument("--shared-control-name", default="TelloPilotControl_v1")
    parser.add_argument("pilot_args", nargs=argparse.REMAINDER)
    return parser.parse_args()


def main() -> int:
    try:
        return run_team_pilot(parse_args())
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
