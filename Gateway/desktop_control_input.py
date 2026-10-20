from __future__ import annotations

import asyncio
import ipaddress
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

from control_protocol import (
    ACK_STATUS_DISPATCHED,
    ACK_STATUS_FAILED,
    ACK_STATUS_OK,
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
    COMMAND_LAND,
    COMMAND_NEUTRAL,
    COMMAND_RC,
    COMMAND_TAKEOFF,
    DESKTOP_CONTROL_FLAG,
    ControlPacket,
    encode_ack,
)
from xr_control_adapter import query_performance_counter


DEFAULT_WATCHDOG_SECONDS = 0.25
RECEIVE_TIMEOUT_SECONDS = 0.01
MOTION_PRIORITY = 1
CONFIRMED_PRIORITY = 2
EMERGENCY_PRIORITY = 3
SUPPORTED_COMMANDS = {
    COMMAND_RC,
    COMMAND_NEUTRAL,
    COMMAND_TAKEOFF,
    COMMAND_LAND,
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
}
COMMAND_NAMES = {
    COMMAND_TAKEOFF: "takeoff",
    COMMAND_LAND: "land",
    COMMAND_EMERGENCY: "emergency",
}


@dataclass(frozen=True)
class _PendingCommand:
    sequence: int
    sender: tuple[str, int]
    command: str


@dataclass(frozen=True)
class _ProcessedAction:
    priority: int
    action: object


class DesktopControlInput:
    """localhost 데스크톱 제어 패킷을 Pilot 입력 객체로 변환한다."""

    # localhost UDP 입력 소켓과 영구 수신 thread를 준비한다.
    def __init__(
        self,
        action_factory: Callable,
        rc_factory: Callable,
        *,
        port: int = 0,
        watchdog_seconds: float = DEFAULT_WATCHDOG_SECONDS,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if watchdog_seconds <= 0:
            raise ValueError("데스크톱 제어 watchdog은 양수여야 합니다.")

        self._action_factory = action_factory
        self._rc_factory = rc_factory
        self._watchdog_seconds = watchdog_seconds
        self._clock = clock
        self._lock = threading.Lock()
        self._mailbox: deque[_ProcessedAction] = deque()
        self._sender: tuple[str, int] | None = None
        self._last_sequence = -1
        self._last_packet_at = clock()
        self._watchdog_sent = True
        self._pending_command: _PendingCommand | None = None
        self._emergency_handler: Callable[[], None] | None = None
        self._emergency_latched = False
        self._ready = False
        self._active = False
        self._closed = False
        self._receiver_stop = threading.Event()

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind(("127.0.0.1", port))
            self._socket.settimeout(RECEIVE_TIMEOUT_SECONDS)
            self._address = self._socket.getsockname()
        except Exception:
            self._socket.close()
            raise

        self._receiver_thread = threading.Thread(
            target=self._receiver_loop,
            name="desktop-control-receiver",
            daemon=True,
        )
        self._receiver_thread.start()

    # 현재 localhost UDP 바인딩 주소를 반환한다.
    @property
    def address(self) -> tuple[str, int]:
        return self._address

    # 유효한 데스크톱 조종 입력이 활성 상태인지 반환한다.
    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._active

    # 런타임 준비 상태를 바꾸고 해제 시 현재 세션을 안전하게 비운다.
    def set_ready(self, ready: bool) -> None:
        if not isinstance(ready, bool):
            raise TypeError("데스크톱 제어 준비 상태는 bool이어야 합니다.")

        pending: _PendingCommand | None = None
        with self._lock:
            if self._closed or self._ready == ready:
                return
            self._ready = ready
            if not ready:
                pending = self._pending_command
                self._pending_command = None
                self._sender = None
                self._last_sequence = -1
                self._active = False
                self._watchdog_sent = True
                self._mailbox.clear()

        if pending is not None:
            self._send_ack(pending.sequence, pending.sender, status=ACK_STATUS_FAILED)

    # 수신 thread가 호출할 비상 mailbox publisher를 등록한다.
    def set_emergency_handler(self, handler: Callable[[], None]) -> None:
        if not callable(handler):
            raise TypeError("데스크톱 비상 handler는 호출 가능해야 합니다.")
        with self._lock:
            if self._closed:
                raise RuntimeError("닫힌 데스크톱 제어 입력에는 handler를 등록할 수 없습니다.")
            self._emergency_handler = handler

    # 비동기 입력 루프에서 내부 mailbox를 한 번 확인한다.
    async def poll_action(self):
        await asyncio.sleep(0.001)
        return self.poll_action_nowait()

    # 내부 mailbox를 비우고 비상 우선 또는 최신 조종 입력을 반환한다.
    def poll_action_nowait(self):
        with self._lock:
            queued = tuple(self._mailbox)
            self._mailbox.clear()

        selected: _ProcessedAction | None = None
        for processed in queued:
            if (
                selected is None
                or processed.priority > selected.priority
                or (
                    processed.priority == MOTION_PRIORITY
                    and selected.priority == MOTION_PRIORITY
                )
            ):
                selected = processed
        return selected.action if selected is not None else None

    # 실제 드론 명령 결과를 원래 데스크톱 요청에 ACK로 전달한다.
    def complete_command(
        self,
        command: str,
        success: bool,
        *,
        dispatched: bool = False,
    ) -> None:
        with self._lock:
            pending = self._pending_command
            if pending is None or pending.command != command:
                return
            self._pending_command = None
            self._active = False
            self._watchdog_sent = True

        status = ACK_STATUS_OK if success else ACK_STATUS_FAILED
        if command == "emergency" and success and dispatched:
            status = ACK_STATUS_DISPATCHED
        self._send_ack(pending.sequence, pending.sender, status=status)

    # 비동기 종료 경로에서 수신 thread와 UDP 소켓을 정리한다.
    async def close(self) -> None:
        self.close_nowait()

    # 수신 thread를 먼저 멈춘 뒤 대기 명령과 UDP 소켓을 정리한다.
    def close_nowait(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._ready = False
            self._active = False
            self._watchdog_sent = True
            pending = self._pending_command
            self._pending_command = None
            self._mailbox.clear()

        self._receiver_stop.set()
        if threading.current_thread() is not self._receiver_thread:
            self._receiver_thread.join()
        if pending is not None:
            self._send_ack(pending.sequence, pending.sender, status=ACK_STATUS_FAILED)
        self._socket.close()

    # 객체 수명 동안 UDP 소켓을 단독으로 읽고 watchdog을 갱신한다.
    def _receiver_loop(self) -> None:
        while not self._receiver_stop.is_set():
            try:
                payload, sender = self._socket.recvfrom(256)
            except socket.timeout:
                self._enqueue_watchdog_if_due()
                continue
            except OSError:
                if self._receiver_stop.is_set():
                    return
                raise

            self._process_packet(payload, sender)
            self._enqueue_watchdog_if_due()

    # 패킷 출처·세션·순서를 검증해 ACK, mailbox 또는 비상 handler로 전달한다.
    def _process_packet(self, payload: bytes, sender: tuple[str, int]) -> None:
        if not self._is_loopback(sender[0]):
            return

        packet = ControlPacket.decode(payload)
        if (
            packet is None
            or packet.flags != DESKTOP_CONTROL_FLAG
            or packet.command not in SUPPORTED_COMMANDS
        ):
            return
        if packet.command == COMMAND_HELLO:
            self._handle_hello(packet, sender)
            return

        rejected = False
        preempted: _PendingCommand | None = None
        emergency_handler: Callable[[], None] | None = None
        emergency_pending: _PendingCommand | None = None
        with self._lock:
            if (
                self._closed
                or not self._ready
                or sender != self._sender
                or packet.sequence <= self._last_sequence
            ):
                return

            self._last_sequence = packet.sequence
            self._last_packet_at = self._clock()
            if self._emergency_latched:
                self._active = False
                self._watchdog_sent = True
                rejected = True
            elif packet.command == COMMAND_EMERGENCY:
                preempted = self._pending_command
                emergency_pending = _PendingCommand(
                    packet.sequence,
                    sender,
                    "emergency",
                )
                self._pending_command = emergency_pending
                self._emergency_latched = True
                self._active = False
                self._watchdog_sent = True
                self._mailbox.clear()
                emergency_handler = self._emergency_handler
                if emergency_handler is None:
                    self._mailbox.append(
                        _ProcessedAction(
                            EMERGENCY_PRIORITY,
                            self._action_factory(
                                "emergency",
                                command="emergency",
                                stop_session=True,
                            ),
                        )
                    )
            elif self._pending_command is not None:
                if packet.command in COMMAND_NAMES:
                    rejected = True
                else:
                    self._active = False
                    self._watchdog_sent = True
            elif packet.command in {COMMAND_TAKEOFF, COMMAND_LAND}:
                command = COMMAND_NAMES[packet.command]
                self._pending_command = _PendingCommand(
                    packet.sequence,
                    sender,
                    command,
                )
                self._active = False
                self._watchdog_sent = True
                self._mailbox.append(
                    _ProcessedAction(
                        CONFIRMED_PRIORITY,
                        self._action_factory(command, command=command),
                    )
                )
            else:
                self._enqueue_motion_locked(packet)

        if rejected:
            self._send_ack(packet.sequence, sender, status=ACK_STATUS_FAILED)
            return
        handler_failed = False
        if emergency_handler is not None and emergency_pending is not None:
            try:
                emergency_handler()
            except Exception:
                with self._lock:
                    if self._pending_command == emergency_pending:
                        self._pending_command = None
                handler_failed = True
        if preempted is not None:
            self._send_ack(
                preempted.sequence,
                preempted.sender,
                status=ACK_STATUS_FAILED,
            )
        if handler_failed and emergency_pending is not None:
            self._send_ack(
                emergency_pending.sequence,
                emergency_pending.sender,
                status=ACK_STATUS_FAILED,
            )

    # HELLO 준비 gate와 단일 sender 조건을 확인해 세션을 시작한다.
    def _handle_hello(
        self,
        packet: ControlPacket,
        sender: tuple[str, int],
    ) -> None:
        rejected = False
        with self._lock:
            if self._closed:
                return
            if (
                not self._ready
                or self._emergency_latched
                or self._pending_command is not None
            ):
                rejected = True
            elif (
                self._sender is not None
                and sender != self._sender
                and (
                    self._active
                    or self._clock() - self._last_packet_at <= self._watchdog_seconds
                )
            ):
                return
            elif not self._send_ack(
                packet.sequence,
                sender,
                status=ACK_STATUS_OK,
            ):
                return
            else:
                self._sender = sender
                self._last_sequence = packet.sequence
                self._last_packet_at = self._clock()
                self._active = False
                self._watchdog_sent = True
                self._mailbox.clear()

        if rejected:
            self._send_ack(packet.sequence, sender, status=ACK_STATUS_FAILED)

    # RC와 neutral을 같은 우선순위의 최신 mailbox 값으로 접는다.
    def _enqueue_motion_locked(self, packet: ControlPacket) -> None:
        bounded_axes = (
            (0, 0, 0, 0)
            if packet.command == COMMAND_NEUTRAL
            else tuple(max(-100, min(100, value)) for value in packet.axes)
        )
        self._active = packet.command != COMMAND_NEUTRAL
        self._watchdog_sent = packet.command == COMMAND_NEUTRAL
        self._mailbox = deque(
            item for item in self._mailbox if item.priority != MOTION_PRIORITY
        )
        label = "desktop-neutral" if packet.command == COMMAND_NEUTRAL else "desktop-rc"
        self._mailbox.append(
            _ProcessedAction(
                MOTION_PRIORITY,
                self._action_factory(
                    label,
                    rc=self._rc_factory(
                        left_right=bounded_axes[0],
                        forward_back=bounded_axes[1],
                        up_down=bounded_axes[2],
                        yaw=bounded_axes[3],
                    ),
                ),
            )
        )

    # 입력 갱신이 끊기면 watchdog 중립 입력을 mailbox에 한 번만 넣는다.
    def _enqueue_watchdog_if_due(self) -> None:
        with self._lock:
            if (
                self._closed
                or not self._ready
                or self._emergency_latched
                or self._pending_command is not None
                or not self._active
                or self._watchdog_sent
                or self._clock() - self._last_packet_at <= self._watchdog_seconds
            ):
                return
            self._active = False
            self._watchdog_sent = True
            self._mailbox = deque(
                item for item in self._mailbox if item.priority != MOTION_PRIORITY
            )
            self._mailbox.append(
                _ProcessedAction(
                    MOTION_PRIORITY,
                    self._action_factory(
                        "desktop-watchdog-neutral",
                        rc=self._rc_factory(),
                    ),
                )
            )

    # 기존 고정 ACK 패킷으로 요청 상태를 응답한다.
    def _send_ack(
        self,
        sequence: int,
        sender: tuple[str, int],
        *,
        status: int,
    ) -> bool:
        try:
            payload = encode_ack(
                sequence,
                query_performance_counter(),
                status=status,
            )
            return self._socket.sendto(payload, sender) == len(payload)
        except OSError:
            return False

    # IPv4 sender가 로컬 loopback 주소인지 확인한다.
    @staticmethod
    def _is_loopback(host: str) -> bool:
        try:
            return ipaddress.ip_address(host).is_loopback
        except ValueError:
            return False
