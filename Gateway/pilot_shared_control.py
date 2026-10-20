from __future__ import annotations

import asyncio
import mmap
import struct
import threading
import time
from typing import Callable

from control_protocol import (
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
    COMMAND_LAND,
    COMMAND_NEUTRAL,
    COMMAND_RC,
    COMMAND_TAKEOFF,
)
from xr_control_adapter import query_performance_counter, query_performance_frequency


CONTROL_MEMORY_NAME = "TelloPilotControl_v1"
CONTROL_MEMORY_MAGIC = 0x31435054
CONTROL_MEMORY_VERSION = 1
CONTROL_MEMORY_SIZE = 96
CONTROL_ACTIVE_FLAG = 1
READER_READY = 1
ACK_NONE = 0
ACK_SUCCESS = 1
ACK_FAILURE = 2

WRITER_SLOT = struct.Struct("<I H H Q q q 4h B 7x")
READER_SLOT = struct.Struct("<Q q q i I q 8x")
WRITER_SLOT_SIZE = WRITER_SLOT.size

SUPPORTED_COMMANDS = {
    COMMAND_RC,
    COMMAND_NEUTRAL,
    COMMAND_TAKEOFF,
    COMMAND_LAND,
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
}


class PilotSharedControlInput:
    """Unity의 최신 조종 상태를 Windows 명명 메모리에서 읽는다."""

    def __init__(
        self,
        action_factory: Callable,
        rc_factory: Callable,
        *,
        name: str = CONTROL_MEMORY_NAME,
        watchdog_seconds: float = 0.2,
        input_freshness_seconds: float | None = None,
        counter: Callable[[], int] = query_performance_counter,
        counter_frequency: int | None = None,
    ) -> None:
        if not name:
            raise ValueError("공유 제어 메모리 이름은 비어 있을 수 없습니다.")
        if watchdog_seconds <= 0:
            raise ValueError("공유 제어 watchdog은 양수여야 합니다.")

        self._action_factory = action_factory
        self._rc_factory = rc_factory
        self._watchdog_seconds = watchdog_seconds
        self._input_freshness_seconds = input_freshness_seconds
        self._counter = counter
        self._counter_frequency = counter_frequency or query_performance_frequency()
        self._mapping = mmap.mmap(
            -1,
            CONTROL_MEMORY_SIZE,
            tagname=name,
            access=mmap.ACCESS_WRITE,
        )
        self._lock = threading.Lock()
        self._session_id = 0
        self._last_sequence = 0
        self._last_frame_at = time.monotonic()
        self._active = False
        self._control_active = False
        self._watchdog_sent = True
        self._pending_command: tuple[int, int, str] | None = None
        self._ack_session_id = 0
        self._ack_sequence = 0
        self._ack_status = ACK_NONE
        self._closed = False
        self._write_reader_state()

    @property
    def instructions(self) -> str:
        return "[Quest3] 공유메모리 XR 입력을 Pilot-PC 제어 루프로 전달합니다."

    @property
    def is_active(self) -> bool:
        return self._active

    @property
    def control_active(self) -> bool:
        return self._control_active

    # 새 프레임 하나만 적용하고 갱신이 끊기면 중립 입력을 한 번 반환한다.
    async def poll_action(self):
        await asyncio.sleep(0.001)
        return self.poll_action_nowait()

    # 동기 입력 루프에서도 같은 공유 메모리 계약을 사용할 수 있게 즉시 한 번 읽는다.
    def poll_action_nowait(self):
        self._write_reader_state()
        frame = self._read_stable_frame()
        if frame is not None:
            action = self._process_frame(frame)
            if action is not None:
                return action

        if (
            self._active
            and not self._watchdog_sent
            and time.monotonic() - self._last_frame_at > self._watchdog_seconds
        ):
            self._active = False
            self._control_active = False
            self._watchdog_sent = True
            return self._action_factory(
                "quest3-shared-watchdog-neutral",
                rc=self._rc_factory(),
            )
        return None

    # 실제 Tello 명령 결과를 공유 ACK 슬롯에 기록한다.
    def complete_command(self, command: str, success: bool) -> None:
        pending = self._pending_command
        if pending is None or pending[2] != command:
            return
        self._pending_command = None
        self._write_ack(
            pending[0],
            pending[1],
            ACK_SUCCESS if success else ACK_FAILURE,
        )
        if command == "emergency" and success:
            self._active = False
            self._control_active = False
            self._session_id = 0
            self._last_sequence = 0
            self._watchdog_sent = True

    # Reader 종료 상태를 남기고 명명 메모리 핸들을 닫는다.
    async def close(self) -> None:
        self.close_nowait()

    # 동기 런타임 종료에서도 Reader 상태와 핸들을 즉시 정리한다.
    def close_nowait(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._active = False
        self._control_active = False
        self._write_reader_state(reader_state=0)
        self._mapping.close()

    # Writer 슬롯을 sequence 전후 확인으로 원자적으로 읽는다.
    def _read_stable_frame(self):
        with self._lock:
            self._mapping.seek(0)
            first = self._mapping.read(WRITER_SLOT_SIZE)
            values = WRITER_SLOT.unpack(first)
            sequence = values[4]
            if sequence <= 0:
                return None
            self._mapping.seek(16)
            if struct.unpack("<q", self._mapping.read(8))[0] != sequence:
                return None
            return values

    # 새 세션·순서·명령을 검증하고 원본 ControlAction으로 변환한다.
    def _process_frame(self, values):
        magic, version, flags, session_id, sequence, input_ticks, *payload = values
        axes = tuple(payload[:4])
        command = payload[4]
        active = (flags & CONTROL_ACTIVE_FLAG) != 0
        if (
            magic != CONTROL_MEMORY_MAGIC
            or version != CONTROL_MEMORY_VERSION
            or command not in SUPPORTED_COMMANDS
            or session_id == 0
            or not self._input_is_fresh(input_ticks)
        ):
            return None

        if command == COMMAND_HELLO:
            if not active:
                self._write_ack(session_id, sequence, ACK_FAILURE)
                return None
            if session_id == self._session_id and sequence <= self._last_sequence:
                return None
            self._session_id = session_id
            self._last_sequence = sequence
            self._last_frame_at = time.monotonic()
            self._active = True
            self._control_active = False
            self._watchdog_sent = False
            self._write_ack(session_id, sequence, ACK_SUCCESS)
            return None

        if session_id != self._session_id or sequence <= self._last_sequence:
            return None

        self._last_sequence = sequence
        self._last_frame_at = time.monotonic()
        self._watchdog_sent = False
        if not active:
            self._active = False
            self._control_active = False
            self._write_ack(session_id, sequence, ACK_SUCCESS)
            return self._action_factory(
                "quest3-shared-release-neutral",
                rc=self._rc_factory(),
            )

        self._active = True
        if command in {COMMAND_TAKEOFF, COMMAND_LAND, COMMAND_EMERGENCY}:
            self._control_active = False
            if self._pending_command is not None:
                self._write_ack(session_id, sequence, ACK_FAILURE)
                return None
            command_name = {
                COMMAND_TAKEOFF: "takeoff",
                COMMAND_LAND: "land",
                COMMAND_EMERGENCY: "emergency",
            }[command]
            self._pending_command = (session_id, sequence, command_name)
            return self._action_factory(
                command_name,
                command=command_name,
                stop_session=command == COMMAND_EMERGENCY,
            )

        bounded_axes = (
            (0, 0, 0, 0)
            if command == COMMAND_NEUTRAL
            else tuple(max(-100, min(100, value)) for value in axes)
        )
        self._control_active = command == COMMAND_RC
        return self._action_factory(
            "quest3-shared-rc",
            rc=self._rc_factory(
                left_right=bounded_axes[0],
                forward_back=bounded_axes[1],
                up_down=bounded_axes[2],
                yaw=bounded_axes[3],
            ),
        )

    # 동일 PC의 Stopwatch tick이 지정 freshness 안에 있을 때만 적용한다.
    def _input_is_fresh(self, input_ticks: int) -> bool:
        maximum_age = self._input_freshness_seconds
        if maximum_age is None:
            return True
        age_ticks = self._counter() - input_ticks
        return (
            -0.05 * self._counter_frequency
            <= age_ticks
            <= maximum_age * self._counter_frequency
        )

    # ACK와 heartbeat를 Reader 소유 슬롯에 기록한다.
    def _write_ack(self, session_id: int, sequence: int, status: int) -> None:
        self._ack_session_id = session_id
        self._ack_sequence = sequence
        self._ack_status = status
        self._write_reader_state()

    # 음수 ACK sequence로 기록 중임을 표시한 뒤 양수 sequence로 확정한다.
    def _write_reader_state(self, reader_state: int = READER_READY) -> None:
        if self._closed and reader_state != 0:
            return
        heartbeat = query_performance_counter()
        sequence = self._ack_sequence
        writing_sequence = -sequence if sequence > 0 else 0
        writing = READER_SLOT.pack(
            self._ack_session_id,
            writing_sequence,
            heartbeat,
            self._ack_status,
            reader_state,
            heartbeat,
        )
        complete = READER_SLOT.pack(
            self._ack_session_id,
            sequence,
            heartbeat,
            self._ack_status,
            reader_state,
            heartbeat,
        )
        with self._lock:
            self._mapping.seek(WRITER_SLOT_SIZE)
            self._mapping.write(writing)
            self._mapping.seek(WRITER_SLOT_SIZE)
            self._mapping.write(complete)
            self._mapping.flush()
