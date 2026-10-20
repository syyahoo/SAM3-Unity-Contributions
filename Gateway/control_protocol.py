from __future__ import annotations

import json
import socket
import struct
import time
from dataclasses import dataclass


CONTROL_MAGIC = 0x31524354
CONTROL_VERSION = 1
CONTROL_PACKET = struct.Struct("<I H H Q q 4h B 7x")
ACK_PACKET = struct.Struct("<I H H Q q B 7x")

COMMAND_RC = 0
COMMAND_NEUTRAL = 1
COMMAND_TAKEOFF = 2
COMMAND_LAND = 3
COMMAND_EMERGENCY = 4
COMMAND_HELLO = 5
INTERNAL_CONTROL_FLAG = 1
DESKTOP_CONTROL_FLAG = 2
ACK_STATUS_OK = 0
ACK_STATUS_FAILED = 1
ACK_STATUS_CANCELLED = 2
ACK_STATUS_DISPATCHED = 3
CONTROL_MOTION_VERSION = 1


@dataclass(frozen=True)
class ControlPacket:
    sequence: int
    input_ticks: int
    axes: tuple[int, int, int, int]
    command: int
    flags: int = 0

    # 고정 크기 Unity 조종 패킷을 해석한다.
    @classmethod
    def decode(cls, payload: bytes) -> "ControlPacket | None":
        if len(payload) != CONTROL_PACKET.size:
            return None

        magic, version, flags, sequence, input_ticks, *values = CONTROL_PACKET.unpack(payload)
        if magic != CONTROL_MAGIC or version != CONTROL_VERSION:
            return None

        return cls(
            sequence=sequence,
            input_ticks=input_ticks,
            axes=(values[0], values[1], values[2], values[3]),
            command=values[4],
            flags=flags,
        )


# 고정 크기 조종 패킷을 만든다.
def encode_control_packet(packet: ControlPacket) -> bytes:
    return CONTROL_PACKET.pack(
        CONTROL_MAGIC,
        CONTROL_VERSION,
        packet.flags,
        packet.sequence,
        packet.input_ticks,
        *packet.axes,
        packet.command,
    )


# Unity로 전송할 명령 결과 ACK를 만든다.
def encode_ack(sequence: int, relay_ticks: int, status: int = 0) -> bytes:
    return ACK_PACKET.pack(
        CONTROL_MAGIC,
        CONTROL_VERSION,
        status,
        sequence,
        relay_ticks,
        1,
    )


class ControlMotionPublisher:
    """실제로 전송된 최신 RC를 위치 예측 프로세스에 best-effort로 공유한다."""

    def __init__(self, port: int = 49003) -> None:
        self._target = ("127.0.0.1", port)
        self._sequence = 0
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._closed = False

    # 위치 예측이 오래된 조종 입력을 재사용하지 않도록 매 전송 시각을 함께 보낸다.
    def publish(self, control) -> bool:
        if self._closed:
            return False
        axes = tuple(
            max(-100, min(100, int(getattr(control, name, 0))))
            for name in ("left_right", "forward_back", "up_down", "yaw")
        )
        self._sequence += 1
        payload = json.dumps(
            {
                "version": CONTROL_MOTION_VERSION,
                "kind": "control-motion",
                "sequence": self._sequence,
                "sourceTimestampNs": time.perf_counter_ns(),
                "axes": axes,
            },
            separators=(",", ":"),
        ).encode("utf-8")
        try:
            self._socket.sendto(payload, self._target)
            return True
        except OSError:
            return False

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._socket.close()
