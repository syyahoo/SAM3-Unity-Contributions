from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum


TVP1_MAGIC = 0x31505654
TVP2_MAGIC = 0x32505654
TVP2_VERSION = 2
TVP2_PACKET = struct.Struct("<I H B B Q Q 3f 4f 3f 3f f f H H")
assert TVP2_PACKET.size == 88
LOCAL_MAP_READY_FLAG = 1 << 0
LOCAL_MAP_DEGRADED_FLAG = 1 << 1
RELOCALIZING_FLAG = 1 << 2
SESSION_MAP_FLAG = 1 << 3


class TrackingMode(IntEnum):
    INITIALIZING = 0
    APRILTAG_LOCKED = 1
    VISUAL_TRACKED = 2
    SENSOR_PREDICTED = 3
    UNCERTAIN = 4


@dataclass(frozen=True)
class PosePacketV2:
    sequence: int
    source_timestamp_ns: int
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]
    velocity: tuple[float, float, float]
    position_stddev: tuple[float, float, float]
    yaw_stddev: float
    confidence: float
    visual_anchor_count: int
    detected_tag_count: int
    mode: TrackingMode
    flags: int = 0

    def encode(self) -> bytes:
        values = (
            *self.position,
            *self.rotation,
            *self.velocity,
            *self.position_stddev,
            self.yaw_stddev,
            self.confidence,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("pose 패킷에 유한하지 않은 값이 있습니다.")
        return TVP2_PACKET.pack(
            TVP2_MAGIC,
            TVP2_VERSION,
            int(self.mode),
            self.flags & 0xFF,
            self.sequence,
            self.source_timestamp_ns,
            *self.position,
            *self.rotation,
            *self.velocity,
            *self.position_stddev,
            self.yaw_stddev,
            max(0.0, min(1.0, self.confidence)),
            max(0, min(65535, self.visual_anchor_count)),
            max(0, min(65535, self.detected_tag_count)),
        )

    @classmethod
    def decode(cls, payload: bytes) -> "PosePacketV2 | None":
        if len(payload) != TVP2_PACKET.size:
            return None
        values = TVP2_PACKET.unpack(payload)
        if values[0] != TVP2_MAGIC or values[1] != TVP2_VERSION:
            return None
        try:
            mode = TrackingMode(values[2])
        except ValueError:
            return None
        packet = cls(
            sequence=values[4],
            source_timestamp_ns=values[5],
            position=tuple(values[6:9]),
            rotation=tuple(values[9:13]),
            velocity=tuple(values[13:16]),
            position_stddev=tuple(values[16:19]),
            yaw_stddev=values[19],
            confidence=values[20],
            visual_anchor_count=values[21],
            detected_tag_count=values[22],
            mode=mode,
            flags=values[3],
        )
        try:
            packet.encode()
        except ValueError:
            return None
        return packet
