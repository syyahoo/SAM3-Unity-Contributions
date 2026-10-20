from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping


VENUE_XZ_METERS = "venue-xz-meters"
NORMALIZED_TOP_LEFT = "normalized-top-left"


@dataclass(frozen=True)
class VenueBounds:
    min_x: float
    max_x: float
    min_z: float
    max_z: float

    @property
    def width(self) -> float:
        return self.max_x - self.min_x

    @property
    def depth(self) -> float:
        return self.max_z - self.min_z

    # 지도 manifest의 X/Z 경계를 유한한 미터 범위로 고정한다.
    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "VenueBounds":
        try:
            bounds = cls(
                min_x=float(value["minX"]),
                max_x=float(value["maxX"]),
                min_z=float(value["minZ"]),
                max_z=float(value["maxZ"]),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("지도 X/Z 경계가 올바르지 않습니다.") from error
        if (
            not all(
                math.isfinite(component)
                for component in (
                    bounds.min_x,
                    bounds.max_x,
                    bounds.min_z,
                    bounds.max_z,
                )
            )
            or bounds.width <= 0.0
            or bounds.depth <= 0.0
        ):
            raise ValueError("지도 X/Z 경계가 올바르지 않습니다.")
        return bounds

    # Venue X/Z 점이 현재 지도 경계 안인지 확인한다.
    def contains(self, map_x: float, map_z: float) -> bool:
        return (
            math.isfinite(map_x)
            and math.isfinite(map_z)
            and self.min_x <= map_x <= self.max_x
            and self.min_z <= map_z <= self.max_z
        )


# 좌상단 정규화 좌표를 Venue X/Z 미터 좌표로 변환한다.
def normalized_to_venue(
    bounds: VenueBounds,
    normalized_x: float,
    normalized_y: float,
) -> tuple[float, float]:
    if (
        not math.isfinite(normalized_x)
        or not math.isfinite(normalized_y)
        or not 0.0 <= normalized_x <= 1.0
        or not 0.0 <= normalized_y <= 1.0
    ):
        raise ValueError("좌상단 정규화 좌표는 0 이상 1 이하여야 합니다.")
    return (
        bounds.min_x + normalized_x * bounds.width,
        bounds.max_z - normalized_y * bounds.depth,
    )


# Venue X/Z 미터 좌표를 좌상단 정규화 좌표로 변환한다.
def venue_to_normalized(
    bounds: VenueBounds,
    map_x: float,
    map_z: float,
) -> tuple[float, float]:
    if not math.isfinite(map_x) or not math.isfinite(map_z):
        raise ValueError("Venue X/Z 좌표는 유한해야 합니다.")
    return (
        (map_x - bounds.min_x) / bounds.width,
        (bounds.max_z - map_z) / bounds.depth,
    )
