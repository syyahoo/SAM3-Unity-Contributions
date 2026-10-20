from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass

import numpy as np


DEPTH_DISTANCE_SOURCE = "depth-anything-v2-metric-indoor-small"


@dataclass(frozen=True)
class DepthRegions:
    left_meters: float
    center_meters: float
    right_meters: float
    coverage: float


@dataclass(frozen=True)
class DepthDistanceSample:
    version: int
    sequence: int
    frame_id: int
    source_timestamp_ns: int
    left_meters: float
    center_meters: float
    right_meters: float
    coverage: float
    inference_ms: float
    source: str = DEPTH_DISTANCE_SOURCE

    # Unity JsonUtility 계약과 같은 camelCase JSON을 생성한다.
    def to_json(self) -> str:
        values = asdict(self)
        payload = {
            "version": values["version"],
            "sequence": values["sequence"],
            "frameId": values["frame_id"],
            "sourceTimestampNs": values["source_timestamp_ns"],
            "leftMeters": round(values["left_meters"], 3),
            "centerMeters": round(values["center_meters"], 3),
            "rightMeters": round(values["right_meters"], 3),
            "coverage": round(values["coverage"], 3),
            "inferenceMs": round(values["inference_ms"], 2),
            "source": values["source"],
        }
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False)


class DepthDistanceSmoother:
    """접근 거리는 빠르게, 멀어지는 거리는 천천히 안정화한다."""

    def __init__(self, approach_alpha: float = 0.65, recede_alpha: float = 0.30) -> None:
        if not 0.0 < approach_alpha <= 1.0 or not 0.0 < recede_alpha <= 1.0:
            raise ValueError("거리 평활 alpha는 0보다 크고 1 이하여야 합니다.")
        self.approach_alpha = approach_alpha
        self.recede_alpha = recede_alpha
        self._previous: DepthRegions | None = None

    # 새 3구간 거리를 이전 표본과 비대칭 지수 평활한다.
    def update(self, current: DepthRegions) -> DepthRegions:
        if self._previous is None:
            self._previous = current
            return current

        previous = self._previous

        def smooth(old: float, new: float) -> float:
            alpha = self.approach_alpha if new < old else self.recede_alpha
            return old + alpha * (new - old)

        result = DepthRegions(
            left_meters=smooth(previous.left_meters, current.left_meters),
            center_meters=smooth(previous.center_meters, current.center_meters),
            right_meters=smooth(previous.right_meters, current.right_meters),
            coverage=current.coverage,
        )
        self._previous = result
        return result


# 프레임 가장자리와 상단 하늘을 제외한 좌·중·우 진행 영역이다.
_REGIONS = (
    (0.08, 0.34, 0.34, 0.78),
    (0.38, 0.62, 0.30, 0.78),
    (0.66, 0.92, 0.34, 0.78),
)


# Metric depth map을 좌·중·우의 가까운 유효 표면 거리로 축약한다.
def summarize_metric_depth(
    depth: np.ndarray,
    *,
    minimum_depth_meters: float = 0.15,
    maximum_depth_meters: float = 20.0,
    minimum_coverage: float = 0.60,
    near_percentile: float = 20.0,
) -> DepthRegions | None:
    if not isinstance(depth, np.ndarray) or depth.ndim != 2 or depth.size == 0:
        raise ValueError("depth map은 비어 있지 않은 2차원 numpy 배열이어야 합니다.")
    if not 0.0 <= minimum_coverage <= 1.0:
        raise ValueError("minimum coverage는 0과 1 사이여야 합니다.")

    height, width = depth.shape
    distances: list[float] = []
    coverages: list[float] = []
    for left, right, top, bottom in _REGIONS:
        x0 = max(0, min(width - 1, int(round(width * left))))
        x1 = max(x0 + 1, min(width, int(round(width * right))))
        y0 = max(0, min(height - 1, int(round(height * top))))
        y1 = max(y0 + 1, min(height, int(round(height * bottom))))
        values = depth[y0:y1, x0:x1]
        valid = np.isfinite(values)
        valid &= values >= minimum_depth_meters
        valid &= values <= maximum_depth_meters
        coverage = float(np.count_nonzero(valid) / values.size)
        if coverage < minimum_coverage:
            return None
        distance = float(np.percentile(values[valid], near_percentile))
        if not math.isfinite(distance) or distance <= 0.0:
            return None
        distances.append(distance)
        coverages.append(coverage)

    return DepthRegions(
        left_meters=distances[0],
        center_meters=distances[1],
        right_meters=distances[2],
        coverage=min(coverages),
    )
