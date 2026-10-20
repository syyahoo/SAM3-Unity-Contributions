from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Iterable

import cv2
import numpy as np

from vision_frame_tcp import LatestFrameTcpReader


DEFAULT_SAMPLE_COUNT = 30
DEFAULT_MAX_RMS = 1.0


@dataclass(frozen=True)
class ViewSignature:
    center_x: float
    center_y: float
    coverage: float
    angle_degrees: float
    detected_mask: int


def _angle_delta(first: float, second: float) -> float:
    return abs((first - second + 180.0) % 360.0 - 180.0)


def build_view_signature(
    detections: Iterable,
    image_size: tuple[int, int],
    tag_ids: tuple[int, ...],
) -> ViewSignature | None:
    detections = [item for item in detections if int(item.tag_id) in tag_ids]
    if len(detections) < 2:
        return None

    width, height = image_size
    if width <= 0 or height <= 0:
        return None

    corners = np.concatenate(
        [np.asarray(item.corners, dtype=np.float64) for item in detections],
        axis=0,
    )
    center = np.mean(corners, axis=0)
    hull = cv2.convexHull(corners.astype(np.float32))
    coverage = math.sqrt(
        max(0.0, float(cv2.contourArea(hull))) / float(width * height)
    )
    by_id = {
        int(item.tag_id): np.asarray(item.center, dtype=np.float64)
        for item in detections
    }
    direction = None
    for first_id, second_id in ((0, 1), (2, 3), (0, 2), (1, 3)):
        if first_id in by_id and second_id in by_id:
            direction = by_id[second_id] - by_id[first_id]
            break
    angle = 0.0 if direction is None else math.degrees(
        math.atan2(float(direction[1]), float(direction[0]))
    )
    mask = 0
    for item in detections:
        mask |= 1 << tag_ids.index(int(item.tag_id))
    return ViewSignature(
        float(center[0]) / width,
        float(center[1]) / height,
        coverage,
        angle,
        mask,
    )


def is_diverse_view(
    candidate: ViewSignature,
    accepted: Iterable[ViewSignature],
    minimum_center_shift: float,
    minimum_coverage_change: float,
    minimum_angle_change: float,
) -> bool:
    for previous in accepted:
        center_shift = math.hypot(
            candidate.center_x - previous.center_x,
            candidate.center_y - previous.center_y,
        )
        coverage_change = abs(candidate.coverage - previous.coverage)
        angle_change = _angle_delta(
            candidate.angle_degrees,
            previous.angle_degrees,
        )
        if (
            candidate.detected_mask == previous.detected_mask
            and center_shift < minimum_center_shift
            and coverage_change < minimum_coverage_change
            and angle_change < minimum_angle_change
        ):
            return False
    return True


def normalize_calibration_result(
    candidate_path: Path,
    output_path: Path,
    sample_count: int,
    maximum_rms: float,
) -> dict:
    data = json.loads(candidate_path.read_text(encoding="utf-8"))
    matrix = np.asarray(data.get("camera_matrix", []), dtype=np.float64).reshape(-1)
    distortion = np.asarray(
        data.get("distortion_coefficients", []),
        dtype=np.float64,
    ).reshape(-1)
    rms = float(data.get("rms_reprojection_error", math.inf))
    width = int(data.get("image_width", 0))
    height = int(data.get("image_height", 0))
    if (
        data.get("calibrated") is not True
        or matrix.size != 9
        or distortion.size < 4
        or not np.all(np.isfinite(matrix))
        or not np.all(np.isfinite(distortion))
        or not math.isfinite(rms)
        or rms < 0.0
        or rms > maximum_rms
        or width <= 0
        or height <= 0
        or matrix[0] <= 0.0
        or matrix[4] <= 0.0
    ):
        raise ValueError(
            f"보정 결과가 기준을 충족하지 않습니다: RMS={rms:.3f}px"
        )

    result = {
        "calibrated": True,
        "rms_reprojection_error": rms,
        "image_width": width,
        "image_height": height,
        "camera_matrix": matrix.tolist(),
        "distortion_coefficients": distortion.tolist(),
        "sample_count": sample_count,
        "tag_family": "tag36h11",
        "tag_size_meters": 0.117,
        "tag_center_spacing_meters": 0.190,
        "tag_ids": [0, 1, 2, 3],
        "source": "pilot-vision-tcp",
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary_path.replace(output_path)
    return result


def load_vision_module(vision_root: Path) -> ModuleType:
    source = (
        vision_root
        / "Assets"
        / "StreamingAssets"
        / "TelloVisionBridge"
        / "Source"
        / "tello_vision_bridge.py"
    )
    if not source.is_file():
        raise RuntimeError(f"Vision 원본 모듈을 찾을 수 없습니다: {source}")
    spec = importlib.util.spec_from_file_location("strikerpc_calibration_source", source)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Vision 원본 모듈을 로드할 수 없습니다: {source}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    if not hasattr(module, "BoardTracker"):
        raise RuntimeError("Vision 원본에 BoardTracker가 없습니다.")
    return module


def run_calibration(args: argparse.Namespace) -> int:
    vision_root = Path(args.vision_root).resolve()
    output_path = Path(args.output).resolve()
    candidate_path = (
        Path(tempfile.gettempdir())
        / "StrikerPC"
        / "Calibration"
        / f"{output_path.stem}.candidate{output_path.suffix}"
    )
    candidate_path.parent.mkdir(parents=True, exist_ok=True)
    module = load_vision_module(vision_root)
    board_args = argparse.Namespace(
        tag_family="tag36h11",
        tag_size=0.117,
        tag_spacing=0.190,
        tag_ids="0,1,2,3",
        max_reprojection_error=3.0,
    )
    tracker = module.BoardTracker(board_args, candidate_path)
    tracker.start_calibration()
    reader = LatestFrameTcpReader(args.video_port)
    signatures: list[ViewSignature] = []
    tag_ids = (0, 1, 2, 3)
    last_status_at = 0.0
    print("STRIKERPC_READY TELLO_CAMERA_CALIBRATION", flush=True)
    print(
        "Tello 카메라에 보드를 비추고 거리, 화면 위치, 높이, 기울기를 천천히 바꾸세요.",
        flush=True,
    )
    try:
        while len(signatures) < args.samples:
            shared = reader.read()
            if shared is None:
                continue
            frame = shared.frame
            tracker.track(frame)
            detections = list(getattr(tracker, "_last_detections", ()))
            image_size = (frame.shape[1], frame.shape[0])
            signature = build_view_signature(detections, image_size, tag_ids)
            now = time.monotonic()
            if signature is None:
                if now - last_status_at >= 2.0:
                    print("보정 대기: AprilTag가 두 개 이상 보여야 합니다.", flush=True)
                    last_status_at = now
                continue
            if not is_diverse_view(
                signature,
                signatures,
                args.minimum_center_shift,
                args.minimum_coverage_change,
                args.minimum_angle_change,
            ):
                if now - last_status_at >= 2.0:
                    print("보정 대기: 보드의 거리, 위치 또는 기울기를 더 바꾸세요.", flush=True)
                    last_status_at = now
                continue
            tracker._capture_calibration_sample(detections, image_size)
            signatures.append(signature)
            print(
                f"보정 샘플 {len(signatures)}/{args.samples}: "
                f"tags={len(detections)}, coverage={signature.coverage:.3f}",
                flush=True,
            )

        if not tracker.save_calibration():
            print(
                f"보정 실패: 후보 결과를 확인하세요: {candidate_path}",
                file=sys.stderr,
                flush=True,
            )
            return 2
        result = normalize_calibration_result(
            candidate_path,
            output_path,
            len(signatures),
            args.maximum_rms,
        )
        candidate_path.unlink(missing_ok=True)
        print(
            f"보정 완료: RMS={result['rms_reprojection_error']:.3f}px, "
            f"output={output_path}",
            flush=True,
        )
        return 0
    except KeyboardInterrupt:
        print("보정이 취소되었습니다.", flush=True)
        return 130
    finally:
        reader.close()


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description="Pilot Adapter의 Tello 영상으로 카메라 내부 파라미터를 보정합니다."
    )
    parser.add_argument(
        "--vision-root",
        default=str(root / "Vision"),
    )
    parser.add_argument("--video-port", type=int, default=49000)
    parser.add_argument(
        "--output",
        default=str(
            root
            / "UnityClient"
            / "Assets"
            / "StreamingAssets"
            / "Maps"
            / "tello-camera-calibration.json"
        ),
    )
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLE_COUNT)
    parser.add_argument("--maximum-rms", type=float, default=DEFAULT_MAX_RMS)
    parser.add_argument("--minimum-center-shift", type=float, default=0.06)
    parser.add_argument("--minimum-coverage-change", type=float, default=0.025)
    parser.add_argument("--minimum-angle-change", type=float, default=7.0)
    args = parser.parse_args()
    if args.samples < 30:
        parser.error("--samples는 30 이상이어야 합니다.")
    if args.maximum_rms <= 0.0:
        parser.error("--maximum-rms는 0보다 커야 합니다.")
    return args


if __name__ == "__main__":
    raise SystemExit(run_calibration(parse_args()))
