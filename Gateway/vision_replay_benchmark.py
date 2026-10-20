from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np

from import_venue_visual_map import matrix_to_quaternion, validate_bundle_integrity
from vision_adapter import SparseAnchorTracker
from visual_map import (
    VisualMapArtifact,
    VisualMapError,
    load_visual_map,
    load_visual_map_identity,
)


FIXTURE_SCHEMA_VERSION = 1
INSPECTION_FILE_NAME = "vision-replay-inspection.json"
FIXTURE_FILE_NAME = "vision-replay-fixture.json"
METRICS_FILE_NAME = "vision-replay-metrics.json"
PREDICTIONS_FILE_NAME = "vision-replay-predictions.json"
DEFAULT_MIN_SUCCESS_RATE = 0.9
DEFAULT_MAX_TRANSLATION_ERROR_METERS = 0.5
DEFAULT_MAX_YAW_ERROR_DEGREES = 15.0
DEFAULT_MAX_FALSE_ACCEPT_RATE = 0.01


class VisionReplayBenchmarkError(RuntimeError):
    pass


class CameraContractMismatchError(VisionReplayBenchmarkError):
    pass


@dataclass(frozen=True)
class ReplaySource:
    bundle_path: Path
    index_path: Path
    bundle_sha256: str
    index_sha256: str
    index: dict[str, Any]
    frames: tuple[dict[str, Any], ...]
    source_width: int
    source_height: int
    source_intrinsics: tuple[float, float, float, float]


# 입력 파일의 내용을 바꾸지 않고 SHA-256을 계산한다.
def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


# JSON 객체 파일을 엄격하게 읽는다.
def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise VisionReplayBenchmarkError(f"JSON 파일을 읽을 수 없습니다: {path}") from error
    if not isinstance(value, dict):
        raise VisionReplayBenchmarkError(f"JSON 루트가 객체가 아닙니다: {path}")
    return value


# ZIP 내부 JSON 객체를 읽는다.
def _read_archive_json(archive: zipfile.ZipFile, path: str) -> dict[str, Any]:
    try:
        value = json.loads(archive.read(path).decode("utf-8"))
    except (KeyError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise VisionReplayBenchmarkError(f"번들 JSON을 읽을 수 없습니다: {path}") from error
    if not isinstance(value, dict):
        raise VisionReplayBenchmarkError(f"번들 JSON 루트가 객체가 아닙니다: {path}")
    return value


# 위치와 회전 라벨의 기본 형식을 검증한다.
def _validate_pose(frame: dict[str, Any], frame_id: str) -> None:
    try:
        position = np.asarray(
            [frame["position"][axis] for axis in ("x", "y", "z")],
            np.float64,
        )
        rotation = np.asarray(
            [frame["rotation"][axis] for axis in ("x", "y", "z", "w")],
            np.float64,
        )
    except (KeyError, TypeError, ValueError) as error:
        raise VisionReplayBenchmarkError(f"프레임 pose 형식이 잘못되었습니다: {frame_id}") from error
    if not np.isfinite(position).all() or not np.isfinite(rotation).all():
        raise VisionReplayBenchmarkError(f"프레임 pose에 유한하지 않은 값이 있습니다: {frame_id}")
    if abs(float(np.linalg.norm(rotation)) - 1.0) > 1e-3:
        raise VisionReplayBenchmarkError(f"프레임 quaternion이 정규화되지 않았습니다: {frame_id}")


# 번들과 인덱스의 SHA, identity, 프레임 대응 관계를 검증한다.
def load_replay_source(bundle_path: Path, index_path: Path) -> ReplaySource:
    bundle_path = bundle_path.resolve()
    index_path = index_path.resolve()
    if not bundle_path.is_file() or not index_path.is_file():
        raise VisionReplayBenchmarkError("번들과 scan-frame index 파일이 모두 필요합니다.")

    bundle_sha256 = sha256_path(bundle_path)
    index_sha256 = sha256_path(index_path)
    index = _read_json(index_path)
    if int(index.get("formatVersion", 0)) != 1:
        raise VisionReplayBenchmarkError("지원하지 않는 scan-frame index 버전입니다.")
    map_id = str(index.get("mapId") or "")
    scan_id = str(index.get("scanId") or "")
    if not map_id or not scan_id:
        raise VisionReplayBenchmarkError("index mapId/scanId가 비어 있습니다.")

    sources = index.get("sources")
    if not isinstance(sources, list):
        raise VisionReplayBenchmarkError("index sources가 배열이 아닙니다.")
    matching_sources = [
        item
        for item in sources
        if isinstance(item, dict) and item.get("bundleSha256") == bundle_sha256
    ]
    if len(matching_sources) != 1:
        raise VisionReplayBenchmarkError("번들 SHA-256이 index source와 정확히 한 번 일치해야 합니다.")
    source_record = matching_sources[0]

    frames_value = index.get("frames")
    if not isinstance(frames_value, list) or len(frames_value) < 2:
        raise VisionReplayBenchmarkError("재생 벤치마크에는 두 개 이상의 index frame이 필요합니다.")
    if int(index.get("frameCount", -1)) != len(frames_value):
        raise VisionReplayBenchmarkError("index frameCount가 frames 배열과 일치하지 않습니다.")

    source_camera = index.get("sourceCamera")
    if not isinstance(source_camera, dict):
        raise VisionReplayBenchmarkError("index sourceCamera가 없습니다.")
    try:
        source_width = int(source_camera["sourceImageWidth"])
        source_height = int(source_camera["sourceImageHeight"])
        source_intrinsics = tuple(
            float(source_camera[key])
            for key in (
                "sourceFocalLengthX",
                "sourceFocalLengthY",
                "sourcePrincipalPointX",
                "sourcePrincipalPointY",
            )
        )
    except (KeyError, TypeError, ValueError) as error:
        raise VisionReplayBenchmarkError("sourceCamera 형식이 잘못되었습니다.") from error
    if source_width < 1 or source_height < 1 or not np.isfinite(source_intrinsics).all():
        raise VisionReplayBenchmarkError("sourceCamera 값이 유효하지 않습니다.")

    frame_ids: set[str] = set()
    image_paths: set[str] = set()
    frames: list[dict[str, Any]] = []
    for item in frames_value:
        if not isinstance(item, dict):
            raise VisionReplayBenchmarkError("index frame이 객체가 아닙니다.")
        frame_id = str(item.get("id") or "")
        image_path = str(item.get("imagePath") or "")
        if not frame_id or frame_id in frame_ids:
            raise VisionReplayBenchmarkError(f"프레임 ID가 비었거나 중복되었습니다: {frame_id}")
        if not image_path or image_path.startswith(("/", "\\")) or ".." in Path(image_path).parts:
            raise VisionReplayBenchmarkError(f"안전하지 않은 imagePath입니다: {image_path}")
        _validate_pose(item, frame_id)
        try:
            timestamp = float(item["timestampSeconds"])
            int(item["mapGridX"])
            int(item["mapGridZ"])
        except (KeyError, TypeError, ValueError) as error:
            raise VisionReplayBenchmarkError(f"프레임 시간/지도 셀이 잘못되었습니다: {frame_id}") from error
        if not math.isfinite(timestamp):
            raise VisionReplayBenchmarkError(f"프레임 시간이 유효하지 않습니다: {frame_id}")
        frame_ids.add(frame_id)
        image_paths.add(image_path)
        frames.append(item)

    with zipfile.ZipFile(bundle_path, "r") as archive:
        try:
            validate_bundle_integrity(archive)
        except Exception as error:
            raise VisionReplayBenchmarkError(f"번들 무결성 검증에 실패했습니다: {error}") from error
        manifest = _read_archive_json(archive, "manifest.json")
        visual = _read_archive_json(archive, "visual/keyframes.json")
        archive_names = set(archive.namelist())
        if not image_paths.issubset(archive_names):
            missing = sorted(image_paths - archive_names)[0]
            raise VisionReplayBenchmarkError(f"index가 가리키는 JPEG가 번들에 없습니다: {missing}")
        visual_frames = visual.get("keyframes")
        if not isinstance(visual_frames, list):
            raise VisionReplayBenchmarkError("visual/keyframes.json keyframes가 배열이 아닙니다.")
        visual_paths = {
            str(item.get("imagePath") or "")
            for item in visual_frames
            if isinstance(item, dict)
        }
        if image_paths != visual_paths:
            raise VisionReplayBenchmarkError("index와 visual keyframe의 imagePath 집합이 다릅니다.")

    if str(manifest.get("scanId") or "") != str(source_record.get("bundleManifestScanId") or ""):
        raise VisionReplayBenchmarkError("번들 manifest scanId가 index source와 다릅니다.")
    if str(manifest.get("scanId") or "") != scan_id:
        raise VisionReplayBenchmarkError("번들 manifest scanId가 index scanId와 다릅니다.")
    if str(visual.get("scanId") or "") != str(source_record.get("visualScanId") or ""):
        raise VisionReplayBenchmarkError("visual scanId가 index source와 다릅니다.")
    if int(source_record.get("frameCount", -1)) != len(frames):
        raise VisionReplayBenchmarkError("index source frameCount가 frames 배열과 다릅니다.")

    return ReplaySource(
        bundle_path=bundle_path,
        index_path=index_path,
        bundle_sha256=bundle_sha256,
        index_sha256=index_sha256,
        index=index,
        frames=tuple(frames),
        source_width=source_width,
        source_height=source_height,
        source_intrinsics=source_intrinsics,
    )


# 런타임 calibration 및 visual-map identity와 재생 소스를 비교한다.
def inspect_fixture(
    bundle_path: Path,
    index_path: Path,
    *,
    calibration_path: Path | None = None,
    visual_map_manifest_path: Path | None = None,
) -> tuple[ReplaySource, dict[str, Any]]:
    source = load_replay_source(bundle_path, index_path)
    index = source.index
    blocking_reasons: list[str] = []
    runtime_camera: dict[str, Any] | None = None
    visual_map_identity: dict[str, Any] | None = None

    calibration: dict[str, Any] | None = None
    calibration_sha256: str | None = None
    if calibration_path is None:
        blocking_reasons.append("runtime-calibration-not-provided")
    else:
        calibration_path = calibration_path.resolve()
        calibration = _read_json(calibration_path)
        calibration_sha256 = sha256_path(calibration_path)
        try:
            runtime_width = int(calibration["image_width"])
            runtime_height = int(calibration["image_height"])
            camera_matrix = np.asarray(calibration["camera_matrix"], np.float64).reshape(3, 3)
        except (KeyError, TypeError, ValueError) as error:
            raise VisionReplayBenchmarkError("runtime calibration 형식이 잘못되었습니다.") from error
        if runtime_width < 1 or runtime_height < 1 or not np.isfinite(camera_matrix).all():
            raise VisionReplayBenchmarkError("runtime calibration 값이 유효하지 않습니다.")
        runtime_intrinsics = (
            float(camera_matrix[0, 0]),
            float(camera_matrix[1, 1]),
            float(camera_matrix[0, 2]),
            float(camera_matrix[1, 2]),
        )
        runtime_camera = {
            "width": runtime_width,
            "height": runtime_height,
            "intrinsics": list(runtime_intrinsics),
            "calibrationSha256": calibration_sha256,
        }
        if (source.source_width, source.source_height) != (runtime_width, runtime_height):
            blocking_reasons.append("source-runtime-image-size-mismatch")
        if not np.allclose(source.source_intrinsics, runtime_intrinsics, rtol=0.0, atol=1e-6):
            blocking_reasons.append("source-runtime-intrinsics-mismatch")

    if visual_map_manifest_path is None:
        blocking_reasons.append("visual-map-manifest-not-provided")
    else:
        manifest_path = visual_map_manifest_path.resolve()
        manifest = _read_json(manifest_path)
        visual_map_identity = {
            "mapId": manifest.get("mapId"),
            "mapVersion": manifest.get("mapVersion"),
            "scanId": manifest.get("scanId"),
            "imageWidth": manifest.get("imageWidth"),
            "imageHeight": manifest.get("imageHeight"),
            "calibrationSha256": manifest.get("calibrationSha256"),
            "manifestSha256": sha256_path(manifest_path),
        }
        expected_identity = {
            "mapId": index.get("mapId"),
            "mapVersion": index.get("mapVersion"),
            "scanId": index.get("scanId"),
        }
        for key, expected in expected_identity.items():
            if manifest.get(key) != expected:
                raise VisionReplayBenchmarkError(f"visual-map {key}가 index와 다릅니다.")
        if calibration_sha256 is not None and manifest.get("calibrationSha256") != calibration_sha256:
            raise VisionReplayBenchmarkError("visual-map calibration SHA-256이 runtime calibration과 다릅니다.")
        if runtime_camera is not None and (
            manifest.get("imageWidth") != runtime_camera["width"]
            or manifest.get("imageHeight") != runtime_camera["height"]
        ):
            raise VisionReplayBenchmarkError("visual-map 이미지 크기가 runtime calibration과 다릅니다.")

    replay_blocking_reasons = list(blocking_reasons)
    accuracy_blocking_reasons = [
        *replay_blocking_reasons,
        "independent-drone-ground-truth-not-provided",
        "train-only-map-provenance-not-provided",
    ]
    inspection = {
        "schemaVersion": FIXTURE_SCHEMA_VERSION,
        "kind": "visionReplayInspection",
        "inputs": {
            "bundle": str(source.bundle_path),
            "bundleSha256": source.bundle_sha256,
            "index": str(source.index_path),
            "indexSha256": source.index_sha256,
        },
        "identity": {
            "mapId": index.get("mapId"),
            "mapVersion": index.get("mapVersion"),
            "scanId": index.get("scanId"),
            "coordinateSystem": index.get("coordinateSystem"),
        },
        "sourceCamera": {
            "width": source.source_width,
            "height": source.source_height,
            "intrinsics": list(source.source_intrinsics),
        },
        "runtimeCamera": runtime_camera,
        "visualMapIdentity": visual_map_identity,
        "frameCount": len(source.frames),
        "replayEligible": not replay_blocking_reasons,
        "replayBlockingReasons": replay_blocking_reasons,
        "accuracyEligible": False,
        "blockingReasons": accuracy_blocking_reasons,
        "poseOracle": {
            "type": "quest-camera-reference-pose",
            "independentDroneGroundTruth": False,
        },
        "resizePolicy": "forbidden-for-accuracy",
        "warnings": [
            "Quest 카메라 reference pose는 독립적인 Tello ground truth가 아닙니다.",
            "전체 source frame으로 만든 canonical map 재생은 accuracy가 아닌 exact-replay smoke입니다.",
            "resize/crop/warp는 intrinsics를 함께 갱신하지 않으면 같은 pose 라벨을 사용할 수 없습니다.",
        ],
    }
    return source, inspection


# 서로 다른 용도의 난수 스트림을 안정적으로 파생한다.
def _derived_seed(seed: int, label: str) -> int:
    payload = f"vision-replay-v1:{seed}:{label}".encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")


# seed로 단일 프레임과 연속 window를 결정적으로 고른다.
def select_frames_and_windows(
    frames: Sequence[dict[str, Any]],
    *,
    seed: int,
    frame_count: int,
    window_count: int,
    window_size: int,
) -> dict[str, Any]:
    if frame_count < 1 or window_count < 0 or window_size < 1:
        raise VisionReplayBenchmarkError("frame/window 선택 수가 유효하지 않습니다.")
    if window_size > len(frames):
        raise VisionReplayBenchmarkError("window 크기가 전체 프레임 수보다 큽니다.")
    frame_rng = random.Random(_derived_seed(seed, "frames"))
    indices = frame_rng.sample(range(len(frames)), min(frame_count, len(frames)))
    maximum_start = len(frames) - window_size
    starts = list(range(maximum_start + 1))
    window_rng = random.Random(_derived_seed(seed, "windows"))
    selected_starts = window_rng.sample(starts, min(window_count, len(starts)))
    return {
        "seed": seed,
        "frameIds": [str(frames[index]["id"]) for index in indices],
        "windows": [
            {
                "startIndex": start,
                "frameIds": [str(item["id"]) for item in frames[start : start + window_size]],
            }
            for start in selected_starts
        ],
        "sequencePolicy": "preserve-time-order-within-window",
        "singleFramePolicy": "reset-tracker-before-each-frame",
    }


# 시간상 연속 구간과 완전히 분리된 지도 셀을 각각 held-out으로 만든다.
def build_route_spatial_splits(
    frames: Sequence[dict[str, Any]],
    *,
    seed: int,
    held_out_ratio: float,
) -> dict[str, Any]:
    if not 0.0 < held_out_ratio < 1.0:
        raise VisionReplayBenchmarkError("held-out 비율은 0과 1 사이여야 합니다.")
    held_count = min(len(frames) - 1, max(1, int(math.ceil(len(frames) * held_out_ratio))))
    route_rng = random.Random(_derived_seed(seed, "route-split"))
    route_start = route_rng.randrange(0, len(frames) - held_count + 1)
    route_indices = set(range(route_start, route_start + held_count))

    cells: dict[tuple[int, int], list[int]] = {}
    for index, frame in enumerate(frames):
        cell = (int(frame["mapGridX"]), int(frame["mapGridZ"]))
        cells.setdefault(cell, []).append(index)
    if len(cells) < 2:
        raise VisionReplayBenchmarkError("spatial held-out에는 서로 다른 지도 셀이 두 개 이상 필요합니다.")
    cell_keys = sorted(cells)
    spatial_rng = random.Random(_derived_seed(seed, "spatial-split"))
    spatial_rng.shuffle(cell_keys)
    selected_cells: list[tuple[int, int]] = []
    selected_count = 0
    for cell in cell_keys[:-1]:
        selected_cells.append(cell)
        selected_count += len(cells[cell])
        if selected_count >= held_count:
            break
    spatial_indices = {index for cell in selected_cells for index in cells[cell]}

    def split(indices: set[int]) -> dict[str, Any]:
        return {
            "trainFrameIds": [str(frame["id"]) for i, frame in enumerate(frames) if i not in indices],
            "heldOutFrameIds": [str(frame["id"]) for i, frame in enumerate(frames) if i in indices],
        }

    return {
        "route": {
            **split(route_indices),
            "heldOutStartIndex": route_start,
            "heldOutLength": held_count,
            "policy": "single-contiguous-time-ordered-segment",
        },
        "spatial": {
            **split(spatial_indices),
            "heldOutCells": [[x, z] for x, z in selected_cells],
            "policy": "whole-map-cells-never-shared-with-train",
        },
    }


# photometric valid와 invalid 입력의 재현 가능한 계획을 만든다.
def build_augmentation_plan(frame_ids: Sequence[str], *, seed: int) -> dict[str, Any]:
    if not frame_ids:
        raise VisionReplayBenchmarkError("augmentation 대상 프레임이 없습니다.")
    photometric: list[dict[str, Any]] = []
    transforms = ("brightness", "gamma", "blur", "jpeg", "occlusion")
    for frame_id in frame_ids:
        for transform in transforms:
            case_seed = _derived_seed(seed, f"{frame_id}:{transform}")
            rng = random.Random(case_seed)
            if transform == "brightness":
                parameters = {"scale": round(rng.uniform(0.7, 1.3), 6)}
            elif transform == "gamma":
                parameters = {"gamma": round(rng.uniform(0.7, 1.4), 6)}
            elif transform == "blur":
                parameters = {"kernelSize": rng.choice((3, 5))}
            elif transform == "jpeg":
                parameters = {"quality": rng.randint(45, 85)}
            else:
                width_fraction = round(rng.uniform(0.1, 0.25), 6)
                height_fraction = round(rng.uniform(0.1, 0.25), 6)
                parameters = {
                    "xFraction": round(rng.uniform(0.0, 1.0 - width_fraction), 6),
                    "yFraction": round(rng.uniform(0.0, 1.0 - height_fraction), 6),
                    "widthFraction": width_fraction,
                    "heightFraction": height_fraction,
                    "fillValue": 0,
                }
            photometric.append(
                {
                    "caseId": f"{frame_id}:photo:{transform}",
                    "frameId": frame_id,
                    "caseClass": "photometric-valid",
                    "transform": transform,
                    "seed": case_seed,
                    "parameters": parameters,
                    "geometryPreserved": True,
                    "poseLabelAllowed": True,
                    "expectedValid": True,
                }
            )

    invalid: list[dict[str, Any]] = []
    for frame_id in frame_ids:
        for transform in ("noise", "solid", "corrupt-jpeg"):
            case_seed = _derived_seed(seed, f"{frame_id}:invalid:{transform}")
            parameters: dict[str, Any]
            if transform == "noise":
                parameters = {"distribution": "uniform-uint8"}
            elif transform == "solid":
                parameters = {"value": case_seed % 256}
            else:
                parameters = {"keepFraction": 0.2}
            invalid.append(
                {
                    "caseId": f"{frame_id}:invalid:{transform}",
                    "frameId": frame_id,
                    "caseClass": "invalid",
                    "transform": transform,
                    "seed": case_seed,
                    "parameters": parameters,
                    "expectedValid": False,
                    "expectedAccepted": False,
                }
            )
    return {
        "photometric": photometric,
        "invalid": invalid,
        "forbiddenSamePoseTransforms": ["resize", "crop", "rotation", "perspective-warp"],
    }


# 기하를 바꾸지 않는 photometric 변환만 실제 이미지에 적용한다.
def apply_photometric_augmentation(image: np.ndarray, case: dict[str, Any]) -> np.ndarray:
    if case.get("caseClass") != "photometric-valid":
        raise VisionReplayBenchmarkError("photometric-valid case만 이 함수에 적용할 수 있습니다.")
    if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
        raise VisionReplayBenchmarkError("BGR uint8 이미지가 필요합니다.")
    source = np.asarray(image, np.uint8)
    transform = str(case.get("transform") or "")
    parameters = case.get("parameters") or {}
    if transform == "brightness":
        result = np.clip(source.astype(np.float32) * float(parameters["scale"]), 0, 255).astype(np.uint8)
    elif transform == "gamma":
        gamma = float(parameters["gamma"])
        if gamma <= 0.0:
            raise VisionReplayBenchmarkError("gamma는 0보다 커야 합니다.")
        table = np.asarray([round(255.0 * ((value / 255.0) ** gamma)) for value in range(256)], np.uint8)
        result = cv2.LUT(source, table)
    elif transform == "blur":
        kernel = int(parameters["kernelSize"])
        if kernel < 1 or kernel % 2 == 0:
            raise VisionReplayBenchmarkError("blur kernel은 양의 홀수여야 합니다.")
        result = cv2.GaussianBlur(source, (kernel, kernel), 0)
    elif transform == "jpeg":
        quality = int(parameters["quality"])
        encoded, payload = cv2.imencode(".jpg", source, [cv2.IMWRITE_JPEG_QUALITY, quality])
        result = cv2.imdecode(payload, cv2.IMREAD_COLOR) if encoded else None
        if result is None:
            raise VisionReplayBenchmarkError("JPEG augmentation에 실패했습니다.")
    elif transform == "occlusion":
        result = source.copy()
        height, width = result.shape[:2]
        x = min(width - 1, max(0, int(round(width * float(parameters["xFraction"])))))
        y = min(height - 1, max(0, int(round(height * float(parameters["yFraction"])))))
        box_width = max(1, int(round(width * float(parameters["widthFraction"]))))
        box_height = max(1, int(round(height * float(parameters["heightFraction"]))))
        result[y : min(height, y + box_height), x : min(width, x + box_width)] = int(
            parameters.get("fillValue", 0)
        )
    else:
        raise VisionReplayBenchmarkError(f"지원하지 않는 photometric 변환입니다: {transform}")
    if result.shape != source.shape:
        raise VisionReplayBenchmarkError("photometric 변환이 이미지 기하를 변경했습니다.")
    return result


# 위치를 가져서는 안 되는 invalid 입력을 생성한다.
def build_invalid_input(image: np.ndarray, case: dict[str, Any]) -> np.ndarray | bytes:
    if case.get("caseClass") != "invalid":
        raise VisionReplayBenchmarkError("invalid case만 이 함수에 적용할 수 있습니다.")
    source = np.asarray(image, np.uint8)
    if source.ndim != 3 or source.shape[2] != 3:
        raise VisionReplayBenchmarkError("BGR uint8 이미지가 필요합니다.")
    transform = str(case.get("transform") or "")
    parameters = case.get("parameters") or {}
    if transform == "noise":
        rng = np.random.default_rng(int(case["seed"]))
        return rng.integers(0, 256, size=source.shape, dtype=np.uint8)
    if transform == "solid":
        return np.full(source.shape, int(parameters.get("value", 0)), np.uint8)
    if transform == "corrupt-jpeg":
        encoded, payload = cv2.imencode(".jpg", source)
        if not encoded:
            raise VisionReplayBenchmarkError("invalid JPEG 원본 인코딩에 실패했습니다.")
        keep = max(1, int(len(payload) * float(parameters.get("keepFraction", 0.2))))
        return payload.tobytes()[:keep]
    raise VisionReplayBenchmarkError(f"지원하지 않는 invalid 변환입니다: {transform}")


# 선택, split, 변환 계획과 차단 사유를 하나의 fixture로 작성한다.
def compile_fixture(
    bundle_path: Path,
    index_path: Path,
    output_dir: Path,
    *,
    seed: int,
    frame_count: int,
    window_count: int,
    window_size: int,
    held_out_ratio: float,
    calibration_path: Path | None = None,
    visual_map_manifest_path: Path | None = None,
    overwrite: bool = False,
) -> tuple[Path, dict[str, Any]]:
    source, inspection = inspect_fixture(
        bundle_path,
        index_path,
        calibration_path=calibration_path,
        visual_map_manifest_path=visual_map_manifest_path,
    )
    selection = select_frames_and_windows(
        source.frames,
        seed=seed,
        frame_count=frame_count,
        window_count=window_count,
        window_size=window_size,
    )
    selected = set(selection["frameIds"])
    selected_records = [
        {
            "id": frame["id"],
            "timestampSeconds": frame["timestampSeconds"],
            "imagePath": frame["imagePath"],
            "position": frame["position"],
            "rotation": frame["rotation"],
            "mapGridX": frame["mapGridX"],
            "mapGridZ": frame["mapGridZ"],
        }
        for frame in source.frames
        if frame["id"] in selected
    ]
    fixture = {
        "schemaVersion": FIXTURE_SCHEMA_VERSION,
        "kind": "visionReplayFixture",
        "inspection": inspection,
        "selection": selection,
        "selectedFrameRecords": selected_records,
        "splits": build_route_spatial_splits(
            source.frames,
            seed=seed,
            held_out_ratio=held_out_ratio,
        ),
        "augmentationPlan": build_augmentation_plan(selection["frameIds"], seed=seed),
        "leakagePolicy": {
            "accuracyRequiresMapBuiltFromTrainFramesOnly": True,
            "heldOutFramesMayNotContributeDescriptorsOrAnchors": True,
            "canonicalAllFrameReplayClassification": "exact-replay-smoke-only",
        },
        "evaluationPolicy": {
            "simpleResizeAllowed": False,
            "pureRandomPixelsExpectedAccepted": False,
            "singleFramesResetTracker": True,
            "windowsPreserveTimestampOrder": True,
        },
    }
    output_path = _write_output(output_dir, FIXTURE_FILE_NAME, fixture, overwrite=overwrite)
    return output_path, fixture


# Unity Y-up quaternion의 yaw를 계산한다.
def _yaw_degrees(rotation: Sequence[float]) -> float:
    quaternion = np.asarray(rotation, np.float64)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise VisionReplayBenchmarkError("quaternion 형식이 잘못되었습니다.")
    length = float(np.linalg.norm(quaternion))
    if length <= 1e-12:
        raise VisionReplayBenchmarkError("quaternion 길이가 0입니다.")
    x, y, z, w = quaternion / length
    forward_x = 2.0 * (x * z + w * y)
    forward_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.atan2(forward_x, forward_z))


# prediction 사례에서 위치, yaw, 성공률, false-accept 지표를 계산한다.
def calculate_metrics(
    cases: Sequence[dict[str, Any]],
    *,
    max_translation_error_meters: float = DEFAULT_MAX_TRANSLATION_ERROR_METERS,
    max_yaw_error_degrees: float = DEFAULT_MAX_YAW_ERROR_DEGREES,
) -> dict[str, Any]:
    if max_translation_error_meters <= 0.0 or max_yaw_error_degrees <= 0.0:
        raise VisionReplayBenchmarkError("pose 성공 임계값은 0보다 커야 합니다.")
    valid_count = 0
    accepted_count = 0
    success_count = 0
    invalid_count = 0
    false_accept_count = 0
    translation_errors: list[float] = []
    yaw_errors: list[float] = []
    per_case: list[dict[str, Any]] = []
    for index, case in enumerate(cases):
        if not isinstance(case, dict):
            raise VisionReplayBenchmarkError(f"prediction case {index}가 객체가 아닙니다.")
        case_id = str(case.get("caseId") or f"case-{index}")
        expected_valid = bool(case.get("expectedValid", True))
        accepted = bool(case.get("accepted", False))
        result: dict[str, Any] = {
            "caseId": case_id,
            "expectedValid": expected_valid,
            "accepted": accepted,
        }
        if not expected_valid:
            invalid_count += 1
            if accepted:
                false_accept_count += 1
            per_case.append(result)
            continue
        valid_count += 1
        if not accepted:
            per_case.append(result)
            continue
        accepted_count += 1
        try:
            expected_position = np.asarray(case["expectedPosition"], np.float64)
            predicted_position = np.asarray(case["predictedPosition"], np.float64)
            expected_rotation = np.asarray(case["expectedRotation"], np.float64)
            predicted_rotation = np.asarray(case["predictedRotation"], np.float64)
        except (KeyError, TypeError, ValueError) as error:
            raise VisionReplayBenchmarkError(f"accepted case pose가 잘못되었습니다: {case_id}") from error
        if (
            expected_position.shape != (3,)
            or predicted_position.shape != (3,)
            or expected_rotation.shape != (4,)
            or predicted_rotation.shape != (4,)
            or not np.isfinite(expected_position).all()
            or not np.isfinite(predicted_position).all()
        ):
            raise VisionReplayBenchmarkError(f"accepted case pose가 유효하지 않습니다: {case_id}")
        translation_error = float(np.linalg.norm(predicted_position - expected_position))
        yaw_delta = _yaw_degrees(predicted_rotation) - _yaw_degrees(expected_rotation)
        yaw_error = abs((yaw_delta + 180.0) % 360.0 - 180.0)
        translation_errors.append(translation_error)
        yaw_errors.append(yaw_error)
        within_thresholds = (
            translation_error <= max_translation_error_meters
            and yaw_error <= max_yaw_error_degrees
        )
        if within_thresholds:
            success_count += 1
        result["translationErrorMeters"] = translation_error
        result["yawErrorDegrees"] = yaw_error
        result["withinPoseThresholds"] = within_thresholds
        per_case.append(result)

    def distribution(values: list[float]) -> dict[str, float | None]:
        return {
            "median": float(np.median(values)) if values else None,
            "p95": float(np.percentile(values, 95)) if values else None,
            "maximum": max(values) if values else None,
        }

    return {
        "schemaVersion": FIXTURE_SCHEMA_VERSION,
        "kind": "visionReplayMetrics",
        "validCaseCount": valid_count,
        "acceptedCaseCount": accepted_count,
        "returnRate": accepted_count / valid_count if valid_count else None,
        "successfulCaseCount": success_count,
        "successRate": success_count / valid_count if valid_count else None,
        "poseThresholds": {
            "maxTranslationErrorMeters": max_translation_error_meters,
            "maxYawErrorDegrees": max_yaw_error_degrees,
        },
        "translationErrorMeters": distribution(translation_errors),
        "yawErrorDegrees": distribution(yaw_errors),
        "invalidCaseCount": invalid_count,
        "falseAcceptCount": false_accept_count,
        "falseAcceptRate": false_accept_count / invalid_count if invalid_count else None,
        "cases": per_case,
    }


# fixture의 accuracy gate와 수치 임계값을 함께 적용한다.
def build_verification_result(
    metrics: dict[str, Any],
    inspection: dict[str, Any],
    *,
    min_success_rate: float = DEFAULT_MIN_SUCCESS_RATE,
    max_translation_p95_meters: float = DEFAULT_MAX_TRANSLATION_ERROR_METERS,
    max_yaw_p95_degrees: float = DEFAULT_MAX_YAW_ERROR_DEGREES,
    max_false_accept_rate: float = DEFAULT_MAX_FALSE_ACCEPT_RATE,
) -> dict[str, Any]:
    if (
        not 0.0 <= min_success_rate <= 1.0
        or max_translation_p95_meters <= 0.0
        or max_yaw_p95_degrees <= 0.0
        or not 0.0 <= max_false_accept_rate <= 1.0
    ):
        raise VisionReplayBenchmarkError("score 합격 임계값이 유효하지 않습니다.")
    translation_p95 = metrics["translationErrorMeters"]["p95"]
    yaw_p95 = metrics["yawErrorDegrees"]["p95"]
    success_rate = metrics.get("successRate")
    false_accept_rate = metrics.get("falseAcceptRate")
    checks = {
        "successRate": success_rate is not None and success_rate >= min_success_rate,
        "translationP95": (
            translation_p95 is not None
            and translation_p95 <= max_translation_p95_meters
        ),
        "yawP95": yaw_p95 is not None and yaw_p95 <= max_yaw_p95_degrees,
        "falseAcceptRate": (
            false_accept_rate is not None
            and false_accept_rate <= max_false_accept_rate
        ),
    }
    thresholds = {
        "minSuccessRate": min_success_rate,
        "maxTranslationP95Meters": max_translation_p95_meters,
        "maxYawP95Degrees": max_yaw_p95_degrees,
        "maxFalseAcceptRate": max_false_accept_rate,
    }
    blocking_reasons = list(inspection.get("blockingReasons") or [])
    if int(metrics.get("validCaseCount", 0)) < 1:
        blocking_reasons.append("valid-cases-not-provided")
    if int(metrics.get("invalidCaseCount", 0)) < 1:
        blocking_reasons.append("invalid-cases-not-provided")
    accuracy_eligible = bool(inspection.get("accuracyEligible", False))
    if not accuracy_eligible or blocking_reasons:
        return {
            "status": "blocked",
            "passed": None,
            "accuracyEligible": False,
            "blockingReasons": list(dict.fromkeys(blocking_reasons)),
            "thresholds": thresholds,
            "checks": checks,
        }
    passed = all(checks.values())
    return {
        "status": "passed" if passed else "failed",
        "passed": passed,
        "accuracyEligible": True,
        "blockingReasons": [],
        "thresholds": thresholds,
        "checks": checks,
    }


# OpenCV world-to-camera pose를 Unity 카메라 pose로 변환한다.
def _cv_tcw_to_unity_pose(
    rotation_cw: np.ndarray,
    translation_cw: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    rotation_wc = np.asarray(rotation_cw, np.float64).T
    position = -rotation_wc @ np.asarray(translation_cw, np.float64).reshape(3)
    forward = rotation_wc[:, 2]
    forward /= max(float(np.linalg.norm(forward)), 1e-12)
    up = -rotation_wc[:, 1]
    up /= max(float(np.linalg.norm(up)), 1e-12)
    right = np.cross(up, forward)
    right /= max(float(np.linalg.norm(right)), 1e-12)
    up = np.cross(forward, right)
    unity_rotation = np.column_stack((right, up, forward))
    return position, matrix_to_quaternion(unity_rotation)


class SparseAnchorTrackerEvaluationSeam:
    """검증된 동일 카메라 계약에서 production tracker를 재사용한다."""

    def __init__(
        self,
        artifact: VisualMapArtifact,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
        *,
        source_image_size: tuple[int, int],
    ) -> None:
        expected_size = (artifact.identity.image_width, artifact.identity.image_height)
        if tuple(source_image_size) != expected_size:
            raise CameraContractMismatchError(
                "source/runtime 이미지 크기가 다릅니다. 단순 resize로 accuracy를 판정할 수 없습니다: "
                f"source={source_image_size[0]}x{source_image_size[1]}, "
                f"runtime={expected_size[0]}x{expected_size[1]}"
            )
        self.artifact = artifact
        self.camera_matrix = np.asarray(camera_matrix, np.float64).reshape(3, 3)
        self.distortion = np.asarray(distortion, np.float64).reshape(-1)
        self.width, self.height = source_image_size
        self._reset_tracker()

    # 각 독립 frame 또는 window 시작 전에 temporal 상태를 초기화한다.
    def _reset_tracker(self) -> None:
        self.tracker = SparseAnchorTracker(self.camera_matrix, self.distortion)
        self.tracker.load_persistent_map(self.artifact)

    # 하나의 BGR frame을 production SparseAnchorTracker로 평가한다.
    def evaluate_bgr(
        self,
        image: np.ndarray,
        frame: dict[str, Any],
        *,
        reset: bool = False,
    ) -> dict[str, Any]:
        if reset:
            self._reset_tracker()
        if image.shape != (self.height, self.width, 3):
            raise CameraContractMismatchError(
                "입력 프레임이 tracker camera contract와 다릅니다. resize 우회는 허용되지 않습니다."
            )
        gray = cv2.cvtColor(np.asarray(image, np.uint8), cv2.COLOR_BGR2GRAY)
        tracked = self.tracker.track(gray, True, [], prefer_temporal_track=False)
        expected_position = [float(frame["position"][axis]) for axis in ("x", "y", "z")]
        expected_rotation = [float(frame["rotation"][axis]) for axis in ("x", "y", "z", "w")]
        result: dict[str, Any] = {
            "caseId": str(frame["id"]),
            "expectedValid": True,
            "accepted": tracked is not None,
            "expectedPosition": expected_position,
            "expectedRotation": expected_rotation,
        }
        if tracked is not None:
            position, rotation = _cv_tcw_to_unity_pose(tracked.rotation_cw, tracked.translation_cw)
            result.update(
                {
                    "predictedPosition": position.tolist(),
                    "predictedRotation": rotation.tolist(),
                    "inlierCount": tracked.inlier_count,
                    "persistentInlierCount": tracked.persistent_inlier_count,
                    "reprojectionError": tracked.reprojection_error,
                }
            )
        return result


# archive JPEG를 geometry 변경 없이 BGR로 읽는다.
def _read_archive_bgr(
    archive: zipfile.ZipFile,
    frame: dict[str, Any],
) -> np.ndarray:
    image_path = str(frame.get("imagePath") or "")
    try:
        payload = archive.read(image_path)
    except KeyError as error:
        raise VisionReplayBenchmarkError(
            f"번들 이미지가 없습니다: {image_path}"
        ) from error
    image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise VisionReplayBenchmarkError(f"번들 JPEG를 해석할 수 없습니다: {image_path}")
    return image


# fixture의 base, photometric, invalid, 연속 window 사례를 실제 tracker 입력으로 만든다.
def evaluate_replay_cases(
    source: ReplaySource,
    fixture: dict[str, Any],
    evaluator: SparseAnchorTrackerEvaluationSeam,
) -> list[dict[str, Any]]:
    frames_by_id = {str(frame["id"]): frame for frame in source.frames}
    image_cache: dict[str, np.ndarray] = {}
    cases: list[dict[str, Any]] = []

    def frame_for(frame_id: str) -> dict[str, Any]:
        try:
            return frames_by_id[frame_id]
        except KeyError as error:
            raise VisionReplayBenchmarkError(
                f"fixture frame이 현재 source에 없습니다: {frame_id}"
            ) from error

    with zipfile.ZipFile(source.bundle_path, "r") as archive:
        def image_for(frame_id: str) -> np.ndarray:
            if frame_id not in image_cache:
                image_cache[frame_id] = _read_archive_bgr(
                    archive,
                    frame_for(frame_id),
                )
            return image_cache[frame_id]

        selection = fixture.get("selection") or {}
        for frame_id_value in selection.get("frameIds") or []:
            frame_id = str(frame_id_value)
            result = evaluator.evaluate_bgr(
                image_for(frame_id),
                frame_for(frame_id),
                reset=True,
            )
            result["caseId"] = f"{frame_id}:base"
            result["caseClass"] = "base-valid"
            cases.append(result)

        augmentation = fixture.get("augmentationPlan") or {}
        for case in augmentation.get("photometric") or []:
            if not isinstance(case, dict):
                raise VisionReplayBenchmarkError("photometric case 형식이 잘못되었습니다.")
            frame_id = str(case.get("frameId") or "")
            image = apply_photometric_augmentation(image_for(frame_id), case)
            result = evaluator.evaluate_bgr(
                image,
                frame_for(frame_id),
                reset=True,
            )
            result["caseId"] = str(case["caseId"])
            result["caseClass"] = "photometric-valid"
            cases.append(result)

        for case in augmentation.get("invalid") or []:
            if not isinstance(case, dict):
                raise VisionReplayBenchmarkError("invalid case 형식이 잘못되었습니다.")
            frame_id = str(case.get("frameId") or "")
            invalid_input = build_invalid_input(image_for(frame_id), case)
            invalid_image = (
                cv2.imdecode(np.frombuffer(invalid_input, np.uint8), cv2.IMREAD_COLOR)
                if isinstance(invalid_input, bytes)
                else invalid_input
            )
            if invalid_image is None:
                result = {
                    "caseId": str(case["caseId"]),
                    "expectedValid": False,
                    "accepted": False,
                }
            else:
                result = evaluator.evaluate_bgr(
                    invalid_image,
                    frame_for(frame_id),
                    reset=True,
                )
                result["caseId"] = str(case["caseId"])
                result["expectedValid"] = False
                result.pop("expectedPosition", None)
                result.pop("expectedRotation", None)
            result["caseClass"] = "invalid"
            cases.append(result)

        for window_index, window in enumerate(selection.get("windows") or []):
            if not isinstance(window, dict):
                raise VisionReplayBenchmarkError("window 형식이 잘못되었습니다.")
            for offset, frame_id_value in enumerate(window.get("frameIds") or []):
                frame_id = str(frame_id_value)
                result = evaluator.evaluate_bgr(
                    image_for(frame_id),
                    frame_for(frame_id),
                    reset=offset == 0,
                )
                result["caseId"] = f"window-{window_index}:{offset}:{frame_id}"
                result["caseClass"] = "temporal-window-valid"
                cases.append(result)
    return cases


# 검증된 동일 camera contract에서 production visual map과 archive를 실행한다.
def run_replay_fixture(
    fixture_path: Path,
    map_path: Path,
    calibration_path: Path,
    visual_map_manifest_path: Path,
    output_dir: Path,
    *,
    overwrite: bool = False,
) -> tuple[Path, dict[str, Any]]:
    fixture_path = fixture_path.resolve()
    fixture = _read_json(fixture_path)
    if fixture.get("kind") != "visionReplayFixture":
        raise VisionReplayBenchmarkError("visionReplayFixture JSON이 필요합니다.")
    inspection = fixture.get("inspection")
    if not isinstance(inspection, dict):
        raise VisionReplayBenchmarkError("fixture inspection이 없습니다.")
    replay_blockers = list(inspection.get("replayBlockingReasons") or [])
    if not inspection.get("replayEligible", False) or replay_blockers:
        raise CameraContractMismatchError(
            "RGB replay camera contract가 차단되었습니다: "
            + ", ".join(replay_blockers or ["replay-not-eligible"])
        )
    inputs = inspection.get("inputs") or {}
    try:
        source = load_replay_source(
            Path(str(inputs["bundle"])),
            Path(str(inputs["index"])),
        )
    except (KeyError, TypeError) as error:
        raise VisionReplayBenchmarkError("fixture 입력 경로가 없습니다.") from error
    if (
        source.bundle_sha256 != inputs.get("bundleSha256")
        or source.index_sha256 != inputs.get("indexSha256")
    ):
        raise VisionReplayBenchmarkError("fixture 입력 SHA-256이 현재 파일과 다릅니다.")

    calibration = _read_json(calibration_path.resolve())
    try:
        camera_matrix = np.asarray(calibration["camera_matrix"], np.float64).reshape(3, 3)
        distortion = np.asarray(
            calibration.get("distortion_coefficients", []),
            np.float64,
        ).reshape(-1)
    except (KeyError, TypeError, ValueError) as error:
        raise VisionReplayBenchmarkError("runtime calibration 형식이 잘못되었습니다.") from error
    try:
        expected_identity = load_visual_map_identity(
            map_path.resolve(),
            calibration_path.resolve(),
        )
    except VisualMapError as error:
        raise VisionReplayBenchmarkError(str(error)) from error
    manifest_path = visual_map_manifest_path.resolve()
    expected_manifest = inspection.get("visualMapIdentity") or {}
    if sha256_path(manifest_path) != expected_manifest.get("manifestSha256"):
        raise VisionReplayBenchmarkError("fixture visual-map manifest SHA-256이 다릅니다.")
    try:
        artifact = load_visual_map(
            manifest_path.with_name("visual-map-v1.npz"),
            manifest_path,
            expected_identity,
        )
    except VisualMapError as error:
        raise VisionReplayBenchmarkError(str(error)) from error
    evaluator = SparseAnchorTrackerEvaluationSeam(
        artifact,
        camera_matrix,
        distortion,
        source_image_size=(source.source_width, source.source_height),
    )
    cases = evaluate_replay_cases(source, fixture, evaluator)
    predictions = {
        "schemaVersion": FIXTURE_SCHEMA_VERSION,
        "kind": "visionReplayPredictions",
        "fixture": str(fixture_path),
        "fixtureSha256": sha256_path(fixture_path),
        "evaluationClassification": (
            "accuracy" if inspection.get("accuracyEligible") else "proxy-smoke"
        ),
        "cases": cases,
    }
    target = _write_output(
        output_dir,
        PREDICTIONS_FILE_NAME,
        predictions,
        overwrite=overwrite,
    )
    return target, predictions


# 격리된 출력 디렉터리만 허용하고 JSON 결과를 기록한다.
def _write_output(
    output_dir: Path,
    file_name: str,
    value: dict[str, Any],
    *,
    overwrite: bool,
) -> Path:
    output_dir = output_dir.resolve()
    lowered_parts = [part.lower() for part in output_dir.parts]
    for index in range(len(lowered_parts) - 1):
        if lowered_parts[index : index + 2] == [".team-runtime", "visual-maps"]:
            raise VisionReplayBenchmarkError("운영 .team-runtime/visual-maps를 output-dir로 사용할 수 없습니다.")
    output_dir.mkdir(parents=True, exist_ok=True)
    target = output_dir / file_name
    if target.exists() and not overwrite:
        raise VisionReplayBenchmarkError(f"출력 파일이 이미 있습니다: {target}")
    target.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return target


# CLI 공통 입력 인자를 추가한다.
def _add_source_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--index", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--visual-map-manifest", type=Path)
    parser.add_argument("--overwrite", action="store_true")


# inspect, compile, score 명령을 구성한다.
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="기록 RGB 기반 결정적 vision replay benchmark")
    commands = parser.add_subparsers(dest="command", required=True)
    inspect_parser = commands.add_parser("inspect")
    _add_source_arguments(inspect_parser)
    compile_parser = commands.add_parser("compile")
    _add_source_arguments(compile_parser)
    compile_parser.add_argument("--seed", type=int, default=20260811)
    compile_parser.add_argument("--frame-count", type=int, default=32)
    compile_parser.add_argument("--window-count", type=int, default=8)
    compile_parser.add_argument("--window-size", type=int, default=12)
    compile_parser.add_argument("--held-out-ratio", type=float, default=0.2)
    run_parser = commands.add_parser("run")
    run_parser.add_argument("--fixture", type=Path, required=True)
    run_parser.add_argument("--map-path", type=Path, required=True)
    run_parser.add_argument("--calibration", type=Path, required=True)
    run_parser.add_argument("--visual-map-manifest", type=Path, required=True)
    run_parser.add_argument("--output-dir", type=Path, required=True)
    run_parser.add_argument("--overwrite", action="store_true")
    score_parser = commands.add_parser("score")
    score_parser.add_argument("--fixture", type=Path, required=True)
    score_parser.add_argument("--predictions", type=Path, required=True)
    score_parser.add_argument("--output-dir", type=Path, required=True)
    score_parser.add_argument("--overwrite", action="store_true")
    score_parser.add_argument(
        "--min-success-rate",
        type=float,
        default=DEFAULT_MIN_SUCCESS_RATE,
    )
    score_parser.add_argument(
        "--max-translation-p95-meters",
        type=float,
        default=DEFAULT_MAX_TRANSLATION_ERROR_METERS,
    )
    score_parser.add_argument(
        "--max-yaw-p95-degrees",
        type=float,
        default=DEFAULT_MAX_YAW_ERROR_DEGREES,
    )
    score_parser.add_argument(
        "--max-false-accept-rate",
        type=float,
        default=DEFAULT_MAX_FALSE_ACCEPT_RATE,
    )
    return parser


# CLI 요청을 실행하고 생성한 JSON 경로를 반환한다.
def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "inspect":
        _, inspection = inspect_fixture(
            args.bundle,
            args.index,
            calibration_path=args.calibration,
            visual_map_manifest_path=args.visual_map_manifest,
        )
        target = _write_output(
            args.output_dir,
            INSPECTION_FILE_NAME,
            inspection,
            overwrite=args.overwrite,
        )
    elif args.command == "compile":
        target, _ = compile_fixture(
            args.bundle,
            args.index,
            args.output_dir,
            seed=args.seed,
            frame_count=args.frame_count,
            window_count=args.window_count,
            window_size=args.window_size,
            held_out_ratio=args.held_out_ratio,
            calibration_path=args.calibration,
            visual_map_manifest_path=args.visual_map_manifest,
            overwrite=args.overwrite,
        )
    elif args.command == "run":
        target, _ = run_replay_fixture(
            args.fixture,
            args.map_path,
            args.calibration,
            args.visual_map_manifest,
            args.output_dir,
            overwrite=args.overwrite,
        )
    else:
        fixture_path = args.fixture.resolve()
        fixture = _read_json(fixture_path)
        if fixture.get("kind") != "visionReplayFixture":
            raise VisionReplayBenchmarkError("visionReplayFixture JSON이 필요합니다.")
        inspection = fixture.get("inspection")
        if not isinstance(inspection, dict):
            raise VisionReplayBenchmarkError("fixture inspection이 없습니다.")
        predictions = _read_json(args.predictions)
        if predictions.get("kind") != "visionReplayPredictions":
            raise VisionReplayBenchmarkError("visionReplayPredictions JSON이 필요합니다.")
        fixture_sha256 = sha256_path(fixture_path)
        if predictions.get("fixtureSha256") != fixture_sha256:
            raise VisionReplayBenchmarkError("predictions와 fixture SHA-256이 다릅니다.")
        cases = predictions.get("cases")
        if not isinstance(cases, list):
            raise VisionReplayBenchmarkError("predictions JSON에 cases 배열이 필요합니다.")
        metrics = calculate_metrics(
            cases,
            max_translation_error_meters=args.max_translation_p95_meters,
            max_yaw_error_degrees=args.max_yaw_p95_degrees,
        )
        metrics["fixture"] = str(fixture_path)
        metrics["fixtureSha256"] = fixture_sha256
        metrics["evaluationClassification"] = (
            "accuracy" if inspection.get("accuracyEligible") else "proxy-smoke"
        )
        metrics["verification"] = build_verification_result(
            metrics,
            inspection,
            min_success_rate=args.min_success_rate,
            max_translation_p95_meters=args.max_translation_p95_meters,
            max_yaw_p95_degrees=args.max_yaw_p95_degrees,
            max_false_accept_rate=args.max_false_accept_rate,
        )
        target = _write_output(
            args.output_dir,
            METRICS_FILE_NAME,
            metrics,
            overwrite=args.overwrite,
        )
    print(target)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except VisionReplayBenchmarkError as error:
        raise SystemExit(f"vision replay benchmark 실패: {error}") from error
