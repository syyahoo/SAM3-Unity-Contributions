from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import zipfile
from pathlib import Path

import cv2
import numpy as np

from visual_map import (
    VisualMapArtifact,
    VisualMapError,
    load_visual_map,
    load_visual_map_identity,
    merge_visual_maps,
    save_visual_map,
    visual_map_grid_cells,
    visual_map_grid_cells_for_points,
    visual_map_paths,
)
from vision_adapter import (
    CAPTURE_MAX_ANCHORS,
    CAPTURE_MAX_KEYFRAMES,
    MAX_FEATURES,
    MIN_KEYFRAME_DISTANCE_METERS,
    RUNTIME_MAX_PERSISTENT_ANCHORS,
    SparseAnchorTracker,
)


INDEX_FORMAT_VERSION = 1
INDEX_FILE_NAME = "scan-frame-index-v1.json"
SUPPORTED_BUNDLE_VERSION = 1
SUPPORTED_VISUAL_VERSION = 2
MAXIMUM_PATH_STEP_METERS = 1.25
TRACKER_CAMERA_POINT_TO_UNITY_BASIS = np.diag([-1.0, -1.0, 1.0])
ALIGNMENT_FORMAT_VERSION = 1


class VenueVisualImportError(RuntimeError):
    pass


def _sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_stream(source) -> str:
    digest = hashlib.sha256()
    for chunk in iter(lambda: source.read(1024 * 1024), b""):
        digest.update(chunk)
    return digest.hexdigest()


def _read_json(archive: zipfile.ZipFile, path: str) -> dict:
    try:
        value = json.loads(archive.read(path).decode("utf-8"))
    except KeyError as error:
        raise VenueVisualImportError(f"번들 항목이 없습니다: {path}") from error
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise VenueVisualImportError(
            f"번들 JSON 형식이 올바르지 않습니다: {path}"
        ) from error
    if not isinstance(value, dict):
        raise VenueVisualImportError(f"번들 JSON 객체가 아닙니다: {path}")
    return value


# 번들의 CRC, 항목 목록, 모든 payload SHA-256을 검증한다.
def validate_bundle_integrity(archive: zipfile.ZipFile) -> None:
    names = [item.filename for item in archive.infolist()]
    if len(names) != len(set(names)):
        raise VenueVisualImportError("번들에 중복 ZIP 항목이 있습니다.")
    crc_error = archive.testzip()
    if crc_error is not None:
        raise VenueVisualImportError(f"번들 CRC 검증에 실패했습니다: {crc_error}")

    checksums = _read_json(archive, "checksums.json").get("files")
    if not isinstance(checksums, list):
        raise VenueVisualImportError("번들 checksum 목록이 없습니다.")
    try:
        checksum_paths = [str(item["path"]) for item in checksums]
    except (KeyError, TypeError) as error:
        raise VenueVisualImportError("번들 checksum 항목 형식이 올바르지 않습니다.") from error
    expected_paths = set(names) - {"checksums.json"}
    if (
        not all(checksum_paths)
        or len(checksum_paths) != len(set(checksum_paths))
        or set(checksum_paths) != expected_paths
    ):
        raise VenueVisualImportError(
            "번들 checksum 항목 목록이 ZIP 항목과 일치하지 않습니다."
        )

    for item in checksums:
        path = str(item["path"])
        expected = str(item.get("sha256") or "").lower()
        if len(expected) != 64 or any(character not in "0123456789abcdef" for character in expected):
            raise VenueVisualImportError(f"checksum SHA-256 형식이 올바르지 않습니다: {path}")
        with archive.open(path) as source:
            actual = _sha256_stream(source)
        if actual != expected:
            raise VenueVisualImportError(f"번들 checksum이 일치하지 않습니다: {path}")


def quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion, np.float64)
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(length) or length <= 1e-12:
        raise VenueVisualImportError("키프레임 quaternion이 유효하지 않습니다.")
    x, y, z, w = x / length, y / length, z / length, w / length
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, np.float64)
    if (
        matrix.shape != (3, 3)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6)
        or not math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=1e-6)
    ):
        raise VenueVisualImportError("Camera rotation is not a proper rigid rotation.")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray(
            (
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ),
            np.float64,
        )
    else:
        diagonal = np.diag(matrix)
        axis = int(np.argmax(diagonal))
        if axis == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.asarray(
                (
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                ),
                np.float64,
            )
        elif axis == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.asarray(
                (
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                ),
                np.float64,
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.asarray(
                (
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ),
                np.float64,
            )
    quaternion /= np.linalg.norm(quaternion)
    return quaternion


def _validate_operational_transform(transform: np.ndarray) -> np.ndarray:
    matrix = np.asarray(transform, np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise VenueVisualImportError("Operational alignment must be a finite 4x4 matrix.")
    rotation = matrix[:3, :3]
    if (
        not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-6)
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
        or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5)
        or not np.allclose(rotation @ np.asarray((0.0, 1.0, 0.0)), (0.0, 1.0, 0.0), atol=1e-5)
    ):
        raise VenueVisualImportError(
            "Operational alignment must be a gravity-preserving rigid transform."
        )
    return matrix


def load_operational_alignment(
    alignment_path: Path,
    archive: zipfile.ZipFile,
    manifest: dict,
) -> tuple[dict, np.ndarray]:
    try:
        alignment = json.loads(alignment_path.read_text(encoding="utf-8"))
        source_board = np.asarray(alignment["sourceBoardToVenue"], np.float64).reshape(4, 4)
        reference_board = np.asarray(
            alignment["referenceBoardToVenue"], np.float64
        ).reshape(4, 4)
        bundle_board = np.asarray(
            _read_json(archive, "registration.json")["boardToVenue"],
            np.float64,
        ).reshape(4, 4)
        transform = np.asarray(
            alignment["sourceToOperationalVenue"],
            np.float64,
        ).reshape(4, 4)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        raise VenueVisualImportError("Operational venue alignment is invalid.") from error
    if (
        int(alignment.get("formatVersion") or 0) != ALIGNMENT_FORMAT_VERSION
        or str(alignment.get("sourceScanId") or "") != str(manifest.get("scanId") or "")
        or not str(alignment.get("referenceMapId") or "")
        or not np.isfinite(source_board).all()
        or not np.isfinite(reference_board).all()
        or not np.allclose(source_board, bundle_board, rtol=0.0, atol=1e-5)
    ):
        raise VenueVisualImportError(
            "Operational venue alignment does not match the source bundle registration."
        )
    transform = _validate_operational_transform(transform)
    transformed_source = transform @ source_board
    transformed_x = transformed_source[:3, 0].copy()
    reference_x = reference_board[:3, 0].copy()
    transformed_x[1] = 0.0
    reference_x[1] = 0.0
    transformed_x /= np.linalg.norm(transformed_x)
    reference_x /= np.linalg.norm(reference_x)
    if (
        not np.allclose(transformed_source[:3, 3], reference_board[:3, 3], atol=1e-5)
        or not np.allclose(transformed_x, reference_x, atol=1e-5)
    ):
        raise VenueVisualImportError(
            "Operational alignment does not preserve the reference board origin and heading."
        )
    return alignment, transform


def transform_keyframes(
    keyframes: list[dict],
    source_to_operational: np.ndarray,
) -> list[dict]:
    transform = _validate_operational_transform(source_to_operational)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    transformed: list[dict] = []
    for source in keyframes:
        position = rotation @ _point(source.get("position")) + translation
        camera_rotation = rotation @ quaternion_to_matrix(_quaternion(source.get("rotation")))
        quaternion = matrix_to_quaternion(camera_rotation)
        item = dict(source)
        item["position"] = {
            axis: float(position[index])
            for index, axis in enumerate(("x", "y", "z"))
        }
        item["rotation"] = {
            axis: float(quaternion[index])
            for index, axis in enumerate(("x", "y", "z", "w"))
        }
        transformed.append(item)
    return transformed


# 원본 descriptor는 유지하고 기하 계산에 쓰는 u 좌표만 수평 반전한다.
def mirror_keypoint_geometry_x(points: np.ndarray, image_width: int) -> np.ndarray:
    result = np.asarray(points, np.float32).copy()
    if result.ndim != 2 or result.shape[1:] != (2,) or image_width < 1:
        raise VenueVisualImportError("기하 계산용 keypoint 형식이 올바르지 않습니다.")
    result[:, 0] = float(image_width - 1) - result[:, 0]
    return result


def _point(value: dict) -> np.ndarray:
    try:
        result = np.asarray([value[axis] for axis in ("x", "y", "z")], np.float64)
    except (KeyError, TypeError, ValueError) as error:
        raise VenueVisualImportError("키프레임 위치 형식이 올바르지 않습니다.") from error
    if not np.isfinite(result).all():
        raise VenueVisualImportError("키프레임 위치에 유한하지 않은 값이 있습니다.")
    return result


def _quaternion(value: dict) -> np.ndarray:
    try:
        result = np.asarray(
            [value[axis] for axis in ("x", "y", "z", "w")],
            np.float64,
        )
    except (KeyError, TypeError, ValueError) as error:
        raise VenueVisualImportError("키프레임 회전 형식이 올바르지 않습니다.") from error
    if not np.isfinite(result).all() or abs(np.linalg.norm(result) - 1.0) > 1e-3:
        raise VenueVisualImportError("키프레임 회전값이 정규화되어 있지 않습니다.")
    return result


def _camera_intrinsics(keyframe: dict) -> tuple[int, int, float, float, float, float]:
    try:
        values = (
            int(keyframe["imageWidth"]),
            int(keyframe["imageHeight"]),
            float(keyframe["focalLengthX"]),
            float(keyframe["focalLengthY"]),
            float(keyframe["principalPointX"]),
            float(keyframe["principalPointY"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise VenueVisualImportError(
            "키프레임 intrinsics 형식이 올바르지 않습니다."
        ) from error
    width, height, focal_x, focal_y, principal_x, principal_y = values
    if (
        width < 1
        or height < 1
        or focal_x <= 0.0
        or focal_y <= 0.0
        or not np.isfinite(values).all()
        or not 0.0 <= principal_x <= width
        or not 0.0 <= principal_y <= height
    ):
        raise VenueVisualImportError("키프레임 intrinsics 값이 유효하지 않습니다.")
    return values


def _validate_manifest_files(
    archive: zipfile.ZipFile,
    manifest: dict,
) -> None:
    records = manifest.get("files")
    if not isinstance(records, list):
        raise VenueVisualImportError("manifest 파일 목록이 없습니다.")
    try:
        paths = [str(item["path"]) for item in records]
    except (KeyError, TypeError) as error:
        raise VenueVisualImportError("manifest 파일 목록 형식이 올바르지 않습니다.") from error
    expected = set(archive.namelist()) - {"manifest.json", "checksums.json"}
    if len(paths) != len(set(paths)) or set(paths) != expected:
        raise VenueVisualImportError("manifest 파일 목록이 ZIP payload와 일치하지 않습니다.")
    for record in records:
        try:
            expected_size = int(record["size"])
            actual_size = archive.getinfo(str(record["path"])).file_size
        except (KeyError, TypeError, ValueError) as error:
            raise VenueVisualImportError("manifest 파일 크기 형식이 올바르지 않습니다.") from error
        if expected_size != actual_size:
            raise VenueVisualImportError(
                f"manifest 파일 크기가 일치하지 않습니다: {record['path']}"
            )


def _validate_visual_header(
    archive: zipfile.ZipFile,
    manifest: dict,
    visual: dict,
    identity,
) -> tuple[list[dict], float, float, float]:
    if (
        manifest.get("format") != "dronevenue"
        or int(manifest.get("version") or 0) != SUPPORTED_BUNDLE_VERSION
        or manifest.get("coordinateSystem") != "UnityLeftHandedYUp"
    ):
        raise VenueVisualImportError("지원하지 않는 Venue bundle manifest입니다.")
    _validate_manifest_files(archive, manifest)
    bundle_scan_id = str(manifest.get("scanId") or "")
    if bundle_scan_id != identity.scan_id:
        raise VenueVisualImportError(
            "번들 scanId가 현재 venue-map scanId와 일치하지 않습니다: "
            f"{bundle_scan_id}/{identity.scan_id}"
        )
    if (
        int(visual.get("version") or 0) != SUPPORTED_VISUAL_VERSION
        or visual.get("coordinateSystem") != "UnityLeftHandedYUp"
        or visual.get("imageOrientation") != "cameraNative"
        or visual.get("intrinsicsSpace") != "imagePixels"
    ):
        raise VenueVisualImportError("지원하지 않는 visual/keyframes.json 형식입니다.")
    try:
        scan_origin_x = float(visual["gridOriginX"])
        scan_origin_z = float(visual["gridOriginZ"])
        scan_cell_size = float(visual["gridCellSize"])
    except (KeyError, TypeError, ValueError) as error:
        raise VenueVisualImportError("visual grid 형식이 올바르지 않습니다.") from error
    if (
        not np.isfinite((scan_origin_x, scan_origin_z, scan_cell_size)).all()
        or scan_cell_size <= 0.0
        or not math.isclose(
            float(manifest.get("visualGridCellSize") or 0.0),
            scan_cell_size,
            rel_tol=0.0,
            abs_tol=1e-8,
        )
    ):
        raise VenueVisualImportError("manifest와 visual grid 값이 일치하지 않습니다.")
    visual_scan_id = str(visual.get("scanId") or "")
    source_scan_ids = manifest.get("sourceScanIds") or []
    if (
        not visual_scan_id
        or (
            visual_scan_id != bundle_scan_id
            and visual_scan_id not in source_scan_ids
        )
    ):
        raise VenueVisualImportError(
            "visual scanId가 manifest scanId 계보에 없습니다."
        )
    keyframes = visual.get("keyframes")
    if not isinstance(keyframes, list) or not keyframes:
        raise VenueVisualImportError("visual keyframe이 없습니다.")
    if int(manifest.get("visualKeyframeCount") or -1) != len(keyframes):
        raise VenueVisualImportError("manifest와 visual keyframe 수가 일치하지 않습니다.")
    return keyframes, scan_origin_x, scan_origin_z, scan_cell_size


# 번들의 bounds와 registration이 Unity가 생성한 현재 지도 계약과 같은지 검증한다.
def _validate_spatial_contract(
    archive: zipfile.ZipFile,
    manifest: dict,
    map_data: dict,
    identity,
    scan_cell_size: float,
    source_to_operational: np.ndarray | None = None,
) -> None:
    transform = _validate_operational_transform(
        np.eye(4, dtype=np.float64)
        if source_to_operational is None
        else source_to_operational
    )
    try:
        bounds = manifest["bounds"]
        source_minimum_x = float(bounds["minX"])
        source_maximum_x = float(bounds["maxX"])
        source_minimum_y = float(bounds["minY"])
        source_maximum_y = float(bounds["maxY"])
        source_minimum_z = float(bounds["minZ"])
        source_maximum_z = float(bounds["maxZ"])
        map_maximum_x = float(map_data["maxX"])
        map_maximum_z = float(map_data["maxZ"])
        map_board_to_venue = np.asarray(map_data["boardToVenue"], np.float64)
        bundle_board_to_venue = np.asarray(
            _read_json(archive, "registration.json")["boardToVenue"],
            np.float64,
        )
    except (KeyError, TypeError, ValueError) as error:
        raise VenueVisualImportError("Venue 공간 계약 형식이 올바르지 않습니다.") from error
    corners = np.asarray(
        [
            (x, y, z, 1.0)
            for x in (source_minimum_x, source_maximum_x)
            for y in (source_minimum_y, source_maximum_y)
            for z in (source_minimum_z, source_maximum_z)
        ],
        np.float64,
    )
    transformed_corners = (transform @ corners.T).T[:, :3]
    minimum_x = float(transformed_corners[:, 0].min())
    maximum_x = float(transformed_corners[:, 0].max())
    minimum_z = float(transformed_corners[:, 2].min())
    maximum_z = float(transformed_corners[:, 2].max())
    spatial_values = (
        minimum_x,
        maximum_x,
        minimum_z,
        maximum_z,
        map_maximum_x,
        map_maximum_z,
        scan_cell_size,
        identity.grid_origin_x,
        identity.grid_origin_z,
        identity.grid_cell_size_meters,
    )
    if not np.isfinite(spatial_values).all():
        raise VenueVisualImportError("Venue 공간 계약에 유한하지 않은 값이 있습니다.")
    expected_width = math.ceil((maximum_x - minimum_x) / scan_cell_size) + 1
    expected_height = math.ceil((maximum_z - minimum_z) / scan_cell_size) + 1
    if (
        not math.isclose(minimum_x, identity.grid_origin_x, abs_tol=1e-6)
        or not math.isclose(minimum_z, identity.grid_origin_z, abs_tol=1e-6)
        or not math.isclose(maximum_x, map_maximum_x, abs_tol=1e-6)
        or not math.isclose(maximum_z, map_maximum_z, abs_tol=1e-6)
        or not math.isclose(
            scan_cell_size,
            identity.grid_cell_size_meters,
            rel_tol=0.0,
            abs_tol=1e-8,
        )
        or expected_width != identity.grid_width
        or expected_height != identity.grid_height
    ):
        raise VenueVisualImportError(
            "번들 bounds/grid가 현재 venue-map 공간 계약과 일치하지 않습니다."
        )
    if (
        map_board_to_venue.shape != (16,)
        or bundle_board_to_venue.shape != (16,)
        or not np.isfinite(map_board_to_venue).all()
        or not np.isfinite(bundle_board_to_venue).all()
        or not np.allclose(
            map_board_to_venue.reshape(4, 4),
            transform @ bundle_board_to_venue.reshape(4, 4),
            rtol=0.0,
            atol=1e-5,
        )
    ):
        raise VenueVisualImportError(
            "번들 registration이 현재 venue-map boardToVenue와 일치하지 않습니다."
        )
    bundle_matrix = bundle_board_to_venue.reshape(4, 4)
    bundle_rotation = bundle_matrix[:3, :3]
    if (
        not np.allclose(bundle_matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-6)
        or not np.allclose(
            bundle_rotation.T @ bundle_rotation,
            np.eye(3),
            atol=1e-5,
        )
        or not math.isclose(
            float(np.linalg.det(bundle_rotation)),
            1.0,
            rel_tol=0.0,
            abs_tol=1e-5,
        )
    ):
        raise VenueVisualImportError("bundle boardToVenue가 유효한 강체 변환이 아닙니다.")


# 프레임의 위치에서 bundle signed 셀과 runtime map 셀을 각각 계산한다.
def build_frame_index(
    keyframes: list[dict],
    identity,
    scan_origin_x: float,
    scan_origin_z: float,
    scan_cell_size: float,
    *,
    source_bundle_sha256: str = "",
    source_to_operational: np.ndarray | None = None,
) -> tuple[list[dict], np.ndarray]:
    if scan_cell_size <= 0.0 or not math.isfinite(scan_cell_size):
        raise VenueVisualImportError("visual gridCellSize가 유효하지 않습니다.")
    source_positions = np.asarray([_point(item.get("position")) for item in keyframes])
    aligned_keyframes = transform_keyframes(
        keyframes,
        np.eye(4, dtype=np.float64)
        if source_to_operational is None
        else source_to_operational,
    )
    positions = np.asarray([_point(item.get("position")) for item in aligned_keyframes])
    map_cells = visual_map_grid_cells_for_points(positions, identity)
    records: list[dict] = []
    seen_ids: set[str] = set()
    seen_paths: set[str] = set()
    previous_timestamp = -math.inf
    for keyframe, aligned_keyframe, source_position, position, map_cell in zip(
        keyframes,
        aligned_keyframes,
        source_positions,
        positions,
        map_cells,
    ):
        try:
            frame_id = str(keyframe["id"])
            timestamp = float(keyframe["timestamp"])
            image_path = str(keyframe["imagePath"])
            stored_scan_cell = (int(keyframe["gridX"]), int(keyframe["gridZ"]))
        except (KeyError, TypeError, ValueError) as error:
            raise VenueVisualImportError("키프레임 인덱스 형식이 올바르지 않습니다.") from error
        if (
            not frame_id
            or frame_id in seen_ids
            or not image_path.startswith("visual/frames/")
            or image_path in seen_paths
        ):
            raise VenueVisualImportError(f"키프레임 ID 또는 imagePath가 유효하지 않습니다: {frame_id}")
        if not math.isfinite(timestamp) or timestamp < previous_timestamp:
            raise VenueVisualImportError("키프레임 timestamp가 단조 증가하지 않습니다.")
        _quaternion(keyframe.get("rotation"))
        scan_cell = (
            math.floor((float(source_position[0]) - scan_origin_x) / scan_cell_size),
            math.floor((float(source_position[2]) - scan_origin_z) / scan_cell_size),
        )
        if stored_scan_cell != scan_cell:
            raise VenueVisualImportError(
                f"키프레임 signed grid가 위치와 일치하지 않습니다: {frame_id}"
            )
        if int(map_cell[0]) < 0 or int(map_cell[1]) < 0:
            raise VenueVisualImportError(
                f"키프레임 위치가 현재 venue-map 범위를 벗어났습니다: {frame_id}"
            )
        records.append(
            {
                "sourceBundleSha256": source_bundle_sha256,
                "id": frame_id,
                "timestampSeconds": timestamp,
                "imagePath": image_path,
                "position": {
                    axis: float(position[index])
                    for index, axis in enumerate(("x", "y", "z"))
                },
                "sourcePosition": {
                    axis: float(source_position[index])
                    for index, axis in enumerate(("x", "y", "z"))
                },
                "rotation": {
                    axis: float(aligned_keyframe["rotation"][axis])
                    for axis in ("x", "y", "z", "w")
                },
                "sourceRotation": {
                    axis: float(keyframe["rotation"][axis])
                    for axis in ("x", "y", "z", "w")
                },
                "scanGridX": scan_cell[0],
                "scanGridZ": scan_cell[1],
                "mapGridX": int(map_cell[0]),
                "mapGridZ": int(map_cell[1]),
            }
        )
        seen_ids.add(frame_id)
        seen_paths.add(image_path)
        previous_timestamp = timestamp
    return records, map_cells


def _empty_artifact(identity) -> VisualMapArtifact:
    return VisualMapArtifact(
        identity=identity,
        anchor_points=np.empty((0, 3), np.float32),
        anchor_descriptors=np.empty((0, 32), np.uint8),
        anchor_support=np.empty((0,), np.uint16),
        keyframe_rotations_cw=np.empty((0, 3, 3), np.float64),
        keyframe_translations_cw=np.empty((0, 3), np.float64),
        duration_seconds=0.0,
        path_length_meters=0.0,
        anchor_conflicts=np.empty((0,), np.uint16),
        anchor_grid_cells=np.empty((0, 2), np.int32),
    )


def _chunk_artifact(
    tracker: SparseAnchorTracker,
    identity,
    venue_bounds_xz: tuple[float, float, float, float],
) -> tuple[VisualMapArtifact, int, int]:
    rotations = np.asarray(
        [item.rotation_cw for item in tracker.keyframes],
        np.float64,
    ).reshape(-1, 3, 3)
    translations = np.asarray(
        [item.translation_cw for item in tracker.keyframes],
        np.float64,
    ).reshape(-1, 3)
    preliminary = VisualMapArtifact(
        identity=identity,
        anchor_points=tracker.anchor_points,
        anchor_descriptors=tracker.anchor_descriptors,
        anchor_support=tracker.anchor_support,
        keyframe_rotations_cw=rotations,
        keyframe_translations_cw=translations,
        duration_seconds=0.0,
        path_length_meters=0.0,
        anchor_conflicts=tracker.anchor_conflicts,
    )
    cells = visual_map_grid_cells(preliminary)
    map_grid_valid = np.all(cells >= 0, axis=1)
    minimum_x, maximum_x, minimum_z, maximum_z = venue_bounds_xz
    venue_bounds_valid = (
        (preliminary.anchor_points[:, 0] >= minimum_x - 1e-5)
        & (preliminary.anchor_points[:, 0] <= maximum_x + 1e-5)
        & (preliminary.anchor_points[:, 2] >= minimum_z - 1e-5)
        & (preliminary.anchor_points[:, 2] <= maximum_z + 1e-5)
    )
    keep = map_grid_valid & venue_bounds_valid
    artifact = VisualMapArtifact(
        identity=identity,
        anchor_points=preliminary.anchor_points[keep],
        anchor_descriptors=preliminary.anchor_descriptors[keep],
        anchor_support=preliminary.anchor_support[keep],
        keyframe_rotations_cw=rotations,
        keyframe_translations_cw=translations,
        duration_seconds=0.0,
        path_length_meters=0.0,
        anchor_conflicts=preliminary.anchor_conflicts[keep],
        anchor_grid_cells=cells[keep],
    )
    return (
        artifact,
        int(np.count_nonzero(~map_grid_valid)),
        int(np.count_nonzero(map_grid_valid & ~venue_bounds_valid)),
    )


# 긴 스캔을 구간별로 삼각측량한 뒤 공간 균형을 유지하며 canonical 지도로 합친다.
def build_visual_artifact(
    archive: zipfile.ZipFile,
    keyframes: list[dict],
    identity,
    *,
    max_features: int,
    maximum_anchors: int,
    tracker_max_anchors: int,
    chunk_size: int,
    minimum_keyframe_distance: float,
    venue_bounds_xz: tuple[float, float, float, float],
) -> tuple[VisualMapArtifact, dict]:
    if chunk_size < 2 or maximum_anchors < 1 or tracker_max_anchors < maximum_anchors:
        raise VenueVisualImportError("시각 지도 용량 설정이 유효하지 않습니다.")
    reference_intrinsics = _camera_intrinsics(keyframes[0])
    width, height, focal_x, focal_y, principal_x, principal_y = reference_intrinsics
    camera_matrix = np.asarray(
        [
            [focal_x, 0.0, float(width - 1) - principal_x],
            [0.0, focal_y, principal_y],
            [0.0, 0.0, 1.0],
        ],
        np.float64,
    )
    combined = _empty_artifact(identity)
    accepted_keyframes = 0
    low_feature_frames = 0
    triangulated_anchors = 0
    map_grid_out_of_bounds_anchors = 0
    venue_bounds_out_of_bounds_anchors = 0
    chunk_count = 0

    for chunk_start in range(0, len(keyframes), chunk_size):
        chunk = keyframes[chunk_start : chunk_start + chunk_size]
        tracker = SparseAnchorTracker(
            camera_matrix,
            np.zeros(5, np.float64),
            max_features=max_features,
            max_anchors=tracker_max_anchors,
            max_keyframes=max(2, len(chunk)),
            minimum_keyframe_distance_meters=minimum_keyframe_distance,
            geometry_keypoint_transform=(
                lambda points: mirror_keypoint_geometry_x(points, width)
            ),
        )
        chunk_accepted = 0
        for keyframe in chunk:
            if _camera_intrinsics(keyframe) != reference_intrinsics:
                raise VenueVisualImportError(
                    "키프레임 intrinsics가 프레임 사이에서 변경되었습니다."
                )
            image_path = str(keyframe["imagePath"])
            try:
                encoded = np.frombuffer(archive.read(image_path), np.uint8)
            except KeyError as error:
                raise VenueVisualImportError(
                    f"키프레임 JPEG가 없습니다: {image_path}"
                ) from error
            gray = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
            if gray is None or gray.shape != (height, width):
                raise VenueVisualImportError(
                    f"키프레임 JPEG를 디코딩할 수 없습니다: {image_path}"
                )
            accepted = tracker.update_marker_keyframe(
                gray,
                _point(keyframe["position"]),
                _quaternion(keyframe["rotation"]),
                quaternion_to_matrix,
                [],
            )
            if accepted:
                chunk_accepted += 1
            if tracker.last_feature_count < 100:
                low_feature_frames += 1
        accepted_keyframes += chunk_accepted
        if chunk_accepted >= 2:
            tracker.rebuild_session_map()
        (
            chunk_map,
            chunk_map_grid_out_of_bounds,
            chunk_venue_bounds_out_of_bounds,
        ) = _chunk_artifact(tracker, identity, venue_bounds_xz)
        triangulated_anchors += tracker.anchor_count
        map_grid_out_of_bounds_anchors += chunk_map_grid_out_of_bounds
        venue_bounds_out_of_bounds_anchors += chunk_venue_bounds_out_of_bounds
        combined = merge_visual_maps(
            combined,
            chunk_map,
            maximum_anchors=maximum_anchors,
        )
        chunk_count += 1

    positions = np.asarray([_point(item["position"]) for item in keyframes])
    timestamps = np.asarray([float(item["timestamp"]) for item in keyframes], np.float64)
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    if not np.isfinite(steps).all() or np.any(steps > MAXIMUM_PATH_STEP_METERS):
        raise VenueVisualImportError("키프레임 경로에 비정상적인 이동 간격이 있습니다.")
    duration = max(0.0, float(timestamps[-1] - timestamps[0]))
    path_length = float(steps.sum()) if len(steps) else 0.0
    artifact = VisualMapArtifact(
        identity=identity,
        anchor_points=combined.anchor_points,
        anchor_descriptors=combined.anchor_descriptors,
        anchor_support=combined.anchor_support,
        keyframe_rotations_cw=combined.keyframe_rotations_cw,
        keyframe_translations_cw=combined.keyframe_translations_cw,
        duration_seconds=duration,
        path_length_meters=path_length,
        anchor_conflicts=combined.anchor_conflicts,
        anchor_grid_cells=visual_map_grid_cells(combined),
    )
    diagnostics = {
        "sourceFrameCount": len(keyframes),
        "chunkCount": chunk_count,
        "acceptedKeyframeCount": accepted_keyframes,
        "storedKeyframeCount": artifact.keyframe_count,
        "lowFeatureFrameCount": low_feature_frames,
        "triangulatedAnchorObservationCount": triangulated_anchors,
        "mapGridOutOfBoundsAnchorCount": map_grid_out_of_bounds_anchors,
        "venueBoundsOutOfBoundsAnchorCount": venue_bounds_out_of_bounds_anchors,
        "durationSeconds": duration,
        "pathLengthMeters": path_length,
        "sourceImageWidth": width,
        "sourceImageHeight": height,
        "sourceFocalLengthX": focal_x,
        "sourceFocalLengthY": focal_y,
        "sourcePrincipalPointX": principal_x,
        "sourcePrincipalPointY": principal_y,
        "sourceDistortionAssumption": "zeroCoefficientsNotPresentInBundleV1",
        "projectionCameraPointToUnityBasis": [
            list(row)
            for row in TRACKER_CAMERA_POINT_TO_UNITY_BASIS
        ],
        "projectionGeometryHorizontalMirror": True,
        "descriptorImageOrientation": "cameraNativeUnmirrored",
    }
    return artifact, diagnostics


def _write_json(path: Path, value: dict) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _archive_source_bundle(bundle_path: Path, staging: Path, bundle_sha256: str) -> Path:
    source_directory = staging / "sources"
    source_directory.mkdir(parents=True, exist_ok=True)
    archived = source_directory / f"{bundle_sha256}.dronevenue"
    shutil.copy2(bundle_path, archived)
    if _sha256_path(archived) != bundle_sha256:
        raise VenueVisualImportError("보관한 source bundle SHA-256이 일치하지 않습니다.")
    return archived


def import_venue_visual_map(
    bundle_path: Path,
    map_path: Path,
    calibration_path: Path,
    visual_map_root: Path,
    alignment_path: Path,
    *,
    max_features: int,
    maximum_anchors: int,
    tracker_max_anchors: int,
    chunk_size: int,
    minimum_keyframe_distance: float,
    minimum_keyframes: int,
    minimum_anchors: int,
) -> dict:
    bundle_path = bundle_path.resolve()
    map_path = map_path.resolve()
    calibration_path = calibration_path.resolve()
    visual_map_root = visual_map_root.resolve()
    alignment_path = alignment_path.resolve()
    if not bundle_path.is_file():
        raise VenueVisualImportError(f"Venue bundle이 없습니다: {bundle_path}")
    identity = load_visual_map_identity(map_path, calibration_path)
    try:
        map_data = json.loads(map_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise VenueVisualImportError("venue-map.json을 읽을 수 없습니다.") from error
    target_npz, target_manifest = visual_map_paths(visual_map_root, identity.map_id)
    target_directory = target_npz.parent
    bundle_sha256 = _sha256_path(bundle_path)
    if target_directory.exists():
        index_path = target_directory / INDEX_FILE_NAME
        if index_path.is_file():
            try:
                index = json.loads(index_path.read_text(encoding="utf-8"))
                imported = {
                    str(item.get("bundleSha256") or "")
                    for item in index.get("sources", [])
                }
            except (OSError, json.JSONDecodeError, TypeError):
                imported = set()
            if bundle_sha256 in imported:
                raise VenueVisualImportError("같은 source bundle이 이미 반영되었습니다.")
        raise VenueVisualImportError(
            f"기존 canonical visual map을 덮어쓰지 않습니다: {target_directory}"
        )

    with zipfile.ZipFile(bundle_path, "r") as archive:
        validate_bundle_integrity(archive)
        manifest = _read_json(archive, "manifest.json")
        visual = _read_json(archive, "visual/keyframes.json")
        alignment, source_to_operational = load_operational_alignment(
            alignment_path,
            archive,
            manifest,
        )
        keyframes, scan_origin_x, scan_origin_z, scan_cell_size = _validate_visual_header(
            archive,
            manifest,
            visual,
            identity,
        )
        _validate_spatial_contract(
            archive,
            manifest,
            map_data,
            identity,
            scan_cell_size,
            source_to_operational,
        )
        frame_records, map_cells = build_frame_index(
            keyframes,
            identity,
            scan_origin_x,
            scan_origin_z,
            scan_cell_size,
            source_bundle_sha256=bundle_sha256,
            source_to_operational=source_to_operational,
        )
        aligned_keyframes = transform_keyframes(keyframes, source_to_operational)
        artifact, diagnostics = build_visual_artifact(
            archive,
            aligned_keyframes,
            identity,
            max_features=max_features,
            maximum_anchors=maximum_anchors,
            tracker_max_anchors=tracker_max_anchors,
            chunk_size=chunk_size,
            minimum_keyframe_distance=minimum_keyframe_distance,
            venue_bounds_xz=(
                float(identity.grid_origin_x),
                float(map_data["maxX"]),
                float(identity.grid_origin_z),
                float(map_data["maxZ"]),
            ),
        )

    if artifact.keyframe_count < minimum_keyframes:
        raise VenueVisualImportError(
            f"시각 지도 keyframe이 부족합니다: {artifact.keyframe_count}/{minimum_keyframes}"
        )
    if artifact.anchor_count < minimum_anchors:
        raise VenueVisualImportError(
            f"시각 지도 anchor가 부족합니다: {artifact.anchor_count}/{minimum_anchors}"
        )

    staging = visual_map_root / f".{identity.map_id}.{os.getpid()}.tmp"
    if staging.exists():
        raise VenueVisualImportError(f"기존 임시 폴더가 있습니다: {staging}")
    visual_map_root.mkdir(parents=True, exist_ok=True)
    staging.mkdir()
    try:
        staging_npz = staging / target_npz.name
        staging_manifest = staging / target_manifest.name
        save_visual_map(staging_npz, staging_manifest, artifact)
        verified = load_visual_map(
            staging_npz,
            staging_manifest,
            identity,
            max_persistent_anchors=maximum_anchors,
        )
        if verified.anchor_count != artifact.anchor_count:
            raise VenueVisualImportError("저장 후 visual map anchor 수가 변경되었습니다.")
        archived_bundle = _archive_source_bundle(bundle_path, staging, bundle_sha256)
        unique_scan_cells = {
            (item["scanGridX"], item["scanGridZ"])
            for item in frame_records
        }
        unique_map_cells = {
            (item["mapGridX"], item["mapGridZ"])
            for item in frame_records
        }
        minimum_scan_x = min(cell[0] for cell in unique_scan_cells)
        maximum_scan_x = max(cell[0] for cell in unique_scan_cells)
        minimum_scan_z = min(cell[1] for cell in unique_scan_cells)
        maximum_scan_z = max(cell[1] for cell in unique_scan_cells)
        index = {
            "formatVersion": INDEX_FORMAT_VERSION,
            "mapId": identity.map_id,
            "mapVersion": identity.map_version,
            "scanId": identity.scan_id,
            "coordinateSystem": "UnityLeftHandedYUp",
            "operationalAlignment": {
                "formatVersion": int(alignment["formatVersion"]),
                "referenceMapId": str(alignment["referenceMapId"]),
                "sourceScanId": str(alignment["sourceScanId"]),
                "sourceToOperationalVenue": source_to_operational.reshape(-1).tolist(),
            },
            "sources": [
                {
                    "bundleSha256": bundle_sha256,
                    "bundleManifestScanId": str(manifest["scanId"]),
                    "visualScanId": str(visual["scanId"]),
                    "frameCount": len(frame_records),
                    "archivePath": archived_bundle.relative_to(staging).as_posix(),
                }
            ],
            "sourceCamera": {
                key: value
                for key, value in diagnostics.items()
                if key.startswith("source")
            },
            "projection": {
                "cameraPointToUnityBasis": diagnostics[
                    "projectionCameraPointToUnityBasis"
                ],
                "geometryHorizontalMirror": diagnostics[
                    "projectionGeometryHorizontalMirror"
                ],
                "descriptorImageOrientation": diagnostics[
                    "descriptorImageOrientation"
                ],
            },
            "scanGrid": {
                "originX": scan_origin_x,
                "originZ": scan_origin_z,
                "cellSizeMeters": scan_cell_size,
                "minimumCellX": minimum_scan_x,
                "maximumCellX": maximum_scan_x,
                "minimumCellZ": minimum_scan_z,
                "maximumCellZ": maximum_scan_z,
                "width": maximum_scan_x - minimum_scan_x + 1,
                "height": maximum_scan_z - minimum_scan_z + 1,
                "indexing": "floor((SourceVenueXZ-origin)/cellSize)",
            },
            "mapGrid": {
                "originX": identity.grid_origin_x,
                "originZ": identity.grid_origin_z,
                "cellSizeMeters": identity.grid_cell_size_meters,
                "width": identity.grid_width,
                "height": identity.grid_height,
                "indexing": "floor((VenueXZ-origin)/cellSize+1e-5)",
            },
            "frameCount": len(frame_records),
            "mappedFrameCount": int(np.count_nonzero(np.all(map_cells >= 0, axis=1))),
            "uniqueScanCellCount": len(unique_scan_cells),
            "uniqueMapCellCount": len(unique_map_cells),
            "frames": frame_records,
        }
        _write_json(staging / INDEX_FILE_NAME, index)
        if _sha256_path(archived_bundle) != bundle_sha256:
            raise VenueVisualImportError("frame index source bundle 검증에 실패했습니다.")
        target_directory.parent.mkdir(parents=True, exist_ok=True)
        staging.replace(target_directory)
    except Exception:
        if staging.exists():
            shutil.rmtree(staging)
        raise

    coverage_cells = {
        tuple(cell)
        for cell in visual_map_grid_cells(artifact)
        if np.all(cell >= 0)
    }
    return {
        "mapId": identity.map_id,
        "scanId": identity.scan_id,
        "visualMapNpz": str(target_npz),
        "visualMapManifest": str(target_manifest),
        "frameIndex": str(target_directory / INDEX_FILE_NAME),
        "sourceBundleSha256": bundle_sha256,
        "canonicalVisualMap": {
            **diagnostics,
            "anchorCount": artifact.anchor_count,
            "keyframeCount": artifact.keyframe_count,
            "coverageCellCount": len(coverage_cells),
        },
        "scanFrameIndex": {
            "frameCount": len(frame_records),
            "mappedFrameCount": int(np.count_nonzero(np.all(map_cells >= 0, axis=1))),
            "uniqueScanCellCount": len(unique_scan_cells),
            "uniqueMapCellCount": len(unique_map_cells),
        },
    }


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description=(
            "Import Quest venue RGB keyframes into the canonical StrikerPC visual map."
        )
    )
    parser.add_argument("bundle", type=Path)
    parser.add_argument(
        "--map-path",
        type=Path,
        default=root / "Assets" / "Maps" / "current" / "venue-map.json",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=(
            root
            / "UnityClient"
            / "Assets"
            / "StreamingAssets"
            / "Maps"
            / "tello-camera-calibration.json"
        ),
    )
    parser.add_argument(
        "--visual-map-root",
        type=Path,
        default=root / ".team-runtime" / "visual-maps",
    )
    parser.add_argument(
        "--alignment",
        type=Path,
        default=root / "Gateway" / "venue_operational_alignment.json",
    )
    parser.add_argument("--max-features", type=int, default=MAX_FEATURES)
    parser.add_argument(
        "--max-anchors",
        type=int,
        default=RUNTIME_MAX_PERSISTENT_ANCHORS,
    )
    parser.add_argument(
        "--tracker-max-anchors",
        type=int,
        default=CAPTURE_MAX_ANCHORS,
    )
    parser.add_argument("--chunk-size", type=int, default=CAPTURE_MAX_KEYFRAMES)
    parser.add_argument(
        "--minimum-keyframe-distance",
        type=float,
        default=MIN_KEYFRAME_DISTANCE_METERS,
    )
    parser.add_argument("--minimum-keyframes", type=int, default=12)
    parser.add_argument("--minimum-anchors", type=int, default=250)
    return parser.parse_args()


def main() -> int:
    arguments = parse_args()
    try:
        report = import_venue_visual_map(
            arguments.bundle,
            arguments.map_path,
            arguments.calibration,
            arguments.visual_map_root,
            arguments.alignment,
            max_features=arguments.max_features,
            maximum_anchors=arguments.max_anchors,
            tracker_max_anchors=arguments.tracker_max_anchors,
            chunk_size=arguments.chunk_size,
            minimum_keyframe_distance=arguments.minimum_keyframe_distance,
            minimum_keyframes=arguments.minimum_keyframes,
            minimum_anchors=arguments.minimum_anchors,
        )
    except (
        OSError,
        ValueError,
        zipfile.BadZipFile,
        cv2.error,
        VisualMapError,
        VenueVisualImportError,
    ) as error:
        print(f"VENUE_VISUAL_MAP_IMPORT FAILED: {error}")
        return 1
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
