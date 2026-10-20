from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


VISUAL_MAP_FORMAT_VERSION = 1
VENUE_POSE_CONVENTION_VERSION = 2
VISUAL_MAP_FILE_NAME = "visual-map-v1.npz"
VISUAL_MAP_MANIFEST_NAME = "visual-map-v1.json"
INCREMENTAL_MAP_FILE_NAME = "incremental-map-v2.npz"
INCREMENTAL_MAP_MANIFEST_NAME = "incremental-map-v2.json"
INCREMENTAL_MAP_HISTORY_DIRECTORY = "history-v2"
INCREMENTAL_MAP_GENERATION_LIMIT = 5


class VisualMapError(RuntimeError):
    pass


@dataclass(frozen=True)
class VisualMapIdentity:
    map_id: str
    map_version: int
    scan_id: str
    calibration_sha256: str
    image_width: int
    image_height: int
    grid_origin_x: float = 0.0
    grid_origin_z: float = 0.0
    grid_cell_size_meters: float = 1.0
    grid_width: int = 0
    grid_height: int = 0
    pose_convention_version: int = VENUE_POSE_CONVENTION_VERSION
    board_to_venue_sha256: str = ""


@dataclass(frozen=True)
class VisualMapArtifact:
    identity: VisualMapIdentity
    anchor_points: np.ndarray
    anchor_descriptors: np.ndarray
    anchor_support: np.ndarray
    keyframe_rotations_cw: np.ndarray
    keyframe_translations_cw: np.ndarray
    duration_seconds: float
    path_length_meters: float
    anchor_conflicts: np.ndarray | None = None
    anchor_grid_cells: np.ndarray | None = None
    persistent_anchor_count: int | None = None

    @property
    def anchor_count(self) -> int:
        return int(len(self.anchor_points))

    @property
    def keyframe_count(self) -> int:
        return int(len(self.keyframe_rotations_cw))


@dataclass(frozen=True)
class VisualMapQuality:
    anchor_count: int
    coverage_cell_count: int
    median_anchor_support: float
    score: float


def load_visual_map_identity(
    map_path: Path,
    calibration_path: Path,
) -> VisualMapIdentity:
    try:
        map_data = json.loads(map_path.read_text(encoding="utf-8"))
        calibration_data = json.loads(calibration_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise VisualMapError(f"시각 지도 식별 정보를 읽을 수 없습니다: {error}") from error
    try:
        identity = VisualMapIdentity(
            map_id=str(map_data["mapId"]),
            map_version=int(map_data["version"]),
            scan_id=str(map_data["scanId"]),
            calibration_sha256=_sha256(calibration_path),
            image_width=int(calibration_data["image_width"]),
            image_height=int(calibration_data["image_height"]),
            grid_origin_x=float(map_data.get("minX", 0.0)),
            grid_origin_z=float(map_data.get("minZ", 0.0)),
            grid_cell_size_meters=float(map_data.get("floorCellSize", 1.0)),
            grid_width=int(map_data.get("floorWidth", 0)),
            grid_height=int(map_data.get("floorHeight", 0)),
            board_to_venue_sha256=_board_to_venue_sha256(
                map_data.get("boardToVenue")
            ),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise VisualMapError("시각 지도 식별 정보 형식이 올바르지 않습니다.") from error
    if (
        not identity.map_id
        or not identity.scan_id
        or identity.map_version < 1
        or identity.image_width < 1
        or identity.image_height < 1
        or identity.grid_cell_size_meters <= 0.0
        or identity.grid_width < 0
        or identity.grid_height < 0
    ):
        raise VisualMapError("시각 지도 식별 정보 값이 올바르지 않습니다.")
    return identity


def visual_map_paths(root: Path, map_id: str) -> tuple[Path, Path]:
    directory = root / map_id
    return directory / VISUAL_MAP_FILE_NAME, directory / VISUAL_MAP_MANIFEST_NAME


def incremental_visual_map_paths(root: Path, map_id: str) -> tuple[Path, Path]:
    directory = root / map_id
    return directory / INCREMENTAL_MAP_FILE_NAME, directory / INCREMENTAL_MAP_MANIFEST_NAME


# 시각 앵커를 스캔 지도의 고정 X/Z 셀 좌표로 변환한다.
def visual_map_grid_cells(artifact: VisualMapArtifact) -> np.ndarray:
    return visual_map_grid_cells_for_points(
        artifact.anchor_points,
        artifact.identity,
    )


# Venue 위치 배열을 현재 스캔 지도의 고정 X/Z 셀 좌표로 변환한다.
def visual_map_grid_cells_for_points(
    points: np.ndarray,
    identity: VisualMapIdentity,
) -> np.ndarray:
    return _grid_cells_for_points(points, identity)


def evaluate_visual_map_quality(
    artifact: VisualMapArtifact,
    *,
    cell_size_meters: float = 1.0,
) -> VisualMapQuality:
    points = np.asarray(artifact.anchor_points, np.float32)
    support = np.asarray(artifact.anchor_support, np.uint16)
    if len(points):
        if _has_fixed_grid(artifact.identity):
            cells = visual_map_grid_cells(artifact)
        else:
            minimum = np.min(points[:, (0, 2)], axis=0)
            cells = np.floor(
                (points[:, (0, 2)] - minimum) / max(0.1, cell_size_meters)
            ).astype(np.int32)
        coverage = len(
            {
                (int(cell[0]), int(cell[1]))
                for cell in cells
                if int(cell[0]) >= 0 and int(cell[1]) >= 0
            }
        )
        median_support = float(np.median(support))
    else:
        coverage = 0
        median_support = 0.0
    score = (
        float(len(points))
        + float(coverage) * 25.0
        + min(20.0, median_support) * 5.0
    )
    return VisualMapQuality(len(points), coverage, median_support, score)


def is_visual_map_quality_acceptable(
    candidate: VisualMapArtifact,
    previous: VisualMapArtifact | None,
) -> bool:
    if previous is None:
        return True
    current = evaluate_visual_map_quality(candidate)
    baseline = evaluate_visual_map_quality(previous)
    return (
        current.anchor_count >= max(1, int(np.ceil(baseline.anchor_count * 0.85)))
        and current.coverage_cell_count
        >= max(1, int(np.ceil(baseline.coverage_cell_count * 0.80)))
        and current.score >= baseline.score * 0.90
    )


def merge_visual_maps(
    base: VisualMapArtifact,
    refinement: VisualMapArtifact,
    *,
    maximum_anchors: int,
    duplicate_distance_meters: float = 0.04,
    duplicate_descriptor_distance: float = 64.0,
) -> VisualMapArtifact:
    """기준 지도와 세션 보강 지도를 중복 없이 하나의 런타임 지도로 합친다."""
    _validate_arrays(base)
    _validate_arrays(refinement)
    if base.identity != refinement.identity:
        raise VisualMapError("병합할 시각 지도의 식별 정보가 일치하지 않습니다.")
    if maximum_anchors < 1:
        raise VisualMapError("병합 시각 지도의 최대 앵커 수는 양수여야 합니다.")

    base_persistent_count = _effective_persistent_anchor_count(base)
    points = [point.copy() for point in base.anchor_points]
    descriptors = [descriptor.copy() for descriptor in base.anchor_descriptors]
    support = [int(value) for value in base.anchor_support]
    conflicts = [int(value) for value in (
        base.anchor_conflicts
        if base.anchor_conflicts is not None
        else np.zeros(base.anchor_count, np.uint16)
    )]
    refinement_conflicts = (
        refinement.anchor_conflicts
        if refinement.anchor_conflicts is not None
        else np.zeros(refinement.anchor_count, np.uint16)
    )

    cell_size = max(0.001, duplicate_distance_meters)

    def cell_for(point: np.ndarray) -> tuple[int, int, int]:
        return tuple(int(value) for value in np.floor(point / cell_size))

    spatial_index: dict[tuple[int, int, int], list[int]] = {}
    for index, point in enumerate(points):
        spatial_index.setdefault(cell_for(point), []).append(index)

    for point, descriptor, observed_support, observed_conflicts in zip(
        refinement.anchor_points,
        refinement.anchor_descriptors,
        refinement.anchor_support,
        refinement_conflicts,
    ):
        merged_index = -1
        best_descriptor_distance = float("inf")
        cell = cell_for(point)
        for offset_x in (-1, 0, 1):
            for offset_y in (-1, 0, 1):
                for offset_z in (-1, 0, 1):
                    nearby_cell = (
                        cell[0] + offset_x,
                        cell[1] + offset_y,
                        cell[2] + offset_z,
                    )
                    for index in spatial_index.get(nearby_cell, ()):
                        if np.linalg.norm(points[index] - point) >= duplicate_distance_meters:
                            continue
                        descriptor_distance = cv2.norm(
                            descriptors[index],
                            descriptor,
                            cv2.NORM_HAMMING,
                        )
                        if descriptor_distance <= duplicate_descriptor_distance \
                                and descriptor_distance < best_descriptor_distance:
                            merged_index = index
                            best_descriptor_distance = descriptor_distance

        if merged_index >= 0:
            if (
                merged_index >= base_persistent_count
                and int(observed_support) >= support[merged_index]
            ):
                previous_cell = cell_for(points[merged_index])
                points[merged_index] = point.copy()
                descriptors[merged_index] = descriptor.copy()
                current_cell = cell_for(points[merged_index])
                if current_cell != previous_cell:
                    spatial_index[previous_cell].remove(merged_index)
                    spatial_index.setdefault(current_cell, []).append(merged_index)
            support[merged_index] = max(
                support[merged_index],
                int(observed_support),
            )
            conflicts[merged_index] = max(
                conflicts[merged_index],
                int(observed_conflicts),
            )
            continue

        new_index = len(points)
        points.append(point.copy())
        descriptors.append(descriptor.copy())
        support.append(int(observed_support))
        conflicts.append(int(observed_conflicts))
        spatial_index.setdefault(cell, []).append(new_index)

    points = np.asarray(points, np.float32).reshape(-1, 3)
    descriptors = np.asarray(descriptors, np.uint8).reshape(-1, 32)
    support = np.asarray(support, np.uint16)
    conflicts = np.asarray(conflicts, np.uint16)

    grid_cells = _grid_cells_for_points(points, base.identity)
    selected, selected_persistent_count = _select_with_persistent_prefix(
        points,
        support,
        maximum_anchors,
        base_persistent_count,
        grid_cells=grid_cells,
    )
    points = points[selected]
    descriptors = descriptors[selected]
    support = support[selected]
    conflicts = conflicts[selected]
    grid_cells = _grid_cells_for_points(points, base.identity)

    rotations = np.concatenate(
        (base.keyframe_rotations_cw, refinement.keyframe_rotations_cw),
        axis=0,
    )
    translations = np.concatenate(
        (base.keyframe_translations_cw, refinement.keyframe_translations_cw),
        axis=0,
    )
    merged = VisualMapArtifact(
        identity=base.identity,
        anchor_points=points,
        anchor_descriptors=descriptors,
        anchor_support=support,
        keyframe_rotations_cw=rotations,
        keyframe_translations_cw=translations,
        duration_seconds=max(base.duration_seconds, refinement.duration_seconds),
        path_length_meters=max(base.path_length_meters, refinement.path_length_meters),
        anchor_conflicts=conflicts,
        anchor_grid_cells=grid_cells,
        persistent_anchor_count=selected_persistent_count,
    )
    _validate_arrays(merged)
    return merged


def save_incremental_visual_map(
    npz_path: Path,
    manifest_path: Path,
    artifact: VisualMapArtifact,
    *,
    previous: VisualMapArtifact | None = None,
    retain_generations: int = INCREMENTAL_MAP_GENERATION_LIMIT,
) -> tuple[Path, Path]:
    if not is_visual_map_quality_acceptable(artifact, previous):
        candidate = evaluate_visual_map_quality(artifact)
        baseline = evaluate_visual_map_quality(previous)
        raise VisualMapError(
            "누적 지도 품질이 기존 지도보다 낮아 저장을 거부했습니다: "
            f"score={candidate.score:.1f}/{baseline.score:.1f}, "
            f"coverage={candidate.coverage_cell_count}/{baseline.coverage_cell_count}"
        )

    history = npz_path.parent / INCREMENTAL_MAP_HISTORY_DIRECTORY
    generation = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    generation_npz = history / f"incremental-map-{generation}.npz"
    generation_manifest = history / f"incremental-map-{generation}.json"
    save_visual_map(generation_npz, generation_manifest, artifact)
    save_visual_map(npz_path, manifest_path, artifact)
    _prune_incremental_generations(history, max(1, retain_generations))
    return generation_npz, generation_manifest


def load_incremental_visual_map(
    npz_path: Path,
    manifest_path: Path,
    expected: VisualMapIdentity,
    *,
    max_persistent_anchors: int = 2500,
) -> tuple[VisualMapArtifact, Path]:
    errors: list[str] = []
    if npz_path.exists() or manifest_path.exists():
        try:
            return (
                load_visual_map(
                    npz_path,
                    manifest_path,
                    expected,
                    max_persistent_anchors=max_persistent_anchors,
                ),
                manifest_path,
            )
        except VisualMapError as error:
            errors.append(f"current={error}")

    history = npz_path.parent / INCREMENTAL_MAP_HISTORY_DIRECTORY
    for candidate_manifest in sorted(
        history.glob("incremental-map-*.json"),
        reverse=True,
    ):
        candidate_npz = candidate_manifest.with_suffix(".npz")
        try:
            return (
                load_visual_map(
                    candidate_npz,
                    candidate_manifest,
                    expected,
                    max_persistent_anchors=max_persistent_anchors,
                ),
                candidate_manifest,
            )
        except VisualMapError as error:
            errors.append(f"{candidate_manifest.name}={error}")
    detail = "; ".join(errors) if errors else "저장된 세대가 없습니다."
    raise VisualMapError(f"사용 가능한 누적 지도를 찾지 못했습니다: {detail}")


def save_visual_map(
    npz_path: Path,
    manifest_path: Path,
    artifact: VisualMapArtifact,
) -> None:
    _validate_arrays(artifact)
    npz_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    npz_temp = npz_path.with_name(f".{npz_path.name}.{os.getpid()}.tmp")
    manifest_temp = manifest_path.with_name(
        f".{manifest_path.name}.{os.getpid()}.tmp"
    )
    try:
        grid_cells = visual_map_grid_cells(artifact)
        with npz_temp.open("wb") as output:
            np.savez_compressed(
                output,
                format_version=np.asarray([VISUAL_MAP_FORMAT_VERSION], np.uint16),
                anchor_points=np.asarray(artifact.anchor_points, np.float32),
                anchor_descriptors=np.asarray(artifact.anchor_descriptors, np.uint8),
                anchor_support=np.asarray(artifact.anchor_support, np.uint16),
                anchor_conflicts=np.asarray(
                    artifact.anchor_conflicts
                    if artifact.anchor_conflicts is not None
                    else np.zeros(artifact.anchor_count, np.uint16),
                    np.uint16,
                ),
                anchor_grid_cells=grid_cells,
                keyframe_rotations_cw=np.asarray(
                    artifact.keyframe_rotations_cw,
                    np.float64,
                ),
                keyframe_translations_cw=np.asarray(
                    artifact.keyframe_translations_cw,
                    np.float64,
                ),
            )
        npz_sha256 = _sha256(npz_temp)
        quality = evaluate_visual_map_quality(artifact)
        coverage_cells = sorted(
            {
                (int(cell[0]), int(cell[1]))
                for cell in grid_cells
                if int(cell[0]) >= 0 and int(cell[1]) >= 0
            }
        )
        manifest = {
            "formatVersion": VISUAL_MAP_FORMAT_VERSION,
            "poseConventionVersion": artifact.identity.pose_convention_version,
            "boardToVenueSha256": artifact.identity.board_to_venue_sha256,
            "mapId": artifact.identity.map_id,
            "mapVersion": artifact.identity.map_version,
            "scanId": artifact.identity.scan_id,
            "calibrationSha256": artifact.identity.calibration_sha256,
            "imageWidth": artifact.identity.image_width,
            "imageHeight": artifact.identity.image_height,
            "npzSha256": npz_sha256,
            "capturedAtUtc": datetime.now(timezone.utc).isoformat(),
            "durationSeconds": float(artifact.duration_seconds),
            "pathLengthMeters": float(artifact.path_length_meters),
            "keyframeCount": artifact.keyframe_count,
            "anchorCount": artifact.anchor_count,
            "persistentAnchorCount": _effective_persistent_anchor_count(artifact),
            "coverageCellCount": quality.coverage_cell_count,
            "medianAnchorSupport": quality.median_anchor_support,
            "qualityScore": quality.score,
            "scanGridOriginX": artifact.identity.grid_origin_x,
            "scanGridOriginZ": artifact.identity.grid_origin_z,
            "scanGridCellSizeMeters": artifact.identity.grid_cell_size_meters,
            "scanGridWidth": artifact.identity.grid_width,
            "scanGridHeight": artifact.identity.grid_height,
            "scanGridCoverageCells": [list(cell) for cell in coverage_cells],
        }
        manifest_temp.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(npz_temp, npz_path)
        os.replace(manifest_temp, manifest_path)
    finally:
        npz_temp.unlink(missing_ok=True)
        manifest_temp.unlink(missing_ok=True)


def load_visual_map(
    npz_path: Path,
    manifest_path: Path,
    expected: VisualMapIdentity,
    *,
    max_persistent_anchors: int = 2500,
) -> VisualMapArtifact:
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise VisualMapError("시각 지도 파일이 없습니다.") from error
    except (OSError, json.JSONDecodeError) as error:
        raise VisualMapError(f"시각 지도 manifest를 읽을 수 없습니다: {error}") from error
    expected_values = {
        "formatVersion": VISUAL_MAP_FORMAT_VERSION,
        "poseConventionVersion": expected.pose_convention_version,
        "boardToVenueSha256": expected.board_to_venue_sha256,
        "mapId": expected.map_id,
        "mapVersion": expected.map_version,
        "scanId": expected.scan_id,
        "calibrationSha256": expected.calibration_sha256,
        "imageWidth": expected.image_width,
        "imageHeight": expected.image_height,
    }
    for key, value in expected_values.items():
        if manifest.get(key) != value:
            raise VisualMapError(f"시각 지도 {key}가 현재 Venue와 일치하지 않습니다.")
    optional_grid_values = {
        "scanGridOriginX": expected.grid_origin_x,
        "scanGridOriginZ": expected.grid_origin_z,
        "scanGridCellSizeMeters": expected.grid_cell_size_meters,
        "scanGridWidth": expected.grid_width,
        "scanGridHeight": expected.grid_height,
    }
    for key, value in optional_grid_values.items():
        if key not in manifest:
            continue
        current = manifest.get(key)
        if isinstance(value, float):
            matches = isinstance(current, (int, float)) and np.isclose(current, value)
        else:
            matches = current == value
        if not matches:
            raise VisualMapError(f"시각 지도 {key}가 현재 Venue와 일치하지 않습니다.")
    persistent_anchor_count: int | None = None
    if "persistentAnchorCount" in manifest:
        current = manifest["persistentAnchorCount"]
        if isinstance(current, bool) or not isinstance(current, int):
            raise VisualMapError(
                "시각 지도 persistentAnchorCount 형식이 올바르지 않습니다."
            )
        persistent_anchor_count = current
    if not npz_path.is_file() or manifest.get("npzSha256") != _sha256(npz_path):
        raise VisualMapError("시각 지도 NPZ 해시가 일치하지 않습니다.")
    try:
        with np.load(npz_path, allow_pickle=False) as data:
            format_version = int(np.asarray(data["format_version"]).reshape(-1)[0])
            conflicts = (
                np.asarray(data["anchor_conflicts"], np.uint16)
                if "anchor_conflicts" in data.files
                else np.zeros(len(data["anchor_points"]), np.uint16)
            )
            grid_cells = (
                np.asarray(data["anchor_grid_cells"], np.int32)
                if "anchor_grid_cells" in data.files
                else None
            )
            artifact = VisualMapArtifact(
                identity=expected,
                anchor_points=np.asarray(data["anchor_points"], np.float32),
                anchor_descriptors=np.asarray(data["anchor_descriptors"], np.uint8),
                anchor_support=np.asarray(data["anchor_support"], np.uint16),
                keyframe_rotations_cw=np.asarray(
                    data["keyframe_rotations_cw"],
                    np.float64,
                ),
                keyframe_translations_cw=np.asarray(
                    data["keyframe_translations_cw"],
                    np.float64,
                ),
                duration_seconds=float(manifest.get("durationSeconds", 0.0)),
                path_length_meters=float(manifest.get("pathLengthMeters", 0.0)),
                anchor_conflicts=conflicts,
                anchor_grid_cells=grid_cells,
                persistent_anchor_count=persistent_anchor_count,
            )
    except (OSError, KeyError, ValueError) as error:
        raise VisualMapError(f"시각 지도 NPZ 형식이 올바르지 않습니다: {error}") from error
    if format_version != VISUAL_MAP_FORMAT_VERSION:
        raise VisualMapError("지원하지 않는 시각 지도 버전입니다.")
    _validate_arrays(artifact)
    expected_grid_cells = visual_map_grid_cells(artifact)
    if artifact.anchor_grid_cells is None:
        artifact = replace(artifact, anchor_grid_cells=expected_grid_cells)
    elif not np.array_equal(artifact.anchor_grid_cells, expected_grid_cells):
        raise VisualMapError("시각 지도 anchor grid 셀이 현재 스캔 지도와 일치하지 않습니다.")
    if (
        manifest.get("anchorCount") != artifact.anchor_count
        or manifest.get("keyframeCount") != artifact.keyframe_count
    ):
        raise VisualMapError("시각 지도 manifest 수량이 NPZ와 일치하지 않습니다.")
    if artifact.persistent_anchor_count is None:
        artifact = replace(
            artifact,
            persistent_anchor_count=artifact.anchor_count,
        )
    if artifact.anchor_count > max_persistent_anchors:
        order, selected_persistent_count = _select_with_persistent_prefix(
            artifact.anchor_points,
            artifact.anchor_support,
            max_persistent_anchors,
            _effective_persistent_anchor_count(artifact),
            grid_cells=artifact.anchor_grid_cells,
        )
        artifact = VisualMapArtifact(
            identity=artifact.identity,
            anchor_points=artifact.anchor_points[order],
            anchor_descriptors=artifact.anchor_descriptors[order],
            anchor_support=artifact.anchor_support[order],
            keyframe_rotations_cw=artifact.keyframe_rotations_cw,
            keyframe_translations_cw=artifact.keyframe_translations_cw,
            duration_seconds=artifact.duration_seconds,
            path_length_meters=artifact.path_length_meters,
            anchor_conflicts=(
                artifact.anchor_conflicts[order]
                if artifact.anchor_conflicts is not None
                else None
            ),
            anchor_grid_cells=(
                _grid_cells_for_points(
                    artifact.anchor_points[order],
                    artifact.identity,
                )
            ),
            persistent_anchor_count=selected_persistent_count,
        )
    return artifact


def _board_to_venue_sha256(raw_matrix: object) -> str:
    if raw_matrix is None:
        return ""
    try:
        matrix = np.asarray(raw_matrix, dtype="<f8")
    except (TypeError, ValueError) as error:
        raise VisualMapError("boardToVenue 형식이 올바르지 않습니다.") from error
    if matrix.shape != (16,) or not np.all(np.isfinite(matrix)):
        raise VisualMapError("boardToVenue 형식이 올바르지 않습니다.")
    return hashlib.sha256(matrix.tobytes()).hexdigest()


def _effective_persistent_anchor_count(artifact: VisualMapArtifact) -> int:
    if artifact.persistent_anchor_count is None:
        return artifact.anchor_count
    return int(artifact.persistent_anchor_count)


def _select_with_persistent_prefix(
    points: np.ndarray,
    support: np.ndarray,
    maximum_count: int,
    persistent_anchor_count: int,
    *,
    grid_cells: np.ndarray | None = None,
) -> tuple[np.ndarray, int]:
    """영구 앵커를 앞쪽에 보존하고 남은 용량만 세션 앵커로 채운다."""
    points = np.asarray(points, np.float32)
    support = np.asarray(support, np.uint16)
    cells = np.asarray(grid_cells, np.int32) if grid_cells is not None else None
    if maximum_count < 1:
        return np.empty((0,), np.int32), 0

    persistent_limit = min(persistent_anchor_count, maximum_count)
    persistent_order = _select_spatially_balanced_anchors(
        points[:persistent_anchor_count],
        support[:persistent_anchor_count],
        persistent_limit,
        grid_cells=(
            cells[:persistent_anchor_count]
            if cells is not None
            else None
        ),
    )
    remaining = maximum_count - len(persistent_order)
    session_order = _select_spatially_balanced_anchors(
        points[persistent_anchor_count:],
        support[persistent_anchor_count:],
        remaining,
        grid_cells=(
            cells[persistent_anchor_count:]
            if cells is not None
            else None
        ),
    )
    if len(session_order):
        session_order = session_order + persistent_anchor_count
    return (
        np.concatenate((persistent_order, session_order)).astype(np.int32),
        len(persistent_order),
    )


def _select_spatially_balanced_anchors(
    points: np.ndarray,
    support: np.ndarray,
    maximum_count: int,
    *,
    cell_size_meters: float = 1.0,
    grid_cells: np.ndarray | None = None,
) -> np.ndarray:
    """Keep strong anchors from every occupied X/Z cell before filling dense cells."""
    points = np.asarray(points, np.float32)
    support = np.asarray(support, np.uint16)
    if maximum_count < 1:
        return np.empty((0,), np.int32)
    if len(points) <= maximum_count:
        return np.arange(len(points), dtype=np.int32)

    if grid_cells is None:
        minimum = np.min(points[:, (0, 2)], axis=0)
        cells = np.floor(
            (points[:, (0, 2)] - minimum) / max(0.1, cell_size_meters)
        ).astype(np.int32)
    else:
        cells = np.asarray(grid_cells, np.int32)
        if cells.shape != (len(points), 2):
            raise VisualMapError("시각 지도 grid cell 형식이 올바르지 않습니다.")
    groups: dict[tuple[int, int], list[int]] = {}
    for index, cell in enumerate(cells):
        groups.setdefault((int(cell[0]), int(cell[1])), []).append(index)
    for indices in groups.values():
        indices.sort(key=lambda index: (-int(support[index]), index))

    selected: list[int] = []
    ordered_cells = sorted(groups)
    if len(ordered_cells) > maximum_count:
        # 셀 수가 cap보다 많아도 정렬 앞부분만 남기지 않고 전체 영역을 균등 표본화한다.
        cell_indices = np.linspace(
            0,
            len(ordered_cells) - 1,
            maximum_count,
            dtype=np.int64,
        )
        return np.asarray(
            sorted(groups[ordered_cells[int(index)]][0] for index in cell_indices),
            np.int32,
        )
    depth = 0
    while len(selected) < maximum_count:
        added = False
        for cell in ordered_cells:
            indices = groups[cell]
            if depth < len(indices):
                selected.append(indices[depth])
                added = True
                if len(selected) == maximum_count:
                    break
        if not added:
            break
        depth += 1
    return np.asarray(sorted(selected), np.int32)


def optimize_closed_loop_poses(
    rotations_cw: np.ndarray,
    translations_cw: np.ndarray,
    final_rotation_wc: np.ndarray,
    final_position: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    try:
        from scipy.optimize import least_squares
    except ImportError as error:
        raise VisualMapError("시각 지도 최적화에 scipy가 필요합니다.") from error
    rotations_cw = np.asarray(rotations_cw, np.float64)
    translations_cw = np.asarray(translations_cw, np.float64)
    if len(rotations_cw) < 2 or rotations_cw.shape != (len(rotations_cw), 3, 3):
        raise VisualMapError("폐루프 최적화에는 2개 이상의 유효한 pose가 필요합니다.")
    rotations_wc = np.transpose(rotations_cw, (0, 2, 1))
    positions = -np.einsum("nij,nj->ni", rotations_wc, translations_cw)
    final_rotation_wc = np.asarray(final_rotation_wc, np.float64).reshape(3, 3)
    final_position = np.asarray(final_position, np.float64).reshape(3)
    closure_position = float(np.linalg.norm(positions[-1] - final_position))
    closure_rotation = _rotation_degrees(rotations_wc[-1], final_rotation_wc)
    original_position_steps = np.diff(positions, axis=0)
    original_rotation_steps = [
        rotations_wc[index - 1].T @ rotations_wc[index]
        for index in range(1, len(rotations_wc))
    ]

    initial = []
    for index in range(1, len(positions)):
        initial.extend(positions[index])
        initial.extend(cv2.Rodrigues(rotations_wc[index])[0].reshape(3))
    initial = np.asarray(initial, np.float64)

    def unpack(values: np.ndarray) -> tuple[list[np.ndarray], list[np.ndarray]]:
        result_positions = [positions[0]]
        result_rotations = [rotations_wc[0]]
        for index in range(len(positions) - 1):
            offset = index * 6
            result_positions.append(values[offset : offset + 3])
            result_rotations.append(cv2.Rodrigues(values[offset + 3 : offset + 6])[0])
        return result_positions, result_rotations

    def residual(values: np.ndarray) -> np.ndarray:
        current_positions, current_rotations = unpack(values)
        terms: list[float] = []
        for index in range(1, len(current_positions)):
            terms.extend(
                ((current_positions[index] - current_positions[index - 1])
                - original_position_steps[index - 1])
                * 5.0
            )
            current_relative = current_rotations[index - 1].T @ current_rotations[index]
            rotation_error = original_rotation_steps[index - 1].T @ current_relative
            terms.extend(cv2.Rodrigues(rotation_error)[0].reshape(3) * 3.0)
        terms.extend((current_positions[-1] - final_position) * 20.0)
        terms.extend(
            cv2.Rodrigues(final_rotation_wc.T @ current_rotations[-1])[0].reshape(3)
            * 10.0
        )
        return np.asarray(terms, np.float64)

    result = least_squares(residual, initial, loss="soft_l1", max_nfev=300)
    if not result.success or not np.isfinite(result.x).all():
        raise VisualMapError(f"시각 지도 pose graph 최적화 실패: {result.message}")
    optimized_positions, optimized_rotations_wc = unpack(result.x)
    optimized_rotations_wc = np.asarray(optimized_rotations_wc, np.float64)
    optimized_rotations_cw = np.transpose(optimized_rotations_wc, (0, 2, 1))
    optimized_positions = np.asarray(optimized_positions, np.float64)
    optimized_translations_cw = -np.einsum(
        "nij,nj->ni",
        optimized_rotations_cw,
        optimized_positions,
    )
    return (
        optimized_rotations_cw,
        optimized_translations_cw,
        closure_position,
        closure_rotation,
    )


def _validate_arrays(artifact: VisualMapArtifact) -> None:
    points = np.asarray(artifact.anchor_points)
    descriptors = np.asarray(artifact.anchor_descriptors)
    support = np.asarray(artifact.anchor_support)
    conflicts = (
        np.asarray(artifact.anchor_conflicts)
        if artifact.anchor_conflicts is not None
        else np.zeros(len(points), np.uint16)
    )
    grid_cells = (
        np.asarray(artifact.anchor_grid_cells)
        if artifact.anchor_grid_cells is not None
        else None
    )
    rotations = np.asarray(artifact.keyframe_rotations_cw)
    translations = np.asarray(artifact.keyframe_translations_cw)
    if points.ndim != 2 or points.shape[1:] != (3,) or not np.isfinite(points).all():
        raise VisualMapError("시각 지도 anchor_points 형식이 올바르지 않습니다.")
    if descriptors.shape != (len(points), 32) or descriptors.dtype != np.uint8:
        raise VisualMapError("시각 지도 descriptor 형식이 올바르지 않습니다.")
    if support.shape != (len(points),):
        raise VisualMapError("시각 지도 anchor support 형식이 올바르지 않습니다.")
    if conflicts.shape != (len(points),):
        raise VisualMapError("시각 지도 anchor conflicts 형식이 올바르지 않습니다.")
    if grid_cells is not None and grid_cells.shape != (len(points), 2):
        raise VisualMapError("시각 지도 anchor grid cell 형식이 올바르지 않습니다.")
    persistent_anchor_count = artifact.persistent_anchor_count
    if persistent_anchor_count is not None and (
        isinstance(persistent_anchor_count, (bool, np.bool_))
        or not isinstance(persistent_anchor_count, (int, np.integer))
        or persistent_anchor_count < 0
        or persistent_anchor_count > len(points)
    ):
        raise VisualMapError(
            "시각 지도 persistent anchor 수가 anchor 범위를 벗어났습니다."
        )
    if rotations.ndim != 3 or rotations.shape[1:] != (3, 3):
        raise VisualMapError("시각 지도 keyframe rotation 형식이 올바르지 않습니다.")
    if translations.shape != (len(rotations), 3):
        raise VisualMapError("시각 지도 keyframe translation 형식이 올바르지 않습니다.")
    if not np.isfinite(rotations).all() or not np.isfinite(translations).all():
        raise VisualMapError("시각 지도 keyframe에 유한하지 않은 값이 있습니다.")


def _rotation_degrees(first: np.ndarray, second: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(first.T @ second) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _has_fixed_grid(identity: VisualMapIdentity) -> bool:
    return (
        identity.grid_cell_size_meters > 0.0
        and identity.grid_width > 0
        and identity.grid_height > 0
    )


def _grid_cells_for_points(
    points: np.ndarray,
    identity: VisualMapIdentity,
) -> np.ndarray:
    points = np.asarray(points, np.float32).reshape(-1, 3)
    if not len(points):
        return np.empty((0, 2), np.int32)
    if not _has_fixed_grid(identity):
        minimum = np.min(points[:, (0, 2)], axis=0)
        return np.floor(
            (points[:, (0, 2)] - minimum)
            / max(0.1, identity.grid_cell_size_meters)
        ).astype(np.int32)

    relative = np.column_stack(
        (
            points[:, 0] - identity.grid_origin_x,
            points[:, 2] - identity.grid_origin_z,
        )
    )
    cells = np.floor(
        relative / identity.grid_cell_size_meters + 1e-5
    ).astype(np.int32)
    valid = (
        (cells[:, 0] >= 0)
        & (cells[:, 0] < identity.grid_width)
        & (cells[:, 1] >= 0)
        & (cells[:, 1] < identity.grid_height)
    )
    cells[~valid] = -1
    return cells


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _prune_incremental_generations(history: Path, retain: int) -> None:
    for npz_path in history.glob("incremental-map-*.npz"):
        if not npz_path.with_suffix(".json").exists():
            npz_path.unlink(missing_ok=True)
    manifests = sorted(history.glob("incremental-map-*.json"), reverse=True)
    for manifest in manifests[retain:]:
        manifest.with_suffix(".npz").unlink(missing_ok=True)
        manifest.unlink(missing_ok=True)
