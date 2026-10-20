from __future__ import annotations

import argparse
import importlib.util
import inspect
import json
import math
import socket
import subprocess
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from vision_frame_tcp import LatestFrameTcpReader
from pose_protocol import (
    LOCAL_MAP_DEGRADED_FLAG,
    LOCAL_MAP_READY_FLAG,
    RELOCALIZING_FLAG,
    SESSION_MAP_FLAG,
)
from visual_map import (
    INCREMENTAL_MAP_HISTORY_DIRECTORY,
    VisualMapArtifact,
    VisualMapError,
    incremental_visual_map_paths,
    load_incremental_visual_map,
    load_visual_map,
    load_visual_map_identity,
    merge_visual_maps,
    optimize_closed_loop_poses,
    save_incremental_visual_map,
    save_visual_map,
    visual_map_paths,
)


EXPECTED_VISION_COMMIT = "0516d950e2f28ecc192b5cb221281a7bd1f4afe5"
MAX_FEATURES = 1500
MIN_KEYFRAME_FEATURES = 100
MAX_ANCHORS = 3000
MAX_KEYFRAMES = 30
CAPTURE_MAX_ANCHORS = 12000
CAPTURE_MAX_KEYFRAMES = 120
CAPTURE_MIN_KEYFRAME_DISTANCE_METERS = 0.5
RUNTIME_MAX_PERSISTENT_ANCHORS = 4500
RUNTIME_MAX_ANCHORS = RUNTIME_MAX_PERSISTENT_ANCHORS + MAX_ANCHORS
MIN_KEYFRAME_DISTANCE_METERS = 0.2
MIN_KEYFRAME_IMAGE_DISPLACEMENT_PIXELS = 4.0
MAX_CAPTURE_PATH_STEP_METERS = 1.25
MAX_ANCHOR_POSE_GAP_SECONDS = 0.2
MAX_ANCHOR_POSITION_DELTA_METERS = 0.75
MAX_ANCHOR_ROTATION_DELTA_DEGREES = 45.0
MAX_ANCHOR_POSITION_METERS_PER_SECOND = 4.0
MAX_ANCHOR_ROTATION_DEGREES_PER_SECOND = 180.0
MIN_ANCHOR_POSITION_DELTA_METERS = 0.08
MIN_ANCHOR_ROTATION_DELTA_DEGREES = 5.0
SEVERE_ANCHOR_REJECTION_RESET_STREAK = 3
MAP_READY_STREAK = 5
MAP_READY_MIN_INLIERS = 30
MAP_MAINTENANCE_MIN_INLIERS = 12
SESSION_BRIDGE_MIN_INLIERS = 12
BOARD_VALIDATED_CANONICAL_MIN_INLIERS = 12
MAP_READY_MAX_REPROJECTION_ERROR = 2.0
MAP_READY_PREPARATION_DISTANCE_METERS = 0.2
MAP_DEGRADED_SECONDS = 0.5
INCREMENTAL_SAVE_INTERVAL_SECONDS = 30.0
INCREMENTAL_KEYFRAME_CHECKPOINT_SECONDS = 5.0
DYNAMIC_ANCHOR_CONFLICT_THRESHOLD = 3
DYNAMIC_ANCHOR_MAX_CONFLICT_THRESHOLD = 20
COLD_START_MIN_INLIERS = 50
COLD_START_MAX_REPROJECTION_ERROR = 1.5
KLT_MAX_FORWARD_ERROR = 20.0
KLT_MAX_FORWARD_BACKWARD_ERROR_PIXELS = 1.5
DESCRIPTOR_REFRESH_MAX_POSITION_DELTA_METERS = 0.12
DESCRIPTOR_REFRESH_MAX_ROTATION_DELTA_DEGREES = 5.0
DESCRIPTOR_REFRESH_RESET_STREAK = 3
FALLBACK_FLOW_INTERVAL_SECONDS = 0.2
FALLBACK_FLOW_MAX_GAP_SECONDS = 0.6
FALLBACK_FLOW_MIN_TRACKED_POINTS = 60
FALLBACK_FLOW_MIN_INLIER_RATIO = 0.7
FALLBACK_FLOW_STATIONARY_MAX_PIXELS = 0.6
FALLBACK_FLOW_STATIONARY_MAX_ROTATION_DEGREES = 0.35
FALLBACK_FLOW_STATIONARY_MAX_SCALE_DELTA = 0.006
DEFAULT_APRILTAG_REFRESH_COOLDOWN_SECONDS = 3.0


# 보정 행렬과 다른 크기의 영상은 잘못된 pose를 만들기 전에 폐기한다.
def _frame_matches_calibration(
    frame: np.ndarray,
    expected_width: int,
    expected_height: int,
) -> bool:
    return (
        isinstance(frame, np.ndarray)
        and frame.ndim == 3
        and frame.shape[2] == 3
        and frame.shape[1] == expected_width
        and frame.shape[0] == expected_height
    )


def _feature_mask(
    shape: tuple[int, ...],
    boxes: list[tuple[int, int, int, int]],
) -> np.ndarray | None:
    if not boxes:
        return None
    mask = np.full(shape[:2], 255, np.uint8)
    height, width = shape[:2]
    for x1, y1, x2, y2 in boxes:
        margin = 12
        left = max(0, min(width, x1 - margin))
        top = max(0, min(height, y1 - margin))
        right = max(0, min(width, x2 + margin))
        bottom = max(0, min(height, y2 + margin))
        mask[top:bottom, left:right] = 0
    return mask


def _points_outside_boxes(
    points: np.ndarray,
    boxes: list[tuple[int, int, int, int]],
) -> np.ndarray:
    valid = np.ones(len(points), dtype=bool)
    for x1, y1, x2, y2 in boxes:
        valid &= ~(
            (points[:, 0] >= x1)
            & (points[:, 0] <= x2)
            & (points[:, 1] >= y1)
            & (points[:, 1] <= y2)
        )
    return valid


# 검출된 AprilTag 전체를 감싸는 영역을 환경 특징점 입력에서 제외한다.
def _detected_board_boxes(
    detections,
    shape: tuple[int, ...],
) -> list[tuple[int, int, int, int]]:
    try:
        detection_items = tuple(detections)
    except TypeError:
        return []
    corners = [
        np.asarray(item.corners, np.float32).reshape(-1, 2)
        for item in detection_items
        if getattr(item, "corners", None) is not None
    ]
    if not corners:
        return []
    points = np.concatenate(corners, axis=0)
    if not np.isfinite(points).all():
        return []
    height, width = shape[:2]
    left = max(0, min(width, int(math.floor(float(points[:, 0].min())))))
    top = max(0, min(height, int(math.floor(float(points[:, 1].min())))))
    right = max(0, min(width, int(math.ceil(float(points[:, 0].max())))))
    bottom = max(0, min(height, int(math.ceil(float(points[:, 1].max())))))
    if right <= left or bottom <= top:
        return []
    return [(left, top, right, bottom)]


# Unity 카메라 회전을 OpenCV world-to-camera 계산용 좌표축으로 복원한다.
def _unity_camera_rotation_to_cv_world(
    rotation: np.ndarray,
    quaternion_to_matrix,
) -> np.ndarray:
    return quaternion_to_matrix(rotation) @ np.diag([-1.0, -1.0, 1.0])


@dataclass
class VisualMeasurement:
    sequence: int
    source_timestamp_ns: int
    position: np.ndarray
    rotation: np.ndarray
    confidence: float
    anchor_count: int
    tag_count: int
    reprojection_error: float
    source: str
    flags: int = 0

    def to_json(self) -> bytes:
        return json.dumps(
            {
                "version": 1,
                "sequence": self.sequence,
                "sourceTimestampNs": self.source_timestamp_ns,
                "position": self.position.tolist(),
                "rotation": self.rotation.tolist(),
                "confidence": self.confidence,
                "anchorCount": self.anchor_count,
                "tagCount": self.tag_count,
                "reprojectionError": self.reprojection_error,
                "source": self.source,
                "flags": self.flags,
            },
            separators=(",", ":"),
        ).encode("utf-8")


@dataclass(frozen=True)
class FrameMotionValidation:
    sequence: int
    source_timestamp_ns: int
    interval_seconds: float
    tracked_points: int
    inlier_count: int
    inlier_ratio: float
    median_flow_x_pixels: float
    median_flow_y_pixels: float
    median_flow_pixels: float
    median_forward_backward_error_pixels: float
    affine_rotation_degrees: float
    affine_scale: float
    stationary: bool
    confidence: float

    def to_json(self) -> bytes:
        return json.dumps(
            {
                "version": 1,
                "kind": "frame-motion",
                "sequence": self.sequence,
                "sourceTimestampNs": self.source_timestamp_ns,
                "intervalSeconds": self.interval_seconds,
                "trackedPoints": self.tracked_points,
                "inlierCount": self.inlier_count,
                "inlierRatio": self.inlier_ratio,
                "medianFlowXPixels": self.median_flow_x_pixels,
                "medianFlowYPixels": self.median_flow_y_pixels,
                "medianFlowPixels": self.median_flow_pixels,
                "medianForwardBackwardErrorPixels": (
                    self.median_forward_backward_error_pixels
                ),
                "affineRotationDegrees": self.affine_rotation_degrees,
                "affineScale": self.affine_scale,
                "stationary": self.stationary,
                "confidence": self.confidence,
            },
            separators=(",", ":"),
        ).encode("utf-8")


class VisualLossMotionValidator:
    """일정 간격 영상의 움직임을 검증하되 절대 위치나 스케일은 만들지 않는다."""

    def __init__(
        self,
        interval_seconds: float = FALLBACK_FLOW_INTERVAL_SECONDS,
        maximum_gap_seconds: float = FALLBACK_FLOW_MAX_GAP_SECONDS,
    ) -> None:
        self.interval_seconds = max(0.05, float(interval_seconds))
        self.maximum_gap_seconds = max(
            self.interval_seconds,
            float(maximum_gap_seconds),
        )
        self._previous_gray: np.ndarray | None = None
        self._previous_dynamic_boxes: list[tuple[int, int, int, int]] = []
        self._previous_sequence = 0
        self._previous_timestamp_ns = 0

    def reset(self) -> None:
        self._previous_gray = None
        self._previous_dynamic_boxes = []
        self._previous_sequence = 0
        self._previous_timestamp_ns = 0

    def update(
        self,
        gray: np.ndarray,
        sequence: int,
        source_timestamp_ns: int,
        dynamic_boxes: list[tuple[int, int, int, int]],
    ) -> FrameMotionValidation | None:
        if (
            not isinstance(gray, np.ndarray)
            or gray.ndim != 2
            or sequence <= 0
            or source_timestamp_ns <= 0
        ):
            self.reset()
            return None

        if self._previous_gray is None:
            self._store_reference(
                gray,
                sequence,
                source_timestamp_ns,
                dynamic_boxes,
            )
            return None

        if sequence <= self._previous_sequence:
            self._store_reference(
                gray,
                sequence,
                source_timestamp_ns,
                dynamic_boxes,
            )
            return None

        interval_seconds = (
            source_timestamp_ns - self._previous_timestamp_ns
        ) / 1_000_000_000.0
        if interval_seconds < self.interval_seconds:
            return None

        previous_gray = self._previous_gray
        previous_boxes = self._previous_dynamic_boxes
        self._store_reference(
            gray,
            sequence,
            source_timestamp_ns,
            dynamic_boxes,
        )
        if (
            interval_seconds <= 0.0
            or interval_seconds > self.maximum_gap_seconds
            or previous_gray.shape != gray.shape
        ):
            return None

        try:
            points = cv2.goodFeaturesToTrack(
                previous_gray,
                maxCorners=400,
                qualityLevel=0.01,
                minDistance=8.0,
                mask=_feature_mask(previous_gray.shape, previous_boxes),
                blockSize=7,
            )
            if points is None or len(points) < FALLBACK_FLOW_MIN_TRACKED_POINTS:
                return None

            moved, status, forward_error = cv2.calcOpticalFlowPyrLK(
                previous_gray,
                gray,
                points,
                None,
                winSize=(21, 21),
                maxLevel=3,
                criteria=(
                    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                    30,
                    0.01,
                ),
            )
            if moved is None or status is None or forward_error is None:
                return None
            backward, backward_status, _ = cv2.calcOpticalFlowPyrLK(
                gray,
                previous_gray,
                moved,
                None,
                winSize=(21, 21),
                maxLevel=3,
                criteria=(
                    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                    30,
                    0.01,
                ),
            )
            if backward is None or backward_status is None:
                return None

            original = points.reshape(-1, 2)
            current = moved.reshape(-1, 2)
            returned = backward.reshape(-1, 2)
            forward_error = forward_error.reshape(-1)
            valid = (
                (status.reshape(-1) > 0)
                & (backward_status.reshape(-1) > 0)
                & np.isfinite(original).all(axis=1)
                & np.isfinite(current).all(axis=1)
                & np.isfinite(returned).all(axis=1)
                & np.isfinite(forward_error)
                & (forward_error <= KLT_MAX_FORWARD_ERROR)
            )
            forward_backward_error = np.linalg.norm(original - returned, axis=1)
            valid &= np.isfinite(forward_backward_error)
            valid &= (
                forward_backward_error
                <= KLT_MAX_FORWARD_BACKWARD_ERROR_PIXELS
            )
            valid &= _points_outside_boxes(current, dynamic_boxes)
            original = original[valid]
            current = current[valid]
            forward_backward_error = forward_backward_error[valid]
            tracked_points = int(len(original))
            if tracked_points < FALLBACK_FLOW_MIN_TRACKED_POINTS:
                return None

            affine, affine_inliers = cv2.estimateAffinePartial2D(
                original,
                current,
                method=cv2.RANSAC,
                ransacReprojThreshold=2.0,
                maxIters=1000,
                confidence=0.99,
                refineIters=10,
            )
            if affine is None or affine_inliers is None:
                return None
            inliers = affine_inliers.reshape(-1) > 0
            inlier_count = int(np.count_nonzero(inliers))
            inlier_ratio = inlier_count / max(1, tracked_points)
            if (
                inlier_count < FALLBACK_FLOW_MIN_TRACKED_POINTS
                or inlier_ratio < FALLBACK_FLOW_MIN_INLIER_RATIO
            ):
                return None

            deltas = current[inliers] - original[inliers]
            magnitudes = np.linalg.norm(deltas, axis=1)
            median_flow_x = float(np.median(deltas[:, 0]))
            median_flow_y = float(np.median(deltas[:, 1]))
            median_flow = float(np.median(magnitudes))
            median_forward_backward_error = float(
                np.median(forward_backward_error[inliers])
            )
            affine_a = float(affine[0, 0])
            affine_b = float(affine[1, 0])
            affine_scale = math.hypot(affine_a, affine_b)
            affine_rotation = math.degrees(math.atan2(affine_b, affine_a))
            stationary = (
                median_flow <= FALLBACK_FLOW_STATIONARY_MAX_PIXELS
                and abs(affine_rotation)
                <= FALLBACK_FLOW_STATIONARY_MAX_ROTATION_DEGREES
                and abs(affine_scale - 1.0)
                <= FALLBACK_FLOW_STATIONARY_MAX_SCALE_DELTA
            )
            confidence = float(
                np.clip(
                    min(1.0, inlier_count / 80.0)
                    * inlier_ratio
                    * (
                        1.0
                        - min(
                            1.0,
                            median_forward_backward_error
                            / KLT_MAX_FORWARD_BACKWARD_ERROR_PIXELS,
                        )
                    ),
                    0.0,
                    1.0,
                )
            )
            return FrameMotionValidation(
                sequence=sequence,
                source_timestamp_ns=source_timestamp_ns,
                interval_seconds=interval_seconds,
                tracked_points=tracked_points,
                inlier_count=inlier_count,
                inlier_ratio=inlier_ratio,
                median_flow_x_pixels=median_flow_x,
                median_flow_y_pixels=median_flow_y,
                median_flow_pixels=median_flow,
                median_forward_backward_error_pixels=(
                    median_forward_backward_error
                ),
                affine_rotation_degrees=affine_rotation,
                affine_scale=affine_scale,
                stationary=stationary,
                confidence=confidence,
            )
        except cv2.error:
            return None

    def _store_reference(
        self,
        gray: np.ndarray,
        sequence: int,
        source_timestamp_ns: int,
        dynamic_boxes: list[tuple[int, int, int, int]],
    ) -> None:
        self._previous_gray = gray.copy(order="C")
        self._previous_dynamic_boxes = list(dynamic_boxes)
        self._previous_sequence = sequence
        self._previous_timestamp_ns = source_timestamp_ns


@dataclass
class Keyframe:
    points: np.ndarray
    descriptors: np.ndarray
    rotation_cw: np.ndarray
    translation_cw: np.ndarray


@dataclass(frozen=True)
class TrackedPose:
    rotation_cw: np.ndarray
    translation_cw: np.ndarray
    reprojection_error: float
    inlier_count: int
    persistent_inlier_count: int

    @property
    def session_inlier_count(self) -> int:
        return max(0, self.inlier_count - self.persistent_inlier_count)


@dataclass(frozen=True)
class _TrackedPoseCandidate:
    pose: TrackedPose
    image_points: np.ndarray
    anchor_indices: np.ndarray


class SparseAnchorTracker:
    """AprilTag 기준 pose에서 삼각측량한 희소 앵커를 KLT/PnP로 추적한다."""

    def __init__(
        self,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
        *,
        max_features: int = MAX_FEATURES,
        max_anchors: int = MAX_ANCHORS,
        max_keyframes: int = MAX_KEYFRAMES,
        minimum_keyframe_distance_meters: float = MIN_KEYFRAME_DISTANCE_METERS,
        geometry_keypoint_transform: Callable[[np.ndarray], np.ndarray] | None = None,
    ) -> None:
        self.camera_matrix = camera_matrix
        self.distortion = distortion
        self.max_anchors = max_anchors
        self.max_keyframes = max(2, max_keyframes)
        self.minimum_keyframe_distance_meters = max(
            MIN_KEYFRAME_DISTANCE_METERS,
            minimum_keyframe_distance_meters,
        )
        self.geometry_keypoint_transform = geometry_keypoint_transform
        self.orb = cv2.ORB_create(nfeatures=max_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.keyframes: list[Keyframe] = []
        self.anchor_points = np.empty((0, 3), np.float32)
        self.anchor_descriptors = np.empty((0, 32), np.uint8)
        self.anchor_support = np.empty((0,), np.uint16)
        self.anchor_conflicts = np.empty((0,), np.uint16)
        self.persistent_anchor_count = 0
        self.previous_gray: np.ndarray | None = None
        self.tracked_pixels = np.empty((0, 2), np.float32)
        self.tracked_anchor_indices = np.empty((0,), np.int32)
        self.last_feature_count = 0
        self.last_keyframe_connected = False
        self.merged_anchor_observations = 0
        self.quarantined_anchor_count = 0
        self._descriptor_disagreement_streak = 0
        self._pending_supported_anchor_indices = np.empty((0,), np.int32)

    @property
    def anchor_count(self) -> int:
        return len(self.anchor_points)

    @property
    def session_anchor_count(self) -> int:
        return self.anchor_count - self.persistent_anchor_count

    def load_persistent_map(self, artifact: VisualMapArtifact) -> None:
        self.anchor_points = artifact.anchor_points.copy()
        self.anchor_descriptors = artifact.anchor_descriptors.copy()
        self.anchor_support = artifact.anchor_support.copy()
        self.anchor_conflicts = (
            artifact.anchor_conflicts.copy()
            if artifact.anchor_conflicts is not None
            else np.zeros(artifact.anchor_count, np.uint16)
        )
        artifact_persistent_count = getattr(
            artifact,
            "persistent_anchor_count",
            None,
        )
        self.persistent_anchor_count = (
            self.anchor_count
            if artifact_persistent_count is None
            else min(
                self.anchor_count,
                max(0, int(artifact_persistent_count)),
            )
        )
        self.keyframes.clear()
        self.previous_gray = None
        self._descriptor_disagreement_streak = 0
        self._invalidate_tracked_points()

    def update_marker_keyframe(
        self,
        gray: np.ndarray,
        position: np.ndarray,
        rotation: np.ndarray,
        quaternion_to_matrix,
        dynamic_boxes: list[tuple[int, int, int, int]],
        *,
        force: bool = False,
    ) -> bool:
        return self._update_keyframe(
            gray,
            position,
            rotation,
            quaternion_to_matrix,
            dynamic_boxes,
            force=force,
        )

    def update_tracked_keyframe(
        self,
        gray: np.ndarray,
        position: np.ndarray,
        rotation: np.ndarray,
        quaternion_to_matrix,
        dynamic_boxes: list[tuple[int, int, int, int]],
    ) -> bool:
        return self._update_keyframe(
            gray,
            position,
            rotation,
            quaternion_to_matrix,
            dynamic_boxes,
        )

    def _update_keyframe(
        self,
        gray: np.ndarray,
        position: np.ndarray,
        rotation: np.ndarray,
        quaternion_to_matrix,
        dynamic_boxes: list[tuple[int, int, int, int]],
        *,
        force: bool = False,
    ) -> bool:
        keypoints, descriptors = self.orb.detectAndCompute(
            gray,
            _feature_mask(gray.shape, dynamic_boxes),
        )
        self.last_feature_count = len(keypoints)
        self.last_keyframe_connected = False
        if descriptors is None or len(keypoints) < MIN_KEYFRAME_FEATURES:
            return False
        rotation_wc = _unity_camera_rotation_to_cv_world(
            rotation,
            quaternion_to_matrix,
        )
        rotation_cw = rotation_wc.T
        translation_cw = -rotation_cw @ position.reshape(3)
        geometry_points = np.float32([item.pt for item in keypoints])
        if self.geometry_keypoint_transform is not None:
            geometry_points = np.asarray(
                self.geometry_keypoint_transform(geometry_points.copy()),
                np.float32,
            )
            if (
                geometry_points.shape != (len(keypoints), 2)
                or not np.isfinite(geometry_points).all()
            ):
                raise ValueError("기하 계산용 keypoint 변환 결과가 유효하지 않습니다.")
        current = Keyframe(
            geometry_points,
            descriptors,
            rotation_cw,
            translation_cw,
        )
        distance = (
            self._keyframe_distance(current, self.keyframes[-1])
            if self.keyframes
            else math.inf
        )
        if self.keyframes and distance >= self.minimum_keyframe_distance_meters:
            previous = self.keyframes[-1]
            self.last_keyframe_connected = self._triangulate(previous, current) > 0
        if (
            not self.keyframes
            or force
            or (
                distance >= self.minimum_keyframe_distance_meters
                and self.last_keyframe_connected
            )
        ):
            self.keyframes.append(current)
            if len(self.keyframes) > self.max_keyframes:
                self.keyframes = [
                    self.keyframes[0],
                    *self.keyframes[-(self.max_keyframes - 1) :],
                ]
            return True
        return False

    def track(
        self,
        gray: np.ndarray,
        run_descriptor_match: bool,
        dynamic_boxes: list[tuple[int, int, int, int]],
        *,
        prefer_temporal_track: bool = False,
    ) -> TrackedPose | None:
        self._pending_supported_anchor_indices = np.empty((0,), np.int32)
        if self.anchor_count < 6:
            self.previous_gray = gray
            return None

        temporal_candidate = None
        if (
            self.previous_gray is not None
            and len(self.tracked_pixels) >= 6
            and len(self.tracked_anchor_indices) == len(self.tracked_pixels)
        ):
            previous_pixels = self.tracked_pixels.reshape(-1, 2)
            moved, status, forward_error = cv2.calcOpticalFlowPyrLK(
                self.previous_gray,
                gray,
                previous_pixels.reshape(-1, 1, 2),
                None,
                winSize=(21, 21),
                maxLevel=3,
            )
            valid = (
                status.reshape(-1).astype(bool)
                if status is not None and len(status) == len(previous_pixels)
                else np.zeros(len(previous_pixels), bool)
            )
            moved_pixels = (
                moved.reshape(-1, 2)
                if moved is not None and len(moved) == len(previous_pixels)
                else np.empty((0, 2), np.float32)
            )
            if len(moved_pixels) != len(previous_pixels):
                valid[:] = False
            else:
                height, width = gray.shape[:2]
                valid &= (
                    np.isfinite(moved_pixels).all(axis=1)
                    & (moved_pixels[:, 0] >= 0.0)
                    & (moved_pixels[:, 0] < width)
                    & (moved_pixels[:, 1] >= 0.0)
                    & (moved_pixels[:, 1] < height)
                )
            if len(moved_pixels) == len(previous_pixels) and forward_error is not None:
                errors = np.asarray(forward_error).reshape(-1)
                if len(errors) == len(valid):
                    valid &= np.isfinite(errors) & (errors <= KLT_MAX_FORWARD_ERROR)
            valid_indices = np.flatnonzero(valid)
            if len(valid_indices) >= 6:
                backward, backward_status, backward_lk_error = cv2.calcOpticalFlowPyrLK(
                    gray,
                    self.previous_gray,
                    moved_pixels[valid_indices].reshape(-1, 1, 2),
                    None,
                    winSize=(21, 21),
                    maxLevel=3,
                )
                backward_valid = (
                    backward_status.reshape(-1).astype(bool)
                    if backward_status is not None
                    and len(backward_status) == len(valid_indices)
                    else np.zeros(len(valid_indices), bool)
                )
                if backward is None or len(backward) != len(valid_indices):
                    backward_valid[:] = False
                else:
                    backward_pixels = backward.reshape(-1, 2)
                    backward_error = np.linalg.norm(
                        backward_pixels - previous_pixels[valid_indices],
                        axis=1,
                    )
                    backward_valid &= (
                        np.isfinite(backward_error)
                        & (
                            backward_error
                            <= KLT_MAX_FORWARD_BACKWARD_ERROR_PIXELS
                        )
                    )
                    if backward_lk_error is not None:
                        backward_errors = np.asarray(backward_lk_error).reshape(-1)
                        if len(backward_errors) == len(backward_valid):
                            backward_valid &= (
                                np.isfinite(backward_errors)
                                & (backward_errors <= KLT_MAX_FORWARD_ERROR)
                            )
                valid_indices = valid_indices[backward_valid]
            if len(valid_indices) >= 6:
                image_points = moved_pixels[valid_indices]
                anchor_indices = self.tracked_anchor_indices[valid_indices]
                static = _points_outside_boxes(image_points, dynamic_boxes)
                image_points = image_points[static]
                anchor_indices = anchor_indices[static]
                in_bounds = (anchor_indices >= 0) & (anchor_indices < self.anchor_count)
                if not in_bounds.all():
                    image_points = image_points[in_bounds]
                    anchor_indices = anchor_indices[in_bounds]
                    self._invalidate_tracked_points()
                temporal_candidate = self._solve_pose_candidate(
                    image_points,
                    anchor_indices,
                )

        descriptor_candidate = None
        temporal_track_ready = (
            temporal_candidate is not None
            and temporal_candidate.pose.inlier_count >= MAP_MAINTENANCE_MIN_INLIERS
            and temporal_candidate.pose.reprojection_error
            <= MAP_READY_MAX_REPROJECTION_ERROR
        )
        if not temporal_track_ready or run_descriptor_match:
            descriptor_points = self._descriptor_correspondences(gray, dynamic_boxes)
            if descriptor_points is not None:
                descriptor_candidate = self._solve_pose_candidate(*descriptor_points)

        self.previous_gray = gray
        chosen = temporal_candidate
        cache_source = temporal_candidate
        support_source = None
        invalidate_after_commit = False
        if descriptor_candidate is not None and (
            chosen is None
            or not prefer_temporal_track
            or not temporal_track_ready
        ):
            chosen = descriptor_candidate
            cache_source = descriptor_candidate
            support_source = descriptor_candidate
            self._descriptor_disagreement_streak = 0
        elif (
            temporal_track_ready
            and descriptor_candidate is not None
            and run_descriptor_match
        ):
            descriptor_refresh_good = (
                descriptor_candidate.pose.inlier_count
                >= MAP_MAINTENANCE_MIN_INLIERS
                and descriptor_candidate.pose.reprojection_error
                <= MAP_READY_MAX_REPROJECTION_ERROR
            )
            if descriptor_refresh_good and self._pose_candidates_are_consistent(
                temporal_candidate,
                descriptor_candidate,
            ):
                cache_source = descriptor_candidate
                support_source = descriptor_candidate
                self._descriptor_disagreement_streak = 0
            elif descriptor_refresh_good:
                self._descriptor_disagreement_streak += 1
                invalidate_after_commit = (
                    self._descriptor_disagreement_streak
                    >= DESCRIPTOR_REFRESH_RESET_STREAK
                )
        if chosen is None or cache_source is None:
            self._invalidate_tracked_points()
            return None
        self.tracked_pixels = cache_source.image_points.astype(np.float32)
        self.tracked_anchor_indices = cache_source.anchor_indices.astype(np.int32)
        if (
            support_source is not None
            and chosen.pose.inlier_count >= MAP_READY_MIN_INLIERS
            and chosen.pose.reprojection_error <= MAP_READY_MAX_REPROJECTION_ERROR
        ):
            self._pending_supported_anchor_indices = (
                support_source.anchor_indices.copy()
            )
        result = chosen.pose
        if invalidate_after_commit:
            self._invalidate_tracked_points(preserve_pending=True)
        return result

    # 현재 프레임의 ORB 특징을 지도 앵커와 중복 없이 대응시킨다.
    def _descriptor_correspondences(
        self,
        gray: np.ndarray,
        dynamic_boxes: list[tuple[int, int, int, int]],
    ) -> tuple[np.ndarray, np.ndarray] | None:
        keypoints, descriptors = self.orb.detectAndCompute(
            gray,
            _feature_mask(gray.shape, dynamic_boxes),
        )
        if descriptors is None or len(keypoints) < 12:
            return None
        matches = self.matcher.knnMatch(self.anchor_descriptors, descriptors, k=2)
        candidates = [
            first
            for pair in matches
            if len(pair) == 2
            for first, second in [pair]
            if first.distance < 0.75 * second.distance
        ]
        by_image_feature = {}
        for candidate in candidates:
            previous = by_image_feature.get(candidate.trainIdx)
            if previous is None or candidate.distance < previous.distance:
                by_image_feature[candidate.trainIdx] = candidate
        good = list(by_image_feature.values())
        if len(good) < 6:
            return None
        return (
            np.float32([keypoints[item.trainIdx].pt for item in good]),
            np.asarray([item.queryIdx for item in good], np.int32),
        )

    # 한 대응 집합의 PnP를 풀되 tracker 상태는 확정하지 않는다.
    def _solve_pose_candidate(
        self,
        image_points: np.ndarray,
        anchor_indices: np.ndarray,
    ) -> _TrackedPoseCandidate | None:
        if len(anchor_indices) < 6:
            return None
        object_points = self.anchor_points[anchor_indices]
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            self.camera_matrix,
            self.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=2.0,
            iterationsCount=80,
            confidence=0.99,
        )
        if not success or inliers is None or len(inliers) < 6:
            return None
        selected = inliers.reshape(-1)
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points[selected],
            image_points[selected],
            self.camera_matrix,
            self.distortion,
            rvec,
            tvec,
        )
        projected, _ = cv2.projectPoints(
            object_points[selected],
            rvec,
            tvec,
            self.camera_matrix,
            self.distortion,
        )
        error = float(
            np.sqrt(
                np.mean(
                    np.sum(
                        (projected.reshape(-1, 2) - image_points[selected]) ** 2,
                        axis=1,
                    )
                )
            )
        )
        selected_indices = anchor_indices[selected]
        rotation_cw, _ = cv2.Rodrigues(rvec)
        persistent_inliers = int(
            np.count_nonzero(selected_indices < self.persistent_anchor_count)
        )
        return _TrackedPoseCandidate(
            TrackedPose(
                rotation_cw,
                tvec.reshape(3),
                error,
                len(selected),
                persistent_inliers,
            ),
            image_points[selected].astype(np.float32),
            selected_indices.astype(np.int32),
        )

    # KLT와 ORB의 독립 PnP가 같은 카메라 pose일 때만 ORB로 추적점을 재시드한다.
    @staticmethod
    def _pose_candidates_are_consistent(
        temporal: _TrackedPoseCandidate,
        descriptor: _TrackedPoseCandidate,
    ) -> bool:
        temporal_position = (
            -temporal.pose.rotation_cw.T @ temporal.pose.translation_cw
        )
        descriptor_position = (
            -descriptor.pose.rotation_cw.T @ descriptor.pose.translation_cw
        )
        position_delta = float(
            np.linalg.norm(temporal_position - descriptor_position)
        )
        relative_rotation = (
            temporal.pose.rotation_cw.T @ descriptor.pose.rotation_cw
        )
        cosine = float(
            np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0)
        )
        rotation_delta = float(np.degrees(np.arccos(cosine)))
        return (
            position_delta <= DESCRIPTOR_REFRESH_MAX_POSITION_DELTA_METERS
            and rotation_delta <= DESCRIPTOR_REFRESH_MAX_ROTATION_DELTA_DEGREES
        )

    # 엄격한 현재 pose에서 관측된 앵커만 저장용 support에 반영한다.
    def _record_supported_anchors(self, anchor_indices: np.ndarray) -> None:
        if len(self.anchor_support) != self.anchor_count:
            self.anchor_support = np.ones(self.anchor_count, np.uint16)
        supported = np.unique(anchor_indices)
        self.anchor_support[supported] = np.minimum(
            65535,
            self.anchor_support[supported].astype(np.uint32) + 1,
        ).astype(np.uint16)
        if len(self.anchor_conflicts) != self.anchor_count:
            self.anchor_conflicts = np.zeros(self.anchor_count, np.uint16)
        self.anchor_conflicts[supported] = np.maximum(
            0,
            self.anchor_conflicts[supported].astype(np.int32) - 1,
        ).astype(np.uint16)

    # 외부 pose 연속성과 보드 정합까지 통과한 descriptor 관측만 support에 확정한다.
    def confirm_tracked_pose(self) -> None:
        indices = self._pending_supported_anchor_indices
        self._pending_supported_anchor_indices = np.empty((0,), np.int32)
        if not len(indices):
            return
        in_bounds = (indices >= 0) & (indices < self.anchor_count)
        if not in_bounds.all():
            return
        self._record_supported_anchors(indices)

    def _triangulate(self, first: Keyframe, second: Keyframe) -> int:
        matches = self.matcher.knnMatch(first.descriptors, second.descriptors, k=2)
        reverse_matches = self.matcher.knnMatch(
            second.descriptors,
            first.descriptors,
            k=2,
        )
        reverse_best = {
            left.queryIdx: left.trainIdx
            for pair in reverse_matches
            if len(pair) == 2
            for left, right in [pair]
            if left.distance < 0.70 * right.distance
        }
        good = [
            left
            for pair in matches
            if len(pair) == 2
            for left, right in [pair]
            if left.distance < 0.70 * right.distance
            and reverse_best.get(left.trainIdx) == left.queryIdx
        ]
        if len(good) < 20:
            return 0
        points_first = np.float32([first.points[item.queryIdx] for item in good])
        points_second = np.float32([second.points[item.trainIdx] for item in good])
        if (
            float(np.median(np.linalg.norm(points_second - points_first, axis=1)))
            < MIN_KEYFRAME_IMAGE_DISPLACEMENT_PIXELS
        ):
            return 0
        normalized_first = cv2.undistortPoints(
            points_first.reshape(-1, 1, 2),
            self.camera_matrix,
            self.distortion,
        ).reshape(-1, 2)
        normalized_second = cv2.undistortPoints(
            points_second.reshape(-1, 1, 2),
            self.camera_matrix,
            self.distortion,
        ).reshape(-1, 2)
        projection_first = np.column_stack((first.rotation_cw, first.translation_cw))
        projection_second = np.column_stack((second.rotation_cw, second.translation_cw))
        homogeneous = cv2.triangulatePoints(
            projection_first,
            projection_second,
            normalized_first.T,
            normalized_second.T,
        )
        homogeneous_scale = homogeneous[3]
        nonzero_scale = np.abs(homogeneous_scale) > 1e-9
        points = np.full((len(good), 3), np.nan, np.float64)
        points[nonzero_scale] = (
            homogeneous[:3, nonzero_scale] / homogeneous_scale[nonzero_scale]
        ).T
        depth_first = (first.rotation_cw @ points.T + first.translation_cw[:, None])[2]
        depth_second = (second.rotation_cw @ points.T + second.translation_cw[:, None])[2]
        camera_first = -first.rotation_cw.T @ first.translation_cw
        camera_second = -second.rotation_cw.T @ second.translation_cw
        ray_first = points - camera_first
        ray_second = points - camera_second
        ray_denominator = np.linalg.norm(ray_first, axis=1) * np.linalg.norm(
            ray_second,
            axis=1,
        )
        parallax_cosine = np.ones(len(points), np.float64)
        usable_rays = ray_denominator > 1e-9
        parallax_cosine[usable_rays] = np.sum(
            ray_first[usable_rays] * ray_second[usable_rays],
            axis=1,
        ) / ray_denominator[usable_rays]
        parallax_degrees = np.degrees(
            np.arccos(np.clip(parallax_cosine, -1.0, 1.0))
        )
        camera_points_first = (
            first.rotation_cw @ points.T + first.translation_cw[:, None]
        ).T
        camera_points_second = (
            second.rotation_cw @ points.T + second.translation_cw[:, None]
        ).T
        projected_first = camera_points_first[:, :2] / camera_points_first[:, 2:3]
        projected_second = camera_points_second[:, :2] / camera_points_second[:, 2:3]
        focal_scale = float(
            (self.camera_matrix[0, 0] + self.camera_matrix[1, 1]) * 0.5
        )
        reprojection_error = np.maximum(
            np.linalg.norm(projected_first - normalized_first, axis=1),
            np.linalg.norm(projected_second - normalized_second, axis=1),
        ) * focal_scale
        valid = (
            np.isfinite(points).all(axis=1)
            & nonzero_scale
            & (depth_first > 0.1)
            & (depth_second > 0.1)
            & (depth_first < 20.0)
            & (depth_second < 20.0)
            & (parallax_degrees >= 1.0)
            & np.isfinite(reprojection_error)
            & (reprojection_error <= 2.0)
        )
        if not valid.any():
            return 0
        descriptors = np.asarray(
            [first.descriptors[good[index].queryIdx] for index in np.flatnonzero(valid)],
            np.uint8,
        )
        return self._append_anchors(points[valid].astype(np.float32), descriptors)

    def _append_anchors(self, points: np.ndarray, descriptors: np.ndarray) -> int:
        self._ensure_anchor_metadata()
        accepted_points = []
        accepted_descriptors = []
        for point, descriptor in zip(points, descriptors):
            if self.anchor_count:
                distances = np.linalg.norm(self.anchor_points - point, axis=1)
                nearby = np.flatnonzero(distances < 0.04)
                if len(nearby):
                    descriptor_distances = np.asarray(
                        [
                            cv2.norm(
                                self.anchor_descriptors[index],
                                descriptor,
                                cv2.NORM_HAMMING,
                            )
                            for index in nearby
                        ],
                        np.float32,
                    )
                    compatible = nearby[descriptor_distances <= 64]
                    if len(compatible):
                        nearest = int(
                            compatible[np.argmin(distances[compatible])]
                        )
                        if len(self.anchor_support) != self.anchor_count:
                            self.anchor_support = np.ones(
                                self.anchor_count,
                                np.uint16,
                            )
                        previous_support = min(
                            64,
                            max(1, int(self.anchor_support[nearest])),
                        )
                        if nearest >= self.persistent_anchor_count:
                            self.anchor_points[nearest] = (
                                self.anchor_points[nearest] * previous_support + point
                            ) / (previous_support + 1)
                        self.anchor_support[nearest] = min(
                            np.iinfo(np.uint16).max,
                            int(self.anchor_support[nearest]) + 1,
                        )
                        if len(self.anchor_conflicts) != self.anchor_count:
                            self.anchor_conflicts = np.zeros(
                                self.anchor_count,
                                np.uint16,
                            )
                        self.anchor_conflicts[nearest] = max(
                            0,
                            int(self.anchor_conflicts[nearest]) - 1,
                        )
                        self.merged_anchor_observations += 1
                        continue

                    # Quest와 Tello의 시점 차이로 descriptor가 달라도 검증된
                    # Tello 삼각측량 관측은 영구 앵커를 훼손하지 않고 별도로 둔다.
                    session_nearby = nearby[
                        nearby >= self.persistent_anchor_count
                    ]
                    if len(session_nearby):
                        nearest_session = int(
                            session_nearby[
                                np.argmin(distances[session_nearby])
                            ]
                        )
                        self._record_anchor_conflict(nearest_session)
                        continue
            accepted_points.append(point)
            accepted_descriptors.append(descriptor)
        if not accepted_points:
            return 0
        self.anchor_points = np.concatenate(
            (self.anchor_points, np.asarray(accepted_points, np.float32))
        )
        self.anchor_descriptors = np.concatenate(
            (self.anchor_descriptors, np.asarray(accepted_descriptors, np.uint8))
        )
        self.anchor_support = np.concatenate(
            (self.anchor_support, np.ones(len(accepted_points), np.uint16))
        )
        self.anchor_conflicts = np.concatenate(
            (self.anchor_conflicts, np.zeros(len(accepted_points), np.uint16))
        )
        if self.anchor_count > self.max_anchors:
            session_capacity = max(0, self.max_anchors - self.persistent_anchor_count)
            persistent = np.arange(self.persistent_anchor_count, dtype=np.int32)
            session = np.arange(self.persistent_anchor_count, self.anchor_count, dtype=np.int32)
            session = session[-session_capacity:] if session_capacity else session[:0]
            keep = np.concatenate((persistent, session))
            self.anchor_points = self.anchor_points[keep]
            self.anchor_descriptors = self.anchor_descriptors[keep]
            self.anchor_support = self.anchor_support[keep]
            self.anchor_conflicts = self.anchor_conflicts[keep]
            self._invalidate_tracked_points()
        return len(accepted_points)

    def _record_anchor_conflict(self, index: int) -> None:
        if len(self.anchor_conflicts) != self.anchor_count:
            self.anchor_conflicts = np.zeros(self.anchor_count, np.uint16)
        self.anchor_conflicts[index] = min(
            np.iinfo(np.uint16).max,
            int(self.anchor_conflicts[index]) + 1,
        )
        required_conflicts = min(
            DYNAMIC_ANCHOR_MAX_CONFLICT_THRESHOLD,
            max(
                DYNAMIC_ANCHOR_CONFLICT_THRESHOLD,
                int(self.anchor_support[index]) // 2,
            ),
        )
        if self.anchor_conflicts[index] < required_conflicts:
            return
        keep = np.ones(self.anchor_count, bool)
        keep[index] = False
        self.anchor_points = self.anchor_points[keep]
        self.anchor_descriptors = self.anchor_descriptors[keep]
        self.anchor_support = self.anchor_support[keep]
        self.anchor_conflicts = self.anchor_conflicts[keep]
        if index < self.persistent_anchor_count:
            self.persistent_anchor_count -= 1
        self.quarantined_anchor_count += 1
        self._invalidate_tracked_points()

    # 불연속 anchor pose가 검출되면 검증되지 않은 로컬 맵을 비운다.
    def reset_local_map(self) -> None:
        self._ensure_anchor_metadata()
        self.keyframes.clear()
        self.anchor_points = self.anchor_points[: self.persistent_anchor_count].copy()
        self.anchor_descriptors = self.anchor_descriptors[
            : self.persistent_anchor_count
        ].copy()
        self.anchor_support = self.anchor_support[: self.persistent_anchor_count].copy()
        self.anchor_conflicts = self.anchor_conflicts[
            : self.persistent_anchor_count
        ].copy()
        self.previous_gray = None
        self._invalidate_tracked_points()

    # 불연속 pose만 버리고 다음 프레임에서 descriptor 재매칭하도록 로컬 맵은 보존한다.
    def reject_tracked_pose(self) -> None:
        self._invalidate_tracked_points()

    def optimize_local_map(self) -> None:
        """최근 5개 키프레임에서 재관측되지 않는 오래된 앵커를 정리한다."""
        if self.anchor_count < 100 or len(self.keyframes) < 2:
            return
        self._ensure_anchor_metadata()
        support = np.zeros(self.anchor_count, np.uint8)
        for keyframe in self.keyframes[-5:]:
            matches = self.matcher.knnMatch(
                self.anchor_descriptors,
                keyframe.descriptors,
                k=2,
            )
            for pair in matches:
                if len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance:
                    support[pair[0].queryIdx] = min(255, support[pair[0].queryIdx] + 1)
        keep = support > 0
        keep[: self.persistent_anchor_count] = True
        if max(50, self.persistent_anchor_count) <= keep.sum() < self.anchor_count:
            self.anchor_points = self.anchor_points[keep]
            self.anchor_descriptors = self.anchor_descriptors[keep]
            self.anchor_support = self.anchor_support[keep]
            self.anchor_conflicts = self.anchor_conflicts[keep]
            self._invalidate_tracked_points()

    def rebuild_session_map(self) -> None:
        keyframes = list(self.keyframes)
        self.anchor_points = np.empty((0, 3), np.float32)
        self.anchor_descriptors = np.empty((0, 32), np.uint8)
        self.anchor_support = np.empty((0,), np.uint16)
        self.anchor_conflicts = np.empty((0,), np.uint16)
        self.persistent_anchor_count = 0
        self._invalidate_tracked_points()
        for index in range(1, len(keyframes)):
            self._triangulate(keyframes[index - 1], keyframes[index])
            if index >= 2:
                self._triangulate(keyframes[index - 2], keyframes[index])

    # 앵커 배열 인덱스가 바뀌면 다음 프레임에서 descriptor 재매칭하도록 KLT 상태를 버린다.
    def _invalidate_tracked_points(self, *, preserve_pending: bool = False) -> None:
        self.tracked_pixels = np.empty((0, 2), np.float32)
        self.tracked_anchor_indices = np.empty((0,), np.int32)
        self._descriptor_disagreement_streak = 0
        if not preserve_pending:
            self._pending_supported_anchor_indices = np.empty((0,), np.int32)

    def _ensure_anchor_metadata(self) -> None:
        if len(self.anchor_support) != self.anchor_count:
            self.anchor_support = np.ones(self.anchor_count, np.uint16)
        if len(self.anchor_conflicts) != self.anchor_count:
            self.anchor_conflicts = np.zeros(self.anchor_count, np.uint16)

    @staticmethod
    def _keyframe_distance(first: Keyframe, second: Keyframe) -> float:
        first_position = -first.rotation_cw.T @ first.translation_cw
        second_position = -second.rotation_cw.T @ second.translation_cw
        return float(np.linalg.norm(first_position - second_position))


class AnchorPoseContinuityGate:
    def __init__(self, max_gap_seconds: float = MAX_ANCHOR_POSE_GAP_SECONDS) -> None:
        self.max_gap_seconds = max(0.0, max_gap_seconds)
        self._last_position: np.ndarray | None = None
        self._last_rotation: np.ndarray | None = None
        self._last_at = 0.0
        self.last_rejection_reason = ""
        self.last_gap_seconds = 0.0
        self.last_position_delta_meters = 0.0
        self.last_rotation_delta_degrees = 0.0
        self.consecutive_rejections = 0

    @property
    def can_attempt_relocalization(self) -> bool:
        return self.last_rejection_reason in {"missing-reference", "pose-gap"}

    # AprilTag 또는 검증된 anchor pose를 다음 프레임의 연속성 기준으로 저장한다.
    def update_reference(
        self,
        position: np.ndarray,
        rotation: np.ndarray,
        observed_at: float,
    ) -> None:
        self._last_position = position.copy()
        self._last_rotation = rotation.copy()
        self._last_at = observed_at
        self._clear_diagnostic()
        self.consecutive_rejections = 0

    # 연속적으로 들어오는 anchor pose는 태그 손실 시간과 무관하게 이어서 허용한다.
    def accept_anchor(
        self,
        position: np.ndarray,
        rotation: np.ndarray,
        observed_at: float,
    ) -> bool:
        self._clear_diagnostic()
        if self._last_position is None or self._last_rotation is None:
            self._reject("missing-reference")
            return False
        self.last_gap_seconds = observed_at - self._last_at
        if self.last_gap_seconds < 0.0:
            self._reject("reversed-time")
            return False
        if self.last_gap_seconds > self.max_gap_seconds:
            self._reject("pose-gap")
            return False
        (
            self.last_position_delta_meters,
            self.last_rotation_delta_degrees,
        ) = _anchor_pose_deltas(
            position,
            rotation,
            self._last_position,
            self._last_rotation,
        )
        if (
            not math.isfinite(self.last_position_delta_meters)
            or self.last_position_delta_meters
            > min(
                MAX_ANCHOR_POSITION_DELTA_METERS,
                MIN_ANCHOR_POSITION_DELTA_METERS
                + MAX_ANCHOR_POSITION_METERS_PER_SECOND * self.last_gap_seconds,
            )
        ):
            self._reject("position-delta", clear_reference=False)
            return False
        if (
            not math.isfinite(self.last_rotation_delta_degrees)
            or self.last_rotation_delta_degrees
            > min(
                MAX_ANCHOR_ROTATION_DELTA_DEGREES,
                MIN_ANCHOR_ROTATION_DELTA_DEGREES
                + MAX_ANCHOR_ROTATION_DEGREES_PER_SECOND * self.last_gap_seconds,
            )
        ):
            self._reject("rotation-delta", clear_reference=False)
            return False
        self.update_reference(position, rotation, observed_at)
        return True

    def reset(self) -> None:
        self._clear_reference()
        self._clear_diagnostic()
        self.consecutive_rejections = 0

    def accept_relocalization(
        self,
        position: np.ndarray,
        rotation: np.ndarray,
        observed_at: float,
        *,
        persistent_inliers: int,
        reprojection_error: float,
    ) -> bool:
        if (
            persistent_inliers < MAP_READY_MIN_INLIERS
            or not math.isfinite(reprojection_error)
            or reprojection_error > MAP_READY_MAX_REPROJECTION_ERROR
        ):
            return False
        self.update_reference(position, rotation, observed_at)
        return True

    def _reject(self, reason: str, *, clear_reference: bool = True) -> None:
        self.last_rejection_reason = reason
        self.consecutive_rejections += 1
        if clear_reference:
            self._clear_reference()

    def _clear_reference(self) -> None:
        self._last_position = None
        self._last_rotation = None
        self._last_at = 0.0

    def _clear_diagnostic(self) -> None:
        self.last_rejection_reason = ""
        self.last_gap_seconds = 0.0
        self.last_position_delta_meters = 0.0
        self.last_rotation_delta_degrees = 0.0


class VisualMapCaptureController:
    def __init__(
        self,
        sparse: SparseAnchorTracker,
        identity,
        npz_path: Path,
        manifest_path: Path,
        quaternion_to_matrix,
        *,
        minimum_duration_seconds: float = 60.0,
        minimum_path_meters: float = 8.0,
        minimum_keyframes: int = 12,
        minimum_anchors: int = 250,
        timeout_seconds: float = 600.0,
    ) -> None:
        self.sparse = sparse
        self.identity = identity
        self.npz_path = npz_path
        self.manifest_path = manifest_path
        self.quaternion_to_matrix = quaternion_to_matrix
        self.minimum_duration_seconds = minimum_duration_seconds
        self.minimum_path_meters = minimum_path_meters
        self.minimum_keyframes = minimum_keyframes
        self.minimum_anchors = minimum_anchors
        self.timeout_seconds = timeout_seconds
        self.state = "WAITING_FOR_BOARD"
        self.board_stable_since: float | None = None
        self.capture_started_at = 0.0
        self.path_length_meters = 0.0
        self.last_path_position: np.ndarray | None = None
        self.last_path_keyframe: Keyframe | None = None
        self.last_tracked_position: np.ndarray | None = None
        self.last_tracked_rotation: np.ndarray | None = None
        self._last_progress_at = 0.0
        self.message = "보드 태그 4개를 1초 동안 보여주세요."

    @property
    def flags(self) -> int:
        if self.state == "COMPLETE":
            return LOCAL_MAP_READY_FLAG
        if self.state == "REJECTED":
            return LOCAL_MAP_DEGRADED_FLAG
        return 0

    def update(
        self,
        now: float,
        marker_position: np.ndarray | None,
        marker_rotation: np.ndarray | None,
        marker_tag_count: int | None,
        tracked_position: np.ndarray | None,
        tracked_rotation: np.ndarray | None,
    ) -> None:
        if self.state in {"COMPLETE", "REJECTED"}:
            return
        marker_sampled = marker_tag_count is not None
        marker_ready = (
            marker_position is not None
            and marker_rotation is not None
            and marker_tag_count == 4
        )
        if self.state == "WAITING_FOR_BOARD":
            if not marker_sampled:
                return
            if marker_ready:
                if self.board_stable_since is None:
                    self.board_stable_since = now
                if now - self.board_stable_since >= 1.0:
                    self._set_state("ARMED", "보드 이탈점까지 천천히 이동하세요.")
            else:
                self.board_stable_since = None
            return
        if self.state == "ARMED":
            if not marker_sampled:
                return
            if marker_ready:
                return
            self.capture_started_at = now
            self.last_path_keyframe = (
                self.sparse.keyframes[-1] if self.sparse.keyframes else None
            )
            self.last_path_position = (
                -self.last_path_keyframe.rotation_cw.T
                @ self.last_path_keyframe.translation_cw
                if self.last_path_keyframe is not None
                else None
            )
            self.board_stable_since = None
            self._set_state("CAPTURING", "시각 지도 촬영 중입니다.")
            return
        if self.state != "CAPTURING":
            return
        duration = now - self.capture_started_at
        if duration > self.timeout_seconds:
            self._reject("10분 안에 보드로 복귀하지 못했습니다.")
            return
        if now - self._last_progress_at >= 1.0:
            self._last_progress_at = now
            print(
                "VISUAL_MAP_CAPTURE PROGRESS: "
                f"duration={duration:.1f}s, "
                f"path={self.path_length_meters:.2f}m, "
                f"keyframes={len(self.sparse.keyframes)}, "
                f"anchors={self.sparse.session_anchor_count}, "
                f"features={self.sparse.last_feature_count}",
                flush=True,
            )
        if tracked_position is not None and tracked_rotation is not None:
            self.last_tracked_position = tracked_position.copy()
            self.last_tracked_rotation = tracked_rotation.copy()
        if self.sparse.keyframes:
            latest_keyframe = self.sparse.keyframes[-1]
            if latest_keyframe is not self.last_path_keyframe:
                keyframe_position = (
                    -latest_keyframe.rotation_cw.T @ latest_keyframe.translation_cw
                )
                if self.last_path_position is not None:
                    step = float(
                        np.linalg.norm(keyframe_position - self.last_path_position)
                    )
                    if 0.1 <= step <= MAX_CAPTURE_PATH_STEP_METERS:
                        self.path_length_meters += step
                self.last_path_position = keyframe_position
                self.last_path_keyframe = latest_keyframe
        eligible = (
            duration >= self.minimum_duration_seconds
            and self.path_length_meters >= self.minimum_path_meters
            and len(self.sparse.keyframes) >= self.minimum_keyframes
            and self.sparse.session_anchor_count >= self.minimum_anchors
        )
        if marker_ready and eligible:
            if self.board_stable_since is None:
                self.board_stable_since = now
            if now - self.board_stable_since >= 1.0:
                self._finalize(marker_position, marker_rotation, duration)
        elif marker_sampled:
            self.board_stable_since = None

    def _finalize(
        self,
        final_position: np.ndarray,
        final_rotation: np.ndarray,
        duration_seconds: float,
    ) -> None:
        if self.last_tracked_position is None or self.last_tracked_rotation is None:
            self._reject("보드 재인식 직전의 시각 pose가 없습니다.")
            return
        closure_position, closure_rotation = _anchor_pose_deltas(
            self.last_tracked_position,
            self.last_tracked_rotation,
            final_position,
            final_rotation,
        )
        if closure_position > 0.5 or closure_rotation > 15.0:
            self._reject(
                f"폐루프 오차 초과: {closure_position:.3f}m, "
                f"{closure_rotation:.2f}deg"
            )
            return
        try:
            rotations = np.asarray(
                [item.rotation_cw for item in self.sparse.keyframes],
                np.float64,
            )
            translations = np.asarray(
                [item.translation_cw for item in self.sparse.keyframes],
                np.float64,
            )
            final_rotation_wc = _unity_camera_rotation_to_cv_world(
                final_rotation,
                self.quaternion_to_matrix,
            )
            optimized_rotations, optimized_translations, _, _ = (
                optimize_closed_loop_poses(
                    rotations,
                    translations,
                    final_rotation_wc,
                    final_position,
                )
            )
            for index, keyframe in enumerate(self.sparse.keyframes):
                keyframe.rotation_cw = optimized_rotations[index]
                keyframe.translation_cw = optimized_translations[index]
            self.sparse.rebuild_session_map()
            if self.sparse.anchor_count < self.minimum_anchors:
                raise VisualMapError(
                    "최적화 후 유효 앵커가 "
                    f"{self.sparse.anchor_count}개로 부족합니다."
                )
            artifact = VisualMapArtifact(
                identity=self.identity,
                anchor_points=self.sparse.anchor_points,
                anchor_descriptors=self.sparse.anchor_descriptors,
                anchor_support=self.sparse.anchor_support,
                keyframe_rotations_cw=optimized_rotations,
                keyframe_translations_cw=optimized_translations,
                duration_seconds=duration_seconds,
                path_length_meters=self.path_length_meters,
                anchor_conflicts=self.sparse.anchor_conflicts,
            )
            save_visual_map(self.npz_path, self.manifest_path, artifact)
        except (OSError, ValueError, VisualMapError) as error:
            self._reject(str(error))
            return
        self._set_state(
            "COMPLETE",
            "시각 지도 저장 완료: "
            f"keyframes={len(self.sparse.keyframes)}, "
            f"anchors={self.sparse.anchor_count}, "
            f"path={self.path_length_meters:.2f}m",
        )

    def _reject(self, reason: str) -> None:
        self._set_state("REJECTED", f"시각 지도 캡처 거부: {reason}")

    def _set_state(self, state: str, message: str) -> None:
        self.state = state
        self.message = message
        print(f"VISUAL_MAP_CAPTURE {state}: {message}", flush=True)


class VisionAdapter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.root = Path(__file__).resolve().parent.parent
        self.vision_root = (self.root / args.vision_root).resolve()
        self.module = self._load_team_module()
        map_path = Path(args.map_path)
        calibration_path = Path(args.calibration)
        self.board_to_venue = self._load_board_to_venue(map_path)
        board_args = argparse.Namespace(
            tag_family="tag36h11",
            tag_size=0.117,
            tag_spacing=0.190,
            tag_ids="0,1,2,3",
            max_reprojection_error=3.0,
        )
        self.board = self.module.BoardTracker(board_args, calibration_path)
        if not self.board.calibration_ready:
            raise RuntimeError(
                "Vision 카메라 보정이 완료되지 않았습니다. calibration.json의 "
                "calibrated=true 및 카메라 행렬이 필요합니다."
            )
        self.apriltag_only = bool(getattr(args, "apriltag_only", False))
        if self.apriltag_only and args.capture_map:
            raise ValueError("apriltag-only와 capture-map은 동시에 사용할 수 없습니다.")
        self.apriltag_refresh_cooldown = float(
            getattr(
                args,
                "apriltag_refresh_cooldown",
                DEFAULT_APRILTAG_REFRESH_COOLDOWN_SECONDS,
            )
        )
        if (
            not math.isfinite(self.apriltag_refresh_cooldown)
            or self.apriltag_refresh_cooldown < 0.0
        ):
            raise ValueError("apriltag-refresh-cooldown은 0 이상의 유한한 값이어야 합니다.")
        tracker_options = (
            {
                "max_anchors": CAPTURE_MAX_ANCHORS,
                "max_keyframes": CAPTURE_MAX_KEYFRAMES,
                "minimum_keyframe_distance_meters": (
                    CAPTURE_MIN_KEYFRAME_DISTANCE_METERS
                ),
            }
            if args.capture_map
            else {"max_anchors": RUNTIME_MAX_ANCHORS}
        )
        self.sparse = SparseAnchorTracker(
            self.board.camera_matrix,
            self.board._distortion,
            **tracker_options,
        )
        self.visual_map_identity = load_visual_map_identity(
            map_path,
            calibration_path,
        )
        self.visual_map_npz, self.visual_map_manifest = visual_map_paths(
            Path(args.visual_map_root),
            self.visual_map_identity.map_id,
        )
        (
            self.incremental_map_npz,
            self.incremental_map_manifest,
        ) = incremental_visual_map_paths(
            Path(args.visual_map_root),
            self.visual_map_identity.map_id,
        )
        self.capture = None
        self.session_only = bool(getattr(args, "session_only", False))
        if self.session_only and args.capture_map:
            raise ValueError("session-only와 capture-map은 동시에 사용할 수 없습니다.")
        self.visual_map_loaded = False
        self.visual_map_invalid = False
        self.visual_map_missing = False
        self.incremental_map_loaded = False
        self._incremental_baseline: VisualMapArtifact | None = None
        if self.apriltag_only:
            print(
                "VISION MODE APRILTAG_SENSOR: 정상 4태그 좌표를 3초 쿨타임으로 "
                "갱신하고 희소 앵커 SLAM은 실행하지 않습니다.",
                flush=True,
            )
        elif args.capture_map:
            self.capture = VisualMapCaptureController(
                self.sparse,
                self.visual_map_identity,
                self.visual_map_npz,
                self.visual_map_manifest,
                self.module.quaternion_to_matrix,
                minimum_duration_seconds=args.capture_minimum_duration,
                minimum_path_meters=args.capture_minimum_path,
                minimum_keyframes=args.capture_minimum_keyframes,
                minimum_anchors=args.capture_minimum_anchors,
                timeout_seconds=args.capture_timeout,
            )
        elif self.session_only:
            self.visual_map_missing = True
            print(
                "VISUAL_MAP SESSION_ONLY_TEST: 저장 지도를 읽거나 쓰지 않고 "
                "현재 실행의 온라인 앵커만 사용합니다.",
                flush=True,
            )
        else:
            canonical_artifact = None
            incremental_artifact = None
            canonical_exists = (
                self.visual_map_npz.exists() or self.visual_map_manifest.exists()
            )
            self.visual_map_missing = not canonical_exists
            if canonical_exists:
                try:
                    canonical_artifact = load_visual_map(
                        self.visual_map_npz,
                        self.visual_map_manifest,
                        self.visual_map_identity,
                        max_persistent_anchors=RUNTIME_MAX_PERSISTENT_ANCHORS,
                    )
                    print(
                        "VISUAL_MAP LOADED: "
                        f"anchors={canonical_artifact.anchor_count}, "
                        f"keyframes={canonical_artifact.keyframe_count}",
                        flush=True,
                    )
                except VisualMapError as error:
                    print(f"VISUAL_MAP INVALID: {error}", flush=True)

            incremental_history = (
                self.incremental_map_npz.parent
                / INCREMENTAL_MAP_HISTORY_DIRECTORY
            )
            incremental_exists = (
                self.incremental_map_npz.exists()
                or self.incremental_map_manifest.exists()
                or any(incremental_history.glob("incremental-map-*.json"))
            )
            if incremental_exists:
                try:
                    incremental_artifact, loaded_from = load_incremental_visual_map(
                        self.incremental_map_npz,
                        self.incremental_map_manifest,
                        self.visual_map_identity,
                        max_persistent_anchors=RUNTIME_MAX_PERSISTENT_ANCHORS,
                    )
                    self.incremental_map_loaded = True
                    print(
                        "VISUAL_MAP INCREMENTAL_LOADED: "
                        f"anchors={incremental_artifact.anchor_count}, "
                        f"source={loaded_from}",
                        flush=True,
                    )
                except VisualMapError as error:
                    print(
                        "VISUAL_MAP INCREMENTAL_INVALID: "
                        f"{error}; 유효한 기준 지도 또는 새 세션 지도를 사용합니다.",
                        flush=True,
                    )

            artifact = canonical_artifact or incremental_artifact
            if canonical_artifact is not None and incremental_artifact is not None:
                artifact = merge_visual_maps(
                    canonical_artifact,
                    incremental_artifact,
                    maximum_anchors=RUNTIME_MAX_PERSISTENT_ANCHORS,
                )
                print(
                    "VISUAL_MAP MERGED: "
                    f"canonical={canonical_artifact.anchor_count}, "
                    f"incremental={incremental_artifact.anchor_count}, "
                    f"result={artifact.anchor_count}",
                    flush=True,
                )

            if artifact is not None:
                self.sparse.load_persistent_map(artifact)
                self.visual_map_loaded = True
                self.visual_map_invalid = False
                self._incremental_baseline = artifact
            elif canonical_exists:
                self.visual_map_invalid = True
            else:
                print(
                    "VISUAL_MAP SESSION_ONLY: 저장 지도 없이 온라인 지도를 생성합니다.",
                    flush=True,
                )
        self.reader = LatestFrameTcpReader(args.video_port)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = ("127.0.0.1", args.measurement_port)
        hover_motion_port = int(getattr(args, "hover_motion_port", 0))
        self.hover_motion_target = (
            ("127.0.0.1", hover_motion_port)
            if hover_motion_port > 0
            else None
        )
        self.motion_validator = VisualLossMotionValidator()
        self._last_board_at = 0.0
        self._last_orb_at = 0.0
        self._last_optimization_at = 0.0
        self.anchor_continuity = AnchorPoseContinuityGate()
        self._map_success_streak = 0
        self._last_map_success_at = 0.0
        self._last_strong_map_success_at = 0.0
        self._local_map_ready = False
        self._map_preparation_origin: np.ndarray | None = None
        self._session_bridge_success_streak = 0
        self._marker_validated_session_map = False
        self._marker_revalidation_required = False
        self._marker_revalidation_success_streak = 0
        self._anchor_map_reference_active = False
        self._clear_anchor_output_alignment()
        self._runtime_started_at = time.monotonic()
        self._had_apriltag_lock = False
        self._last_sensor_alignment_at = 0.0
        self._next_sensor_alignment_at = 0.0
        self._cold_start_verified = False
        self._last_incremental_save_at = 0.0
        self._rejected_frame_shape: tuple[int, ...] | None = None
        self._board_seen_in_current_lock = False
        self._pending_board_hidden_log = False
        self._diag_board_validate_count = 0
        self._diag_map_degraded = False

    # 검증된 영상 이동량을 Position Sync와 Pilot Hover에 각각 전달한다.
    def _send_motion_validation(
        self,
        motion_validation: FrameMotionValidation,
    ) -> None:
        payload = motion_validation.to_json()
        self.sender.sendto(payload, self.target)
        hover_target = getattr(self, "hover_motion_target", None)
        if hover_target is not None and hover_target != self.target:
            self.sender.sendto(payload, hover_target)

    def run(self) -> None:
        print("STRIKERPC_READY VISION_ADAPTER", flush=True)
        while True:
            shared = self.reader.read()
            if shared is None:
                continue
            frame = shared.frame
            frame_shape = tuple(int(value) for value in frame.shape)
            if not _frame_matches_calibration(
                frame,
                self.visual_map_identity.image_width,
                self.visual_map_identity.image_height,
            ):
                if frame_shape != self._rejected_frame_shape:
                    if not getattr(self, "apriltag_only", False):
                        self.sparse.reject_tracked_pose()
                    motion_validator = getattr(self, "motion_validator", None)
                    if motion_validator is not None:
                        motion_validator.reset()
                    print(
                        "VISION FRAME REJECTED: "
                        f"expected={self.visual_map_identity.image_width}x"
                        f"{self.visual_map_identity.image_height}x3, "
                        f"actual={frame_shape}; calibration mismatch",
                        flush=True,
                    )
                    self._rejected_frame_shape = frame_shape
                continue
            if self._rejected_frame_shape is not None:
                print(
                    "VISION FRAME RESUMED: calibrated frame size restored",
                    flush=True,
                )
                self._rejected_frame_shape = None
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            dynamic_boxes = list(shared.dynamic_boxes)
            motion_validator = getattr(self, "motion_validator", None)
            motion_validation = (
                motion_validator.update(
                    gray,
                    shared.sequence,
                    shared.source_timestamp_ns,
                    dynamic_boxes,
                )
                if motion_validator is not None
                else None
            )
            now = time.monotonic()
            was_local_map_ready = self._local_map_ready
            marker_pose = None
            marker_sampled = now - self._last_board_at >= 1.0 / self.args.apriltag_rate
            if marker_sampled:
                marker_pose, _ = self.board.track(frame)
                self._last_board_at = now
            board_boxes = (
                _detected_board_boxes(
                    getattr(self.board, "_last_detections", ()),
                    frame.shape,
                )
            )
            if getattr(self, "apriltag_only", False):
                if (
                    marker_pose is not None
                    and marker_pose.tag_count == 4
                    and now >= getattr(self, "_next_sensor_alignment_at", 0.0)
                ):
                    refreshed = getattr(self, "_had_apriltag_lock", False)
                    self._had_apriltag_lock = True
                    self._last_sensor_alignment_at = now
                    cooldown = max(
                        0.0,
                        float(
                            getattr(
                                self,
                                "apriltag_refresh_cooldown",
                                DEFAULT_APRILTAG_REFRESH_COOLDOWN_SECONDS,
                            )
                        ),
                    )
                    self._next_sensor_alignment_at = now + cooldown
                    position, rotation = self._to_venue(
                        marker_pose.position,
                        marker_pose.rotation,
                    )
                    measurement = VisualMeasurement(
                        shared.sequence,
                        shared.source_timestamp_ns,
                        position,
                        rotation,
                        marker_pose.confidence,
                        0,
                        marker_pose.tag_count,
                        marker_pose.reprojection_error,
                        "apriltag",
                        0,
                    )
                    self.sender.sendto(measurement.to_json(), self.target)
                    print(
                        "VISION SENSOR ALIGNMENT "
                        f"{'REFRESHED' if refreshed else 'CAPTURED'}: "
                        f"cooldown={cooldown:.2f}s",
                        flush=True,
                    )
                if motion_validation is not None:
                    self._send_motion_validation(motion_validation)
                continue
            tracking_exclusion_boxes = [*dynamic_boxes, *board_boxes]

            measurement = None
            keyframe_accepted = False
            tracked_position = None
            tracked_rotation = None
            marker_position = None
            marker_rotation = None
            tracking_marker_position = None
            tracking_marker_rotation = None
            board_anchor_reference_good = False
            run_orb = now - self._last_orb_at >= 1.0 / self.args.orb_rate
            tracked = None
            if marker_pose is None or run_orb:
                prefer_temporal_track = (
                    self.capture is None
                    and self._has_recent_stable_map_pose(now)
                    and not getattr(self, "_marker_revalidation_required", False)
                )
                tracked = self.sparse.track(
                    gray,
                    run_orb,
                    tracking_exclusion_boxes,
                    prefer_temporal_track=prefer_temporal_track,
                )
                if run_orb:
                    self._last_orb_at = now
            if marker_pose is not None:
                self._had_apriltag_lock = True
                marker_position, marker_rotation = self._to_venue(
                    marker_pose.position,
                    marker_pose.rotation,
                )
                (
                    tracking_marker_position,
                    tracking_marker_rotation,
                ) = _tracking_marker_pose(
                    marker_pose,
                    marker_position,
                    marker_rotation,
                    bool(getattr(self, "session_only", False)),
                )
                board_reference_eligible = marker_pose.tag_count == 4
                reference_position = (
                    tracking_marker_position
                    if board_reference_eligible
                    else None
                )
                reference_rotation = (
                    tracking_marker_rotation
                    if board_reference_eligible
                    else None
                )
                position, rotation = marker_position, marker_rotation
                board_anchor_reference_good = self._update_map_readiness(
                    tracked,
                    board_reference_eligible,
                    now,
                    reference_position,
                    reference_rotation,
                    board_sampled=marker_sampled,
                    tracker_sampled=run_orb,
                )
                if board_anchor_reference_good and tracked is not None:
                    raw_anchor_position, raw_anchor_rotation = (
                        self.module.cv_tcw_to_unity_pose(
                            tracked.rotation_cw,
                            tracked.translation_cw,
                        )
                    )
                    self._update_anchor_output_alignment(
                        raw_anchor_position,
                        raw_anchor_rotation,
                        position,
                        rotation,
                    )
                    anchor_position, anchor_rotation = (
                        self._align_anchor_output_pose(
                            raw_anchor_position,
                            raw_anchor_rotation,
                        )
                    )
                    self.anchor_continuity.update_reference(
                        anchor_position,
                        anchor_rotation,
                        now,
                    )
                    self._anchor_map_reference_active = True
                elif (
                    board_reference_eligible
                    and not self._anchor_map_reference_active
                ):
                    self.anchor_continuity.update_reference(position, rotation, now)
                can_update_marker_keyframe = (
                    board_reference_eligible
                    and (
                        not self.visual_map_loaded
                        or board_anchor_reference_good
                    )
                )
                if can_update_marker_keyframe and (
                    self.capture is None
                    or self.capture.state in {
                        "WAITING_FOR_BOARD",
                        "ARMED",
                    }
                ):
                    keyframe_position = (
                        raw_anchor_position
                        if self.visual_map_loaded
                        else tracking_marker_position
                    )
                    keyframe_rotation = (
                        raw_anchor_rotation
                        if self.visual_map_loaded
                        else tracking_marker_rotation
                    )
                    keyframe_accepted = self.sparse.update_marker_keyframe(
                        gray,
                        keyframe_position,
                        keyframe_rotation,
                        self.module.quaternion_to_matrix,
                        tracking_exclusion_boxes,
                    )
                    self._log_capture_keyframe(keyframe_accepted)
                measurement = VisualMeasurement(
                    shared.sequence,
                    shared.source_timestamp_ns,
                    position,
                    rotation,
                    marker_pose.confidence,
                    self.sparse.anchor_count,
                    marker_pose.tag_count,
                    marker_pose.reprojection_error,
                    "apriltag",
                    self._current_flags(now, board_visible=True),
                )
            else:
                tracked_is_strong = (
                    tracked is not None and self._tracked_pose_is_strong(tracked)
                )
                tracked_can_maintain = (
                    tracked is not None
                    and self._tracked_pose_can_maintain_ready_map(tracked, now)
                )
                if tracked is not None and not (
                    tracked_is_strong or tracked_can_maintain
                ):
                    self.sparse.reject_tracked_pose()
                    self._update_map_readiness(
                        tracked,
                        False,
                        now,
                        board_sampled=marker_sampled,
                        tracker_sampled=run_orb,
                    )
                elif tracked is not None:
                    raw_anchor_position, raw_anchor_rotation = (
                        self.module.cv_tcw_to_unity_pose(
                            tracked.rotation_cw,
                            tracked.translation_cw,
                        )
                    )
                    position, rotation = self._align_anchor_output_pose(
                        raw_anchor_position,
                        raw_anchor_rotation,
                    )
                    accepted = self.anchor_continuity.accept_anchor(
                        position,
                        rotation,
                        now,
                    )
                    relocalization_inliers, relocalization_source = (
                        self._relocalization_candidate(tracked)
                    )
                    if (
                        not accepted
                        and self.anchor_continuity.can_attempt_relocalization
                        and relocalization_source is not None
                        and self.anchor_continuity.accept_relocalization(
                            position,
                            rotation,
                            now,
                            persistent_inliers=relocalization_inliers,
                            reprojection_error=tracked.reprojection_error,
                        )
                    ):
                        accepted = True
                        print(
                            "Vision relocalized from "
                            f"{relocalization_source} map: "
                            f"inliers={relocalization_inliers}, "
                            f"error={tracked.reprojection_error:.2f}px",
                            flush=True,
                        )
                    if not accepted:
                        session_map_reset = self._reject_unstable_anchor_pose()
                        self._update_map_readiness(
                            None,
                            False,
                            now,
                            board_sampled=marker_sampled,
                            tracker_sampled=run_orb,
                        )
                        if (
                            self.anchor_continuity.consecutive_rejections == 1
                            or session_map_reset
                        ):
                            print(
                                "Vision anchor pose rejected: "
                                "reason="
                                f"{self.anchor_continuity.last_rejection_reason}, "
                                f"gap={self.anchor_continuity.last_gap_seconds:.3f}s, "
                                "positionDelta="
                                f"{self.anchor_continuity.last_position_delta_meters:.3f}m, "
                                "rotationDelta="
                                f"{self.anchor_continuity.last_rotation_delta_degrees:.2f}deg; "
                                f"local session map {'reset' if session_map_reset else 'preserved'}.",
                                flush=True,
                            )
                    else:
                        if tracked_is_strong:
                            self._update_map_readiness(
                                tracked,
                                False,
                                now,
                                board_sampled=marker_sampled,
                                tracker_sampled=run_orb,
                            )
                        else:
                            self._last_map_success_at = now
                        if (
                            self.capture is None
                            and not self._has_recent_stable_map_pose(now)
                        ):
                            self.sparse.reject_tracked_pose()
                        else:
                            tracked_position = position
                            tracked_rotation = rotation
                            if tracked_is_strong:
                                keyframe_accepted = self.sparse.update_tracked_keyframe(
                                    gray,
                                    raw_anchor_position,
                                    raw_anchor_rotation,
                                    self.module.quaternion_to_matrix,
                                    tracking_exclusion_boxes,
                                )
                            self._log_capture_keyframe(keyframe_accepted)
                            confidence = float(
                                np.clip(
                                    1.0 - tracked.reprojection_error / 3.0,
                                    0.0,
                                    1.0,
                                )
                            )
                            measurement = VisualMeasurement(
                                shared.sequence,
                                shared.source_timestamp_ns,
                                position,
                                rotation,
                                confidence,
                                self.sparse.anchor_count,
                                0,
                                tracked.reprojection_error,
                                "anchor",
                                self._current_flags(now, board_visible=False),
                            )
                else:
                    self._update_map_readiness(
                        None,
                        False,
                        now,
                        board_sampled=marker_sampled,
                        tracker_sampled=run_orb,
                    )
            self._log_tracking_diagnostics(
                now,
                was_local_map_ready=was_local_map_ready,
                marker_sampled=marker_sampled,
                marker_position=(
                    tracking_marker_position
                    if marker_pose is not None and marker_pose.tag_count == 4
                    else None
                ),
                marker_rotation=(
                    tracking_marker_rotation
                    if marker_pose is not None and marker_pose.tag_count == 4
                    else None
                ),
                tracked=tracked,
                measurement=measurement,
                board_anchor_reference_good=board_anchor_reference_good,
            )
            if self.capture is not None:
                self.capture.update(
                    now,
                    measurement.position
                    if marker_pose is not None and measurement is not None
                    else None,
                    measurement.rotation
                    if marker_pose is not None and measurement is not None
                    else None,
                    marker_pose.tag_count
                    if marker_pose is not None
                    else 0
                    if marker_sampled
                    else None,
                    tracked_position,
                    tracked_rotation,
                )
                if measurement is not None:
                    measurement.flags = self.capture.flags
            if measurement is not None:
                self.sender.sendto(measurement.to_json(), self.target)
            if motion_validation is not None:
                self._send_motion_validation(motion_validation)
            if (
                self.capture is None
                and now - self._last_optimization_at
                >= 1.0 / self.args.local_optimization_rate
            ):
                self.sparse.optimize_local_map()
                self._last_optimization_at = now
            if self._should_checkpoint_incremental_map(now, keyframe_accepted):
                self._last_incremental_save_at = now
                self._save_incremental_map(now)

    # 실기 한 번으로 보드 검증과 이탈 추적 전환을 판정할 수 있게 상태 변화만 기록한다.
    def _log_tracking_diagnostics(
        self,
        now: float,
        *,
        was_local_map_ready: bool,
        marker_sampled: bool,
        marker_position: np.ndarray | None,
        marker_rotation: np.ndarray | None,
        tracked: TrackedPose | None,
        measurement: VisualMeasurement | None,
        board_anchor_reference_good: bool,
    ) -> None:
        if marker_sampled and marker_position is not None:
            self._board_seen_in_current_lock = True
            self._pending_board_hidden_log = False
            if (
                not was_local_map_ready
                or getattr(self, "_marker_revalidation_required", False)
            ) and getattr(self, "_diag_board_validate_count", 0) < MAP_READY_STREAK:
                if tracked is None:
                    position_delta = -1.0
                    rotation_delta = -1.0
                    persistent_inliers = 0
                    session_inliers = 0
                    total_inliers = 0
                    reprojection_error = -1.0
                    strong = False
                    reason = "no-anchor-track"
                else:
                    tracked_position, tracked_rotation = (
                        self.module.cv_tcw_to_unity_pose(
                            tracked.rotation_cw,
                            tracked.translation_cw,
                        )
                    )
                    position_delta, rotation_delta = _anchor_pose_deltas(
                        tracked_position,
                        tracked_rotation,
                        marker_position,
                        marker_rotation,
                    )
                    persistent_inliers = tracked.persistent_inlier_count
                    session_inliers = tracked.session_inlier_count
                    total_inliers = tracked.inlier_count
                    reprojection_error = tracked.reprojection_error
                    strong = self._tracked_pose_is_strong(tracked)
                    reason = (
                        "accepted"
                        if board_anchor_reference_good
                        else "quality-or-consistency"
                    )
                validation_streak = max(
                    int(getattr(self, "_map_success_streak", 0)),
                    int(getattr(self, "_session_bridge_success_streak", 0)),
                    int(getattr(self, "_marker_revalidation_success_streak", 0)),
                )
                self._diag_board_validate_count = (
                    getattr(self, "_diag_board_validate_count", 0) + 1
                )
                print(
                    "VISION_STATE BOARD_VALIDATE "
                    f"sample={self._diag_board_validate_count}/{MAP_READY_STREAK} "
                    f"accepted={str(bool(board_anchor_reference_good)).lower()} "
                    f"reason={reason} "
                    f"streak={validation_streak}/{MAP_READY_STREAK} "
                    f"persistentInliers={persistent_inliers} "
                    f"sessionInliers={session_inliers} "
                    f"totalInliers={total_inliers} "
                    f"errorPx={reprojection_error:.2f} "
                    f"positionDeltaM={position_delta:.3f} "
                    f"rotationDeltaDeg={rotation_delta:.2f} "
                    f"strong={str(strong).lower()}",
                    flush=True,
                )
        elif marker_sampled and getattr(self, "_board_seen_in_current_lock", False):
            self._board_seen_in_current_lock = False
            self._pending_board_hidden_log = True

        if not was_local_map_ready and self._local_map_ready:
            ready_streak = max(
                int(getattr(self, "_map_success_streak", 0)),
                int(getattr(self, "_session_bridge_success_streak", 0)),
                int(getattr(self, "_marker_revalidation_success_streak", 0)),
            )
            print(
                "VISION_STATE MAP_READY "
                f"streak={ready_streak}/{MAP_READY_STREAK} "
                f"anchors={self.sparse.anchor_count} "
                f"persistentAnchors={self.sparse.persistent_anchor_count} "
                "outputAlignment="
                f"{'active' if getattr(self, '_anchor_output_alignment_active', False) else 'inactive'} "
                "markerValidatedSession="
                f"{str(bool(getattr(self, '_marker_validated_session_map', False))).lower()} "
                f"coldStart={str(bool(getattr(self, '_cold_start_verified', False))).lower()}",
                flush=True,
            )

        if (
            getattr(self, "_pending_board_hidden_log", False)
            and measurement is not None
            and measurement.source == "anchor"
            and tracked is not None
        ):
            print(
                "VISION_STATE BOARD_MISS_ANCHOR_MAINTAINED "
                f"sequence={measurement.sequence} "
                "mode="
                f"{'strong' if self._tracked_pose_is_strong(tracked) else 'maintenance'} "
                f"persistentInliers={tracked.persistent_inlier_count} "
                f"sessionInliers={tracked.session_inlier_count} "
                f"totalInliers={tracked.inlier_count} "
                f"errorPx={tracked.reprojection_error:.2f} "
                "outputAlignment="
                f"{'active' if getattr(self, '_anchor_output_alignment_active', False) else 'inactive'}",
                flush=True,
            )
            self._pending_board_hidden_log = False

        readiness_reset = was_local_map_ready and not self._local_map_ready
        stale = self._local_map_ready and not self._has_recent_stable_map_pose(now)
        degraded_now = readiness_reset or stale
        if degraded_now and not getattr(self, "_diag_map_degraded", False):
            last_success_at = getattr(self, "_last_map_success_at", 0.0)
            last_strong_success_at = getattr(
                self,
                "_last_strong_map_success_at",
                0.0,
            )
            age = -1.0 if last_success_at <= 0.0 else max(0.0, now - last_success_at)
            strong_age = (
                -1.0
                if last_strong_success_at <= 0.0
                else max(0.0, now - last_strong_success_at)
            )
            print(
                "VISION_STATE MAP_DEGRADED "
                f"reason={'readiness-reset' if readiness_reset else 'stale'} "
                f"ageSeconds={age:.3f} "
                f"strongAgeSeconds={strong_age:.3f} "
                "continuityRejections="
                f"{getattr(getattr(self, 'anchor_continuity', None), 'consecutive_rejections', 0)} "
                f"anchors={self.sparse.anchor_count} "
                "outputAlignment="
                f"{'active' if getattr(self, '_anchor_output_alignment_active', False) else 'inactive'}",
                flush=True,
            )
        self._diag_map_degraded = degraded_now
        if readiness_reset:
            self._diag_board_validate_count = 0

    # 큰 pose 불연속은 현재 세션 앵커만 비우고 검증된 영구 지도는 유지한다.
    def _reject_unstable_anchor_pose(self) -> bool:
        reason = self.anchor_continuity.last_rejection_reason
        if (
            reason in {"position-delta", "rotation-delta"}
            and self.anchor_continuity.consecutive_rejections
            < SEVERE_ANCHOR_REJECTION_RESET_STREAK
        ):
            self.sparse.reject_tracked_pose()
            return False
        if reason in {"position-delta", "rotation-delta", "reversed-time"}:
            self.sparse.reset_local_map()
            self._local_map_ready = False
            self._map_success_streak = 0
            self._last_map_success_at = 0.0
            self._last_strong_map_success_at = 0.0
            self._map_preparation_origin = None
            self._session_bridge_success_streak = 0
            self._marker_validated_session_map = False
            self._marker_revalidation_required = bool(self.visual_map_loaded)
            self._marker_revalidation_success_streak = 0
            self._anchor_map_reference_active = False
            self._clear_anchor_output_alignment()
            self._cold_start_verified = False
            return True
        self.sparse.reject_tracked_pose()
        return False

    # 저장 직전까지 연속성 검증을 통과한 시각 pose가 있었는지 확인한다.
    def _has_recent_stable_map_pose(self, now: float) -> bool:
        last_success_at = getattr(self, "_last_map_success_at", 0.0)
        age = now - last_success_at
        return (
            self._local_map_ready
            and last_success_at > 0.0
            and 0.0 <= age <= MAP_DEGRADED_SECONDS
        )

    # 보드와 앵커가 동시에 보일 때 내부 지도 pose를 표시용 보드 pose에 정렬한다.
    def _update_anchor_output_alignment(
        self,
        anchor_position: np.ndarray,
        anchor_rotation: np.ndarray,
        marker_position: np.ndarray,
        marker_rotation: np.ndarray,
    ) -> bool:
        anchor_matrix = self.module.quaternion_to_matrix(anchor_rotation)
        marker_matrix = self.module.quaternion_to_matrix(marker_rotation)
        alignment_rotation = marker_matrix @ anchor_matrix.T
        alignment_translation = (
            np.asarray(marker_position, np.float64)
            - alignment_rotation @ np.asarray(anchor_position, np.float64)
        )
        if not (
            np.isfinite(alignment_rotation).all()
            and np.isfinite(alignment_translation).all()
        ):
            return False
        self._anchor_output_alignment_rotation = alignment_rotation
        self._anchor_output_alignment_translation = alignment_translation
        self._anchor_output_alignment_active = True
        return True

    # 내부 앵커 지도는 그대로 두고 외부로 내보내는 pose에만 보드 정렬을 적용한다.
    def _align_anchor_output_pose(
        self,
        position: np.ndarray,
        rotation: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        if not getattr(self, "_anchor_output_alignment_active", False):
            return position.copy(), rotation.copy()
        alignment_rotation = self._anchor_output_alignment_rotation
        aligned_position = (
            alignment_rotation @ np.asarray(position, np.float64)
            + self._anchor_output_alignment_translation
        )
        aligned_rotation_matrix = (
            alignment_rotation @ self.module.quaternion_to_matrix(rotation)
        )
        return aligned_position, self.module.matrix_to_quaternion(
            aligned_rotation_matrix
        )

    # 새 지도 검증 전에는 이전 세션의 표시 정렬을 재사용하지 않는다.
    def _clear_anchor_output_alignment(self) -> None:
        self._anchor_output_alignment_rotation = np.eye(3, dtype=np.float64)
        self._anchor_output_alignment_translation = np.zeros(3, dtype=np.float64)
        self._anchor_output_alignment_active = False

    # 증분 지도 저장은 현재 프레임이 30개 이상 대응된 경우에만 허용한다.
    def _has_recent_strong_map_pose(self, now: float) -> bool:
        last_success_at = getattr(self, "_last_strong_map_success_at", 0.0)
        age = now - last_success_at
        return (
            self._local_map_ready
            and last_success_at > 0.0
            and 0.0 <= age <= MAP_DEGRADED_SECONDS
        )

    # 정식 지도 유무와 관계없이 검증된 세션 보강 정보를 주기적으로 저장한다.
    def _should_checkpoint_incremental_map(
        self,
        now: float,
        keyframe_accepted: bool,
    ) -> bool:
        elapsed = now - self._last_incremental_save_at
        return (
            self.capture is None
            and self._has_recent_strong_map_pose(now)
            and not self._has_pending_continuity_rejection()
            and (
                elapsed >= INCREMENTAL_SAVE_INTERVAL_SECONDS
                or (
                    keyframe_accepted
                    and elapsed >= INCREMENTAL_KEYFRAME_CHECKPOINT_SECONDS
                )
            )
        )

    def _update_map_readiness(
        self,
        tracked: TrackedPose | None,
        board_visible: bool,
        now: float,
        marker_position: np.ndarray | None = None,
        marker_rotation: np.ndarray | None = None,
        *,
        board_sampled: bool = False,
        tracker_sampled: bool = False,
    ) -> bool:
        if not self.visual_map_loaded and not self.visual_map_missing:
            return False
        if self._map_preparation_origin is None and marker_position is not None:
            self._map_preparation_origin = marker_position.copy()
        tracked_position = None
        tracked_rotation = None
        if tracked is not None:
            tracked_position, tracked_rotation = self.module.cv_tcw_to_unity_pose(
                tracked.rotation_cw,
                tracked.translation_cw,
            )
        reference_position = (
            marker_position if marker_position is not None else tracked_position
        )
        preparation_complete = (
            self._map_preparation_origin is not None
            and reference_position is not None
            and np.linalg.norm(reference_position - self._map_preparation_origin)
            >= MAP_READY_PREPARATION_DISTANCE_METERS
        )
        cold_start_candidate = (
            marker_position is None
            and getattr(self, "incremental_map_loaded", False)
            and tracked is not None
            and tracked.persistent_inlier_count >= COLD_START_MIN_INLIERS
            and tracked.reprojection_error <= COLD_START_MAX_REPROJECTION_ERROR
        )
        base_pose_quality_good = (
            tracked is not None
            and math.isfinite(tracked.reprojection_error)
            and tracked.reprojection_error <= MAP_READY_MAX_REPROJECTION_ERROR
        )
        strict_pose_quality_good = (
            base_pose_quality_good
            and tracked.inlier_count >= MAP_READY_MIN_INLIERS
        )
        board_validated_canonical_candidate = (
            base_pose_quality_good
            and self.visual_map_loaded
            and board_visible
            and marker_position is not None
            and marker_rotation is not None
            and tracked.persistent_inlier_count
            >= BOARD_VALIDATED_CANONICAL_MIN_INLIERS
        )
        board_validated_session_candidate = (
            strict_pose_quality_good
            and not self.visual_map_loaded
            and board_visible
            and marker_position is not None
            and marker_rotation is not None
        )
        pose_quality_good = (
            board_validated_canonical_candidate
            or board_validated_session_candidate
            or (
                strict_pose_quality_good
                and (
                    preparation_complete
                    or cold_start_candidate
                    or self._local_map_ready
                )
            )
        )
        marker_consistent = True
        if (
            pose_quality_good
            and marker_position is not None
            and marker_rotation is not None
        ):
            position_delta, rotation_delta = _anchor_pose_deltas(
                tracked_position,
                tracked_rotation,
                marker_position,
                marker_rotation,
            )
            marker_consistent = position_delta <= 0.3 and rotation_delta <= 10.0

        persistent_good = (
            tracked is not None
            and pose_quality_good
            and marker_consistent
            and tracked.persistent_inlier_count >= MAP_READY_MIN_INLIERS
        )
        bridge_observation_good = (
            self.visual_map_loaded
            and board_visible
            and marker_position is not None
            and marker_rotation is not None
            and tracked is not None
            and pose_quality_good
            and strict_pose_quality_good
            and marker_consistent
            and tracked.session_inlier_count >= SESSION_BRIDGE_MIN_INLIERS
        )
        board_validated_canonical_good = (
            board_validated_canonical_candidate and marker_consistent
        )
        board_validated_session_good = (
            board_validated_session_candidate and marker_consistent
        )
        board_anchor_reference_good = (
            board_visible
            and marker_position is not None
            and marker_rotation is not None
            and (
                board_validated_canonical_good
                or board_validated_session_good
                or persistent_good
                or bridge_observation_good
            )
        )
        if not marker_consistent:
            self._marker_validated_session_map = False
            self._session_bridge_success_streak = 0
            self._marker_revalidation_required = bool(self.visual_map_loaded)
            self._marker_revalidation_success_streak = 0
            self._anchor_map_reference_active = False
            self._clear_anchor_output_alignment()
            self._local_map_ready = False
            self._map_success_streak = 0
            self._last_map_success_at = 0.0
            self._last_strong_map_success_at = 0.0
            self._cold_start_verified = False
            self._map_preparation_origin = marker_position.copy()
            if not self.visual_map_loaded:
                self.sparse.reset_local_map()
            self.sparse.reject_tracked_pose()
            return False

        if getattr(self, "_marker_revalidation_required", False):
            revalidation_observation_good = (
                board_visible
                and pose_quality_good
                and marker_consistent
                and (
                    not self.visual_map_loaded
                    or persistent_good
                    or bridge_observation_good
                    or board_validated_canonical_good
                )
            )
            if revalidation_observation_good:
                self._confirm_tracked_pose_support()
                self._marker_revalidation_success_streak = (
                    getattr(self, "_marker_revalidation_success_streak", 0) + 1
                )
                self._session_bridge_success_streak = (
                    getattr(self, "_session_bridge_success_streak", 0) + 1
                    if bridge_observation_good
                    else 0
                )
            elif board_sampled and (not board_visible or tracker_sampled):
                self._marker_revalidation_success_streak = 0
                self._session_bridge_success_streak = 0

            if self._marker_revalidation_success_streak >= MAP_READY_STREAK:
                self._marker_revalidation_required = False
                self._marker_revalidation_success_streak = 0
                self._marker_validated_session_map = (
                    self._session_bridge_success_streak >= MAP_READY_STREAK
                )
                self._local_map_ready = True
                self._map_success_streak = MAP_READY_STREAK
                self._last_map_success_at = now
                if strict_pose_quality_good:
                    self._last_strong_map_success_at = now
            return board_anchor_reference_good

        if bridge_observation_good:
            self._confirm_tracked_pose_support()
            self._session_bridge_success_streak = (
                getattr(self, "_session_bridge_success_streak", 0) + 1
            )
            if self._session_bridge_success_streak >= MAP_READY_STREAK:
                self._marker_validated_session_map = True
                self._local_map_ready = True
        elif board_sampled and (not board_visible or tracker_sampled):
            self._session_bridge_success_streak = 0

        verified_session_good = (
            pose_quality_good
            and marker_consistent
            and getattr(self, "_marker_validated_session_map", False)
        )
        session_tracking_good = (
            not self.visual_map_loaded
            and not board_visible
            and strict_pose_quality_good
            and preparation_complete
        )
        good = (
            persistent_good
            if self.visual_map_loaded
            else session_tracking_good
        ) or verified_session_good or board_validated_canonical_good
        pending_board_lock_between_samples = (
            self.visual_map_loaded
            and not self._local_map_ready
            and getattr(self, "_anchor_map_reference_active", False)
            and not board_sampled
        )
        if good:
            self._confirm_tracked_pose_support()
            self._map_success_streak += 1
            self._last_map_success_at = now
            if strict_pose_quality_good:
                self._last_strong_map_success_at = now
            if self._map_success_streak >= MAP_READY_STREAK:
                self._local_map_ready = True
                if cold_start_candidate:
                    self._cold_start_verified = True
        elif not pending_board_lock_between_samples and (
            tracked is not None
            or not board_visible
            or (tracker_sampled and tracked is None)
        ):
            self._map_success_streak = 0
        return board_anchor_reference_good

    # tracker의 pending descriptor 관측을 검증된 pose에서만 support로 확정한다.
    def _confirm_tracked_pose_support(self) -> None:
        confirm = getattr(getattr(self, "sparse", None), "confirm_tracked_pose", None)
        if callable(confirm):
            confirm()

    def _relocalization_candidate(
        self,
        tracked: TrackedPose,
    ) -> tuple[int, str | None]:
        if getattr(self, "_marker_revalidation_required", False):
            return 0, None
        if self.visual_map_loaded:
            if (
                tracked.persistent_inlier_count < MAP_READY_MIN_INLIERS
                and getattr(self, "_marker_validated_session_map", False)
            ):
                return tracked.inlier_count, "session"
            return (
                tracked.persistent_inlier_count,
                "incremental" if self.incremental_map_loaded else "persistent",
            )
        if self.capture is not None:
            return tracked.inlier_count, "session"
        if self.visual_map_missing and self._local_map_ready:
            return tracked.inlier_count, "session"
        return 0, None

    # 현재 프레임 자체가 지도 신뢰 기준을 통과한 경우에만 pose를 전파한다.
    def _tracked_pose_is_strong(self, tracked: TrackedPose) -> bool:
        if (
            getattr(self, "_marker_revalidation_required", False)
            or not math.isfinite(tracked.reprojection_error)
            or tracked.reprojection_error > MAP_READY_MAX_REPROJECTION_ERROR
        ):
            return False
        inlier_count = (
            tracked.persistent_inlier_count
            if getattr(self, "visual_map_loaded", False)
            and not getattr(self, "_marker_validated_session_map", False)
            else tracked.inlier_count
        )
        return inlier_count >= MAP_READY_MIN_INLIERS

    # 이미 확정된 지도의 짧은 연속 구간만 낮은 inlier pose로 유지한다.
    def _tracked_pose_can_maintain_ready_map(
        self,
        tracked: TrackedPose,
        now: float,
    ) -> bool:
        if (
            getattr(self, "capture", None) is not None
            or getattr(self, "_marker_revalidation_required", False)
            or not self._has_recent_stable_map_pose(now)
            or not math.isfinite(tracked.reprojection_error)
            or tracked.reprojection_error > MAP_READY_MAX_REPROJECTION_ERROR
        ):
            return False
        inlier_count = (
            tracked.persistent_inlier_count
            if getattr(self, "visual_map_loaded", False)
            and not getattr(self, "_marker_validated_session_map", False)
            else tracked.inlier_count
        )
        return inlier_count >= MAP_MAINTENANCE_MIN_INLIERS

    def _log_capture_keyframe(self, accepted: bool) -> None:
        if self.capture is None or not accepted:
            return
        connection = (
            "seed"
            if len(self.sparse.keyframes) == 1
            else "connected"
            if self.sparse.last_keyframe_connected
            else "unconnected"
        )
        print(
            "VISUAL_MAP_KEYFRAME ACCEPTED: "
            f"index={len(self.sparse.keyframes)}, "
            f"features={self.sparse.last_feature_count}, "
            f"anchors={self.sparse.session_anchor_count}, "
            f"link={connection}",
            flush=True,
        )

    def _current_flags(self, now: float, *, board_visible: bool) -> int:
        if self.capture is not None:
            return self.capture.flags
        if self.visual_map_invalid:
            return LOCAL_MAP_DEGRADED_FLAG
        session_map_active = (
            self.visual_map_missing
            or getattr(self, "incremental_map_loaded", False)
            or getattr(self, "_marker_validated_session_map", False)
        )
        if not self.visual_map_loaded and not session_map_active:
            return 0
        flags = SESSION_MAP_FLAG if session_map_active else 0
        if self._local_map_ready:
            flags |= LOCAL_MAP_READY_FLAG
        map_recent = now - self._last_map_success_at <= MAP_DEGRADED_SECONDS
        if not board_visible and self._local_map_ready and not map_recent:
            flags |= RELOCALIZING_FLAG
        if self._local_map_ready and not board_visible and not map_recent:
            flags |= LOCAL_MAP_DEGRADED_FLAG
        return flags

    def _load_team_module(self):
        self._verify_repository()
        source = (
            self.vision_root
            / "Assets"
            / "StreamingAssets"
            / "TelloVisionBridge"
            / "Source"
            / "tello_vision_bridge.py"
        )
        spec = importlib.util.spec_from_file_location("strikerpc_team_vision", source)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Vision 원본을 불러올 수 없습니다: {source}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        required = {
            "BoardTracker": ("args", "config_path"),
            "cv_tcw_to_unity_pose": ("rotation_cw", "translation_cw"),
            "matrix_to_quaternion": ("matrix",),
            "quaternion_to_matrix": ("quaternion",),
        }
        for name, parameters in required.items():
            value = getattr(module, name, None)
            if value is None:
                raise RuntimeError(f"Vision 원본 API 누락: {name}")
            actual = tuple(inspect.signature(value).parameters)
            if actual[: len(parameters)] != parameters:
                raise RuntimeError(f"Vision 원본 API 시그니처 불일치: {name}{actual}")
        return module

    def _verify_repository(self) -> None:
        try:
            relative = self.vision_root.relative_to(self.root)
        except ValueError:
            relative = None
        if relative is not None and not (self.vision_root / ".git").exists():
            self._verify_embedded_repository(relative)
            return

        result = subprocess.run(
            [
                "git",
                "-c",
                f"safe.directory={self.vision_root.as_posix()}",
                "-C",
                str(self.vision_root),
                "rev-parse",
                "HEAD",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode or result.stdout.strip() != EXPECTED_VISION_COMMIT:
            raise RuntimeError(f"Vision 고정 커밋 불일치: {result.stdout.strip()}")
        if subprocess.run(
            [
                "git",
                "-c",
                f"safe.directory={self.vision_root.as_posix()}",
                "-C",
                str(self.vision_root),
                "diff",
                "--quiet",
                "HEAD",
            ],
            check=False,
        ).returncode:
            raise RuntimeError("Vision 원본 tracked 파일이 수정되어 있습니다.")

    # 통합 Vision은 Unity 설정 Prefab을 제외한 원본 package tree를 확인한다.
    def _verify_embedded_repository(self, relative: Path) -> None:
        marker = self.vision_root / "Assets" / "TelloVisionTracking"
        if not marker.is_dir():
            raise RuntimeError(f"Vision 런타임 소스가 없습니다: {marker}")

        command = [
            "git",
            "-c",
            f"safe.directory={self.root.as_posix()}",
            "-C",
            str(self.root),
        ]
        ancestor = subprocess.run(
            [
                *command,
                "merge-base",
                "--is-ancestor",
                EXPECTED_VISION_COMMIT,
                "HEAD",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if ancestor.returncode != 0:
            raise RuntimeError(
                "Vision 원본 커밋이 현재 이력에 없습니다: "
                f"{EXPECTED_VISION_COMMIT}"
            )

        source_tree = subprocess.run(
            [
                *command,
                "rev-parse",
                f"{EXPECTED_VISION_COMMIT}:Assets/TelloVisionTracking",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        embedded_tree = subprocess.run(
            [
                *command,
                "rev-parse",
                f"HEAD:{relative.as_posix()}/Assets/TelloVisionTracking",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if source_tree.returncode != 0 or embedded_tree.returncode != 0:
            raise RuntimeError("Vision 내장 소스가 고정 커밋과 다릅니다.")

        source_prefab = ":(exclude)Prefabs/TelloVisionTracking.prefab"
        source_changed = subprocess.run(
            [
                *command,
                "diff",
                "--quiet",
                source_tree.stdout.strip(),
                embedded_tree.stdout.strip(),
                "--",
                ".",
                source_prefab,
            ],
            check=False,
        ).returncode
        if source_changed:
            raise RuntimeError("Vision 내장 소스가 고정 커밋과 다릅니다.")

        relative_text = relative.as_posix()
        unity_prefab = (
            f":(exclude){relative_text}/Assets/TelloVisionTracking/Prefabs/"
            "TelloVisionTracking.prefab"
        )
        worktree_changed = subprocess.run(
            [
                *command,
                "diff",
                "--quiet",
                "HEAD",
                "--",
                relative_text,
                unity_prefab,
            ],
            check=False,
        ).returncode
        staged_changed = subprocess.run(
            [
                *command,
                "diff",
                "--cached",
                "--quiet",
                "HEAD",
                "--",
                relative_text,
                unity_prefab,
            ],
            check=False,
        ).returncode
        if worktree_changed or staged_changed:
            raise RuntimeError("Vision 원본 tracked 파일이 수정되어 있습니다.")

    @staticmethod
    def _load_board_to_venue(path: Path) -> np.ndarray:
        data = json.loads(path.read_text(encoding="utf-8"))
        values = np.asarray(data["boardToVenue"], np.float64)
        if values.size != 16:
            raise RuntimeError("지도 boardToVenue은 16개 값이어야 합니다.")
        return values.reshape(4, 4)

    def _to_venue(self, position: np.ndarray, rotation: np.ndarray):
        return _board_pose_to_venue(
            self.board_to_venue,
            position,
            rotation,
            self.module.quaternion_to_matrix,
            self.module.matrix_to_quaternion,
        )

    def close(self) -> None:
        try:
            self._save_incremental_map()
        finally:
            self.reader.close()
            self.sender.close()

    def _save_incremental_map(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        if (
            self.capture is not None
            or getattr(self, "session_only", False)
            or self.visual_map_invalid
            or self._has_pending_continuity_rejection()
            or not (
                getattr(self, "visual_map_loaded", False)
                or self.visual_map_missing
            )
            or not self._has_recent_strong_map_pose(now)
            or not (
                self._had_apriltag_lock
                or getattr(self, "_cold_start_verified", False)
            )
            or (
                getattr(self, "visual_map_loaded", False)
                and not (
                    getattr(self, "_marker_validated_session_map", False)
                    or getattr(self, "_cold_start_verified", False)
                )
            )
            or len(self.sparse.keyframes) < 2
            or self.sparse.anchor_count < MAP_READY_MIN_INLIERS
        ):
            return False

        support = np.asarray(self.sparse.anchor_support, np.uint16)
        conflicts = np.asarray(
            getattr(
                self.sparse,
                "anchor_conflicts",
                np.zeros(self.sparse.anchor_count, np.uint16),
            ),
            np.uint16,
        )
        if support.shape != (self.sparse.anchor_count,):
            return False
        if conflicts.shape != (self.sparse.anchor_count,):
            return False
        keep = support >= 2
        persistent_anchor_count = min(
            self.sparse.anchor_count,
            max(0, int(getattr(self.sparse, "persistent_anchor_count", 0))),
        )
        keep[:persistent_anchor_count] = True
        if int(np.count_nonzero(keep)) < MAP_READY_MIN_INLIERS:
            print(
                "VISUAL_MAP INCREMENTAL_SKIPPED: 반복 관측 앵커가 부족합니다.",
                flush=True,
            )
            return False

        rotations = np.asarray(
            [item.rotation_cw for item in self.sparse.keyframes],
            np.float64,
        )
        translations = np.asarray(
            [item.translation_cw for item in self.sparse.keyframes],
            np.float64,
        )
        positions = np.asarray(
            [-rotation.T @ translation for rotation, translation in zip(rotations, translations)],
            np.float64,
        )
        if not self._incremental_keyframes_are_plausible(positions):
            print(
                "VISUAL_MAP INCREMENTAL_SKIPPED: 키프레임 경로가 스캔 그리드 또는 이동 한계를 벗어났습니다.",
                flush=True,
            )
            return False
        path_length = float(
            np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()
        )
        artifact = VisualMapArtifact(
            identity=self.visual_map_identity,
            anchor_points=self.sparse.anchor_points[keep],
            anchor_descriptors=self.sparse.anchor_descriptors[keep],
            anchor_support=support[keep],
            keyframe_rotations_cw=rotations,
            keyframe_translations_cw=translations,
            duration_seconds=max(0.0, now - self._runtime_started_at),
            path_length_meters=path_length,
            anchor_conflicts=conflicts[keep],
            persistent_anchor_count=int(np.count_nonzero(keep[:persistent_anchor_count])),
        )
        try:
            generation_npz, _ = save_incremental_visual_map(
                self.incremental_map_npz,
                self.incremental_map_manifest,
                artifact,
                previous=getattr(self, "_incremental_baseline", None),
            )
        except (OSError, ValueError, VisualMapError) as error:
            print(f"VISUAL_MAP INCREMENTAL_SAVE_FAILED: {error}", flush=True)
            return False
        self._incremental_baseline = artifact
        print(
            "VISUAL_MAP INCREMENTAL_SAVED: "
            f"anchors={artifact.anchor_count}, "
            f"keyframes={artifact.keyframe_count}, "
            f"merged={getattr(self.sparse, 'merged_anchor_observations', 0)}, "
            f"quarantined={getattr(self.sparse, 'quarantined_anchor_count', 0)}, "
            f"generation={generation_npz.name}",
            flush=True,
        )
        return True

    def _has_pending_continuity_rejection(self) -> bool:
        continuity = getattr(self, "anchor_continuity", None)
        count = getattr(continuity, "consecutive_rejections", 0)
        return isinstance(count, (int, float)) and count > 0

    # 잘못 추정된 큰 이동과 스캔 영역 밖 pose가 지도 파일에 기록되지 않게 한다.
    def _incremental_keyframes_are_plausible(self, positions: np.ndarray) -> bool:
        if positions.ndim != 2 or positions.shape[1:] != (3,) or not np.isfinite(positions).all():
            return False
        if len(positions) >= 2:
            steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
            if not np.isfinite(steps).all() or np.any(steps > MAX_CAPTURE_PATH_STEP_METERS):
                return False

        identity = self.visual_map_identity
        if (
            identity.grid_cell_size_meters <= 0.0
            or identity.grid_width <= 0
            or identity.grid_height <= 0
        ):
            return True
        margin = max(0.5, identity.grid_cell_size_meters)
        maximum_x = (
            identity.grid_origin_x
            + identity.grid_width * identity.grid_cell_size_meters
        )
        maximum_z = (
            identity.grid_origin_z
            + identity.grid_height * identity.grid_cell_size_meters
        )
        return bool(
            np.all(positions[:, 0] >= identity.grid_origin_x - margin)
            and np.all(positions[:, 0] <= maximum_x + margin)
            and np.all(positions[:, 2] >= identity.grid_origin_z - margin)
            and np.all(positions[:, 2] <= maximum_z + margin)
        )


# 저장하지 않는 세션 검증은 삼각측량을 강체인 원본 보드 좌표에서 수행한다.
def _tracking_marker_pose(
    marker_pose,
    venue_position: np.ndarray,
    venue_rotation: np.ndarray,
    use_raw_board_pose: bool,
) -> tuple[np.ndarray, np.ndarray]:
    if use_raw_board_pose:
        return marker_pose.position, marker_pose.rotation
    return venue_position, venue_rotation


# 보드 중심 camera pose를 스캐너의 AprilTag ID0 원점 기준으로 변환한다.
def _board_pose_to_venue(
    board_to_venue: np.ndarray,
    position: np.ndarray,
    rotation: np.ndarray,
    quaternion_to_matrix,
    matrix_to_quaternion,
):
    position_center_to_id0 = np.eye(4, dtype=np.float64)
    position_center_to_id0[:3, :3] = np.diag([1.0, -1.0, 1.0])
    position_center_to_id0[:3, 3] = (0.095, 0.095, 0.0)
    position_transform = board_to_venue @ position_center_to_id0
    venue_position = (position_transform @ np.append(position, 1.0))[:3]

    rotation_center_to_id0 = np.diag([1.0, -1.0, -1.0])
    venue_rotation_matrix = (
        board_to_venue[:3, :3]
        @ rotation_center_to_id0
        @ quaternion_to_matrix(rotation)
    )
    venue_rotation = matrix_to_quaternion(venue_rotation_matrix)
    return venue_position, venue_rotation


# 최근 AprilTag pose와 물리적으로 불연속인 anchor pose를 거부한다.
def _anchor_pose_deltas(
    position: np.ndarray,
    rotation: np.ndarray,
    marker_position: np.ndarray | None,
    marker_rotation: np.ndarray | None,
) -> tuple[float, float]:
    if marker_position is None or marker_rotation is None:
        return math.inf, math.inf
    position_delta = float(np.linalg.norm(position - marker_position))
    first = rotation / max(float(np.linalg.norm(rotation)), 1e-12)
    second = marker_rotation / max(float(np.linalg.norm(marker_rotation)), 1e-12)
    rotation_dot = float(np.clip(abs(np.dot(first, second)), 0.0, 1.0))
    rotation_delta = math.degrees(2.0 * math.acos(rotation_dot))
    return position_delta, rotation_delta


def _anchor_pose_is_consistent(
    position: np.ndarray,
    rotation: np.ndarray,
    marker_position: np.ndarray | None,
    marker_rotation: np.ndarray | None,
) -> bool:
    position_delta, rotation_delta = _anchor_pose_deltas(
        position,
        rotation,
        marker_position,
        marker_rotation,
    )
    return (
        math.isfinite(position_delta)
        and math.isfinite(rotation_delta)
        and position_delta <= MAX_ANCHOR_POSITION_DELTA_METERS
        and rotation_delta <= MAX_ANCHOR_ROTATION_DELTA_DEGREES
    )


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="StrikerPC read-only Vision Adapter")
    parser.add_argument("--vision-root", default="Vision")
    parser.add_argument("--video-port", type=int, default=49000)
    parser.add_argument("--measurement-port", type=int, default=49003)
    parser.add_argument(
        "--hover-motion-port",
        type=int,
        default=49006,
        help="검증된 frame-motion을 Pilot Hover에 별도 전달할 UDP 포트입니다.",
    )
    parser.add_argument(
        "--calibration",
        default=str(
            root
            / "UnityClient"
            / "Assets"
            / "StreamingAssets"
            / "Maps"
            / "tello-camera-calibration.json"
        ),
    )
    parser.add_argument(
        "--map-path",
        default=str(root / "Assets" / "Maps" / "current" / "venue-map.json"),
    )
    parser.add_argument(
        "--visual-map-root",
        default=str(root / ".team-runtime" / "visual-maps"),
    )
    parser.add_argument("--capture-map", action="store_true")
    parser.add_argument(
        "--apriltag-only",
        action="store_true",
        help="AprilTag 좌표 정렬만 전송하고 희소 앵커 SLAM은 실행하지 않습니다.",
    )
    parser.add_argument(
        "--apriltag-refresh-cooldown",
        type=float,
        default=DEFAULT_APRILTAG_REFRESH_COOLDOWN_SECONDS,
        help="정상 4태그 정렬을 다시 반영하기 전 대기 시간(초)입니다.",
    )
    parser.add_argument(
        "--session-only",
        action="store_true",
        help="저장 visual map을 읽거나 쓰지 않고 현재 실행의 앵커만 사용합니다.",
    )
    parser.add_argument("--capture-minimum-duration", type=float, default=60.0)
    parser.add_argument("--capture-minimum-path", type=float, default=8.0)
    parser.add_argument("--capture-minimum-keyframes", type=int, default=12)
    parser.add_argument("--capture-minimum-anchors", type=int, default=250)
    parser.add_argument("--capture-timeout", type=float, default=600.0)
    parser.add_argument("--apriltag-rate", type=float, default=7.5)
    parser.add_argument("--orb-rate", type=float, default=7.5)
    parser.add_argument("--local-optimization-rate", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    adapter = VisionAdapter(parse_args())
    try:
        adapter.run()
    except KeyboardInterrupt:
        return 0
    finally:
        adapter.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
