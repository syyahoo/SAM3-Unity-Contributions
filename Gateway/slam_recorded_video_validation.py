from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np


SOURCE_KIND = "recorded-oculus-fpv-crop"
DEFAULT_CROP = (545, 360, 800, 540)
DEFAULT_START_SECONDS = 58.0
DEFAULT_END_SECONDS = 76.5
DEFAULT_OUTPUT_SIZE = (960, 720)
DEFAULT_RATE_DIVISOR = 4
DEFAULT_BOTTOM_MASK_PIXELS = 170


# 녹화 파일이 바뀌지 않았음을 보고서에 남긴다.
def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


# Vision 팀 런타임의 BoardTracker와 운영 SparseAnchorTracker를 불러온다.
def load_production_trackers(root: Path, calibration_path: Path):
    gateway_root = root / "Gateway"
    vision_source = (
        root
        / "Vision"
        / "Assets"
        / "StreamingAssets"
        / "TelloVisionBridge"
        / "Source"
    )
    for module_root in (gateway_root, vision_source):
        value = str(module_root)
        if value not in sys.path:
            sys.path.insert(0, value)

    from tello_vision_bridge import BoardTracker, quaternion_to_matrix
    from vision_adapter import (
        MAP_MAINTENANCE_MIN_INLIERS,
        MAP_READY_MAX_REPROJECTION_ERROR,
        SparseAnchorTracker,
    )

    board_args = argparse.Namespace(
        tag_family="tag36h11",
        tag_size=0.117,
        tag_spacing=0.190,
        tag_ids="0,1,2,3",
        max_reprojection_error=3.0,
    )
    board = BoardTracker(board_args, calibration_path)
    if not board.calibration_ready:
        raise RuntimeError("Tello 카메라 보정값을 읽을 수 없습니다.")
    sparse = SparseAnchorTracker(
        board.camera_matrix,
        board._distortion,
        max_features=1500,
        max_anchors=3000,
    )
    return (
        board,
        sparse,
        quaternion_to_matrix,
        int(MAP_MAINTENANCE_MIN_INLIERS),
        float(MAP_READY_MAX_REPROJECTION_ERROR),
    )


# XR 화면 녹화에만 존재하는 하단 알림과 포인터를 특징점 입력에서 제외한다.
def recording_overlay_boxes(
    width: int,
    height: int,
    bottom_mask_pixels: int,
) -> list[tuple[int, int, int, int]]:
    masked_height = min(height, max(0, bottom_mask_pixels))
    if masked_height == 0:
        return []
    return [(0, height - masked_height, width, height)]


# 운영 추적점과 보드 이탈 상태를 녹화 프레임 위에 표시한다.
def draw_tracking_overlay(
    frame: np.ndarray,
    *,
    timestamp_seconds: float,
    points: np.ndarray,
    anchor_count: int,
    detection_count: int,
    marker_locked: bool,
    operational_tracking: bool,
    raw_tracking: bool,
    inlier_count: int,
    reprojection_error: float | None,
    bottom_mask_pixels: int,
) -> np.ndarray:
    overlay = frame.copy()
    selected = np.asarray(points, np.float32).reshape(-1, 2)
    if len(selected) > 100:
        selected = selected[
            np.linspace(0, len(selected) - 1, 100).astype(np.int32)
        ]
    for index, point in enumerate(selected):
        x, y = int(round(float(point[0]))), int(round(float(point[1])))
        cv2.circle(overlay, (x, y), 4, (50, 245, 255), 1, cv2.LINE_AA)
        cv2.circle(overlay, (x, y), 1, (30, 255, 80), -1, cv2.LINE_AA)
        if index % 15 == 0:
            cv2.putText(
                overlay,
                f"A{index:02d}",
                (x + 5, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (50, 245, 255),
                1,
                cv2.LINE_AA,
            )

    if marker_locked:
        phase = "BOARD LOCK / MAP BUILD"
        color = (40, 220, 255)
    elif detection_count > 0:
        phase = "BOARD DETECTING"
        color = (40, 220, 255)
    elif operational_tracking:
        phase = "BOARD HIDDEN / ANCHOR TRACKING"
        color = (60, 255, 90)
    elif raw_tracking:
        phase = "BOARD HIDDEN / WEAK ANCHOR POSE"
        color = (40, 180, 255)
    else:
        phase = "BOARD HIDDEN / TRACKING LOST"
        color = (60, 60, 255)

    cv2.rectangle(overlay, (0, 0), (overlay.shape[1], 46), (9, 13, 18), -1)
    cv2.putText(
        overlay,
        f"RECORDED FPV SLAM  |  {phase}",
        (16, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        color,
        2,
        cv2.LINE_AA,
    )
    if bottom_mask_pixels > 0:
        top = overlay.shape[0] - bottom_mask_pixels
        shaded = overlay.copy()
        cv2.rectangle(
            shaded,
            (0, top),
            (overlay.shape[1], overlay.shape[0]),
            (8, 8, 8),
            -1,
        )
        overlay = cv2.addWeighted(shaded, 0.72, overlay, 0.28, 0.0)
        cv2.putText(
            overlay,
            "XR RECORDING UI MASKED",
            (16, top + 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (190, 190, 190),
            1,
            cv2.LINE_AA,
        )

    error_text = "-" if reprojection_error is None else f"{reprojection_error:.2f}px"
    cv2.putText(
        overlay,
        (
            f"t={timestamp_seconds:06.2f}s  anchors={anchor_count}  "
            f"inliers={inlier_count}  error={error_text}"
        ),
        (16, overlay.shape[0] - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (245, 245, 245),
        2,
        cv2.LINE_AA,
    )
    return overlay


# 중앙 FPV 크롭을 연속 디코딩해 보드 이탈 후 운영 SLAM 기준을 측정한다.
def run_validation(
    video_path: Path,
    calibration_path: Path,
    output_dir: Path,
    *,
    crop: tuple[int, int, int, int] = DEFAULT_CROP,
    start_seconds: float = DEFAULT_START_SECONDS,
    end_seconds: float = DEFAULT_END_SECONDS,
    output_size: tuple[int, int] = DEFAULT_OUTPUT_SIZE,
    rate_divisor: int = DEFAULT_RATE_DIVISOR,
    bottom_mask_pixels: int = DEFAULT_BOTTOM_MASK_PIXELS,
    overwrite: bool = False,
) -> dict:
    video_path = video_path.resolve()
    calibration_path = calibration_path.resolve()
    output_dir = output_dir.resolve()
    if not video_path.is_file():
        raise FileNotFoundError(video_path)
    if end_seconds <= start_seconds:
        raise ValueError("종료 시각은 시작 시각보다 커야 합니다.")
    if rate_divisor < 1:
        raise ValueError("rate-divisor는 1 이상이어야 합니다.")
    if output_dir.exists() and overwrite:
        shutil.rmtree(output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"출력 폴더가 비어 있지 않습니다: {output_dir}")
    overlay_dir = output_dir / "overlay"
    overlay_dir.mkdir(parents=True, exist_ok=True)

    root = Path(__file__).resolve().parent.parent
    (
        board,
        sparse,
        quaternion_to_matrix,
        minimum_inliers,
        maximum_reprojection_error,
    ) = load_production_trackers(root, calibration_path)
    from vision_adapter import _detected_board_boxes

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"MP4를 열 수 없습니다: {video_path}")
    source_fps = float(capture.get(cv2.CAP_PROP_FPS))
    source_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    source_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    x, y, crop_width, crop_height = crop
    if (
        x < 0
        or y < 0
        or crop_width < 1
        or crop_height < 1
        or x + crop_width > source_width
        or y + crop_height > source_height
    ):
        capture.release()
        raise ValueError("FPV 크롭이 원본 영상 범위를 벗어납니다.")

    output_width, output_height = output_size
    dynamic_boxes = recording_overlay_boxes(
        output_width,
        output_height,
        bottom_mask_pixels,
    )
    capture.set(cv2.CAP_PROP_POS_MSEC, start_seconds * 1000.0)
    frames: list[dict] = []
    frame_index = 0
    last_detection_count = 0
    board_lock_frame = -1
    board_hidden_start_frame = -1
    try:
        while True:
            success, source_frame = capture.read()
            if not success:
                break
            timestamp_seconds = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000.0
            if timestamp_seconds > end_seconds:
                break

            cropped = source_frame[y : y + crop_height, x : x + crop_width]
            frame = cv2.resize(
                cropped,
                (output_width, output_height),
                interpolation=cv2.INTER_AREA,
            )
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sampled = frame_index % rate_divisor == 0
            marker_pose = None
            if sampled:
                marker_pose, _ = board.track(frame)
                last_detection_count = len(getattr(board, "_last_detections", []))
            board_boxes = (
                _detected_board_boxes(
                    getattr(board, "_last_detections", ()),
                    frame.shape,
                )
            )
            tracking_exclusion_boxes = [*dynamic_boxes, *board_boxes]

            tracked = sparse.track(
                gray,
                sampled,
                tracking_exclusion_boxes,
                prefer_temporal_track=True,
            )
            if marker_pose is not None:
                if board_lock_frame < 0:
                    board_lock_frame = frame_index
                sparse.update_marker_keyframe(
                    gray,
                    marker_pose.position,
                    marker_pose.rotation,
                    quaternion_to_matrix,
                    tracking_exclusion_boxes,
                    force=len(sparse.keyframes) == 0,
                )
            elif (
                board_lock_frame >= 0
                and board_hidden_start_frame < 0
                and sampled
                and last_detection_count == 0
            ):
                board_hidden_start_frame = frame_index

            raw_tracking = tracked is not None
            inlier_count = 0 if tracked is None else int(tracked.inlier_count)
            reprojection_error = (
                None if tracked is None else float(tracked.reprojection_error)
            )
            operational_tracking = bool(
                tracked is not None
                and inlier_count >= minimum_inliers
                and reprojection_error is not None
                and reprojection_error <= maximum_reprojection_error
            )
            points = sparse.tracked_pixels.copy()
            overlay = draw_tracking_overlay(
                frame,
                timestamp_seconds=timestamp_seconds,
                points=points,
                anchor_count=int(sparse.anchor_count),
                detection_count=last_detection_count,
                marker_locked=marker_pose is not None,
                operational_tracking=operational_tracking,
                raw_tracking=raw_tracking,
                inlier_count=inlier_count,
                reprojection_error=reprojection_error,
                bottom_mask_pixels=bottom_mask_pixels,
            )
            overlay_path = overlay_dir / f"frame-{frame_index:04d}.jpg"
            cv2.imwrite(
                str(overlay_path),
                overlay,
                [cv2.IMWRITE_JPEG_QUALITY, 82],
            )
            frames.append(
                {
                    "index": frame_index,
                    "timestampSeconds": timestamp_seconds,
                    "overlayFile": overlay_path.relative_to(output_dir).as_posix(),
                    "markerSampled": sampled,
                    "boardDetectionCount": last_detection_count,
                    "markerLocked": marker_pose is not None,
                    "anchorCount": int(sparse.anchor_count),
                    "rawTracking": raw_tracking,
                    "operationalTracking": operational_tracking,
                    "inlierCount": inlier_count,
                    "reprojectionErrorPixels": reprojection_error,
                }
            )
            frame_index += 1
    finally:
        capture.release()

    if board_lock_frame < 0:
        raise RuntimeError("선택 구간에서 AprilTag 보드 pose를 잠그지 못했습니다.")
    if board_hidden_start_frame < 0:
        raise RuntimeError("보드 잠금 이후 완전히 가려지는 구간을 찾지 못했습니다.")

    hidden_frames = frames[board_hidden_start_frame:]
    operational_frames = [item for item in hidden_frames if item["operationalTracking"]]
    raw_frames = [item for item in hidden_frames if item["rawTracking"]]
    tracking_dropouts = len(hidden_frames) - len(operational_frames)
    first_tracking_lost_frame = next(
        (
            int(item["index"])
            for item in hidden_frames
            if not item["operationalTracking"]
        ),
        -1,
    )
    last_operational_frame = (
        int(operational_frames[-1]["index"]) if operational_frames else -1
    )
    maintained_seconds = (
        float(operational_frames[-1]["timestampSeconds"])
        - float(hidden_frames[0]["timestampSeconds"])
        if operational_frames
        else 0.0
    )
    report = {
        "version": 1,
        "source": SOURCE_KIND,
        "sourceVideo": str(video_path),
        "sourceVideoSha256": sha256_path(video_path),
        "productionTracker": "Gateway.vision_adapter.SparseAnchorTracker",
        "boardTracker": "Vision.tello_vision_bridge.BoardTracker",
        "metricPoseEligible": False,
        "metricPoseBlockingReason": (
            "XR 화면 녹화 크롭을 960x720으로 재표본화했으므로 실제 Tello 렌즈 "
            "캘리브레이션 기반 절대 위치 정확도는 판정하지 않습니다."
        ),
        "sourceWidth": source_width,
        "sourceHeight": source_height,
        "sourceFps": source_fps,
        "cropX": x,
        "cropY": y,
        "cropWidth": crop_width,
        "cropHeight": crop_height,
        "imageWidth": output_width,
        "imageHeight": output_height,
        "startSeconds": start_seconds,
        "endSeconds": end_seconds,
        "frameCount": len(frames),
        "rateDivisor": rate_divisor,
        "bottomMaskPixels": bottom_mask_pixels,
        "minimumOperationalInliers": minimum_inliers,
        "maximumOperationalReprojectionErrorPixels": maximum_reprojection_error,
        "boardLockFrame": board_lock_frame,
        "boardHiddenStartFrame": board_hidden_start_frame,
        "boardHiddenFrameCount": len(hidden_frames),
        "rawTrackedFrameCount": len(raw_frames),
        "operationalTrackedFrameCount": len(operational_frames),
        "trackingDropouts": tracking_dropouts,
        "firstTrackingLostFrame": first_tracking_lost_frame,
        "lastOperationalFrame": last_operational_frame,
        "maintainedAfterBoardHiddenSeconds": maintained_seconds,
        "anchorCount": int(sparse.anchor_count),
        "keyframeCount": len(sparse.keyframes),
        "passed": tracking_dropouts == 0 and bool(operational_frames),
        "frames": frames,
    }
    (output_dir / "slam-recorded-video-report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return report


# MP4 진단 실행 인자를 구성한다.
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description="Oculus 녹화 중앙 FPV의 보드 이탈 후 SLAM 연속성 진단"
    )
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=root
        / "UnityClient"
        / "Assets"
        / "StreamingAssets"
        / "Maps"
        / "tello-camera-calibration.json",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=root / ".team-runtime" / "slam-recorded-video-validation",
    )
    parser.add_argument("--crop", nargs=4, type=int, default=DEFAULT_CROP)
    parser.add_argument("--start-seconds", type=float, default=DEFAULT_START_SECONDS)
    parser.add_argument("--end-seconds", type=float, default=DEFAULT_END_SECONDS)
    parser.add_argument("--rate-divisor", type=int, default=DEFAULT_RATE_DIVISOR)
    parser.add_argument(
        "--bottom-mask-pixels",
        type=int,
        default=DEFAULT_BOTTOM_MASK_PIXELS,
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


# 보고서 핵심 수치를 표준 출력으로 반환한다.
def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_validation(
        args.video,
        args.calibration,
        args.output_dir,
        crop=tuple(args.crop),
        start_seconds=args.start_seconds,
        end_seconds=args.end_seconds,
        rate_divisor=args.rate_divisor,
        bottom_mask_pixels=args.bottom_mask_pixels,
        overwrite=args.overwrite,
    )
    print(
        json.dumps(
            {
                key: report[key]
                for key in (
                    "frameCount",
                    "boardHiddenFrameCount",
                    "operationalTrackedFrameCount",
                    "trackingDropouts",
                    "maintainedAfterBoardHiddenSeconds",
                    "anchorCount",
                    "keyframeCount",
                    "passed",
                )
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
