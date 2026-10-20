from __future__ import annotations

import argparse
import json
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from vision_adapter import SparseAnchorTracker


DEFAULT_SEED = 20260814
BOARD_VISIBLE_FRAMES = 15
TRACKING_MIN_INLIERS = 12
TRACKING_MAX_REPROJECTION_ERROR = 2.0
BOARD_BOX = (38, 48, 274, 304)


@dataclass(frozen=True)
class CameraContract:
    width: int
    height: int
    camera_matrix: np.ndarray
    distortion: np.ndarray


# 운영 Tello 보정 파일에서 영상 크기와 내부 파라미터를 읽는다.
def load_camera_contract(path: Path) -> CameraContract:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not data.get("calibrated"):
        raise ValueError("보정이 완료된 Tello 카메라 파일이 필요합니다.")
    return CameraContract(
        width=int(data["image_width"]),
        height=int(data["image_height"]),
        camera_matrix=np.asarray(data["camera_matrix"], np.float64).reshape(3, 3),
        distortion=np.asarray(data["distortion_coefficients"], np.float64),
    )


# 반복 무늬를 피한 고대비 표면을 만들어 ORB 식별점을 안정적으로 제공한다.
def build_anchor_surface(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    surface = rng.normal(118, 52, (1200, 1800, 3)).clip(0, 255).astype(np.uint8)
    for y in range(40, 1160, 80):
        for x in range(40, 1760, 80):
            color = tuple(int(value) for value in rng.integers(30, 240, 3))
            inverse = tuple(255 - value for value in color)
            cv2.rectangle(surface, (x - 18, y - 18), (x + 18, y + 18), color, 2)
            cv2.line(surface, (x - 15, y - 12), (x + 14, y + 13), inverse, 2)
            cv2.circle(surface, (x, y), 5, (0, 0, 0), -1)
    cv2.putText(
        surface,
        "STRIKER SLAM VISUAL ANCHOR FIELD",
        (70, 100),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.4,
        (255, 255, 255),
        3,
        cv2.LINE_AA,
    )
    return surface


# 보정 영상에서 왜곡 영상으로 가는 고정 remap을 계산한다.
def build_distortion_map(contract: CameraContract) -> tuple[np.ndarray, np.ndarray]:
    rows, columns = np.indices((contract.height, contract.width), dtype=np.float32)
    pixels = np.stack((columns, rows), axis=-1).reshape(-1, 1, 2)
    mapped = cv2.undistortPoints(
        pixels,
        contract.camera_matrix,
        contract.distortion,
        P=contract.camera_matrix,
    ).reshape(contract.height, contract.width, 2)
    return mapped[:, :, 0].astype(np.float32), mapped[:, :, 1].astype(np.float32)


# 초기 구간에만 보이는 2x2 보드 표식을 영상 좌상단에 합성한다.
def draw_board(frame: np.ndarray) -> None:
    left, top, right, bottom = BOARD_BOX
    cv2.rectangle(frame, (left, top), (right, bottom), (235, 235, 235), -1)
    cv2.rectangle(frame, (left, top), (right, bottom), (20, 20, 20), 4)
    tag_size = 82
    for index, (x, y) in enumerate(
        ((58, 70), (160, 70), (58, 172), (160, 172))
    ):
        cv2.rectangle(frame, (x, y), (x + tag_size, y + tag_size), (5, 5, 5), -1)
        cv2.rectangle(
            frame,
            (x + 14, y + 14),
            (x + tag_size - 14, y + tag_size - 14),
            (245, 245, 245),
            -1,
        )
        cv2.rectangle(
            frame,
            (x + 28, y + 28),
            (x + tag_size - 28, y + tag_size - 28),
            (5, 5, 5),
            -1,
        )
        cv2.putText(
            frame,
            str(index),
            (x + 31, y + 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    cv2.putText(
        frame,
        "APRILTAG BOARD",
        (56, 289),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (20, 20, 20),
        2,
        cv2.LINE_AA,
    )


# 이동 카메라 한 프레임을 960x720 저화질 JPEG RGB 프록시로 만든다.
def render_rgb_frame(
    frame_index: int,
    surface: np.ndarray,
    contract: CameraContract,
    distortion_map: tuple[np.ndarray, np.ndarray],
    jpeg_quality: int,
    seed: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    camera_position = np.asarray(
        [0.018 * frame_index, 0.04 * np.sin(frame_index / 13.0), 0.0],
        np.float64,
    )
    rotation_cw = np.diag([-1.0, -1.0, 1.0])
    world_corners = np.asarray(
        [[-4.0, -3.0, 4.0], [4.0, -3.0, 4.0], [4.0, 3.0, 4.0], [-4.0, 3.0, 4.0]],
        np.float64,
    )
    camera_corners = (rotation_cw @ (world_corners - camera_position).T).T
    projected = (contract.camera_matrix @ camera_corners.T).T
    projected = (projected[:, :2] / projected[:, 2:3]).astype(np.float32)
    source_corners = np.asarray(
        [[0.0, 0.0], [1799.0, 0.0], [1799.0, 1199.0], [0.0, 1199.0]],
        np.float32,
    )
    homography = cv2.getPerspectiveTransform(source_corners, projected)
    undistorted = cv2.warpPerspective(
        surface,
        homography,
        (contract.width, contract.height),
        borderValue=(18, 22, 26),
    )
    frame = cv2.remap(
        undistorted,
        distortion_map[0],
        distortion_map[1],
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(18, 22, 26),
    )
    brightness = 0.94 + 0.08 * np.sin(frame_index / 9.0)
    frame = np.clip(frame.astype(np.float32) * brightness, 0, 255).astype(np.uint8)
    noise = np.random.default_rng(seed + frame_index).normal(0, 2.2, frame.shape)
    frame = np.clip(frame.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    frame = cv2.GaussianBlur(frame, (3, 3), 0.6)
    if frame_index < BOARD_VISIBLE_FRAMES:
        draw_board(frame)
    success, encoded = cv2.imencode(
        ".jpg",
        frame,
        [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
    )
    if not success:
        raise RuntimeError("더미 Tello JPEG 프레임 인코딩에 실패했습니다.")
    decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    translation_cw = -rotation_cw @ camera_position
    return decoded, camera_position, rotation_cw, translation_cw


# 추적 중인 식별점과 상태를 실제 RGB 프레임 위에 그린다.
def draw_tracking_overlay(
    frame: np.ndarray,
    frame_index: int,
    points: np.ndarray,
    anchor_count: int,
    inlier_count: int,
    reprojection_error: float | None,
) -> np.ndarray:
    overlay = frame.copy()
    selected = np.asarray(points, np.float32).reshape(-1, 2)
    if len(selected) > 80:
        selected = selected[np.linspace(0, len(selected) - 1, 80).astype(np.int32)]
    for index, point in enumerate(selected):
        x, y = int(round(float(point[0]))), int(round(float(point[1])))
        cv2.circle(overlay, (x, y), 4, (50, 245, 255), 1, cv2.LINE_AA)
        cv2.circle(overlay, (x, y), 1, (30, 255, 80), -1, cv2.LINE_AA)
        if index % 12 == 0:
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
    board_visible = frame_index < BOARD_VISIBLE_FRAMES
    phase = "BOARD VISIBLE / BUILDING MAP" if board_visible else "BOARD HIDDEN / ANCHOR TRACKING"
    color = (40, 220, 255) if board_visible else (60, 255, 90)
    cv2.rectangle(overlay, (0, 0), (overlay.shape[1], 43), (9, 13, 18), -1)
    cv2.putText(
        overlay,
        f"SLAM RGB REPLAY  |  {phase}",
        (16, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        color,
        2,
        cv2.LINE_AA,
    )
    error_text = "-" if reprojection_error is None else f"{reprojection_error:.2f}px"
    cv2.putText(
        overlay,
        f"frame {frame_index:03d}   anchors {anchor_count}   inliers {inlier_count}   error {error_text}",
        (16, overlay.shape[0] - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (245, 245, 245),
        2,
        cv2.LINE_AA,
    )
    return overlay


# 운영 SparseAnchorTracker로 보드 이탈 전후 시퀀스를 실행하고 증거 파일을 기록한다.
def run_validation(
    calibration_path: Path,
    output_dir: Path,
    *,
    frame_count: int = 60,
    jpeg_quality: int = 55,
    seed: int = DEFAULT_SEED,
    overwrite: bool = False,
) -> dict:
    if frame_count < 24:
        raise ValueError("보드 이탈 추적에는 최소 24프레임이 필요합니다.")
    if output_dir.exists() and overwrite:
        shutil.rmtree(output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"출력 폴더가 비어 있지 않습니다: {output_dir}")
    rgb_dir = output_dir / "rgb"
    overlay_dir = output_dir / "overlay"
    rgb_dir.mkdir(parents=True, exist_ok=True)
    overlay_dir.mkdir(parents=True, exist_ok=True)

    contract = load_camera_contract(calibration_path)
    surface = build_anchor_surface(seed)
    distortion_map = build_distortion_map(contract)
    tracker = SparseAnchorTracker(
        contract.camera_matrix,
        contract.distortion,
        max_features=1500,
        max_anchors=3000,
    )
    quaternion = np.asarray([0.0, 0.0, 0.0, 1.0], np.float64)
    quaternion_to_matrix = lambda _value: np.eye(3, dtype=np.float64)
    seed_indices = {0, BOARD_VISIBLE_FRAMES - 1}
    frames: list[dict] = []
    failures = 0
    inlier_values: list[int] = []
    reprojection_values: list[float] = []
    position_errors: list[float] = []

    for frame_index in range(frame_count):
        frame, expected_position, rotation_cw, _ = render_rgb_frame(
            frame_index,
            surface,
            contract,
            distortion_map,
            jpeg_quality,
            seed,
        )
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tracked = None
        points = np.empty((0, 2), np.float32)
        if frame_index in seed_indices:
            tracker.update_marker_keyframe(
                gray,
                expected_position,
                quaternion,
                quaternion_to_matrix,
                [BOARD_BOX],
                force=frame_index == 0,
            )
            if frame_index == BOARD_VISIBLE_FRAMES - 1 and tracker.anchor_count:
                rvec, _ = cv2.Rodrigues(rotation_cw)
                translation_cw = -rotation_cw @ expected_position
                projected, _ = cv2.projectPoints(
                    tracker.anchor_points,
                    rvec,
                    translation_cw,
                    contract.camera_matrix,
                    contract.distortion,
                )
                points = projected.reshape(-1, 2)
        elif frame_index >= BOARD_VISIBLE_FRAMES:
            tracked = tracker.track(
                gray,
                run_descriptor_match=frame_index % 8 == 0,
                dynamic_boxes=[],
                prefer_temporal_track=True,
            )
            points = tracker.tracked_pixels.copy()

        inliers = 0 if tracked is None else int(tracked.inlier_count)
        reprojection_error = (
            None if tracked is None else float(tracked.reprojection_error)
        )
        position_error = None
        if frame_index >= BOARD_VISIBLE_FRAMES:
            if (
                tracked is None
                or tracked.inlier_count < TRACKING_MIN_INLIERS
                or tracked.reprojection_error > TRACKING_MAX_REPROJECTION_ERROR
            ):
                failures += 1
            else:
                estimated_position = (
                    -tracked.rotation_cw.T @ tracked.translation_cw
                )
                position_error = float(
                    np.linalg.norm(estimated_position - expected_position)
                )
                inlier_values.append(inliers)
                reprojection_values.append(reprojection_error)
                position_errors.append(position_error)

        rgb_path = rgb_dir / f"frame-{frame_index:03d}.jpg"
        overlay_path = overlay_dir / f"frame-{frame_index:03d}.jpg"
        cv2.imwrite(str(rgb_path), frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        overlay = draw_tracking_overlay(
            frame,
            frame_index,
            points,
            tracker.anchor_count,
            inliers,
            reprojection_error,
        )
        cv2.imwrite(str(overlay_path), overlay, [cv2.IMWRITE_JPEG_QUALITY, 82])
        frames.append(
            {
                "index": frame_index,
                "rgbFile": rgb_path.relative_to(output_dir).as_posix(),
                "overlayFile": overlay_path.relative_to(output_dir).as_posix(),
                "boardVisible": frame_index < BOARD_VISIBLE_FRAMES,
                "anchorCount": int(tracker.anchor_count),
                "inlierCount": inliers,
                "reprojectionErrorPixels": reprojection_error,
                "positionErrorMeters": position_error,
            }
        )

    tracked_frame_count = frame_count - BOARD_VISIBLE_FRAMES
    report = {
        "version": 1,
        "source": "deterministic-synthetic-tello-rgb-proxy",
        "productionTracker": "Gateway.vision_adapter.SparseAnchorTracker",
        "imageWidth": contract.width,
        "imageHeight": contract.height,
        "jpegQuality": jpeg_quality,
        "frameCount": frame_count,
        "boardVisibleFrameCount": BOARD_VISIBLE_FRAMES,
        "trackedFrameCount": tracked_frame_count,
        "successfulTrackedFrames": tracked_frame_count - failures,
        "trackingDropouts": failures,
        "anchorCount": int(tracker.anchor_count),
        "minimumInliers": min(inlier_values) if inlier_values else 0,
        "maximumReprojectionErrorPixels": (
            max(reprojection_values) if reprojection_values else None
        ),
        "maximumPositionErrorMeters": max(position_errors) if position_errors else None,
        "passed": failures == 0 and bool(inlier_values),
        "frames": frames,
    }
    (output_dir / "slam-visual-replay-report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return report


# 검증용 명령행 인자를 구성한다.
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description="보드 이탈 후 RGB 비주얼 앵커 연속 추적 검증"
    )
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
        default=root / ".team-runtime" / "slam-visual-validation",
    )
    parser.add_argument("--frame-count", type=int, default=60)
    parser.add_argument("--jpeg-quality", type=int, default=55)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


# 검증을 실행하고 핵심 수치를 표준 출력으로 반환한다.
def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = run_validation(
        args.calibration,
        args.output_dir,
        frame_count=args.frame_count,
        jpeg_quality=args.jpeg_quality,
        seed=args.seed,
        overwrite=args.overwrite,
    )
    print(json.dumps({key: value for key, value in report.items() if key != "frames"}))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
