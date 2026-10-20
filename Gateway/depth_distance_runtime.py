from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from contextlib import nullcontext
from pathlib import Path

import cv2
import numpy as np

from depth_distance_protocol import (
    DepthDistanceSample,
    DepthDistanceSmoother,
    summarize_metric_depth,
)
from pilot_shared_frame import PilotSharedFrameReader


class MetricDepthEstimator:
    """로컬 Depth Anything V2 Metric Indoor 모델을 지연 로드한다."""

    def __init__(self, model_path: Path, device_name: str = "auto") -> None:
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation

        if not model_path.is_dir():
            raise FileNotFoundError(f"깊이 모델 폴더가 없습니다: {model_path}")
        if device_name == "auto":
            device_name = "cuda" if torch.cuda.is_available() else "cpu"
        if device_name == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA 장치를 사용할 수 없습니다.")

        self.torch = torch
        self.device = torch.device(device_name)
        self.processor = AutoImageProcessor.from_pretrained(
            model_path,
            local_files_only=True,
            use_fast=False,
        )
        self.model = AutoModelForDepthEstimation.from_pretrained(
            model_path,
            local_files_only=True,
        ).to(self.device)
        self.model.eval()

    # 한 BGR 프레임을 원본 해상도의 metric depth map으로 변환한다.
    def infer(self, frame: np.ndarray) -> tuple[np.ndarray, float]:
        torch = self.torch
        height, width = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb, return_tensors="pt")
        inputs = {key: value.to(self.device) for key, value in inputs.items()}
        autocast = (
            torch.autocast(device_type="cuda", dtype=torch.float16)
            if self.device.type == "cuda"
            else nullcontext()
        )
        started = time.perf_counter()
        with torch.inference_mode(), autocast:
            predicted = self.model(**inputs).predicted_depth
            resized = torch.nn.functional.interpolate(
                predicted.unsqueeze(1),
                size=(height, width),
                mode="bicubic",
                align_corners=False,
            ).squeeze(1)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        inference_ms = (time.perf_counter() - started) * 1000.0
        return resized[0].float().cpu().numpy(), inference_ms


# manifest의 SharedMemory 이름과 거리 HUD 포트를 읽는다.
def load_runtime_contract(manifest_path: Path) -> tuple[str, int]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    shared_memory_name = manifest.get("sharedMemory", {}).get(
        "video",
        "TelloPilotFrame_v2",
    )
    port = manifest.get("ports", {}).get("depthDistance")
    if not isinstance(shared_memory_name, str) or not shared_memory_name.strip():
        raise RuntimeError("team-sources.json의 sharedMemory.video가 올바르지 않습니다.")
    if not isinstance(port, int) or not 1 <= port <= 65535:
        raise RuntimeError("team-sources.json의 ports.depthDistance가 올바르지 않습니다.")
    return shared_memory_name, port


# 한 depth 결과를 Unity UDP 계약 표본으로 만든다.
def build_sample(
    sequence: int,
    frame_id: int,
    source_timestamp_ns: int,
    regions,
    inference_ms: float,
) -> DepthDistanceSample:
    return DepthDistanceSample(
        version=1,
        sequence=sequence,
        frame_id=frame_id,
        source_timestamp_ns=source_timestamp_ns,
        left_meters=regions.left_meters,
        center_meters=regions.center_meters,
        right_meters=regions.right_meters,
        coverage=regions.coverage,
        inference_ms=inference_ms,
    )


# 녹화 또는 정지 이미지 한 장의 추론 계약을 stdout으로 검증한다.
def run_image_once(estimator: MetricDepthEstimator, image_path: Path) -> int:
    frame = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"이미지를 읽을 수 없습니다: {image_path}")
    depth, inference_ms = estimator.infer(frame)
    regions = summarize_metric_depth(depth)
    if regions is None:
        raise RuntimeError("유효 깊이 픽셀 coverage가 부족합니다.")
    sample = build_sample(1, 1, time.perf_counter_ns(), regions, inference_ms)
    print(sample.to_json(), flush=True)
    return 0


# Pilot SharedMemory의 최신 프레임만 일정 주기로 추론해 localhost로 보낸다.
def run_live(
    estimator: MetricDepthEstimator,
    shared_memory_name: str,
    udp_port: int,
    frequency_hz: float,
) -> int:
    reader = PilotSharedFrameReader(shared_memory_name)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = ("127.0.0.1", udp_port)
    smoother = DepthDistanceSmoother()
    minimum_interval = 1.0 / max(1.0, frequency_hz)
    next_inference_at = 0.0
    sequence = 0
    last_warning_at = 0.0
    print("STRIKERPC_READY DEPTH_DISTANCE", flush=True)
    try:
        while True:
            success, frame = reader.read(timeout_seconds=0.2)
            if not success:
                continue
            now = time.monotonic()
            if now < next_inference_at:
                continue
            next_inference_at = now + minimum_interval

            depth, inference_ms = estimator.infer(frame)
            regions = summarize_metric_depth(depth)
            if regions is None:
                if now - last_warning_at >= 2.0:
                    print("[Depth Distance] 유효 깊이 coverage가 부족합니다.", file=sys.stderr)
                    last_warning_at = now
                continue

            sequence += 1
            smoothed = smoother.update(regions)
            sample = build_sample(
                sequence,
                reader.last_frame_id,
                reader.last_source_timestamp_ns,
                smoothed,
                inference_ms,
            )
            sender.sendto(sample.to_json().encode("utf-8"), destination)
    except KeyboardInterrupt:
        return 0
    finally:
        reader.close()
        sender.close()


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Pilot FPV용 CUDA 단안 metric depth 거리 보조 모듈",
    )
    parser.add_argument("--manifest", type=Path, default=root / "team-sources.json")
    parser.add_argument("--model-path", type=Path, required=True)
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--frequency-hz", type=float, default=5.0)
    parser.add_argument("--udp-port", type=int)
    parser.add_argument("--image", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    shared_memory_name, manifest_port = load_runtime_contract(args.manifest.resolve())
    udp_port = args.udp_port if args.udp_port is not None else manifest_port
    if not 1 <= udp_port <= 65535:
        raise RuntimeError("UDP 포트는 1에서 65535 사이여야 합니다.")
    estimator = MetricDepthEstimator(args.model_path.resolve(), args.device)
    if args.image is not None:
        return run_image_once(estimator, args.image.resolve())
    return run_live(estimator, shared_memory_name, udp_port, args.frequency_hz)


if __name__ == "__main__":
    raise SystemExit(main())
