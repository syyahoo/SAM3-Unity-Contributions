from __future__ import annotations

import argparse
import socket

import cv2

from vision_adapter import VisualLossMotionValidator
from vision_frame_tcp import LatestFrameTcpReader


class ImageHoverRuntime:
    """FPV 특징점의 프레임 간 이동만 Pilot 호버 보조로 전달한다."""

    def __init__(self, video_port: int, hover_motion_port: int) -> None:
        self.reader = LatestFrameTcpReader(video_port)
        self.motion = VisualLossMotionValidator()
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = ("127.0.0.1", hover_motion_port)

    def run(self) -> None:
        print("STRIKERPC_READY IMAGE_HOVER", flush=True)
        while True:
            shared = self.reader.read()
            if shared is None:
                continue
            frame = shared.frame
            if frame.ndim != 3 or frame.shape[2] != 3:
                self.motion.reset()
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            measurement = self.motion.update(
                gray,
                shared.sequence,
                shared.source_timestamp_ns,
                list(shared.dynamic_boxes),
            )
            if measurement is not None:
                self.sender.sendto(measurement.to_json(), self.target)

    def close(self) -> None:
        self.reader.close()
        self.sender.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="StrikerPC image-based hover tracking")
    parser.add_argument("--video-port", type=int, default=49000)
    parser.add_argument("--hover-motion-port", type=int, default=49006)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    runtime = ImageHoverRuntime(args.video_port, args.hover_motion_port)
    try:
        runtime.run()
    except KeyboardInterrupt:
        return 0
    finally:
        runtime.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
