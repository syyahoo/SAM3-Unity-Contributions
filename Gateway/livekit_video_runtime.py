from __future__ import annotations

import argparse
import asyncio
import importlib
import logging
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Callable

from mission_platform_runtime import (
    _verify_import_path,
    load_livekit_environment,
    load_mission_source,
    verify_team_source,
)
from pilot_runtime import create_resilient_video_publisher
from pilot_shared_frame import PilotSharedFrameReader


class LiveKitVideoReadyLogHandler(logging.Handler):
    def __init__(self) -> None:
        super().__init__()
        self._ready_sent = False

    # 실제 영상 track 발행 완료 로그를 실행기 준비 신호로 변환한다.
    def emit(self, record: logging.LogRecord) -> None:
        if self._ready_sent:
            return
        if record.getMessage().startswith("LiveKit video track published:"):
            self._ready_sent = True
            print("STRIKERPC_READY LIVEKIT_VIDEO", flush=True)


class TelloTelemetryReceiver:
    """Position Sync의 전용 relay에서 LiveKit 영상 telemetry를 복원한다."""

    def __init__(
        self,
        parser: Callable[[str], dict[str, int | float]],
        *,
        port: int = 8894,
        freshness_seconds: float = 5.0,
        clock: Callable[[], float] = time.monotonic,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self._parser = parser
        self._freshness_seconds = max(0.1, freshness_seconds)
        self._clock = clock
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._state: dict[str, int | float] = {}
        self._last_state_at = 0.0
        self._thread: threading.Thread | None = None
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind(("127.0.0.1", port))
            self._socket.settimeout(0.1)
        except Exception:
            self._socket.close()
            raise

    @property
    def address(self) -> tuple[str, int]:
        return self._socket.getsockname()

    # telemetry 수신 스레드를 한 번만 시작한다.
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="StrikerPCLiveKitTelemetry",
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                payload, _ = self._socket.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if not self._stop_event.is_set():
                    logging.getLogger(__name__).warning(
                        "LiveKit telemetry relay stopped unexpectedly"
                    )
                return
            state = self._parser(payload.decode("ascii", errors="replace"))
            if not state:
                continue
            with self._lock:
                self._state = dict(state)
                self._last_state_at = self._clock()

    # 기존 TelloAdapter.telemetry와 같은 connected/age 필드를 제공한다.
    def snapshot(self) -> dict[str, int | float | bool]:
        with self._lock:
            state = dict(self._state)
            last_state_at = self._last_state_at
        age = self._clock() - last_state_at if last_state_at else float("inf")
        state["connected"] = bool(last_state_at) and age <= self._freshness_seconds
        if last_state_at:
            state["state_age_ms"] = round(age * 1000)
        return state

    # 전용 UDP relay와 수신 스레드를 종료한다.
    def close(self) -> None:
        self._stop_event.set()
        self._socket.close()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None


# 로컬 Shared Memory를 선택적 LiveKit 영상 track으로 발행한다.
async def run_livekit_video(args: argparse.Namespace) -> int:
    manifest_path = Path(args.manifest).resolve()
    mission_root, expected, manifest = load_mission_source(manifest_path)
    if manifest.get("runtime", {}).get("pilotProvider") != "missionPlatform":
        raise RuntimeError("livekit-video 모듈은 missionPlatform Pilot에서만 지원합니다.")
    verify_team_source(mission_root, expected)

    runtime = manifest.get("runtime", {})
    env_path = (
        Path(args.env_file).resolve()
        if args.env_file
        else (manifest_path.parent / runtime["missionPlatformEnv"]).resolve()
    )
    identity = args.identity or manifest["livekit"]["videoPublisherIdentity"]
    track_name = args.track_name or manifest["livekit"]["videoTrackName"]
    shared_video_name = args.shared_video_name or manifest["sharedMemory"]["video"]
    load_livekit_environment(env_path, identity, args.room)

    sys.path.insert(0, str(mission_root))
    capture: PilotSharedFrameReader | None = None
    telemetry: TelloTelemetryReceiver | None = None
    ready_handler = LiveKitVideoReadyLogHandler()
    logging.getLogger().addHandler(ready_handler)
    try:
        video_module = importlib.import_module("apps.strike_gateway.video_publisher")
        tello_module = importlib.import_module("adapters.tello_adapter")
        channels_module = importlib.import_module("shared.protocol.channel_names")
        for module in (video_module, tello_module, channels_module):
            _verify_import_path(module, mission_root)

        capture = PilotSharedFrameReader(shared_video_name)
        telemetry = TelloTelemetryReceiver(
            tello_module.parse_tello_state,
            port=args.state_port,
        )
        telemetry.start()
        publisher_type = create_resilient_video_publisher(video_module.VideoPublisher)
        publisher = publisher_type(
            channel_name=track_name,
            identity=identity,
            telemetry=telemetry.snapshot,
            telemetry_topic=channels_module.TELEMETRY_STRIKE,
        )
        stop_event = asyncio.Event()
        await publisher.run(capture, stop_event)
    finally:
        logging.getLogger().removeHandler(ready_handler)
        if telemetry is not None:
            telemetry.close()
        if capture is not None:
            capture.close()
        sys.path.remove(str(mission_root))
    return 0


# 실행 옵션을 읽는다.
def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="StrikerPC 선택형 LiveKit 영상 발행")
    parser.add_argument("--manifest", default=str(root / "team-sources.json"))
    parser.add_argument("--env-file")
    parser.add_argument("--identity")
    parser.add_argument("--room")
    parser.add_argument("--track-name")
    parser.add_argument("--shared-video-name")
    parser.add_argument("--state-port", type=int, default=8894)
    return parser.parse_args()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    try:
        return asyncio.run(run_livekit_video(parse_args()))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
