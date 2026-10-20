from __future__ import annotations

import argparse
import importlib
import logging
import os
import sys
from pathlib import Path

from mission_platform_runtime import (
    _verify_import_path,
    load_livekit_environment,
    load_mission_source,
    verify_team_source,
)


class CommandCenterReadyLogHandler(logging.Handler):
    def __init__(self) -> None:
        super().__init__()
        self._ready_sent = False

    def emit(self, record: logging.LogRecord) -> None:
        if self._ready_sent:
            return
        if record.getMessage().startswith("Command Center is waiting for video tracks:"):
            self._ready_sent = True
            print("STRIKERPC_READY COMMAND_CENTER", flush=True)


# 신규 command_center를 고정 소스와 기존 LiveKit 설정으로 실행한다.
def run_command_center(args: argparse.Namespace) -> int:
    manifest_path = Path(args.manifest).resolve()
    mission_root, expected, manifest = load_mission_source(manifest_path)
    verify_team_source(mission_root, expected)
    pilot_root = (
        manifest_path.parent / manifest["repositories"]["pilot"]["path"]
    ).resolve()
    env_path = Path(args.env_file).resolve() if args.env_file else pilot_root / ".env"
    load_livekit_environment(env_path, args.identity, args.room)

    sys.path.insert(0, str(mission_root))
    try:
        command_center = importlib.import_module("apps.command_center.main")
        _verify_import_path(command_center, mission_root)

        command_args = [
            str(command_center.__file__),
            "--model",
            str(Path(args.model).resolve()),
            "--device",
            args.device,
            "--confidence",
            str(args.confidence),
        "--detect-fps",
        str(args.detect_fps),
        "--events",
        str(Path(args.events).resolve()),
        ]
        if args.preview:
            command_args.append("--preview")

        ready_handler = CommandCenterReadyLogHandler()
        logging.getLogger().addHandler(ready_handler)
        original_argv = sys.argv
        original_cwd = Path.cwd()
        try:
            sys.argv = command_args
            os.chdir(mission_root)
            command_center.main()
        finally:
            logging.getLogger().removeHandler(ready_handler)
            sys.argv = original_argv
            os.chdir(original_cwd)
    finally:
        sys.path.remove(str(mission_root))
    return 0


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="drone-mission-platform Command Center 통합 호스트")
    parser.add_argument("--manifest", default=str(root / "team-sources.json"))
    parser.add_argument("--env-file")
    parser.add_argument("--identity", default="c2-01")
    parser.add_argument("--room")
    parser.add_argument("--model", required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--detect-fps", type=float, default=10.0)
    parser.add_argument(
        "--events",
        default=str(root / ".team-runtime" / "mission-command-center-events.jsonl"),
    )
    parser.add_argument("--preview", action="store_true")
    return parser.parse_args()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    try:
        return run_command_center(parse_args())
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
