from __future__ import annotations

import argparse
from pathlib import Path

from huggingface_hub import snapshot_download


MODEL_ID = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"
REQUIRED_FILES = (
    "config.json",
    "preprocessor_config.json",
    "model.safetensors",
)


# 검증된 공식 모델 파일만 프로젝트 로컬 런타임 폴더에 내려받는다.
def install_model(model_path: Path) -> None:
    model_path.mkdir(parents=True, exist_ok=True)
    snapshot_download(
        repo_id=MODEL_ID,
        local_dir=model_path,
        allow_patterns=[*REQUIRED_FILES, "README.md"],
    )
    missing = [name for name in REQUIRED_FILES if not (model_path / name).is_file()]
    if missing:
        raise RuntimeError("깊이 모델 파일이 누락되었습니다: " + ", ".join(missing))


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="거리 보조용 metric depth 모델 설치")
    parser.add_argument(
        "--model-path",
        type=Path,
        default=root / ".team-runtime" / "models" / "depth-distance",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    destination = args.model_path.resolve()
    install_model(destination)
    print(f"Depth model ready: {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
