from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path


LINKS = (
    ("vision", "Assets/TelloVisionTracking", "UnityClient/Assets/TelloVisionTracking"),
    (
        "vision",
        "Assets/StreamingAssets/TelloVisionBridge",
        "UnityClient/Assets/StreamingAssets/TelloVisionBridge",
    ),
)


# 팀 저장소 경로와 고정 커밋을 읽는다.
def load_manifest(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


# 경로가 지정 저장소 아래인지 확인한다.
def ensure_child(path: Path, parent: Path) -> None:
    if parent != path and parent not in path.parents:
        raise RuntimeError(f"허용 범위를 벗어난 경로입니다: {path}")


# Windows junction의 실제 대상을 확인한다.
def junction_target(path: Path) -> Path | None:
    if not os.path.lexists(path):
        return None
    try:
        return Path(os.path.realpath(path)).resolve()
    except OSError:
        return None


# Unity 복사본을 팀 Vision 원본 junction으로 교체한다.
def configure_links(manifest_path: Path, apply: bool) -> int:
    root = manifest_path.parent.resolve()
    manifest = load_manifest(manifest_path)
    unity_assets = (root / "UnityClient" / "Assets").resolve()
    failures = []

    for repository_name, source_relative, target_relative in LINKS:
        repository = manifest["repositories"][repository_name]
        repository_root = (root / repository["path"]).resolve()
        source = (repository_root / source_relative).resolve()
        target = (root / target_relative).absolute()
        ensure_child(source, repository_root)
        ensure_child(target, unity_assets)
        if not source.is_dir():
            raise RuntimeError(f"팀 Vision 원본 경로가 없습니다: {source}")

        if junction_target(target) == source:
            print(f"[OK] {target} -> {source}")
            continue
        if not apply:
            failures.append(f"{target}가 팀 원본 junction이 아닙니다.")
            continue

        idea = target / "Runtime" / ".idea"
        if idea.is_dir():
            backup = root / "UnityClient" / "UserSettings" / "TeamSourceBackup" / "TelloVisionTracking.Runtime.idea"
            backup.parent.mkdir(parents=True, exist_ok=True)
            if backup.exists():
                shutil.rmtree(backup)
            shutil.copytree(idea, backup)

        if os.path.lexists(target):
            is_junction = hasattr(os.path, "isjunction") and os.path.isjunction(target)
            if target.is_symlink() or is_junction:
                os.rmdir(target)
            else:
                shutil.rmtree(target)
        target.parent.mkdir(parents=True, exist_ok=True)
        result = subprocess.run(
            ["cmd.exe", "/c", "mklink", "/J", str(target), str(source)],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip())
        print(f"[LINK] {target} -> {source}")

    if failures:
        for failure in failures:
            print(f"[FAIL] {failure}", file=sys.stderr)
        return 1
    return 0


# 실행 옵션을 읽는다.
def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="팀 Vision Unity junction 설정")
    parser.add_argument("--manifest", default=str(root / "team-sources.json"))
    parser.add_argument("--apply", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return configure_links(Path(args.manifest).resolve(), args.apply)


if __name__ == "__main__":
    raise SystemExit(main())
