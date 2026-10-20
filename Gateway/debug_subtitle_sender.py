from __future__ import annotations

import argparse
import json
import socket
import time


DEFAULT_PORT = 49303
MAXIMUM_TEXT_CHARACTERS = 500
MAXIMUM_ROLE_CHARACTERS = 24


def build_payload(
    role: str,
    text: str | None,
    *,
    visible: bool = True,
    clear: bool = False,
) -> bytes:
    normalized_role = (role or "Codex").strip()[:MAXIMUM_ROLE_CHARACTERS] or "Codex"
    normalized_text = (text or "").strip()[:MAXIMUM_TEXT_CHARACTERS]
    if not normalized_text and not clear and visible:
        raise ValueError("표시할 자막 본문이 없습니다.")

    return json.dumps(
        {
            "version": 1,
            "role": normalized_role,
            "text": normalized_text,
            "clear": bool(clear),
            "visible": bool(visible),
            "timestampNs": time.time_ns(),
        },
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")


def send_payload(payload: bytes, port: int = DEFAULT_PORT) -> None:
    if not 1 <= port <= 65535:
        raise ValueError("UDP 포트는 1~65535 범위여야 합니다.")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.sendto(payload, ("127.0.0.1", port))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Unity XR 자막바 디버그 메시지 송신")
    parser.add_argument("--role", default="Codex")
    parser.add_argument("--text")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--clear", action="store_true")
    parser.add_argument("--hide", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = build_payload(
        args.role,
        args.text,
        visible=not args.hide,
        clear=args.clear,
    )
    send_payload(payload, args.port)
    print(f"DEBUG_SUBTITLE_SENT port={args.port} bytes={len(payload)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
