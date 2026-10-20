from __future__ import annotations

import argparse
import json
import locale
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable


SSID_PATTERN = re.compile(r"^\s*SSID\s*:\s*(.*?)\s*$", re.IGNORECASE | re.MULTILINE)
DEFAULT_CONFIG_NAME = "wifi_runtime_autostart.json"
RUNTIME_DIR_NAME = ".team-runtime"
LAUNCHER_STOP_NAME = "wifi-runtime-autostart.stop"
SUPERVISOR_EXIT_NAME = "wifi-runtime-autostart.exit"


@dataclass(frozen=True)
class AutostartConfig:
    ssid: str
    modules: tuple[str, ...]
    poll_seconds: float
    connect_stable_seconds: float
    disconnect_grace_seconds: float
    restart_delay_seconds: float
    shutdown_timeout_seconds: float
    stop_on_disconnect: bool
    log_path: Path


# Windows WLAN 출력에서 현재 연결된 SSID만 추출한다.
def parse_windows_connected_ssids(output: str) -> tuple[str, ...]:
    values = []
    for match in SSID_PATTERN.finditer(output):
        ssid = match.group(1).strip()
        if ssid and ssid not in values:
            values.append(ssid)
    return tuple(values)


# 로컬 Windows 언어에 맞춰 netsh 출력을 안전하게 디코딩한다.
def decode_windows_output(data: bytes) -> str:
    encodings = (locale.getpreferredencoding(False), "utf-8", "cp949")
    for encoding in dict.fromkeys(encodings):
        try:
            return data.decode(encoding)
        except (LookupError, UnicodeDecodeError):
            continue
    return data.decode("utf-8", errors="replace")


# 현재 Windows 무선 인터페이스의 연결 SSID를 읽는다.
def read_windows_connected_ssids() -> tuple[str, ...]:
    if sys.platform != "win32":
        return ()
    creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    try:
        result = subprocess.run(
            ["netsh", "wlan", "show", "interfaces"],
            capture_output=True,
            check=False,
            timeout=2.0,
            creationflags=creation_flags,
        )
    except (OSError, subprocess.SubprocessError):
        return ()
    if result.returncode != 0:
        return ()
    return parse_windows_connected_ssids(decode_windows_output(result.stdout))


def _positive_number(data: dict, key: str, default: float) -> float:
    value = data.get(key, default)
    if type(value) not in (int, float) or float(value) <= 0:
        raise ValueError(f"{key}는 0보다 큰 숫자여야 합니다.")
    return float(value)


# 사용자가 수정하는 JSON 설정을 검증해 실행 설정으로 변환한다.
def load_config(path: Path, root: Path) -> AutostartConfig:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"설정 파일을 읽을 수 없습니다: {error}") from error

    ssid = data.get("ssid")
    if not isinstance(ssid, str) or not ssid.strip():
        raise ValueError("ssid는 비어 있지 않은 문자열이어야 합니다.")

    modules = data.get("modules")
    if (
        not isinstance(modules, list)
        or not modules
        or not all(isinstance(item, str) and item.strip() for item in modules)
    ):
        raise ValueError("modules는 하나 이상의 모듈 문자열 배열이어야 합니다.")
    normalized_modules = tuple(dict.fromkeys(item.strip() for item in modules))

    stop_on_disconnect = data.get("stopOnDisconnect", True)
    if not isinstance(stop_on_disconnect, bool):
        raise ValueError("stopOnDisconnect는 true 또는 false여야 합니다.")

    log_value = data.get("logPath", "Logs/wifi-runtime-autostart.log")
    if not isinstance(log_value, str) or not log_value.strip():
        raise ValueError("logPath는 비어 있지 않은 문자열이어야 합니다.")
    log_path = Path(log_value)
    if not log_path.is_absolute():
        log_path = root / log_path

    return AutostartConfig(
        ssid=ssid.strip(),
        modules=normalized_modules,
        poll_seconds=_positive_number(data, "pollSeconds", 2.0),
        connect_stable_seconds=_positive_number(data, "connectStableSeconds", 2.0),
        disconnect_grace_seconds=_positive_number(data, "disconnectGraceSeconds", 5.0),
        restart_delay_seconds=_positive_number(data, "restartDelaySeconds", 5.0),
        shutdown_timeout_seconds=_positive_number(data, "shutdownTimeoutSeconds", 15.0),
        stop_on_disconnect=stop_on_disconnect,
        log_path=log_path.resolve(),
    )


class WifiRuntimeSupervisor:
    def __init__(
        self,
        root: Path,
        config: AutostartConfig,
        *,
        ssid_reader: Callable[[], tuple[str, ...]] = read_windows_connected_ssids,
        popen_factory: Callable[..., subprocess.Popen] = subprocess.Popen,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        python_path: Path | None = None,
        require_visible_console: bool = True,
    ) -> None:
        self.root = root.resolve()
        self.config = config
        self.ssid_reader = ssid_reader
        self.popen_factory = popen_factory
        self.monotonic = monotonic
        self.sleep = sleep
        self.python_path = (
            python_path.resolve()
            if python_path is not None
            else (self.root / ".upstreams" / "-Pilot-PC" / ".venv" / "Scripts" / "python.exe")
        )
        self.require_visible_console = require_visible_console
        self.runtime_dir = self.root / RUNTIME_DIR_NAME
        self.stop_file = self.runtime_dir / LAUNCHER_STOP_NAME
        self.exit_file = self.runtime_dir / SUPERVISOR_EXIT_NAME
        self.process: subprocess.Popen | None = None
        self.connected_since: float | None = None
        self.disconnected_since: float | None = None
        self.restart_not_before = 0.0
        self.was_connected: bool | None = None

    # 감시기와 런처의 상태 전환만 로그에 기록한다.
    def log(self, message: str) -> None:
        self.config.log_path.parent.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
        with self.config.log_path.open("a", encoding="utf-8") as stream:
            stream.write(f"[{timestamp}] {message}\n")

    # 기존 선택형 런처를 숨김 프로세스로 시작한다.
    def start_runtime(self) -> None:
        if not self.python_path.is_file():
            raise RuntimeError(f"Pilot Python을 찾을 수 없습니다: {self.python_path}")
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.stop_file.unlink(missing_ok=True)
        command = [
            str(self.python_path),
            str(self.root / "Gateway" / "strikerpc_launcher.py"),
        ]
        if self.require_visible_console:
            command.append("--require-visible-console")
        command.extend(
            (
                "--modules",
                ",".join(self.config.modules),
                "--stop-file",
                str(self.stop_file),
            )
        )
        creation_flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
        popen_options = {
            "cwd": str(self.root),
            "creationflags": creation_flags,
        }
        if self.require_visible_console:
            creation_flags |= getattr(subprocess, "CREATE_NEW_CONSOLE", 0)
            popen_options["creationflags"] = creation_flags
            if sys.platform == "win32":
                startup_info = subprocess.STARTUPINFO()
                startup_info.dwFlags |= subprocess.STARTF_USESHOWWINDOW
                startup_info.wShowWindow = 1  # SW_SHOWNORMAL
                popen_options["startupinfo"] = startup_info
        else:
            popen_options["creationflags"] = (
                creation_flags | getattr(subprocess, "CREATE_NO_WINDOW", 0)
            )
            popen_options.update(
                {
                    "stdin": subprocess.DEVNULL,
                    "stdout": subprocess.DEVNULL,
                    "stderr": subprocess.DEVNULL,
                }
            )
        self.process = self.popen_factory(command, **popen_options)
        self.log(
            f"런타임 시작 PID={self.process.pid}, modules={','.join(self.config.modules)}"
        )

    # stop-file을 통해 런처의 중립 RC 및 역순 종료 절차를 실행한다.
    def stop_runtime(self, reason: str) -> None:
        process = self.process
        if process is None or process.poll() is not None:
            self.process = None
            return
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.stop_file.write_text(reason, encoding="utf-8")
        self.log(f"안전 종료 요청: {reason}")
        try:
            process.wait(timeout=self.config.shutdown_timeout_seconds)
        except subprocess.TimeoutExpired:
            self.log("안전 종료 시간 초과로 정확한 런처 프로세스 트리를 종료합니다.")
            if sys.platform == "win32":
                subprocess.run(
                    ["taskkill", "/PID", str(process.pid), "/T", "/F"],
                    capture_output=True,
                    check=False,
                )
            else:
                process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        finally:
            self.process = None
            self.stop_file.unlink(missing_ok=True)

    # Wi-Fi 상태 한 번을 평가해 시작·종료 전환을 수행한다.
    def poll_once(self, now: float | None = None) -> None:
        observed_at = self.monotonic() if now is None else now
        if self.process is not None and self.process.poll() is not None:
            return_code = self.process.poll()
            self.log(f"런타임 종료 감지 code={return_code}")
            self.process = None
            self.restart_not_before = observed_at + self.config.restart_delay_seconds

        connected = self.config.ssid in self.ssid_reader()
        if connected != self.was_connected:
            self.log(
                f"SSID {self.config.ssid} "
                + ("연결 감지" if connected else "연결 안 됨")
            )
            self.was_connected = connected

        if connected:
            self.disconnected_since = None
            if self.connected_since is None:
                self.connected_since = observed_at
            stable = observed_at - self.connected_since
            if (
                self.process is None
                and observed_at >= self.restart_not_before
                and stable >= self.config.connect_stable_seconds
            ):
                try:
                    self.start_runtime()
                except Exception as error:
                    self.log(f"런타임 시작 실패: {error}")
                    self.restart_not_before = observed_at + self.config.restart_delay_seconds
            return

        self.connected_since = None
        if self.process is None or not self.config.stop_on_disconnect:
            self.disconnected_since = None
            return
        if self.disconnected_since is None:
            self.disconnected_since = observed_at
            return
        if observed_at - self.disconnected_since >= self.config.disconnect_grace_seconds:
            self.stop_runtime("Tello Wi-Fi 연결 해제")
            self.restart_not_before = observed_at + self.config.restart_delay_seconds
            self.disconnected_since = None

    # Windows 로그인 동안 감시하고 종료 요청 시 자식 런타임도 정리한다.
    def run(self) -> None:
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.exit_file.unlink(missing_ok=True)
        self.log(
            f"감시 시작 ssid={self.config.ssid}, stopOnDisconnect={self.config.stop_on_disconnect}"
        )
        try:
            while not self.exit_file.exists():
                self.poll_once()
                self.sleep(self.config.poll_seconds)
        except KeyboardInterrupt:
            self.log("사용자 종료 요청")
        finally:
            self.stop_runtime("Wi-Fi 감시기 종료")
            self.exit_file.unlink(missing_ok=True)
            self.log("감시 종료")


def runtime_paths(root: Path) -> tuple[Path, Path]:
    runtime_dir = root / RUNTIME_DIR_NAME
    return runtime_dir / LAUNCHER_STOP_NAME, runtime_dir / SUPERVISOR_EXIT_NAME


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="Tello Wi-Fi 기반 StrikerPC 자동 실행 감시기")
    parser.add_argument(
        "--config",
        default=str(root / "Gateway" / DEFAULT_CONFIG_NAME),
    )
    parser.add_argument("--check-config", action="store_true")
    parser.add_argument("--request-stop", action="store_true")
    return parser.parse_args()


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    args = parse_args()
    if args.request_stop:
        _, exit_file = runtime_paths(root)
        exit_file.parent.mkdir(parents=True, exist_ok=True)
        exit_file.write_text("stop requested", encoding="utf-8")
        return 0

    try:
        config = load_config(Path(args.config).resolve(), root)
    except ValueError as error:
        print(f"[WiFiAutostart] 설정 오류: {error}", file=sys.stderr)
        return 2
    if args.check_config:
        print(
            f"[WiFiAutostart] OK ssid={config.ssid} "
            f"modules={','.join(config.modules)}"
        )
        return 0

    supervisor = WifiRuntimeSupervisor(root, config)
    supervisor.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
