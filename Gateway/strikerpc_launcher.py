from __future__ import annotations

import argparse
import ctypes
import importlib
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

from control_protocol import (
    COMMAND_NEUTRAL,
    INTERNAL_CONTROL_FLAG,
    ControlPacket,
    encode_control_packet,
)


RETRY_SECONDS = 3.0
STARTUP_TIMEOUT_SECONDS = 30.0
TELLO_RESPONSE_TIMEOUT = 2.0
SHUTDOWN_TIMEOUT_SECONDS = 5.0
PILOT_SHUTDOWN_TIMEOUT_SECONDS = 12.0
REQUIRED_LIVEKIT_KEYS = (
    "LIVEKIT_URL",
    "LIVEKIT_API_KEY",
    "LIVEKIT_API_SECRET",
    "LIVEKIT_ROOM",
)
LIVEKIT_RUNTIME_MODULES = frozenset(("livekit-video", "gateway", "command-center"))
MENU_MODULES = (
    ("1", "pilot"),
    ("2", "gateway"),
    ("3", "xr"),
    ("4", "position"),
    ("5", "vision"),
)
MODULE_DEPENDENCIES = {
    "position": (),
    "pilot": (),
    "vision": ("position", "pilot"),
    "livekit-video": ("pilot",),
    "gateway": (),
    "xr": ("pilot",),
}
MODULE_ALIASES = {
    "adapter": "xr",
    "operation-gateway": "gateway",
    "position-sync": "position",
    "vision-adapter": "vision",
    "pilot-pc": "pilot",
    "video-uplink": "livekit-video",
    "uplink": "livekit-video",
}


# Wi-Fi 자동 실행 시 비상 키 입력용 콘솔을 화면 최상단에 표시한다.
def activate_emergency_console() -> bool:
    if sys.platform != "win32":
        return False

    kernel32 = ctypes.windll.kernel32
    user32 = ctypes.windll.user32
    window = kernel32.GetConsoleWindow()
    if not window:
        return False

    sw_restore = 9
    hwnd_topmost = -1
    swp_no_move = 0x0002
    swp_no_size = 0x0001
    swp_show_window = 0x0040
    kernel32.SetConsoleTitleW("StrikerPC Emergency Control - ESC=EMERGENCY")
    user32.ShowWindowAsync(window, sw_restore)
    user32.SetWindowPos(
        window,
        hwnd_topmost,
        0,
        0,
        0,
        0,
        swp_no_move | swp_no_size | swp_show_window,
    )
    user32.SetForegroundWindow(window)
    return bool(user32.IsWindowVisible(window)) and not bool(user32.IsIconic(window))


def print_emergency_console_banner() -> None:
    print("", flush=True)
    print("=" * 72, flush=True)
    print("STRIKERPC EMERGENCY CONTROL WINDOW", flush=True)
    print("ESC = EMERGENCY STOP   BACKSPACE = LAND   TAB = TAKEOFF", flush=True)
    print("KEEP THIS WINDOW OPEN AND VISIBLE WHILE THE DRONE IS CONNECTED.", flush=True)
    print("=" * 72, flush=True)
    print("", flush=True)


class PortConflictError(RuntimeError):
    pass


class LauncherStopRequested(RuntimeError):
    pass


class ModuleSelectionUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class ProcessSpec:
    key: str
    name: str
    command: tuple[str, ...]
    cwd: Path
    log_name: str
    ready_token: str
    dependencies: tuple[str, ...] = ()
    conflicts: tuple[str, ...] = ()
    menu_order: int = 0
    start_order: int = 0
    aliases: tuple[str, ...] = ()
    validator: str = "generic"
    requires_drone: bool = False
    neutral_on_stop: bool = False
    preflight: dict | None = None


@dataclass(frozen=True)
class ModuleSelectionEntry:
    key: str
    name: str
    dependencies: tuple[str, ...]
    requires_drone: bool


# 새 준비 토큰을 우선 사용하되 전환 기간에는 기존 토큰도 허용한다.
def compatible_ready_tokens(token: str) -> tuple[str, ...]:
    if token.startswith("STRIKERPC_"):
        return token, "DRONEOPS_" + token.removeprefix("STRIKERPC_")
    return (token,)


class ManagedChild:
    def __init__(
        self,
        spec: ProcessSpec,
        process: subprocess.Popen,
        log_file,
    ) -> None:
        self.spec = spec
        self.process = process
        self.log_file = log_file
        self.ready = threading.Event()
        self.thread: threading.Thread | None = None
        self.status = "시작 중"


# 지정 포트를 다른 프로세스가 점유했는지 확인한다.
def verify_port(port: int, socket_type: int, host: str = "127.0.0.1") -> None:
    probe = socket.socket(socket.AF_INET, socket_type)
    try:
        probe.bind((host, port))
    except OSError as error:
        protocol = "TCP" if socket_type == socket.SOCK_STREAM else "UDP"
        raise PortConflictError(
            f"{protocol} {port} 포트를 다른 프로세스가 사용 중입니다."
        ) from error
    finally:
        probe.close()


# Pilot-PC 원본의 Tello 연결 클래스를 불러온다.
def load_pilot_tello_types(pilot_root: Path):
    pilot_path = str(pilot_root)
    sys.path.insert(0, pilot_path)
    try:
        config_module = importlib.import_module("tello_app.config")
        tello_module = importlib.import_module("tello_app.tello")
    finally:
        sys.path.remove(pilot_path)

    imported_path = Path(tello_module.__file__).resolve()
    if pilot_root.resolve() not in imported_path.parents:
        raise RuntimeError(f"Pilot-PC 원본이 아닌 Tello 모듈입니다: {imported_path}")
    return tello_module.TelloCommandClient, config_module.TelloConfig


# 실제 Pilot 명령 포트로 SDK 모드 응답만 확인한다.
def check_drone_once(
    pilot_root: Path,
    *,
    tello_host: str = "192.168.10.1",
    command_port: int = 8889,
    local_port: int = 9000,
    response_timeout: float = TELLO_RESPONSE_TIMEOUT,
    client_type=None,
    config_type=None,
) -> None:
    if client_type is None or config_type is None:
        client_type, config_type = load_pilot_tello_types(pilot_root)

    try:
        client = client_type(
            config_type(
                host=tello_host,
                command_port=command_port,
                local_port=local_port,
                response_timeout=response_timeout,
            )
        )
    except OSError as error:
        raise PortConflictError(
            f"UDP {local_port} Tello 명령 포트를 사용할 수 없습니다."
        ) from error
    try:
        client.enter_sdk_mode()
    finally:
        client.close()


# 연결될 때까지 지정 간격으로 Tello command 응답을 확인한다.
def wait_for_drone(
    checker: Callable[[], None],
    emit: Callable[[str], None],
    *,
    retry_seconds: float = RETRY_SECONDS,
    sleeper: Callable[[float], None] = time.sleep,
    should_stop: Callable[[], bool] | None = None,
) -> int:
    attempt = 0
    while True:
        if should_stop is not None and should_stop():
            raise LauncherStopRequested("드론 연결 대기 중 종료 요청")
        attempt += 1
        emit(f"[Drone] 연결 확인 중... (시도 {attempt})")
        try:
            checker()
        except PortConflictError:
            raise
        except Exception as error:
            emit(f"[Drone] 연결되지 않았습니다: {error}")
            emit(f"[Drone] {retry_seconds:g}초 후 다시 확인합니다. 종료는 Ctrl+C")
            if should_stop is None:
                sleeper(retry_seconds)
            else:
                remaining = retry_seconds
                while remaining > 0:
                    if should_stop():
                        raise LauncherStopRequested("드론 연결 대기 중 종료 요청")
                    duration = min(0.1, remaining)
                    sleeper(duration)
                    remaining -= duration
            continue
        emit("[Drone] 연결 확인 완료")
        return attempt


# .env에서 키 이름과 값 존재 여부만 읽는다.
def read_env_values(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip("'\"")
    return values


# 메뉴 번호 또는 모듈 키 목록을 중복 없이 변환한다.
def parse_module_selection(
    raw_value: str,
    *,
    number_to_key: dict[str, str] | None = None,
    available_keys: set[str] | None = None,
    aliases: dict[str, str] | None = None,
) -> tuple[str, ...]:
    number_to_key = number_to_key or dict(MENU_MODULES)
    available_keys = available_keys or set(MODULE_DEPENDENCIES)
    aliases = aliases or MODULE_ALIASES
    selected: list[str] = []
    for value in raw_value.replace(",", " ").split():
        normalized = value.strip().lower()
        key = number_to_key.get(
            normalized,
            aliases.get(normalized, normalized),
        )
        if key not in available_keys:
            raise ValueError(f"알 수 없는 모듈입니다: {value}")
        if key not in selected:
            selected.append(key)
    if not selected:
        raise ValueError("모듈을 하나 이상 선택해야 합니다.")
    return tuple(selected)


class StrikerPCLauncher:
    def __init__(
        self,
        manifest_path: Path,
        *,
        module_config_path: Path | None = None,
        retry_seconds: float = RETRY_SECONDS,
        startup_timeout: float = STARTUP_TIMEOUT_SECONDS,
    ) -> None:
        self.manifest_path = manifest_path.resolve()
        self.root = self.manifest_path.parent
        self.manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        self.module_config_path = (
            module_config_path.resolve()
            if module_config_path is not None
            else self.root / "Gateway" / "launcher_modules.json"
        )
        pilot_item = self.manifest["repositories"]["pilot"]
        vision_item = self.manifest["repositories"]["vision"]
        mission_item = self.manifest["repositories"].get("missionPlatform")
        self.pilot_root = (self.root / pilot_item["path"]).resolve()
        self.vision_root = (self.root / vision_item["path"]).resolve()
        self.mission_root = (
            (self.root / mission_item["path"]).resolve()
            if mission_item is not None
            else self.root / "drone-mission-platform"
        )
        runtime = self.manifest.get("runtime", {})
        self.pilot_provider = runtime.get(
            "pilotProvider",
            "legacyPilot",
        )
        mission_env_value = runtime.get("missionPlatformEnv")
        self.mission_env_path: Path | None = None
        if mission_env_value:
            mission_env_path = (self.root / str(mission_env_value)).resolve()
            try:
                mission_env_path.relative_to(self.root)
            except ValueError as error:
                raise RuntimeError(
                    "Mission Platform 환경 파일은 통합 저장소 내부에 있어야 합니다."
                ) from error
            self.mission_env_path = mission_env_path
        self.pilot_python = self.pilot_root / ".venv" / "Scripts" / "python.exe"
        self.vision_python = self.vision_root / ".venv" / "Scripts" / "python.exe"
        self.gateway_python = self.root / "Gateway" / ".venv" / "Scripts" / "python.exe"
        self.retry_seconds = retry_seconds
        self.startup_timeout = startup_timeout
        self.children: list[ManagedChild] = []
        self.log_dir: Path | None = None
        self.launcher_log = None
        self._pilot_types = None
        self._stopping = False
        self.failures: dict[str, str] = {}
        self._log_counts: dict[str, int] = {}
        self._latest_logs: dict[str, Path] = {}
        self._extension_specs: tuple[ProcessSpec, ...] | None = None

    # 메뉴 표시에 필요한 manifest 구조만 먼저 확인한다.
    def validate_bootstrap(self) -> None:
        repositories = self.manifest.get("repositories", {})
        ports = self.manifest.get("ports", {})
        livekit = self.manifest.get("livekit", {})
        required_repositories = {"pilot", "viewer", "vision"}
        if self.pilot_provider == "missionPlatform":
            required_repositories.add("missionPlatform")
        required_ports = {
            "telloVideoInput",
            "telloCommandLocal",
            "telloStateInput",
            "telloStateRelay",
            "pilotStateRelay",
            "desktopStateRelay",
            "livekitStateRelay",
            "visionVideo",
            "visionPose",
            "visionMeasurement",
            "visionOverlayPose",
            "pilotHoverPose",
            "controlAuthority",
            "controlRelay",
            "pilotControlInput",
            "desktopControl",
            "unityAnnotations",
            "unityEvents",
            "unityHeartbeat",
        }
        required_livekit = {
            "videoPublisherIdentity",
            "videoTrackName",
            "commandCenterIdentity",
        }
        missing_repositories = sorted(required_repositories - repositories.keys())
        missing_ports = sorted(required_ports - ports.keys())
        missing_livekit = sorted(required_livekit - livekit.keys())
        if missing_repositories or missing_ports or missing_livekit:
            raise RuntimeError(
                "team-sources.json 필수 항목이 없습니다: "
                + ", ".join((*missing_repositories, *missing_ports, *missing_livekit))
            )
        if self.pilot_provider not in {"missionPlatform", "legacyPilot"}:
            raise RuntimeError(
                "runtime.pilotProvider must be missionPlatform or legacyPilot: "
                f"{self.pilot_provider}"
            )
        self.process_specs()

    # 선택한 모듈에 필요한 파일과 환경만 검증한다.
    def validate_module(self, key: str) -> None:
        spec = self.process_specs()[key]
        required = (Path(spec.command[0]), Path(spec.command[1]))
        missing = [str(path) for path in required if not path.exists()]
        if missing:
            raise RuntimeError("실행 파일이 없습니다:\n- " + "\n- ".join(missing))
        if not spec.cwd.exists():
            raise RuntimeError(f"작업 폴더가 없습니다: {spec.cwd}")

        ports = self.manifest["ports"]
        if spec.validator == "pilot":
            if self.pilot_provider == "missionPlatform":
                self._verify_python(
                    self.pilot_python,
                    "import sys; "
                    f"sys.path.insert(0, {str(self.mission_root)!r}); "
                    "import cv2, dotenv, livekit, livekit.api, numpy; "
                    "from apps.strike_gateway import main; "
                    "from adapters.gamepad_adapter import GamepadInput; "
                    "from adapters.tello_adapter import TelloAdapter, RcControl; "
                    "assert callable(main.run)",
                    "Mission Platform",
                )
                self._verify_team_repository("missionPlatform")
            else:
                self._verify_python(
                    self.pilot_python,
                    "import cv2, livekit, torch, ultralytics; "
                    "assert torch.cuda.is_available(), 'CUDA is not available'",
                    "Pilot-PC",
                )
                model = self.pilot_root / "yolov8n.pt"
                if not model.exists():
                    raise RuntimeError(f"Pilot 추적 모델이 없습니다: {model}")
                self._verify_team_repository("pilot")
            if self.pilot_provider != "missionPlatform":
                self._verify_livekit_env("Pilot-PC", self._pilot_livekit_env_path())
            verify_port(ports["telloCommandLocal"], socket.SOCK_DGRAM, "0.0.0.0")
            verify_port(ports["telloVideoInput"], socket.SOCK_DGRAM, "0.0.0.0")
            verify_port(ports["pilotControlInput"], socket.SOCK_DGRAM)
            verify_port(ports["desktopControl"], socket.SOCK_DGRAM)
            verify_port(ports["pilotStateRelay"], socket.SOCK_DGRAM)
            if self.pilot_provider == "missionPlatform":
                verify_port(ports["pilotHoverPose"], socket.SOCK_DGRAM)
            verify_port(ports["unityHeartbeat"], socket.SOCK_DGRAM)
            verify_port(ports["visionVideo"], socket.SOCK_STREAM)
        elif spec.validator == "livekit-video":
            self._verify_python(
                self.pilot_python,
                "import sys; "
                f"sys.path.insert(0, {str(self.mission_root)!r}); "
                "import cv2, dotenv, livekit, livekit.api, numpy; "
                "from apps.strike_gateway.video_publisher import VideoPublisher; "
                "assert callable(VideoPublisher)",
                "LiveKit Video",
            )
            self._verify_livekit_env("LiveKit Video", self._pilot_livekit_env_path())
            self._verify_team_repository("missionPlatform")
            verify_port(ports["livekitStateRelay"], socket.SOCK_DGRAM)
        elif spec.validator == "gateway":
            self._verify_python(
                self.gateway_python,
                "import dotenv, livekit, livekit.api",
                "Gateway",
            )
            self._verify_livekit_env("Gateway", self._gateway_livekit_env_path())
            self._verify_team_repository("viewer")
            verify_port(ports["unityEvents"], socket.SOCK_DGRAM)
        elif spec.validator == "xr":
            self._verify_python(self.gateway_python, "import sys", "Gateway")
            verify_port(ports["controlAuthority"], socket.SOCK_STREAM)
            verify_port(ports["controlRelay"], socket.SOCK_DGRAM)
        elif spec.validator == "position":
            self._verify_python(self.gateway_python, "import sys", "Gateway")
            verify_port(ports["telloStateInput"], socket.SOCK_DGRAM, "0.0.0.0")
            verify_port(ports["visionMeasurement"], socket.SOCK_DGRAM)
        elif spec.validator == "vision":
            self._verify_python(
                self.vision_python,
                "import av, cv2, numpy, pupil_apriltags",
                "Vision",
            )
            self._verify_team_repository("vision")
            self._verify_vision_files()
        else:
            self._validate_generic_module(spec)
        self.emit(f"[Environment] {spec.name} 검증 완료")

    # 확장 모듈의 공통 파일·Python·환경·포트 검사를 수행한다.
    def _validate_generic_module(self, spec: ProcessSpec) -> None:
        preflight = spec.preflight or {}
        required_files = preflight.get("requiredFiles", [])
        if not isinstance(required_files, list):
            raise RuntimeError(f"{spec.key} requiredFiles는 배열이어야 합니다.")
        missing = [
            self._format_module_value(str(value))
            for value in required_files
            if not Path(self._format_module_value(str(value))).exists()
        ]
        if missing:
            raise RuntimeError("필수 파일이 없습니다:\n- " + "\n- ".join(missing))

        python_name = preflight.get("python")
        imports = preflight.get("imports", [])
        if python_name is not None or imports:
            python_path = self._resolve_module_python(str(python_name or "gateway"))
            if not isinstance(imports, list) or not all(
                isinstance(value, str)
                and re.fullmatch(r"[A-Za-z_][A-Za-z0-9_.]*", value)
                for value in imports
            ):
                raise RuntimeError(f"{spec.key} imports 형식이 올바르지 않습니다.")
            import_script = "; ".join(f"import {value}" for value in imports) or "pass"
            self._verify_python(python_path, import_script, spec.name)

            probe_value = preflight.get("pythonProbe")
            if probe_value:
                probe_path = Path(self._format_module_value(str(probe_value)))
                probe = subprocess.run(
                    [python_path, probe_path],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                if probe.returncode != 0:
                    detail = (probe.stderr or probe.stdout).strip()
                    raise RuntimeError(f"{spec.name} 추가 검증 실패: {detail}")
                for line in (probe.stdout or "").splitlines():
                    if line.strip():
                        self.emit(f"[Environment] {line.strip()}")

        env = preflight.get("env")
        if env is not None:
            if not isinstance(env, dict) or not isinstance(env.get("requiredKeys"), list):
                raise RuntimeError(f"{spec.key} env 형식이 올바르지 않습니다.")
            path = Path(self._format_module_value(str(env.get("path", ""))))
            required_keys = tuple(str(value) for value in env["requiredKeys"])
            self._verify_env_keys(spec.name, path, required_keys)

        repositories = preflight.get("repositories", [])
        if not isinstance(repositories, list):
            raise RuntimeError(f"{spec.key} repositories는 배열이어야 합니다.")
        for repository in repositories:
            key = str(repository)
            if key not in {"pilot", "viewer", "vision", "missionPlatform"}:
                raise RuntimeError(f"{spec.key} repository가 올바르지 않습니다: {key}")
            self._verify_team_repository(key)

        ports = preflight.get("ports", [])
        if not isinstance(ports, list):
            raise RuntimeError(f"{spec.key} ports는 배열이어야 합니다.")
        for item in ports:
            if not isinstance(item, dict):
                raise RuntimeError(f"{spec.key} port 형식이 올바르지 않습니다.")
            value = self._format_module_value(str(item.get("value", "")))
            try:
                port = int(value)
            except ValueError as error:
                raise RuntimeError(f"{spec.key} port 값이 올바르지 않습니다: {value}") from error
            protocol = str(item.get("protocol", "udp")).lower()
            if protocol not in {"udp", "tcp"}:
                raise RuntimeError(f"{spec.key} protocol은 udp 또는 tcp여야 합니다.")
            socket_type = socket.SOCK_STREAM if protocol == "tcp" else socket.SOCK_DGRAM
            verify_port(port, socket_type, str(item.get("host", "127.0.0.1")))

    # 확장 모듈이 지정한 Python 가상환경 이름을 실제 경로로 바꾼다.
    def _resolve_module_python(self, value: str) -> Path:
        known = {
            "gateway": self.gateway_python,
            "pilot": self.pilot_python,
            "vision": self.vision_python,
        }
        if value in known:
            return known[value]
        return Path(self._format_module_value(value))

    # 지정 환경 파일에서 필요한 키의 존재 여부만 검사한다.
    def _verify_env_keys(
        self,
        name: str,
        path: Path,
        required_keys: tuple[str, ...],
    ) -> None:
        if not path.exists():
            raise RuntimeError(f"{name} 환경 설정 파일이 없습니다: {path}")
        values = read_env_values(path)
        missing = [key for key in required_keys if not values.get(key)]
        if missing:
            raise RuntimeError(f"{name} 환경 필수 키가 없습니다: {', '.join(missing)}")

    # LiveKit 비밀값을 출력하지 않고 필수 키만 검증한다.
    def _verify_livekit_env(self, name: str, path: Path) -> None:
        if not path.exists():
            raise RuntimeError(f"{name} LiveKit 설정 파일이 없습니다: {path}")
        values = read_env_values(path)
        missing = [key for key in REQUIRED_LIVEKIT_KEYS if not values.get(key)]
        if missing:
            raise RuntimeError(f"{name} .env 필수 키가 없습니다: {', '.join(missing)}")

    # Mission Platform은 팀 저장소 대신 통합 전용 LiveKit 환경 파일을 사용한다.
    def _pilot_livekit_env_path(self) -> Path:
        if self.pilot_provider == "missionPlatform" and self.mission_env_path is not None:
            return self.mission_env_path
        return self.pilot_root / ".env"

    # Gateway도 Mission Platform과 동일한 LiveKit 프로젝트를 사용한다.
    def _gateway_livekit_env_path(self) -> Path:
        if self.pilot_provider == "missionPlatform" and self.mission_env_path is not None:
            return self.mission_env_path
        return self.root / "Gateway" / ".env"

    # LiveKit 모듈 자식 프로세스에 필수 네 키만 전달한다.
    def _child_environment(self, spec: ProcessSpec) -> dict[str, str]:
        environment = os.environ.copy()
        if (
            self.pilot_provider == "missionPlatform"
            and self.mission_env_path is not None
            and spec.key in LIVEKIT_RUNTIME_MODULES
        ):
            values = read_env_values(self.mission_env_path)
            missing = [key for key in REQUIRED_LIVEKIT_KEYS if not values.get(key)]
            if missing:
                raise RuntimeError(
                    "Mission Platform .env 필수 키가 없습니다: " + ", ".join(missing)
                )
            environment.update({key: values[key] for key in REQUIRED_LIVEKIT_KEYS})
        return environment

    # Vision 보정 파일과 지도 파일의 형식을 확인한다.
    def _verify_vision_files(self) -> None:
        calibration = (
            self.root
            / "UnityClient"
            / "Assets"
            / "StreamingAssets"
            / "Maps"
            / "tello-camera-calibration.json"
        )
        map_path = self.root / "Assets" / "Maps" / "current" / "venue-map.json"
        try:
            calibration_data = json.loads(calibration.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(f"Vision 카메라 보정 파일을 읽을 수 없습니다: {error}") from error
        matrix = calibration_data.get("camera_matrix", [])
        distortion = calibration_data.get("distortion_coefficients", [])
        rms = calibration_data.get("rms_reprojection_error", float("inf"))
        valid_vectors = (
            isinstance(matrix, list)
            and len(matrix) == 9
            and isinstance(distortion, list)
            and len(distortion) >= 4
            and all(
                type(value) in (int, float) and math.isfinite(float(value))
                for value in matrix + distortion
            )
        )
        valid_rms = (
            type(rms) in (int, float)
            and math.isfinite(float(rms))
            and 0.0 <= float(rms) <= 1.0
        )
        if (
            calibration_data.get("calibrated") is not True
            or not valid_vectors
            or not valid_rms
        ):
            raise RuntimeError(
                "Vision 카메라 보정이 완료되지 않았습니다. "
                "Gateway/tello_camera_calibration.py를 먼저 실행해야 합니다."
            )
        try:
            map_data = json.loads(map_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(f"Venue 지도 파일을 읽을 수 없습니다: {error}") from error
        if len(map_data.get("boardToVenue", [])) != 16:
            raise RuntimeError("Venue 지도 boardToVenue 형식이 올바르지 않습니다.")

    # 실제 Tello 응답을 첫 단계에서 확인한다.
    def wait_for_connection(
        self,
        should_stop: Callable[[], bool] | None = None,
    ) -> int:
        local_port = self.manifest["ports"]["telloCommandLocal"]
        verify_port(local_port, socket.SOCK_DGRAM, "0.0.0.0")
        if self._pilot_types is None:
            self._pilot_types = load_pilot_tello_types(self.pilot_root)
        client_type, config_type = self._pilot_types
        return wait_for_drone(
            lambda: check_drone_once(
                self.pilot_root,
                local_port=local_port,
                client_type=client_type,
                config_type=config_type,
            ),
            self.emit,
            retry_seconds=self.retry_seconds,
            should_stop=should_stop,
        )

    # 독립 모듈의 실행 명령을 안전한 기본 시작 순서로 만든다.
    def build_processes(self) -> tuple[ProcessSpec, ...]:
        ports = self.manifest["ports"]
        gateway = self.root / "Gateway"
        if self.pilot_provider == "missionPlatform":
            pilot_name = "Strike Gateway"
            pilot_dependencies = ("position",)
            pilot_command = (
                str(self.pilot_python),
                str(gateway / "mission_platform_runtime.py"),
                "--manifest",
                str(self.manifest_path),
                "--local-command-port",
                str(ports["telloCommandLocal"]),
                "--video-port",
                str(ports["telloVideoInput"]),
                "--pilot-port",
                str(ports["pilotControlInput"]),
                "--desktop-control-port",
                str(ports["desktopControl"]),
                "--state-port",
                str(ports["pilotStateRelay"]),
                "--desktop-state-port",
                str(ports["desktopStateRelay"]),
                "--dashboard-state-port",
                str(ports["gatewayDashboard"]),
                "--heartbeat-port",
                str(ports["unityHeartbeat"]),
                "--operation-event-port",
                str(ports["unityEvents"]),
                "--vision-tcp-port",
                str(ports["visionVideo"]),
                "--measurement-port",
                str(ports["visionMeasurement"]),
                "--hover-motion-port",
                str(ports["pilotHoverPose"]),
                "--hover-hold",
                "--identity",
                str(self.manifest["livekit"]["videoPublisherIdentity"]),
                "--local-only",
            )
            pilot_cwd = self.mission_root
        else:
            pilot_name = "Pilot-PC"
            pilot_dependencies = MODULE_DEPENDENCIES["pilot"]
            pilot_command = (
                str(self.pilot_python),
                str(gateway / "pilot_runtime.py"),
                "--manifest",
                str(self.manifest_path),
                "--local-command-port",
                str(ports["telloCommandLocal"]),
                "--video-port",
                str(ports["telloVideoInput"]),
                "--pilot-port",
                str(ports["pilotControlInput"]),
                "--desktop-control-port",
                str(ports["desktopControl"]),
                "--vision-tcp-port",
                str(ports["visionVideo"]),
                "--measurement-port",
                str(ports["visionMeasurement"]),
                "--",
                "--model",
                "yolov8n",
                "--classes",
                "person",
                "--device",
                "cuda",
                "--control-source",
                "keyboard",
            )
            pilot_cwd = self.pilot_root
        builtins = (
            ProcessSpec(
                "position",
                "Position Sync",
                (
                    str(self.gateway_python),
                    str(gateway / "drone_position_sync.py"),
                    "--state-port",
                    str(ports["telloStateInput"]),
                    "--state-relay-port",
                    str(ports["telloStateRelay"]),
                    "--pilot-state-relay-port",
                    str(ports["pilotStateRelay"]),
                    "--desktop-state-relay-port",
                    str(ports["desktopStateRelay"]),
                    "--livekit-state-relay-port",
                    str(ports["livekitStateRelay"]),
                    "--measurement-port",
                    str(ports["visionMeasurement"]),
                    "--pose-port",
                    str(ports["visionPose"]),
                    "--pilot-pose-relay-port",
                    str(ports["pilotHoverPose"]),
                    "--visual-pose-port",
                    str(ports["visionOverlayPose"]),
                ),
                gateway,
                "position-sync.log",
                "STRIKERPC_READY POSITION_SYNC",
                MODULE_DEPENDENCIES["position"],
                menu_order=4,
                start_order=10,
                aliases=("position-sync",),
                validator="position",
            ),
            ProcessSpec(
                "pilot",
                pilot_name,
                pilot_command,
                pilot_cwd,
                "pilot-pc.log",
                "STRIKERPC_READY PILOT_PC",
                pilot_dependencies,
                menu_order=1,
                start_order=20,
                aliases=("pilot-pc",),
                validator="pilot",
                requires_drone=True,
                neutral_on_stop=True,
            ),
            ProcessSpec(
                "vision",
                "AprilTag + Image Hover Tracking",
                (
                    str(self.vision_python),
                    str(gateway / "vision_adapter.py"),
                    "--vision-root",
                    str(self.vision_root),
                    "--video-port",
                    str(ports["visionVideo"]),
                    "--measurement-port",
                    str(ports["visionMeasurement"]),
                    "--map-path",
                    str(self.root / "Assets" / "Maps" / "current" / "venue-map.json"),
                    "--apriltag-only",
                    "--apriltag-refresh-cooldown",
                    "3.0",
                    "--hover-motion-port",
                    str(ports["pilotHoverPose"]),
                ),
                self.vision_root,
                "vision-adapter.log",
                "STRIKERPC_READY VISION_ADAPTER",
                MODULE_DEPENDENCIES["vision"],
                menu_order=5,
                start_order=30,
                aliases=("vision-adapter",),
                validator="vision",
            ),
            *(
                (
                    ProcessSpec(
                        "livekit-video",
                        "LiveKit Video Uplink",
                        (
                            str(self.pilot_python),
                            str(gateway / "livekit_video_runtime.py"),
                            "--manifest",
                            str(self.manifest_path),
                            "--env-file",
                            str(self._pilot_livekit_env_path()),
                            "--identity",
                            str(self.manifest["livekit"]["videoPublisherIdentity"]),
                            "--track-name",
                            str(self.manifest["livekit"]["videoTrackName"]),
                            "--shared-video-name",
                            str(self.manifest["sharedMemory"]["video"]),
                            "--state-port",
                            str(ports["livekitStateRelay"]),
                        ),
                        self.mission_root,
                        "livekit-video.log",
                        "STRIKERPC_READY LIVEKIT_VIDEO",
                        MODULE_DEPENDENCIES["livekit-video"],
                        menu_order=7,
                        start_order=35,
                        aliases=("video-uplink", "uplink", "livekit"),
                        validator="livekit-video",
                    ),
                )
                if self.pilot_provider == "missionPlatform"
                else ()
            ),
            ProcessSpec(
                "gateway",
                "Operation Gateway",
                (
                    str(self.gateway_python),
                    str(gateway / "operation_gateway.py"),
                    "--unity-command-port",
                    str(ports["unityAnnotations"]),
                    "--unity-event-port",
                    str(ports["unityEvents"]),
                    "--command-center-identity",
                    str(self.manifest["livekit"]["commandCenterIdentity"]),
                    "--drone-identity",
                    str(self.manifest["livekit"]["videoPublisherIdentity"]),
                    "--video-track",
                    str(self.manifest["livekit"]["videoTrackName"]),
                ),
                gateway,
                "operation-gateway.log",
                "STRIKERPC_READY OPERATION_GATEWAY",
                MODULE_DEPENDENCIES["gateway"],
                menu_order=2,
                start_order=40,
                aliases=("operation-gateway",),
                validator="gateway",
            ),
            ProcessSpec(
                "xr",
                "XR Adapter",
                (
                    str(self.gateway_python),
                    str(gateway / "xr_control_adapter.py"),
                    "--relay-port",
                    str(ports["controlRelay"]),
                    "--authority-port",
                    str(ports["controlAuthority"]),
                    "--pilot-port",
                    str(ports["pilotControlInput"]),
                    "--watchdog-seconds",
                    "0.2",
                    "--pilot-timeout",
                    "8.0",
                ),
                self.root,
                "xr-control-adapter.log",
                "STRIKERPC_READY XR_ADAPTER",
                MODULE_DEPENDENCIES["xr"],
                menu_order=3,
                start_order=50,
                aliases=("adapter", "xr-adapter"),
                validator="xr",
                neutral_on_stop=True,
            ),
        )
        return (*builtins, *self._load_extension_specs())

    # 확장 모듈 설정에서 사용할 경로와 포트 placeholder를 만든다.
    def _module_context(self) -> dict[str, object]:
        return {
            "root": str(self.root),
            "manifest": str(self.manifest_path),
            "gateway_python": str(self.gateway_python),
            "pilot_python": str(self.pilot_python),
            "vision_python": str(self.vision_python),
            "pilot_root": str(self.pilot_root),
            "mission_root": str(self.mission_root),
            "pilot_provider": self.pilot_provider,
            "vision_root": str(self.vision_root),
            "ports": self.manifest["ports"],
            "livekit": self.manifest["livekit"],
            "mission_env": str(
                self.mission_env_path or (self.root / "Gateway" / ".env")
            ),
        }

    # 설정 문자열의 placeholder를 현재 작업 경로로 치환한다.
    def _format_module_value(self, value: str) -> str:
        try:
            return value.format_map(self._module_context())
        except (KeyError, ValueError) as error:
            raise RuntimeError(f"모듈 placeholder가 올바르지 않습니다: {value}") from error

    # JSON에 등록한 외부 기능 모듈을 검증해 실행 사양으로 변환한다.
    def _load_extension_specs(self) -> tuple[ProcessSpec, ...]:
        if self._extension_specs is not None:
            return self._extension_specs
        try:
            data = json.loads(self.module_config_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(f"모듈 설정 파일을 읽을 수 없습니다: {error}") from error
        if data.get("version") != 1 or not isinstance(data.get("modules"), list):
            raise RuntimeError("launcher_modules.json은 version 1과 modules 배열이 필요합니다.")

        specs: list[ProcessSpec] = []
        known_keys = set(MODULE_DEPENDENCIES) | set(MODULE_ALIASES)
        known_log_names = {
            "position-sync.log",
            "pilot-pc.log",
            "vision-adapter.log",
            "operation-gateway.log",
            "xr-control-adapter.log",
        }
        for index, item in enumerate(data["modules"]):
            if not isinstance(item, dict):
                raise RuntimeError(f"확장 모듈 #{index + 1} 형식이 올바르지 않습니다.")
            if item.get("enabled", True) is False:
                continue
            key = str(item.get("key", "")).strip().lower()
            if not re.fullmatch(r"[a-z][a-z0-9_-]*", key):
                raise RuntimeError(f"확장 모듈 key가 올바르지 않습니다: {key}")
            if key in known_keys:
                raise RuntimeError(f"중복된 모듈 key입니다: {key}")
            known_keys.add(key)

            command_values = item.get("command")
            if (
                not isinstance(command_values, list)
                or len(command_values) < 2
                or not all(isinstance(value, str) for value in command_values)
            ):
                raise RuntimeError(f"{key} command는 2개 이상의 문자열 배열이어야 합니다.")
            dependency_values = item.get("dependencies", [])
            conflict_values = item.get("conflicts", [])
            alias_values = item.get("aliases", [])
            if not isinstance(dependency_values, list) or not all(
                isinstance(value, str) for value in dependency_values
            ):
                raise RuntimeError(f"{key} dependencies는 문자열 배열이어야 합니다.")
            if not isinstance(alias_values, list) or not all(
                isinstance(value, str) for value in alias_values
            ):
                raise RuntimeError(f"{key} aliases는 문자열 배열이어야 합니다.")
            if not isinstance(conflict_values, list) or not all(
                isinstance(value, str) for value in conflict_values
            ):
                raise RuntimeError(f"{key} conflicts는 문자열 배열이어야 합니다.")
            dependencies = tuple(
                value.strip().lower() for value in dependency_values
            )
            conflicts = tuple(value.strip().lower() for value in conflict_values)
            aliases = tuple(value.strip().lower() for value in alias_values)
            if any(
                not re.fullmatch(r"[a-z][a-z0-9_-]*", alias)
                for alias in aliases
            ):
                raise RuntimeError(f"{key} alias 형식이 올바르지 않습니다.")
            log_name = str(item.get("logName", f"{key}.log"))
            if Path(log_name).name != log_name:
                raise RuntimeError(f"{key} logName에는 파일 이름만 사용할 수 있습니다.")
            if log_name in known_log_names:
                raise RuntimeError(f"중복된 모듈 logName입니다: {log_name}")
            known_log_names.add(log_name)
            ready_token = str(item.get("readyToken", "")).strip()
            if not ready_token:
                raise RuntimeError(f"{key} readyToken이 필요합니다.")
            preflight = item.get("preflight", {})
            if not isinstance(preflight, dict):
                raise RuntimeError(f"{key} preflight는 객체여야 합니다.")

            specs.append(
                ProcessSpec(
                    key=key,
                    name=str(item.get("name", key)),
                    command=tuple(
                        self._format_module_value(value)
                        for value in command_values
                    ),
                    cwd=Path(
                        self._format_module_value(str(item.get("cwd", "{root}")))
                    ),
                    log_name=log_name,
                    ready_token=ready_token,
                    dependencies=dependencies,
                    conflicts=conflicts,
                    menu_order=int(item.get("menuOrder", 100 + index)),
                    start_order=int(item.get("startOrder", 100 + index)),
                    aliases=aliases,
                    validator="generic",
                    requires_drone=bool(item.get("requiresDrone", False)),
                    neutral_on_stop=bool(item.get("neutralOnStop", False)),
                    preflight=preflight,
                )
            )

        all_dependencies = {
            **MODULE_DEPENDENCIES,
            **{spec.key: spec.dependencies for spec in specs},
        }
        all_keys = set(all_dependencies)
        alias_owners = {alias: key for alias, key in MODULE_ALIASES.items()}
        for spec in specs:
            for alias in spec.aliases:
                if alias in all_keys and alias != spec.key:
                    raise RuntimeError(f"모듈 alias가 key와 충돌합니다: {alias}")
                owner = alias_owners.get(alias)
                if owner is not None and owner != spec.key:
                    raise RuntimeError(f"중복된 모듈 alias입니다: {alias}")
                alias_owners[alias] = spec.key
        for key, dependencies in all_dependencies.items():
            missing = [value for value in dependencies if value not in all_keys]
            if missing:
                raise RuntimeError(
                    f"{key} 의존 모듈이 등록되지 않았습니다: {', '.join(missing)}"
                )
        for spec in specs:
            missing_conflicts = [
                value for value in spec.conflicts if value not in all_keys
            ]
            if missing_conflicts:
                raise RuntimeError(
                    f"{spec.key} 충돌 모듈이 등록되지 않았습니다: "
                    f"{', '.join(missing_conflicts)}"
                )
            if spec.key in spec.conflicts:
                raise RuntimeError(f"{spec.key}는 자기 자신과 충돌할 수 없습니다.")
        self._validate_dependency_graph(all_dependencies)
        self._extension_specs = tuple(specs)
        return self._extension_specs

    # 순환 의존성이 있으면 프로세스를 시작하기 전에 중단한다.
    @staticmethod
    def _validate_dependency_graph(
        dependencies: dict[str, tuple[str, ...]],
    ) -> None:
        visited: set[str] = set()
        visiting: set[str] = set()

        def visit(key: str) -> None:
            if key in visiting:
                raise RuntimeError(f"모듈 순환 의존성이 있습니다: {key}")
            if key in visited:
                return
            visiting.add(key)
            for dependency in dependencies[key]:
                visit(dependency)
            visiting.remove(key)
            visited.add(key)

        for key in dependencies:
            visit(key)

    # 모듈 키로 실행 사양을 찾는다.
    def process_specs(self) -> dict[str, ProcessSpec]:
        return {spec.key: spec for spec in self.build_processes()}

    # 실제 종료 순서 계산에 사용하는 전체 시작 순서를 반환한다.
    def start_order(self) -> tuple[str, ...]:
        specs = self.process_specs()
        return self.resolve_modules(tuple(specs), enforce_conflicts=False)

    # 메뉴 표시 순서와 번호를 동적으로 만든다.
    def menu_modules(self) -> tuple[tuple[str, str], ...]:
        specs = sorted(
            self.process_specs().values(),
            key=lambda spec: (spec.menu_order, spec.key),
        )
        return tuple((str(index), spec.key) for index, spec in enumerate(specs, 1))

    # 체크박스 선택 창에 표시할 동적 모듈 정보를 메뉴 순서대로 반환한다.
    def module_selection_entries(self) -> tuple[ModuleSelectionEntry, ...]:
        specs = self.process_specs()
        return tuple(
            ModuleSelectionEntry(
                key=key,
                name=specs[key].name,
                dependencies=specs[key].dependencies,
                requires_drone=specs[key].requires_drone,
            )
            for _, key in self.menu_modules()
        )

    # 내장 별칭과 확장 모듈 별칭을 합쳐 사용자 입력을 변환한다.
    def parse_selection(self, raw_value: str) -> tuple[str, ...]:
        specs = self.process_specs()
        aliases = dict(MODULE_ALIASES)
        for spec in specs.values():
            aliases.update({alias: spec.key for alias in spec.aliases})
        return parse_module_selection(
            raw_value,
            number_to_key=dict(self.menu_modules()),
            available_keys=set(specs),
            aliases=aliases,
        )

    # 선택 모듈의 전이 의존성을 포함해 시작 순서로 정렬한다.
    def resolve_modules(
        self,
        selected: tuple[str, ...],
        *,
        enforce_conflicts: bool = True,
    ) -> tuple[str, ...]:
        specs = self.process_specs()
        ordered = sorted(
            selected,
            key=lambda key: (
                specs[key].start_order if key in specs else sys.maxsize,
                key,
            ),
        )
        resolved: list[str] = []
        visiting: set[str] = set()

        def add_with_dependencies(key: str) -> None:
            if key not in specs:
                raise ValueError(f"알 수 없는 모듈입니다: {key}")
            if key in visiting:
                raise RuntimeError(f"모듈 순환 의존성이 있습니다: {key}")
            if key in resolved:
                return
            visiting.add(key)
            for dependency in specs[key].dependencies:
                add_with_dependencies(dependency)
            visiting.remove(key)
            resolved.append(key)

        for key in ordered:
            add_with_dependencies(key)
        if enforce_conflicts:
            selected_set = set(resolved)
            for key in resolved:
                conflicts = selected_set.intersection(specs[key].conflicts)
                if conflicts:
                    conflict = sorted(conflicts)[0]
                    raise RuntimeError(
                        f"동시에 실행할 수 없는 모듈입니다: {key}, {conflict}"
                    )
        return tuple(resolved)

    # 지정 모듈에 의존하는 실행 중 모듈을 찾는다.
    def dependent_modules(self, selected: set[str]) -> set[str]:
        dependencies = {
            key: spec.dependencies
            for key, spec in self.process_specs().items()
        }
        dependents: set[str] = set()
        changed = True
        while changed:
            changed = False
            for key, values in dependencies.items():
                if key in dependents or key in selected:
                    continue
                if any(
                    dependency in selected or dependency in dependents
                    for dependency in values
                ):
                    dependents.add(key)
                    changed = True
        return dependents

    # 실행 로그 디렉터리와 launcher.log를 연다.
    def open_logs(self) -> Path:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        log_dir = self.root / ".team-runtime" / "logs" / f"{stamp}_launcher"
        suffix = 2
        while log_dir.exists():
            log_dir = log_dir.with_name(f"{stamp}_launcher_{suffix}")
            suffix += 1
        log_dir.mkdir(parents=True)
        self.log_dir = log_dir
        self.launcher_log = (log_dir / "launcher.log").open(
            "a",
            encoding="utf-8",
            buffering=1,
        )
        return log_dir

    # 콘솔 메시지를 launcher.log에도 기록한다.
    def emit(self, message: str) -> None:
        print(message, flush=True)
        if self.launcher_log is not None:
            timestamp = datetime.now().isoformat(timespec="seconds")
            self.launcher_log.write(f"{timestamp} {message}\n")

    # 자식 프로세스를 별도 그룹으로 시작하고 출력을 수집한다.
    def start_process(self, spec: ProcessSpec) -> ManagedChild:
        if self.log_dir is None:
            raise RuntimeError("로그 디렉터리가 준비되지 않았습니다.")
        count = self._log_counts.get(spec.key, 0) + 1
        self._log_counts[spec.key] = count
        base = Path(spec.log_name)
        log_name = f"{base.stem}_{count:02d}{base.suffix}"
        log_path = self.log_dir / log_name
        self._latest_logs[spec.key] = log_path
        log_file = log_path.open(
            "a",
            encoding="utf-8",
            buffering=1,
        )
        environment = self._child_environment(spec)
        environment["PYTHONUNBUFFERED"] = "1"
        environment["PYTHONUTF8"] = "1"
        creationflags = (
            subprocess.CREATE_NEW_PROCESS_GROUP
            if sys.platform == "win32"
            else 0
        )
        try:
            process = subprocess.Popen(
                list(spec.command),
                cwd=spec.cwd,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                creationflags=creationflags,
            )
        except Exception:
            log_file.close()
            raise
        child = ManagedChild(spec, process, log_file)
        child.thread = threading.Thread(
            target=self._pump_output,
            args=(child,),
            name=f"{spec.key}-output",
            daemon=True,
        )
        child.thread.start()
        self.children.append(child)
        self.emit(f"[Launcher] {spec.name} 시작 PID={process.pid}, 로그={log_name}")
        return child

    # 준비 신호 또는 조기 종료를 기다린다.
    def wait_until_ready(
        self,
        child: ManagedChild,
        should_stop: Callable[[], bool] | None = None,
    ) -> None:
        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            if should_stop is not None and should_stop():
                raise LauncherStopRequested("모듈 시작 중 종료 요청")
            if child.ready.wait(0.1):
                return_code = child.process.poll()
                if return_code is not None:
                    raise RuntimeError(
                        f"{child.spec.name} 준비 직후 종료: exit {return_code}"
                    )
                self.emit(f"[Launcher] {child.spec.name} 준비 완료")
                child.status = "실행 중"
                return
            return_code = child.process.poll()
            if return_code is not None:
                raise RuntimeError(
                    f"{child.spec.name} 준비 전 종료: exit {return_code}"
                )
        raise TimeoutError(
            f"{child.spec.name} 준비 시간이 {self.startup_timeout:g}초를 초과했습니다."
        )

    # 지정 키의 현재 자식 프로세스를 찾는다.
    def child_for(self, key: str) -> ManagedChild | None:
        return next((child for child in self.children if child.spec.key == key), None)

    # 모듈이 현재 실행 중인지 확인한다.
    def is_running(self, key: str) -> bool:
        child = self.child_for(key)
        return child is not None and child.process.poll() is None

    # 모듈별 실행 상태 문자열을 반환한다.
    def module_status(self, key: str) -> str:
        child = self.child_for(key)
        if child is not None and child.process.poll() is None:
            return child.status
        if key in self.failures:
            return "오류"
        return "중지"

    # 선택 모듈과 필요한 의존 모듈을 순서대로 시작한다.
    def start_modules(
        self,
        selected: tuple[str, ...],
        should_stop: Callable[[], bool] | None = None,
    ) -> tuple[str, ...]:
        resolved = self.resolve_modules(selected)
        added = tuple(key for key in resolved if key not in selected)
        if added:
            names = ", ".join(self.process_specs()[key].name for key in added)
            self.emit(f"[Launcher] 의존 모듈 자동 추가: {names}")

        started: list[str] = []
        specs = self.process_specs()
        try:
            self.refresh_processes()
            for key in resolved:
                if self.is_running(key):
                    self.emit(f"[Launcher] {specs[key].name} 이미 실행 중")
                    continue
                self.validate_module(key)
                if specs[key].requires_drone and (
                    key == "pilot" or not self.is_running("pilot")
                ):
                    if should_stop is None:
                        self.wait_for_connection()
                    else:
                        self.wait_for_connection(should_stop)
                    time.sleep(0.1)
                child = self.start_process(specs[key])
                started.append(key)
                if should_stop is None:
                    self.wait_until_ready(child)
                else:
                    self.wait_until_ready(child, should_stop)
                self.failures.pop(key, None)
        except LauncherStopRequested:
            if started:
                self.emit("[Launcher] 종료 요청으로 시작한 모듈을 롤백합니다.")
                self.stop_modules(
                    set(started),
                    include_dependents=False,
                    send_neutral=True,
                )
            raise
        except KeyboardInterrupt:
            if started:
                self.emit("[Launcher] 이번 요청에서 시작한 모듈을 롤백합니다.")
                self.stop_modules(
                    set(started),
                    include_dependents=False,
                    send_neutral=True,
                )
            raise
        except Exception as error:
            if started:
                self.emit("[Launcher] 이번 요청에서 시작한 모듈을 롤백합니다.")
                self.stop_modules(
                    set(started),
                    include_dependents=False,
                    send_neutral=True,
                )
            self.failures[key] = str(error)
            raise

        names = ", ".join(specs[key].name for key in resolved)
        self.emit(f"[Launcher] 선택 모듈 준비 완료: {names}")
        return resolved

    # 종료된 모듈을 정리하고 실행 중인 종속 모듈만 안전 종료한다.
    def refresh_processes(self) -> tuple[str, ...]:
        exited = [
            child
            for child in tuple(self.children)
            if child.process.poll() is not None
        ]
        if not exited:
            return ()

        exited_keys: list[str] = []
        for child in exited:
            key = child.spec.key
            exited_keys.append(key)
            return_code = child.process.poll()
            self._finalize_child(child)
            if not self._stopping:
                reason = f"exit {return_code}"
                self.failures[key] = reason
                self.emit(f"[Launcher] {child.spec.name} 비정상 종료: {reason}")

        if not self._stopping:
            exited_set = set(exited_keys)
            if "xr" in exited_set and self.is_running("pilot"):
                self._send_neutral_to_pilot()
            dependents = {
                key
                for key in self.dependent_modules(exited_set)
                if self.is_running(key)
            }
            if dependents:
                names = ", ".join(
                    self.process_specs()[key].name
                    for key in reversed(self.start_order())
                    if key in dependents
                )
                self.emit(f"[Launcher] 종속 모듈 안전 종료: {names}")
                self.stop_modules(dependents, include_dependents=True)
        return tuple(exited_keys)

    # 선택 모듈과 실행 중인 종속 모듈을 역순으로 종료한다.
    def stop_modules(
        self,
        selected: set[str],
        *,
        include_dependents: bool = True,
        send_neutral: bool = True,
    ) -> tuple[str, ...]:
        targets = set(selected)
        if include_dependents:
            targets.update(self.dependent_modules(targets))
        present = {key for key in targets if self.child_for(key) is not None}
        if not present:
            for key in targets:
                self.failures.pop(key, None)
            return ()

        specs = self.process_specs()
        needs_neutral = any(specs[key].neutral_on_stop for key in present)
        if send_neutral and needs_neutral and self.is_running("pilot"):
            self.emit("[Launcher] 제어 모듈 종료 전 중립 RC를 전송합니다.")
            self._send_neutral_to_pilot()

        stopped: list[str] = []
        for key in reversed(self.start_order()):
            if key not in present:
                continue
            child = self.child_for(key)
            if child is None:
                continue
            child.status = "중지 중"
            self._stop_child(child)
            self.failures.pop(key, None)
            stopped.append(key)
            self.emit(f"[Launcher] {child.spec.name} 종료 완료")
        return tuple(stopped)

    # 단일 자식 프로세스에 정상 종료 후 강제 종료 단계를 적용한다.
    def _stop_child(self, child: ManagedChild) -> None:
        self._request_stop(child)
        timeout_seconds = (
            PILOT_SHUTDOWN_TIMEOUT_SECONDS
            if child.spec.key == "pilot"
            else SHUTDOWN_TIMEOUT_SECONDS
        )
        try:
            child.process.wait(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            child.process.terminate()
            try:
                child.process.wait(timeout=SHUTDOWN_TIMEOUT_SECONDS)
            except subprocess.TimeoutExpired:
                child.process.kill()
                child.process.wait()
        self._finalize_child(child)

    # 종료된 자식의 출력 thread와 로그 파일을 정리한다.
    def _finalize_child(self, child: ManagedChild) -> None:
        if child.thread is not None:
            child.thread.join(timeout=1)
        if not child.log_file.closed:
            child.log_file.close()
        if child in self.children:
            self.children.remove(child)

    # 실행 중인 모듈을 감시하며 외부 안전 종료 요청도 처리한다.
    def monitor(self, should_stop: Callable[[], bool] | None = None) -> None:
        while self.children:
            if should_stop is not None and should_stop():
                self.emit("[Launcher] 외부 안전 종료 요청을 받았습니다.")
                return
            self.refresh_processes()
            time.sleep(0.2)

    # 현재 상태를 포함한 콘솔 메뉴를 표시한다.
    def show_menu(self) -> None:
        specs = self.process_specs()
        print("\n=== StrikerPC 모듈 실행기 ===")
        for number, key in self.menu_modules():
            print(f"{number}. {specs[key].name:<18} [{self.module_status(key)}]")
        print("\nS. 모듈 시작  T. 모듈 중지  L. 로그 경로  R. 새로고침  0. 전체 종료")

    # 실행 중에도 시작·중지할 수 있는 상태 관리 메뉴를 실행한다.
    def run_menu(self) -> None:
        while True:
            self.refresh_processes()
            self.show_menu()
            action = input("명령을 선택하세요: ").strip().lower()
            if action == "0":
                return
            if action == "r":
                continue
            if action == "l":
                self.emit(f"[Launcher] 로그: {self.log_dir}")
                specs = self.process_specs()
                for key in self.start_order():
                    path = self._latest_logs.get(key)
                    if path is not None:
                        self.emit(f"[Launcher] {specs[key].name}: {path}")
                continue
            if action not in {"s", "t"}:
                self.emit("[Launcher] S, T, L, R 또는 0을 입력하세요.")
                continue

            try:
                selected = self.parse_selection(
                    input("모듈 번호를 쉼표로 구분해 입력하세요: ")
                )
            except ValueError as error:
                self.emit(f"[Launcher] 선택 오류: {error}")
                continue

            if action == "s":
                try:
                    self.start_modules(selected)
                except KeyboardInterrupt:
                    self.emit("[Launcher] 모듈 시작 요청을 취소했습니다.")
                except Exception as error:
                    self.emit(f"[Launcher] 모듈 시작 실패: {error}")
            else:
                self.stop_modules(set(selected), include_dependents=True)

    # 중립 RC 후 이번 실행기가 시작한 모든 모듈만 종료한다.
    def stop_all(self) -> None:
        if self._stopping:
            return
        self._stopping = True
        if self.children:
            self.emit("[Launcher] 전체 모듈을 안전 종료합니다.")
            if self.is_running("pilot"):
                self._send_neutral_to_pilot()
        self.stop_modules(
            set(self.start_order()),
            include_dependents=False,
            send_neutral=False,
        )

    # 대화형 메뉴 또는 비대화형 모듈 목록을 실행한다.
    def run(
        self,
        selected: tuple[str, ...] | None = None,
        should_stop: Callable[[], bool] | None = None,
    ) -> int:
        self.open_logs()
        result = 0
        try:
            self.validate_bootstrap()
            if selected is None:
                self.run_menu()
            else:
                self.start_modules(selected, should_stop)
                self.emit("[Launcher] 비대화형 모듈 실행 중. 종료는 Ctrl+C")
                self.monitor(should_stop)
                if self.failures:
                    result = 1
        except LauncherStopRequested:
            self.emit("[Launcher] 외부 안전 종료 요청을 받았습니다.")
        except KeyboardInterrupt:
            self.emit("[Launcher] 사용자 종료 요청")
        except EOFError:
            self.emit("[Launcher] 입력이 종료되어 실행기를 닫습니다.")
        except Exception as error:
            self.emit(f"[Launcher] 실행 실패: {error}")
            result = 1
        finally:
            self.stop_all()
            if self.log_dir is not None:
                self.emit(f"[Launcher] 로그: {self.log_dir}")
            if self.launcher_log is not None:
                self.launcher_log.close()
                self.launcher_log = None
        return result

    # Python 버전과 필수 패키지를 해당 가상환경에서 확인한다.
    @staticmethod
    def _verify_python(python_path: Path, import_script: str, name: str) -> None:
        version = subprocess.run(
            [python_path, "--version"],
            capture_output=True,
            text=True,
            check=False,
        )
        version_text = (version.stdout or version.stderr).strip()
        if version.returncode != 0 or not version_text.startswith("Python 3.12."):
            raise RuntimeError(f"{name} Python 3.12 환경이 아닙니다: {version_text}")
        imports = subprocess.run(
            [python_path, "-c", import_script],
            capture_output=True,
            text=True,
            check=False,
        )
        if imports.returncode != 0:
            detail = (imports.stderr or imports.stdout).strip().splitlines()
            last_line = detail[-1] if detail else "필수 패키지 확인 실패"
            raise RuntimeError(f"{name} 가상환경 검증 실패: {last_line}")

    # 팀 원본의 고정 커밋과 tracked 무변경 상태를 확인한다.
    def _verify_team_repository(self, key: str) -> None:
        expected = self.manifest["repositories"][key]
        name = {
            "pilot": "Pilot-PC",
            "viewer": "Externel-Viewer",
            "vision": "Vision",
            "missionPlatform": "drone-mission-platform",
        }[key]
        repository = (self.root / expected["path"]).resolve()
        if expected.get("ownership") == "owned-embedded":
            self._verify_embedded_vision(name, expected, repository)
            return

        command = [
            "git",
            "-c",
            f"safe.directory={repository.as_posix()}",
            "-C",
            str(repository),
        ]
        commit = subprocess.run(
            [*command, "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=False,
        )
        if commit.returncode != 0 or commit.stdout.strip() != expected["commit"]:
            raise RuntimeError(
                f"{name} 고정 커밋이 아닙니다: {commit.stdout.strip()}"
            )
        origin = subprocess.run(
            [*command, "remote", "get-url", "origin"],
            capture_output=True,
            text=True,
            check=False,
        )
        if origin.returncode != 0 or origin.stdout.strip() != expected["origin"]:
            raise RuntimeError(f"{name} origin이 다릅니다: {origin.stdout.strip()}")
        worktree_changed = subprocess.run(
            [*command, "diff", "--quiet", "HEAD", "--"],
            check=False,
        ).returncode
        staged_changed = subprocess.run(
            [*command, "diff", "--cached", "--quiet", "HEAD", "--"],
            check=False,
        ).returncode
        if worktree_changed or staged_changed:
            raise RuntimeError(f"{name} tracked 파일이 수정되어 있습니다.")

    # 내장 Vision은 Unity 설정 Prefab을 제외한 원본 subtree 동일성을 확인한다.
    def _verify_embedded_vision(
        self,
        name: str,
        expected: dict,
        repository: Path,
    ) -> None:
        try:
            relative = repository.relative_to(self.root)
        except ValueError as error:
            raise RuntimeError(f"{name} 경로가 통합 저장소를 벗어납니다: {repository}") from error
        marker = repository / "Assets" / "TelloVisionTracking"
        if not marker.is_dir():
            raise RuntimeError(f"{name} 런타임 소스가 없습니다: {marker}")

        command = [
            "git",
            "-c",
            f"safe.directory={self.root.as_posix()}",
            "-C",
            str(self.root),
        ]
        commit = str(expected["commit"])
        ancestor = subprocess.run(
            [*command, "merge-base", "--is-ancestor", commit, "HEAD"],
            capture_output=True,
            text=True,
            check=False,
        )
        if ancestor.returncode != 0:
            raise RuntimeError(f"{name} 원본 커밋이 현재 이력에 없습니다: {commit}")

        source_tree = subprocess.run(
            [*command, "rev-parse", f"{commit}:Assets/TelloVisionTracking"],
            capture_output=True,
            text=True,
            check=False,
        )
        embedded_tree = subprocess.run(
            [
                *command,
                "rev-parse",
                f"HEAD:{relative.as_posix()}/Assets/TelloVisionTracking",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if source_tree.returncode != 0 or embedded_tree.returncode != 0:
            raise RuntimeError(f"{name} 내장 소스가 고정 커밋과 다릅니다.")

        source_prefab = ":(exclude)Prefabs/TelloVisionTracking.prefab"
        source_changed = subprocess.run(
            [
                *command,
                "diff",
                "--quiet",
                source_tree.stdout.strip(),
                embedded_tree.stdout.strip(),
                "--",
                ".",
                source_prefab,
            ],
            check=False,
        ).returncode
        if source_changed:
            raise RuntimeError(f"{name} 내장 소스가 고정 커밋과 다릅니다.")

        relative_text = relative.as_posix()
        unity_prefab = (
            f":(exclude){relative_text}/Assets/TelloVisionTracking/Prefabs/"
            "TelloVisionTracking.prefab"
        )
        worktree_changed = subprocess.run(
            [
                *command,
                "diff",
                "--quiet",
                "HEAD",
                "--",
                relative_text,
                unity_prefab,
            ],
            check=False,
        ).returncode
        staged_changed = subprocess.run(
            [
                *command,
                "diff",
                "--cached",
                "--quiet",
                "HEAD",
                "--",
                relative_text,
                unity_prefab,
            ],
            check=False,
        ).returncode
        if worktree_changed or staged_changed:
            raise RuntimeError(f"{name} tracked 파일이 수정되어 있습니다.")

    # 자식 출력을 콘솔과 전용 로그에 동시에 기록한다.
    def _pump_output(self, child: ManagedChild) -> None:
        if child.process.stdout is None:
            return
        try:
            for raw_line in child.process.stdout:
                line = raw_line.rstrip("\r\n")
                prefixed = f"[{child.spec.name}] {line}"
                print(prefixed, flush=True)
                child.log_file.write(prefixed + "\n")
                child.log_file.flush()
                if any(token in line for token in compatible_ready_tokens(child.spec.ready_token)):
                    child.ready.set()
        finally:
            child.process.stdout.close()

    # Pilot IPC로 중립 RC를 직접 전달한다.
    def _send_neutral_to_pilot(self) -> None:
        try:
            port = self.manifest["ports"]["pilotControlInput"]
            packet = encode_control_packet(
                ControlPacket(
                    sequence=0,
                    input_ticks=time.perf_counter_ns(),
                    axes=(0, 0, 0, 0),
                    command=COMMAND_NEUTRAL,
                    flags=INTERNAL_CONTROL_FLAG,
                )
            )
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sender.sendto(packet, ("127.0.0.1", port))
                time.sleep(0.1)
            finally:
                sender.close()
        except (KeyError, OSError):
            pass

    # 자식 프로세스 그룹에 정상 종료 신호를 보낸다.
    @staticmethod
    def _request_stop(child: ManagedChild) -> None:
        if child.process.poll() is not None:
            return
        try:
            if sys.platform == "win32":
                child.process.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                child.process.send_signal(signal.SIGINT)
        except (OSError, ValueError):
            if child.process.poll() is None:
                try:
                    child.process.terminate()
                except OSError:
                    pass


# 모듈을 복수 선택하고 자동 의존성과 충돌을 시작 전에 확인하는 창을 표시한다.
def select_modules_with_window(
    launcher: StrikerPCLauncher,
) -> tuple[str, ...] | None:
    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError as error:
        raise ModuleSelectionUnavailable(
            "Python Tcl/Tk를 불러올 수 없습니다."
        ) from error

    try:
        root = tk.Tk()
    except tk.TclError as error:
        raise ModuleSelectionUnavailable(
            "Windows 모듈 선택 창을 만들 수 없습니다."
        ) from error

    entries = launcher.module_selection_entries()
    selection: tuple[str, ...] | None = None
    variables: dict[str, tk.BooleanVar] = {}

    root.title("StrikerPC 모듈 선택")
    root.resizable(True, True)
    root.minsize(560, 430)

    outer = ttk.Frame(root, padding=18)
    outer.pack(fill="both", expand=True)
    ttk.Label(
        outer,
        text="실행할 모듈을 선택하세요",
        font=("Malgun Gothic", 15, "bold"),
    ).pack(anchor="w")
    ttk.Label(
        outer,
        text="필요한 의존 모듈은 자동으로 함께 시작됩니다.",
    ).pack(anchor="w", pady=(3, 12))

    list_frame = ttk.Frame(outer)
    list_frame.pack(fill="both", expand=True)
    canvas = tk.Canvas(list_frame, highlightthickness=0, borderwidth=0)
    scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=canvas.yview)
    module_frame = ttk.Frame(canvas, padding=(2, 2, 8, 2))
    module_window = canvas.create_window((0, 0), window=module_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)
    canvas.pack(side="left", fill="both", expand=True)
    scrollbar.pack(side="right", fill="y")

    summary = tk.StringVar(value="모듈을 하나 이상 선택하세요.")
    summary_label = tk.Label(
        outer,
        textvariable=summary,
        anchor="w",
        justify="left",
        fg="#334155",
    )
    summary_label.pack(fill="x", pady=(12, 10))

    button_frame = ttk.Frame(outer)
    button_frame.pack(fill="x")
    start_button = ttk.Button(button_frame, text="선택 모듈 시작")
    start_button.pack(side="right")
    ttk.Button(button_frame, text="취소", command=root.destroy).pack(
        side="right",
        padx=(0, 8),
    )

    # 현재 체크 상태의 실행 순서와 충돌을 즉시 갱신한다.
    def update_preview() -> None:
        selected = tuple(
            entry.key for entry in entries if variables[entry.key].get()
        )
        if not selected:
            summary.set("모듈을 하나 이상 선택하세요.")
            summary_label.configure(fg="#334155")
            start_button.state(["disabled"])
            return
        try:
            resolved = launcher.resolve_modules(selected)
        except (RuntimeError, ValueError) as error:
            summary.set(f"선택 오류: {error}")
            summary_label.configure(fg="#b91c1c")
            start_button.state(["disabled"])
            return

        auto_added = tuple(key for key in resolved if key not in selected)
        detail = " → ".join(resolved)
        if auto_added:
            detail += f"\n자동 포함: {', '.join(auto_added)}"
        summary.set(f"실행 순서: {detail}")
        summary_label.configure(fg="#166534")
        start_button.state(["!disabled"])

    # 검증된 직접 선택 목록을 반환하고 창을 닫는다.
    def confirm_selection() -> None:
        nonlocal selection
        selected = tuple(
            entry.key for entry in entries if variables[entry.key].get()
        )
        if not selected:
            return
        try:
            launcher.resolve_modules(selected)
        except (RuntimeError, ValueError):
            update_preview()
            return
        selection = selected
        root.destroy()

    start_button.configure(command=confirm_selection)
    for row, entry in enumerate(entries):
        variable = tk.BooleanVar(master=root, value=False)
        variables[entry.key] = variable
        item = ttk.Frame(module_frame, padding=(6, 6))
        item.grid(row=row * 2, column=0, sticky="ew")
        item.columnconfigure(0, weight=1)
        ttk.Checkbutton(
            item,
            text=f"{entry.name}  ({entry.key})",
            variable=variable,
            command=update_preview,
        ).grid(row=0, column=0, sticky="w")
        notes: list[str] = []
        if entry.dependencies:
            notes.append("의존: " + ", ".join(entry.dependencies))
        if entry.requires_drone:
            notes.append("Tello 연결 필요")
        if notes:
            ttk.Label(
                item,
                text=" · ".join(notes),
                foreground="#64748b",
            ).grid(row=1, column=0, sticky="w", padx=(24, 0), pady=(2, 0))
        ttk.Separator(module_frame, orient="horizontal").grid(
            row=row * 2 + 1,
            column=0,
            sticky="ew",
        )

    module_frame.columnconfigure(0, weight=1)

    # 스크롤 영역과 창 폭을 현재 체크박스 콘텐츠에 맞춘다.
    def synchronize_scroll_region(_event=None) -> None:
        canvas.configure(scrollregion=canvas.bbox("all"))

    def synchronize_content_width(event) -> None:
        canvas.itemconfigure(module_window, width=event.width)

    module_frame.bind("<Configure>", synchronize_scroll_region)
    canvas.bind("<Configure>", synchronize_content_width)
    canvas.bind_all(
        "<MouseWheel>",
        lambda event: canvas.yview_scroll(int(-event.delta / 120), "units"),
    )
    root.bind("<Return>", lambda _event: start_button.invoke())
    root.bind("<Escape>", lambda _event: root.destroy())

    update_preview()
    root.update_idletasks()
    width = 680
    height = min(720, max(500, 235 + len(entries) * 48))
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = max(0, (screen_width - width) // 2)
    y = max(0, (screen_height - height) // 2)
    root.geometry(f"{width}x{height}+{x}+{y}")
    root.attributes("-topmost", True)
    root.after(250, lambda: root.attributes("-topmost", False))
    root.lift()
    root.focus_force()
    root.mainloop()
    return selection


# 명령행 옵션을 읽는다.
def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="StrikerPC 선택형 모듈 실행기")
    parser.add_argument("--manifest", default=str(root / "team-sources.json"))
    parser.add_argument(
        "--module-config",
        default=str(root / "Gateway" / "launcher_modules.json"),
    )
    parser.add_argument("--retry-seconds", type=float, default=RETRY_SECONDS)
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=STARTUP_TIMEOUT_SECONDS,
    )
    parser.add_argument(
        "--modules",
        help="비대화형 실행 모듈 목록(예: pilot,gateway,xr)",
    )
    parser.add_argument(
        "--text-menu",
        action="store_true",
        help="체크박스 창 대신 기존 콘솔 모듈 메뉴를 사용합니다.",
    )
    parser.add_argument(
        "--stop-file",
        help="파일이 생성되면 모듈을 중립 상태로 안전 종료합니다.",
    )
    parser.add_argument(
        "--require-visible-console",
        action="store_true",
        help="Pilot 시작 전에 화면에 보이는 비상 제어 콘솔을 요구합니다.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.require_visible_console:
        if not activate_emergency_console():
            print(
                "[Launcher] Visible emergency console is unavailable; "
                "drone runtimes will not start.",
                file=sys.stderr,
                flush=True,
            )
            return 3
        print_emergency_console_banner()
    launcher = StrikerPCLauncher(
        Path(args.manifest),
        module_config_path=Path(args.module_config),
        retry_seconds=args.retry_seconds,
        startup_timeout=args.startup_timeout,
    )
    if args.modules:
        try:
            selected = launcher.parse_selection(args.modules)
        except (RuntimeError, ValueError) as error:
            print(f"[Launcher] 모듈 선택 오류: {error}", file=sys.stderr)
            return 2
    elif args.text_menu:
        selected = None
    else:
        try:
            launcher.validate_bootstrap()
            window_selection = select_modules_with_window(launcher)
        except ModuleSelectionUnavailable as error:
            print(
                f"[Launcher] {error} 기존 콘솔 메뉴로 전환합니다.",
                file=sys.stderr,
                flush=True,
            )
            selected = None
        except (RuntimeError, ValueError) as error:
            print(f"[Launcher] 모듈 선택 오류: {error}", file=sys.stderr)
            return 2
        else:
            if window_selection is None:
                return 0
            selected = window_selection
    stop_file = Path(args.stop_file).resolve() if args.stop_file else None
    should_stop = (lambda: stop_file.exists()) if stop_file is not None else None
    return launcher.run(selected, should_stop=should_stop)


if __name__ == "__main__":
    raise SystemExit(main())
