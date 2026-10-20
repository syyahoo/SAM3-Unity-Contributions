from __future__ import annotations

import argparse
import json
import os
import queue
import re
import socket
import sys
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import font as tkfont
from tkinter import messagebox, ttk
from typing import TYPE_CHECKING, Callable

from strikerpc_launcher import StrikerPCLauncher

if TYPE_CHECKING:
    from strikerpc_control_gui import RuntimeSupervisorController


DEFAULT_DASHBOARD_STATE_PORT = 8895
LOG_ACTIVE_SECONDS = 6.0
SAFETY_FRESH_SECONDS = 3.0
CONTROL_STATUS_PATTERN = re.compile(
    r"Control 60Hz status sent=(\d+) failed=(\d+) missed=(\d+) "
    r"source=([^\s]+) inputAgeMs=(\d+)"
)
LAUNCHER_EVENT_PATTERN = re.compile(
    r"\[Launcher\] (?P<name>.+?) "
    r"(?P<event>시작 PID=|준비 완료|비정상 종료:|종료 완료)"
)
LOG_TIME_PATTERN = re.compile(
    r"^(?P<stamp>\d{4}-\d{2}-\d{2}[T ][0-9:.,+-]+)\s+(?P<body>.*)$"
)

COLORS = {
    "window": "#07090b",
    "panel": "#0e1317",
    "panel_alt": "#0b0f13",
    "border": "#232c35",
    "border_bright": "#39434d",
    "text": "#e9eef3",
    "muted": "#93a1ae",
    "cyan": "#35b2ac",
    "blue": "#3b82d9",
    "green": "#4fa97c",
    "amber": "#e0a33a",
    "red": "#d9483f",
}

STATUS_TEXT_KO = {
    "ALERT": "경고",
    "CONNECTED": "연결됨",
    "ERROR": "오류",
    "HEALTHY": "정상",
    "MONITORING": "감시 중",
    "NEUTRAL LOCK": "중립 잠금",
    "OFFLINE": "연결 안 됨",
    "SAFE": "안전",
    "STALE": "갱신 지연",
    "STANDBY": "대기",
    "STARTING": "시작 중",
    "WAITING": "대기",
    "WATCHDOG GRACE": "감시 유예",
    "LINK RECOVERY NEUTRAL": "링크 복구 중립",
    "LANDING": "착륙 중",
}


@dataclass(frozen=True)
class ControlMetrics:
    sent: int = 0
    failed: int = 0
    missed: int = 0
    source: str = "--"
    input_age_ms: int | None = None


@dataclass(frozen=True)
class DashboardSnapshot:
    log_directory: Path | None
    log_age_seconds: float | None
    module_states: dict[str, str]
    metrics: ControlMetrics
    console_lines: tuple[str, ...]
    warning_count: int
    error_count: int


# 가장 최근에 갱신된 launcher 세션 디렉터리를 찾는다.
def find_latest_log_directory(log_root: Path) -> Path | None:
    if not log_root.exists():
        return None
    candidates = [path for path in log_root.iterdir() if path.is_dir()]
    if not candidates:
        return None
    return max(candidates, key=latest_directory_mtime)


# 디렉터리 안 로그 파일 중 가장 최근 수정 시간을 반환한다.
def latest_directory_mtime(directory: Path) -> float:
    mtimes = [directory.stat().st_mtime]
    try:
        mtimes.extend(path.stat().st_mtime for path in directory.glob("*.log"))
    except OSError:
        pass
    return max(mtimes)


# UTF-8 로그의 마지막 줄만 제한해 읽는다.
def read_tail_lines(path: Path, limit: int = 200) -> tuple[str, ...]:
    if limit <= 0 or not path.exists():
        return ()
    try:
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            return tuple(deque((line.rstrip() for line in stream), maxlen=limit))
    except OSError:
        return ()


# launcher 이벤트를 모듈별 OFFLINE/STARTING/CONNECTED/ERROR 상태로 변환한다.
def parse_launcher_module_states(
    lines: tuple[str, ...],
    module_names: dict[str, str],
    *,
    stale: bool = False,
) -> dict[str, str]:
    name_to_key = {name: key for key, name in module_names.items()}
    states = {key: "OFFLINE" for key in module_names}
    for line in lines:
        match = LAUNCHER_EVENT_PATTERN.search(line)
        if match is None:
            continue
        key = name_to_key.get(match.group("name"))
        if key is None:
            continue
        event = match.group("event")
        if event == "시작 PID=":
            states[key] = "STARTING"
        elif event == "준비 완료":
            states[key] = "CONNECTED"
        elif event == "비정상 종료:":
            states[key] = "ERROR"
        else:
            states[key] = "OFFLINE"
    if stale:
        return {
            key: "STALE" if state in {"STARTING", "CONNECTED"} else state
            for key, state in states.items()
        }
    return states


# Pilot 로그의 최신 60Hz 제어 누적 지표를 추출한다.
def parse_control_metrics(lines: tuple[str, ...]) -> ControlMetrics:
    for line in reversed(lines):
        match = CONTROL_STATUS_PATTERN.search(line)
        if match is None:
            continue
        return ControlMetrics(
            sent=int(match.group(1)),
            failed=int(match.group(2)),
            missed=int(match.group(3)),
            source=match.group(4),
            input_age_ms=int(match.group(5)),
        )
    return ControlMetrics()


# 여러 모듈 로그를 시간순 콘솔 한 줄 형식으로 합친다.
def collect_console_lines(directory: Path | None, limit: int = 180) -> tuple[str, ...]:
    if directory is None:
        return ()
    collected: list[tuple[str, str]] = []
    for path in directory.glob("*.log"):
        for line in read_tail_lines(path, 80):
            match = LOG_TIME_PATTERN.match(line)
            if match is None:
                stamp = datetime.fromtimestamp(path.stat().st_mtime).isoformat()
                body = line
            else:
                stamp = match.group("stamp").replace(" ", "T")
                body = match.group("body")
            collected.append((stamp, f"{stamp[11:23]:<12} [{path.stem}] {body}"))
    collected.sort(key=lambda item: item[0])
    return tuple(line for _, line in collected[-limit:])


# 안전 상태 payload의 최신성 및 잠금 상태를 화면 등급으로 분류한다.
def classify_safety_status(
    payload: dict[str, object] | None,
    age_seconds: float | None,
) -> tuple[str, str]:
    if payload is None or age_seconds is None:
        return "WAITING", "muted"
    if age_seconds > SAFETY_FRESH_SECONDS:
        return "STALE", "amber"
    if bool(payload.get("forceNeutral")):
        return "NEUTRAL LOCK", "red"
    if payload.get("lastError"):
        return "ALERT", "red"
    state = str(payload.get("state", "unknown"))
    if state in {"healthy", "grounded"}:
        return "SAFE", "green"
    if state in {"landing", "watchdog-grace", "link-recovery-neutral"}:
        return state.replace("-", " ").upper(), "amber"
    return state.replace("-", " ").upper(), "cyan"


# 비행 중이거나 최신 착륙 확인이 없으면 런타임 종료 확인을 요구한다.
def runtime_stop_confirmation_required(
    payload: dict[str, object] | None,
    age_seconds: float | None,
) -> bool:
    if payload is None or age_seconds is None or age_seconds > SAFETY_FRESH_SECONDS:
        return True
    return bool(
        payload.get("airborne")
        or not payload.get("groundedConfirmed")
        or payload.get("forceNeutral")
        or payload.get("lastError")
    )


class SafetyStatusReceiver:
    def __init__(self, port: int) -> None:
        self.port = port
        self.local_port = port
        self.error = ""
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._payload: dict[str, object] | None = None
        self._received_at: float | None = None
        self._receipts: deque[float] = deque()

    # 전용 fan-out UDP 포트 수신을 시작한다.
    def start(self) -> None:
        if self._thread is not None:
            return
        current = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        current.settimeout(0.25)
        try:
            current.bind(("127.0.0.1", self.port))
        except OSError as error:
            current.close()
            self.error = str(error)
            return
        self.local_port = current.getsockname()[1]
        self._socket = current
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._receive_loop,
            name="StrikeGatewayDashboardStatus",
            daemon=True,
        )
        self._thread.start()

    # 수신 스레드와 UDP 소켓을 종료한다.
    def close(self) -> None:
        self._stop.set()
        current = self._socket
        self._socket = None
        if current is not None:
            current.close()
        thread = self._thread
        self._thread = None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    # 최신 payload, 수신 나이와 최근 60초 수신량을 원자적으로 반환한다.
    def snapshot(self) -> tuple[dict[str, object] | None, float | None, int]:
        now = time.monotonic()
        with self._lock:
            while self._receipts and now - self._receipts[0] > 60.0:
                self._receipts.popleft()
            payload = None if self._payload is None else dict(self._payload)
            age = None if self._received_at is None else now - self._received_at
            return payload, age, len(self._receipts)

    # 올바른 flight-safety-state JSON만 최신 상태로 보존한다.
    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            current = self._socket
            if current is None:
                return
            try:
                data, _ = current.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                payload = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(payload, dict) or payload.get("type") != "flight-safety-state":
                continue
            now = time.monotonic()
            with self._lock:
                self._payload = payload
                self._received_at = now
                self._receipts.append(now)


class GatewayDashboardModel:
    def __init__(self, root: Path) -> None:
        self.root = root.resolve()
        self.log_root = self.root / ".team-runtime" / "logs"
        launcher = StrikerPCLauncher(self.root / "team-sources.json")
        self.module_names = {
            spec.key: spec.name for spec in launcher.build_processes()
        }

    # 현재 로그 세션에서 화면에 필요한 모듈·지표·콘솔 상태를 만든다.
    def snapshot(self) -> DashboardSnapshot:
        directory = find_latest_log_directory(self.log_root)
        if directory is None:
            return DashboardSnapshot(None, None, {}, ControlMetrics(), (), 0, 0)
        age = max(0.0, time.time() - latest_directory_mtime(directory))
        launcher_lines = read_tail_lines(directory / "launcher.log", 500)
        states = parse_launcher_module_states(
            launcher_lines,
            self.module_names,
            stale=age > LOG_ACTIVE_SECONDS,
        )
        pilot_paths = sorted(directory.glob("pilot-pc_*.log"))
        pilot_lines = read_tail_lines(pilot_paths[-1], 300) if pilot_paths else ()
        console_lines = collect_console_lines(directory)
        warnings = sum("WARNING" in line or "[WARN]" in line for line in console_lines)
        errors = sum(
            "ERROR" in line or "비정상 종료" in line or "Traceback" in line
            for line in console_lines
        )
        return DashboardSnapshot(
            directory,
            age,
            states,
            parse_control_metrics(pilot_lines),
            console_lines,
            warnings,
            errors,
        )


# 통합 GUI에서는 기존 런타임 감독 객체를 공유하고 단독 실행에서만 새로 만든다.
def resolve_runtime_controller(
    root: Path,
    on_status: Callable[[str], None],
    controller: RuntimeSupervisorController | None = None,
) -> RuntimeSupervisorController:
    if controller is not None:
        return controller
    from strikerpc_control_gui import RuntimeSupervisorController

    return RuntimeSupervisorController(
        root,
        root / "Gateway" / "wifi_runtime_autostart.json",
        on_status=on_status,
    )


class StrikeGatewayDashboardApp:
    def __init__(
        self,
        window: tk.Tk,
        model: GatewayDashboardModel,
        receiver: SafetyStatusReceiver,
        refresh_ms: int,
        runtime_controller: RuntimeSupervisorController | None = None,
        owns_runtime_lifecycle: bool = True,
    ) -> None:
        self.window = window
        self.model = model
        self.receiver = receiver
        self.refresh_ms = max(250, refresh_ms)
        self.runtime_events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.runtime = resolve_runtime_controller(
            model.root,
            on_status=lambda message: self.runtime_events.put(("status", message)),
            controller=runtime_controller,
        )
        self._owns_runtime_lifecycle = owns_runtime_lifecycle
        self._runtime_stopping = False
        self._close_after_stop = False
        self._closing = False
        self._last_safety_payload: dict[str, object] | None = None
        self._last_safety_age: float | None = None
        available_fonts = set(tkfont.families(window))
        self.ui_font = next(
            (
                name
                for name in ("Pretendard", "Noto Sans KR", "Malgun Gothic", "Segoe UI")
                if name in available_fonts
            ),
            "TkDefaultFont",
        )
        self.mono_font = next(
            (
                name
                for name in ("IBM Plex Mono", "Cascadia Mono", "Consolas")
                if name in available_fonts
            ),
            "TkFixedFont",
        )
        self.status_labels: dict[str, tuple[tk.Label, tk.Label]] = {}
        self.metric_values: dict[str, tk.StringVar] = {}
        self.safety_values: dict[str, tk.StringVar] = {}
        self.header_status = tk.StringVar(value="초기화 중")
        self.updated_at = tk.StringVar(value="--:--:--")
        self.log_path = tk.StringVar(value="최근 로그 세션 없음")
        self.runtime_status = tk.StringVar(value="중지됨")
        self.always_on_top = tk.BooleanVar(value=False)
        self._configure_window()
        self._build_ui()
        self.window.after(50, self._refresh)

    # 참고 이미지 비율에 맞춘 어두운 모니터 창을 설정한다.
    def _configure_window(self) -> None:
        self.window.title("타격 게이트웨이")
        self.window.geometry("780x900")
        self.window.minsize(700, 720)
        self.window.configure(bg=COLORS["window"])
        self.window.protocol("WM_DELETE_WINDOW", self._on_close)

    # 연결·건강·지표·안전·콘솔 패널을 위에서 아래로 배치한다.
    def _build_ui(self) -> None:
        outer = tk.Frame(self.window, bg=COLORS["window"], padx=12, pady=10)
        outer.pack(fill="both", expand=True)

        header = tk.Frame(outer, bg=COLORS["window"])
        header.pack(fill="x", pady=(0, 8))
        tk.Label(
            header,
            text="타격 게이트웨이",
            bg=COLORS["window"],
            fg=COLORS["text"],
            font=(self.ui_font, 18, "bold"),
        ).pack(side="left")
        tk.Checkbutton(
            header,
            text="항상 위",
            variable=self.always_on_top,
            command=self._toggle_topmost,
            bg=COLORS["window"],
            fg=COLORS["muted"],
            activebackground=COLORS["window"],
            activeforeground=COLORS["text"],
            selectcolor=COLORS["panel"],
            font=(self.ui_font, 8, "bold"),
        ).pack(side="right", padx=(12, 0))
        tk.Label(
            header,
            textvariable=self.updated_at,
            bg=COLORS["window"],
            fg=COLORS["muted"],
            font=(self.mono_font, 9),
        ).pack(side="right", padx=(10, 0))
        tk.Label(
            header,
            textvariable=self.header_status,
            bg=COLORS["window"],
            fg=COLORS["cyan"],
            font=(self.ui_font, 9, "bold"),
        ).pack(side="right")

        runtime_bar = tk.Frame(
            outer,
            bg=COLORS["panel"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
            padx=10,
            pady=7,
        )
        runtime_bar.pack(fill="x", pady=(0, 8))
        tk.Label(
            runtime_bar,
            text="런타임",
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            font=(self.ui_font, 8, "bold"),
        ).pack(side="left")
        tk.Label(
            runtime_bar,
            textvariable=self.runtime_status,
            bg=COLORS["panel"],
            fg=COLORS["cyan"],
            font=(self.ui_font, 8, "bold"),
        ).pack(side="left", padx=(10, 0))
        self.stop_button = tk.Button(
            runtime_bar,
            text="안전 종료",
            command=self._request_runtime_stop,
            bg=COLORS["panel_alt"],
            fg=COLORS["amber"],
            activebackground=COLORS["border"],
            activeforeground=COLORS["text"],
            disabledforeground=COLORS["border_bright"],
            relief="flat",
            borderwidth=0,
            padx=12,
            pady=5,
            font=(self.ui_font, 8, "bold"),
        )
        self.stop_button.pack(side="right")
        self.start_button = tk.Button(
            runtime_bar,
            text="런타임 시작",
            command=self._start_runtime,
            bg=COLORS["cyan"],
            fg=COLORS["window"],
            activebackground=COLORS["green"],
            activeforeground=COLORS["window"],
            disabledforeground=COLORS["border_bright"],
            relief="flat",
            borderwidth=0,
            padx=12,
            pady=5,
            font=(self.ui_font, 8, "bold"),
        )
        self.start_button.pack(side="right", padx=(0, 6))

        self._build_status_panel(
            outer,
            "연결 상태",
            (
                ("gateway", "게이트웨이"),
                ("xr", "XR 제어"),
                ("tello", "텔로 링크"),
                ("vision", "비전"),
                ("command-center", "지휘통제"),
            ),
        )
        self._build_status_panel(
            outer,
            "모듈 상태",
            (
                ("launcher", "런처"),
                ("pilot", "파일럿"),
                ("video", "영상 송출"),
                ("telemetry", "텔레메트리"),
                ("safety", "안전 계층"),
            ),
        )

        middle = tk.Frame(outer, bg=COLORS["window"])
        middle.pack(fill="x", pady=(0, 8))
        middle.columnconfigure(0, weight=1, uniform="middle")
        middle.columnconfigure(1, weight=1, uniform="middle")
        metrics_panel = self._panel(middle, "제어 활동")
        metrics_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        safety_panel = self._panel(middle, "안전 상태")
        safety_panel.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        for key, label in (
            ("sent", "송신 틱"),
            ("failed", "실패"),
            ("missed", "마감 누락"),
            ("source", "입력 소스"),
            ("status_rx", "상태 수신 / 60초"),
        ):
            self._metric_row(metrics_panel, key, label)
        for key, label in (
            ("state", "상태"),
            ("control", "제어 링크"),
            ("heartbeat", "유니티 하트비트"),
            ("neutral", "강제 중립"),
            ("error", "최근 오류"),
        ):
            self._safety_row(safety_panel, key, label)

        console_panel = self._panel(outer, "콘솔 로그")
        console_panel.pack(fill="both", expand=True)
        path_label = tk.Label(
            console_panel,
            textvariable=self.log_path,
            anchor="w",
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            font=(self.mono_font, 8),
        )
        path_label.pack(fill="x", padx=10, pady=(0, 6))
        text_frame = tk.Frame(console_panel, bg=COLORS["panel"])
        text_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.console = tk.Text(
            text_frame,
            bg="#07090b",
            fg="#93a1ae",
            insertbackground=COLORS["text"],
            relief="flat",
            wrap="none",
            font=(self.mono_font, 8),
            padx=8,
            pady=8,
            state="disabled",
        )
        scrollbar = ttk.Scrollbar(text_frame, orient="vertical", command=self.console.yview)
        self.console.configure(yscrollcommand=scrollbar.set)
        self.console.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        self.console.tag_configure("warning", foreground=COLORS["amber"])
        self.console.tag_configure("error", foreground=COLORS["red"])

    # 공통 테두리와 제목을 가진 패널을 만든다.
    def _panel(self, parent: tk.Misc, title: str) -> tk.Frame:
        frame = tk.Frame(
            parent,
            bg=COLORS["panel"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
        )
        tk.Label(
            frame,
            text=title,
            bg=COLORS["panel_alt"],
            fg=COLORS["text"],
            anchor="w",
            padx=10,
            pady=6,
            font=(self.ui_font, 8, "bold"),
        ).pack(fill="x")
        return frame

    # 동일 너비 상태 셀을 한 줄 패널로 만든다.
    def _build_status_panel(
        self,
        parent: tk.Misc,
        title: str,
        entries: tuple[tuple[str, str], ...],
    ) -> None:
        panel = self._panel(parent, title)
        panel.pack(fill="x", pady=(0, 8))
        row = tk.Frame(panel, bg=COLORS["panel"])
        row.pack(fill="x", padx=5, pady=9)
        for index, (key, label) in enumerate(entries):
            row.columnconfigure(index, weight=1, uniform=title)
            cell = tk.Frame(row, bg=COLORS["panel"])
            cell.grid(row=0, column=index, sticky="nsew", padx=5)
            tk.Label(
                cell,
                text=label,
                bg=COLORS["panel"],
                fg=COLORS["muted"],
                font=(self.ui_font, 7, "bold"),
            ).pack(anchor="w")
            value_row = tk.Frame(cell, bg=COLORS["panel"])
            value_row.pack(fill="x", pady=(3, 0))
            dot = tk.Label(
                value_row,
                text="●",
                bg=COLORS["panel"],
                fg=COLORS["muted"],
                font=(self.ui_font, 8),
            )
            dot.pack(side="left")
            value = tk.Label(
                value_row,
                text="대기",
                bg=COLORS["panel"],
                fg=COLORS["muted"],
                font=(self.ui_font, 8, "bold"),
            )
            value.pack(side="left", padx=(3, 0))
            self.status_labels[key] = (dot, value)

    # 제어 지표의 라벨과 값을 한 행으로 배치한다.
    def _metric_row(self, parent: tk.Misc, key: str, label: str) -> None:
        variable = tk.StringVar(value="--")
        self.metric_values[key] = variable
        row = tk.Frame(parent, bg=COLORS["panel"])
        row.pack(fill="x", padx=10, pady=4)
        tk.Label(
            row,
            text=label,
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            font=(self.ui_font, 8),
        ).pack(side="left")
        tk.Label(
            row,
            textvariable=variable,
            bg=COLORS["panel"],
            fg=COLORS["cyan"],
            font=(self.mono_font, 9, "bold"),
        ).pack(side="right")

    # 안전 지표의 라벨과 값을 한 행으로 배치한다.
    def _safety_row(self, parent: tk.Misc, key: str, label: str) -> None:
        variable = tk.StringVar(value="--")
        self.safety_values[key] = variable
        row = tk.Frame(parent, bg=COLORS["panel"])
        row.pack(fill="x", padx=10, pady=4)
        tk.Label(
            row,
            text=label,
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            font=(self.ui_font, 8),
        ).pack(side="left")
        tk.Label(
            row,
            textvariable=variable,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=(self.mono_font, 8, "bold"),
        ).pack(side="right")

    # 로그와 UDP 안전 상태를 주기적으로 화면에 반영한다.
    def _refresh(self) -> None:
        if self._closing:
            return
        try:
            self._pump_runtime_events()
            snapshot = self.model.snapshot()
            payload, safety_age, status_rx = self.receiver.snapshot()
            self._render_snapshot(snapshot, payload, safety_age, status_rx)
        except Exception as error:
            self.header_status.set(f"모니터 오류: {error}")
        if not self._closing:
            self.window.after(self.refresh_ms, self._refresh)

    # 한 시점의 모델을 모든 패널에 일관되게 표시한다.
    def _render_snapshot(
        self,
        snapshot: DashboardSnapshot,
        payload: dict[str, object] | None,
        safety_age: float | None,
        status_rx: int,
    ) -> None:
        active = snapshot.log_age_seconds is not None and snapshot.log_age_seconds <= LOG_ACTIVE_SECONDS
        self._last_safety_payload = payload
        self._last_safety_age = safety_age
        self.header_status.set("감시 중" if active else "대기")
        self.updated_at.set(datetime.now().strftime("%H:%M:%S"))
        self.log_path.set(
            "최근 로그 세션 없음"
            if snapshot.log_directory is None
            else f"최근 로그 · {snapshot.log_directory}"
        )
        module_states = snapshot.module_states
        self._set_status("gateway", module_states.get("pilot", "OFFLINE"))
        self._set_status("xr", self._control_connection_state(payload, safety_age, module_states))
        self._set_status("tello", self._tello_connection_state(payload, safety_age))
        self._set_status("vision", module_states.get("vision", "OFFLINE"))
        self._set_status("command-center", module_states.get("command-center", "OFFLINE"))
        self._set_status("launcher", "HEALTHY" if active else "OFFLINE")
        self._set_status("pilot", module_states.get("pilot", "OFFLINE"))
        self._set_status("video", module_states.get("livekit-video", "OFFLINE"))
        self._set_status("telemetry", "HEALTHY" if safety_age is not None and safety_age <= SAFETY_FRESH_SECONDS else "STALE")
        safety_label, safety_tone = classify_safety_status(payload, safety_age)
        self._set_status("safety", safety_label, safety_tone)

        metrics = snapshot.metrics
        self.metric_values["sent"].set(f"{metrics.sent:,}")
        self.metric_values["failed"].set(f"{metrics.failed:,}")
        self.metric_values["missed"].set(f"{metrics.missed:,}")
        self.metric_values["source"].set(
            {
                "desktop": "데스크톱",
                "neutral": "중립",
                "reconnecting": "재연결 중",
                "xr": "XR",
            }.get(metrics.source.lower(), metrics.source)
        )
        self.metric_values["status_rx"].set(str(status_rx))
        self.safety_values["state"].set(STATUS_TEXT_KO.get(safety_label, safety_label))
        self.safety_values["control"].set(self._boolean_label(payload, "controlConnected"))
        self.safety_values["heartbeat"].set(self._boolean_label(payload, "heartbeatFresh"))
        self.safety_values["neutral"].set(self._boolean_label(payload, "forceNeutral", inverse=True))
        self.safety_values["error"].set(str(payload.get("lastError") or "없음") if payload else "--")
        self._set_console(snapshot.console_lines)
        self._refresh_runtime_controls(active)

    # 저장된 SSID와 모듈 설정으로 기존 런타임 감시기를 시작한다.
    def _start_runtime(self) -> None:
        if self._runtime_stopping or self.runtime.active:
            return
        snapshot = self.model.snapshot()
        external_active = bool(
            snapshot.log_age_seconds is not None
            and snapshot.log_age_seconds <= LOG_ACTIVE_SECONDS
        )
        if external_active:
            messagebox.showinfo(
                "런타임 실행 중",
                "다른 런처에서 실행 중인 런타임이 감지되었습니다.",
                parent=self.window,
            )
            return
        try:
            self.runtime.start()
            self.runtime_status.set("Wi-Fi 연결 확인 중")
        except Exception as error:
            messagebox.showerror("런타임 시작 실패", str(error), parent=self.window)
        self._refresh_runtime_controls(False)

    # 안전 상태를 확인한 뒤 대시보드가 시작한 런타임 종료를 요청한다.
    def _request_runtime_stop(self) -> None:
        if not self.runtime.active or self._runtime_stopping:
            return
        if runtime_stop_confirmation_required(
            self._last_safety_payload,
            self._last_safety_age,
        ):
            if not messagebox.askyesno(
                "안전 종료 확인",
                (
                    "안전 종료는 착륙 명령이 아닙니다.\n"
                    "기체가 비행 중이면 먼저 착륙을 완료하세요.\n\n"
                    "중립 RC 전송 후 런타임을 종료하시겠습니까?"
                ),
                parent=self.window,
            ):
                return
        self._begin_runtime_stop(close_after=False)

    # stop-file 기반 안전 종료를 작업 스레드에서 실행한다.
    def _begin_runtime_stop(self, *, close_after: bool) -> None:
        if self._runtime_stopping:
            self._close_after_stop = self._close_after_stop or close_after
            return
        self._runtime_stopping = True
        self._close_after_stop = close_after
        self.runtime_status.set("안전 종료 중")
        self._refresh_runtime_controls(False)

        def stop_worker() -> None:
            try:
                self.runtime.stop()
                result: object = None
            except Exception as error:
                result = error
            self.runtime_events.put(("stopped", result))

        threading.Thread(
            target=stop_worker,
            name="StrikeGatewayDashboardSafeStop",
            daemon=False,
        ).start()

    # 런타임 작업 스레드의 상태를 Tk 메인 스레드에 반영한다.
    def _pump_runtime_events(self) -> None:
        while True:
            try:
                event, value = self.runtime_events.get_nowait()
            except queue.Empty:
                return
            if event == "status":
                self.runtime_status.set(str(value))
                continue
            if event != "stopped":
                continue
            self._runtime_stopping = False
            if isinstance(value, Exception):
                self._closing = False
                self._close_after_stop = False
                self.runtime_status.set(f"종료 오류: {value}")
                messagebox.showerror("런타임 종료 실패", str(value), parent=self.window)
                continue
            self.runtime_status.set("중지됨")
            if self._close_after_stop:
                self.close_monitor()
                return

    # 내부 또는 외부 런타임 상태에 맞춰 시작·종료 버튼을 갱신한다.
    def _refresh_runtime_controls(self, external_active: bool) -> None:
        if self._runtime_stopping:
            self.start_button.configure(state="disabled", bg=COLORS["panel_alt"])
            self.stop_button.configure(state="disabled")
            return
        if self.runtime.active:
            self.runtime_status.set(self.runtime.status())
            self.start_button.configure(state="disabled", bg=COLORS["panel_alt"])
            self.stop_button.configure(state="normal")
            return
        if external_active:
            self.runtime_status.set("외부 런타임 감지")
            self.start_button.configure(state="disabled", bg=COLORS["panel_alt"])
            self.stop_button.configure(state="disabled")
            return
        current_status = self.runtime.status()
        self.runtime_status.set(current_status)
        self.start_button.configure(state="normal", bg=COLORS["cyan"])
        self.stop_button.configure(state="disabled")

    # 연결 의미에 맞는 색상으로 상태 셀을 갱신한다.
    def _set_status(self, key: str, value: str, tone: str | None = None) -> None:
        widgets = self.status_labels.get(key)
        if widgets is None:
            return
        normalized = value.upper()
        if tone is None:
            if normalized in {"CONNECTED", "HEALTHY", "SAFE"}:
                tone = "green"
            elif normalized in {"ERROR", "ALERT", "NEUTRAL LOCK"}:
                tone = "red"
            elif normalized in {"STARTING", "STALE", "WAITING", "STANDBY"}:
                tone = "amber"
            else:
                tone = "muted"
        color = COLORS[tone]
        widgets[0].configure(fg=color)
        widgets[1].configure(text=STATUS_TEXT_KO.get(normalized, normalized), fg=color)

    # 안전 payload에서 XR 제어 연결 상태를 계산한다.
    @staticmethod
    def _control_connection_state(
        payload: dict[str, object] | None,
        age: float | None,
        module_states: dict[str, str],
    ) -> str:
        if module_states.get("xr") not in {"CONNECTED", "STARTING"}:
            return module_states.get("xr", "OFFLINE")
        if payload is None or age is None or age > SAFETY_FRESH_SECONDS:
            return "STALE"
        return "CONNECTED" if payload.get("controlConnected") else "WAITING"

    # Tello state 패킷 나이로 기체 링크 상태를 계산한다.
    @staticmethod
    def _tello_connection_state(
        payload: dict[str, object] | None,
        age: float | None,
    ) -> str:
        if payload is None or age is None or age > SAFETY_FRESH_SECONDS:
            return "OFFLINE"
        state_age = payload.get("stateAgeMs")
        if isinstance(state_age, (int, float)) and state_age <= 1500:
            return "CONNECTED"
        return "WAITING"

    # bool 안전 필드를 사람이 읽을 수 있는 짧은 값으로 변환한다.
    @staticmethod
    def _boolean_label(
        payload: dict[str, object] | None,
        key: str,
        *,
        inverse: bool = False,
    ) -> str:
        if payload is None:
            return "--"
        value = bool(payload.get(key))
        if inverse:
            return "해제" if not value else "활성"
        return "정상" if value else "대기"

    # 새 로그를 색상 태그와 함께 콘솔에 표시한다.
    def _set_console(self, lines: tuple[str, ...]) -> None:
        at_bottom = self.console.yview()[1] >= 0.98
        self.console.configure(state="normal")
        self.console.delete("1.0", "end")
        if not lines:
            self.console.insert("end", "런처 로그를 기다리는 중...\n")
        for line in lines:
            tag = "error" if "ERROR" in line or "비정상 종료" in line else "warning" if "WARNING" in line or "[WARN]" in line else None
            self.console.insert("end", f"{line}\n", tag)
        self.console.configure(state="disabled")
        if at_bottom:
            self.console.see("end")

    # 항상 위 설정을 즉시 Windows 창 속성에 반영한다.
    def _toggle_topmost(self) -> None:
        self.window.attributes("-topmost", bool(self.always_on_top.get()))

    # 대시보드가 시작한 런타임을 안전 종료한 뒤 창을 닫는다.
    def _on_close(self) -> None:
        if self._closing:
            return
        if self._owns_runtime_lifecycle and self.runtime.active:
            if not messagebox.askyesno(
                "창 닫기 확인",
                (
                    "대시보드가 시작한 런타임을 안전 종료한 뒤 창을 닫습니다.\n"
                    "이 동작은 착륙 명령이 아닙니다. 계속하시겠습니까?"
                ),
                parent=self.window,
            ):
                return
            self._closing = True
            self._begin_runtime_stop(close_after=True)
            return
        self.close_monitor()

    # 통합 GUI 종료 시 공유 런타임은 유지하고 상태 수신기와 창만 닫는다.
    def close_monitor(self) -> None:
        self._closing = True
        self.receiver.close()
        try:
            if self.window.winfo_exists():
                self.window.destroy()
        except tk.TclError:
            pass


# 환경 변수와 파일 위치에서 StrikerPC 루트를 찾는다.
def find_default_root() -> Path:
    configured = os.environ.get("STRIKERPC_ROOT", "").strip()
    if configured:
        return Path(configured).resolve()
    candidates = [Path.cwd()]
    for source in (Path(__file__).resolve(), Path(sys.executable).resolve()):
        candidates.extend(source.parents[:6])
    seen: set[Path] = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if (resolved / "team-sources.json").is_file() and (resolved / "Gateway").is_dir():
            return resolved
    return Path(__file__).resolve().parent.parent


# manifest의 대시보드 전용 상태 포트를 읽는다.
def load_dashboard_port(root: Path) -> int:
    manifest = json.loads((root / "team-sources.json").read_text(encoding="utf-8"))
    value = manifest.get("ports", {}).get("gatewayDashboard", DEFAULT_DASHBOARD_STATE_PORT)
    port = int(value)
    if port <= 0 or port > 65535:
        raise ValueError("gatewayDashboard 포트는 1~65535 범위여야 합니다.")
    return port


# CLI 인자를 파싱해 GUI 또는 점검 모드로 실행한다.
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Strike Gateway 상태 대시보드")
    parser.add_argument("--root", default=str(find_default_root()))
    parser.add_argument("--state-port", type=int)
    parser.add_argument("--refresh-ms", type=int, default=1000)
    parser.add_argument("--check", action="store_true")
    return parser.parse_args(argv)


# 설정 점검 또는 Tk 대시보드 실행을 시작한다.
def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    root = Path(args.root).resolve()
    if not (root / "team-sources.json").exists():
        raise FileNotFoundError(f"StrikerPC manifest를 찾을 수 없습니다: {root}")
    port = args.state_port if args.state_port is not None else load_dashboard_port(root)
    model = GatewayDashboardModel(root)
    if args.check:
        snapshot = model.snapshot()
        print(
            json.dumps(
                {
                    "root": str(root),
                    "statePort": port,
                    "logDirectory": None if snapshot.log_directory is None else str(snapshot.log_directory),
                    "moduleStates": snapshot.module_states,
                    "metrics": snapshot.metrics.__dict__,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0
    receiver = SafetyStatusReceiver(port)
    receiver.start()
    window = tk.Tk()
    StrikeGatewayDashboardApp(window, model, receiver, args.refresh_ms)
    window.mainloop()
    receiver.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
