from __future__ import annotations

import argparse
import ctypes
import json
import os
import queue
import socket
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, replace
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable

from control_protocol import (
    ACK_PACKET,
    ACK_STATUS_DISPATCHED,
    ACK_STATUS_OK,
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
    COMMAND_LAND,
    COMMAND_NEUTRAL,
    COMMAND_RC,
    COMMAND_TAKEOFF,
    CONTROL_MAGIC,
    CONTROL_VERSION,
    DESKTOP_CONTROL_FLAG,
    ControlPacket,
    encode_control_packet,
)
from desktop_telemetry import (
    FLIGHT_AIRBORNE,
    FLIGHT_GROUNDED,
    FLIGHT_LANDING,
    FLIGHT_UNKNOWN,
    DesktopTelemetryMonitor,
    DesktopTelemetryStatus,
)
from strikerpc_launcher import ModuleSelectionEntry, StrikerPCLauncher
from wifi_runtime_autostart import (
    WifiRuntimeSupervisor,
    load_config,
    read_windows_connected_ssids,
    runtime_paths,
)


DEFAULT_DESKTOP_CONTROL_PORT = 49211
DEFAULT_DESKTOP_STATE_PORT = 8893
DEFAULT_RC_SPEED = 40
MUTEX_NAME = "Local\\StrikerPCDesktopControl"
GUI_STOP_NAME = "desktop-control-gui.stop"
KEYBOARD_AXIS_DIRECTIONS = {
    "w": (0, 1, 0, 0),
    "s": (0, -1, 0, 0),
    "a": (-1, 0, 0, 0),
    "d": (1, 0, 0, 0),
    "r": (0, 0, 1, 0),
    "f": (0, 0, -1, 0),
    "q": (0, 0, 0, -1),
    "e": (0, 0, 0, 1),
}
KEYBOARD_COMMAND_KEYS = frozenset(("t", "l", "space"))
ACK_NAMES = {
    COMMAND_HELLO: "연결",
    COMMAND_TAKEOFF: "이륙",
    COMMAND_LAND: "착륙",
    COMMAND_EMERGENCY: "비상 정지",
}


# 활성 비행 세션이 확실히 착륙한 경우에만 종료 확인을 생략한다.
def shutdown_confirmation_required(
    *,
    runtime_active: bool,
    control_connected: bool,
    command_pending: bool,
    command_outcome_unknown: bool,
    emergency_latched: bool,
    telemetry_connected: bool,
    confirmed_grounded: bool,
) -> bool:
    active_or_uncertain = (
        runtime_active
        or control_connected
        or command_pending
        or command_outcome_unknown
        or emergency_latched
        or telemetry_connected
    )
    if not active_or_uncertain:
        return False
    return (
        not confirmed_grounded
        or command_pending
        or command_outcome_unknown
        or emergency_latched
    )


@dataclass(frozen=True)
class DroneProfile:
    name: str
    ssid: str

    # 프로필 문자열을 정리하고 빈 값과 제어 문자를 거부한다.
    def __post_init__(self) -> None:
        name = self.name.strip()
        ssid = self.ssid.strip()
        if not name:
            raise ValueError("드론 이름은 비어 있을 수 없습니다.")
        if not ssid:
            raise ValueError("SSID는 비어 있을 수 없습니다.")
        if any(character in name + ssid for character in ("\r", "\n", "\0")):
            raise ValueError("드론 이름과 SSID에는 제어 문자를 사용할 수 없습니다.")
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "ssid", ssid)


@dataclass(frozen=True)
class ControlAck:
    command: int
    sequence: int
    success: bool
    status: int
    message: str


@dataclass
class _PendingCommand:
    command: int
    sequence: int
    payload: bytes
    deadline: float
    next_retry_at: float | None
    callback: Callable[[ControlAck], None] | None


# 동시에 누른 키의 방향을 합쳐 대각선 입력을 포함한 RC 축으로 변환한다.
def keyboard_axes(
    pressed_keys: set[str] | frozenset[str],
    speed: int,
) -> tuple[int, int, int, int]:
    normalized_speed = max(10, min(100, int(speed)))
    totals = [0, 0, 0, 0]
    for key in pressed_keys:
        direction = KEYBOARD_AXIS_DIRECTIONS.get(key.lower())
        if direction is None:
            continue
        for index, component in enumerate(direction):
            totals[index] += component
    return tuple(
        max(-1, min(1, component)) * normalized_speed
        for component in totals
    )  # type: ignore[return-value]


# 기존 단일 SSID와 새 다중 드론 목록을 함께 읽는다.
def load_drone_profiles(config_path: Path) -> tuple[DroneProfile, ...]:
    path = Path(config_path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"드론 설정을 읽을 수 없습니다: {error}") from error
    if not isinstance(data, dict):
        raise ValueError("드론 설정 루트는 JSON 객체여야 합니다.")

    profiles: list[DroneProfile] = []
    raw_profiles = data.get("drones")
    if raw_profiles is not None:
        if not isinstance(raw_profiles, list):
            raise ValueError("drones는 드론 객체 배열이어야 합니다.")
        for index, item in enumerate(raw_profiles, start=1):
            if not isinstance(item, dict):
                raise ValueError(f"drones[{index}]는 객체여야 합니다.")
            try:
                name = item["name"]
                ssid = item["ssid"]
            except KeyError as error:
                raise ValueError(
                    f"drones[{index}]에 name과 ssid가 필요합니다."
                ) from error
            if not isinstance(name, str) or not isinstance(ssid, str):
                raise ValueError(f"drones[{index}]의 name과 ssid는 문자열이어야 합니다.")
            profile = DroneProfile(name, ssid)
            if any(
                current.name == profile.name or current.ssid == profile.ssid
                for current in profiles
            ):
                raise ValueError("드론 이름과 SSID는 각각 중복될 수 없습니다.")
            profiles.append(profile)

    legacy_ssid = data.get("ssid")
    if isinstance(legacy_ssid, str) and legacy_ssid.strip():
        normalized_ssid = legacy_ssid.strip()
        if not any(profile.ssid == normalized_ssid for profile in profiles):
            legacy_name = "기본 Tello"
            suffix = 2
            while any(profile.name == legacy_name for profile in profiles):
                legacy_name = f"기본 Tello {suffix}"
                suffix += 1
            profiles.insert(0, DroneProfile(legacy_name, normalized_ssid))

    if not profiles:
        raise ValueError("하나 이상의 ssid 또는 drones 프로필이 필요합니다.")
    return tuple(profiles)


# legacy ssid가 가리키는 현재 선택 프로필을 찾는다.
def select_config_profile(
    config_path: Path,
    profiles: tuple[DroneProfile, ...],
) -> DroneProfile:
    if not profiles:
        raise ValueError("선택할 드론 프로필이 없습니다.")
    try:
        data = json.loads(Path(config_path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"드론 설정을 읽을 수 없습니다: {error}") from error
    selected_ssid = data.get("ssid") if isinstance(data, dict) else None
    if isinstance(selected_ssid, str):
        normalized = selected_ssid.strip()
        match = next(
            (profile for profile in profiles if profile.ssid == normalized),
            None,
        )
        if match is not None:
            return match
    return profiles[0]


# 프로필을 추가하거나 갱신하고 기존 런타임 설정 필드를 보존한다.
def save_drone_profile(config_path: Path, profile: DroneProfile) -> tuple[DroneProfile, ...]:
    path = Path(config_path)
    if path.exists():
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"드론 설정을 읽을 수 없습니다: {error}") from error
        if not isinstance(data, dict):
            raise ValueError("드론 설정 루트는 JSON 객체여야 합니다.")
        try:
            existing = list(load_drone_profiles(path))
        except ValueError:
            has_profiles = data.get("drones") is not None
            has_legacy_ssid = bool(
                isinstance(data.get("ssid"), str) and data["ssid"].strip()
            )
            if has_profiles or has_legacy_ssid:
                raise
            existing = []
    else:
        data = {}
        existing = []

    name_match = next(
        (current for current in existing if current.name == profile.name),
        None,
    )
    ssid_match = next(
        (current for current in existing if current.ssid == profile.ssid),
        None,
    )
    if name_match is not None and ssid_match is not None and name_match != ssid_match:
        raise ValueError("다른 두 프로필의 이름과 SSID를 동시에 사용할 수 없습니다.")

    updated: list[DroneProfile] = []
    replaced = False
    for current in existing:
        if current.name == profile.name or current.ssid == profile.ssid:
            if not replaced:
                updated.append(profile)
                replaced = True
            continue
        updated.append(current)
    if not replaced:
        updated.append(profile)

    data["ssid"] = profile.ssid
    data["drones"] = [
        {"name": current.name, "ssid": current.ssid} for current in updated
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return tuple(updated)


# 체크박스로 선택한 모듈을 다른 자동 실행 설정을 보존한 채 저장한다.
def save_module_selection(config_path: Path, modules: tuple[str, ...]) -> None:
    normalized = tuple(dict.fromkeys(value.strip().lower() for value in modules))
    if not normalized or any(
        not value or any(character in value for character in ("\r", "\n", "\0"))
        for value in normalized
    ):
        raise ValueError("모듈을 하나 이상 올바르게 선택해야 합니다.")

    path = Path(config_path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"런타임 설정을 읽을 수 없습니다: {error}") from error
    if not isinstance(data, dict):
        raise ValueError("런타임 설정 루트는 JSON 객체여야 합니다.")

    data["modules"] = list(normalized)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


class DesktopControlClient:
    def __init__(
        self,
        target: tuple[str, int] = ("127.0.0.1", DEFAULT_DESKTOP_CONTROL_PORT),
        *,
        ack_timeout: float = 8.0,
        hello_retry_seconds: float = 0.25,
        rc_interval: float = 0.08,
        socket_factory: Callable[..., socket.socket] = socket.socket,
        monotonic: Callable[[], float] = time.monotonic,
        on_ack: Callable[[ControlAck], None] | None = None,
        on_status: Callable[[str], None] | None = None,
    ) -> None:
        self.target = target
        self.ack_timeout = ack_timeout
        self.hello_retry_seconds = hello_retry_seconds
        self.rc_interval = rc_interval
        self.socket_factory = socket_factory
        self.monotonic = monotonic
        self.on_ack = on_ack
        self.on_status = on_status
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._sequence = 0
        self._pending: _PendingCommand | None = None
        self._movement_axes: tuple[int, int, int, int] | None = None
        self._next_rc_at = 0.0
        self._connected = False
        self._closed = False
        self._emergency_latched = False
        self._command_outcome_unknown = False
        self._clear_latch_on_hello = False
        self._clear_unknown_on_hello = False

    @property
    # HELLO ACK로 확인한 현재 제어 연결 상태를 반환한다.
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    # 비상 명령 이후 수동 제어 잠금 상태를 반환한다.
    def emergency_latched(self) -> bool:
        with self._lock:
            return self._emergency_latched

    @property
    # 확인 명령 ACK를 현재 기다리고 있는지 반환한다.
    def command_pending(self) -> bool:
        with self._lock:
            return self._pending is not None

    @property
    # 확인 명령 결과가 불명이라 명시적 재연결이 필요한지 반환한다.
    def command_outcome_unknown(self) -> bool:
        with self._lock:
            return self._command_outcome_unknown

    # UDP 소켓과 ACK 수신 스레드를 시작한다.
    def start(self) -> None:
        with self._lock:
            if self._closed:
                raise RuntimeError("닫힌 제어 클라이언트는 다시 시작할 수 없습니다.")
            if self._thread is not None and self._thread.is_alive():
                return
            self._socket = self.socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.bind(("127.0.0.1", 0))
            self._socket.settimeout(0.02)
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run,
                name="StrikerPCDesktopControl",
                daemon=True,
            )
            self._thread.start()

    # 최초 HELLO를 보내고 실제 ACK를 비동기로 기다린다.
    def connect(self, callback: Callable[[ControlAck], None] | None = None) -> int:
        with self._lock:
            self._clear_latch_on_hello = False
            self._clear_unknown_on_hello = False
        return self._begin_confirmed(COMMAND_HELLO, callback, require_connection=False)

    # 명시적 HELLO 재연결 성공 시에만 비상 잠금을 해제한다.
    def reconnect(self, callback: Callable[[ControlAck], None] | None = None) -> int:
        with self._lock:
            if self._pending is not None:
                raise RuntimeError("이전 확인 명령의 ACK를 기다리는 중입니다.")
        self.stop_movement()
        with self._lock:
            self._connected = False
            self._clear_latch_on_hello = True
            self._clear_unknown_on_hello = True
        return self._begin_confirmed(COMMAND_HELLO, callback, require_connection=False)

    # 현재 축을 즉시 전송하고 버튼을 누르는 동안 반복한다.
    def start_movement(self, axes: tuple[int, int, int, int]) -> int:
        normalized = self._normalize_axes(axes)
        with self._lock:
            self._require_manual_control_locked()
            self._movement_axes = normalized
            self._next_rc_at = self.monotonic() + self.rc_interval
        return self._send_unconfirmed(COMMAND_RC, normalized)

    # 반복 이동을 멈추고 즉시 중립 RC를 전송한다.
    def stop_movement(self) -> int | None:
        with self._lock:
            had_movement = self._movement_axes is not None
            self._movement_axes = None
        if had_movement:
            return self.send_neutral()
        return None

    # 현재 수동 RC 축을 한 번 전송한다.
    def send_rc(self, axes: tuple[int, int, int, int]) -> int:
        normalized = self._normalize_axes(axes)
        with self._lock:
            self._require_manual_control_locked()
        return self._send_unconfirmed(COMMAND_RC, normalized)

    # 연결과 비상 잠금 여부와 무관하게 중립 RC를 전송한다.
    def send_neutral(self) -> int | None:
        with self._lock:
            if self._socket is None or self._closed:
                return None
        return self._send_unconfirmed(COMMAND_NEUTRAL, (0, 0, 0, 0))

    # 이륙 명령을 보내고 실제 실행 ACK를 비동기로 기다린다.
    def send_takeoff(
        self,
        callback: Callable[[ControlAck], None] | None = None,
    ) -> int:
        with self._lock:
            self._require_manual_control_locked()
        return self._begin_confirmed(COMMAND_TAKEOFF, callback)

    # 착륙 명령은 비상 잠금과 분리해 계속 사용할 수 있게 한다.
    def send_land(
        self,
        callback: Callable[[ControlAck], None] | None = None,
    ) -> int:
        return self._begin_confirmed(COMMAND_LAND, callback)

    # 비상 명령 전송 즉시 RC와 이륙을 잠그고 ACK를 기다린다.
    def send_emergency(
        self,
        callback: Callable[[ControlAck], None] | None = None,
    ) -> int:
        preempted: _PendingCommand | None = None
        with self._lock:
            if not self._connected:
                raise RuntimeError("제어 연결 ACK를 먼저 확인하세요.")
            if self._pending is not None:
                if self._pending.command == COMMAND_EMERGENCY:
                    raise RuntimeError("비상 정지 ACK를 기다리는 중입니다.")
                preempted = self._pending
                self._pending = None
            self._emergency_latched = True
            self._movement_axes = None
        if preempted is not None:
            self._deliver_ack(
                preempted,
                ControlAck(
                    command=preempted.command,
                    sequence=preempted.sequence,
                    success=False,
                    status=2,
                    message="비상 정지로 이전 확인 명령을 취소했습니다.",
                ),
            )
        try:
            self.send_neutral()
        except (OSError, RuntimeError):
            self._notify_status("비상 정지 전 중립 RC 전송에 실패했습니다.")
        self._notify_status("비상 정지 전송: 수동 제어가 잠겼습니다.")
        return self._begin_confirmed(COMMAND_EMERGENCY, callback)

    # 중립 RC 후 ACK 스레드와 UDP 소켓을 닫는다.
    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
        try:
            self.stop_movement()
            self.send_neutral()
        except OSError:
            pass
        with self._lock:
            self._closed = True
            self._connected = False
            self._stop_event.set()
            thread = self._thread
            udp_socket = self._socket
        if thread is not None:
            thread.join(timeout=1.0)
        if udp_socket is not None:
            try:
                udp_socket.close()
            except OSError:
                pass

    # ACK 수신, HELLO 재시도와 반복 RC를 한 스레드에서 처리한다.
    def _run(self) -> None:
        while not self._stop_event.is_set():
            udp_socket = self._socket
            if udp_socket is None:
                return
            try:
                payload, sender = udp_socket.recvfrom(256)
            except socket.timeout:
                payload = b""
                sender = ("", 0)
            except OSError:
                if self._stop_event.is_set():
                    return
                self._notify_status("제어 UDP 수신 오류가 발생했습니다.")
                payload = b""
                sender = ("", 0)
            if payload and sender == self.target:
                self._handle_ack(payload)
            self._service_pending()
            self._service_movement()

    # 확인 명령을 하나만 등록하고 UDP로 전송한다.
    def _begin_confirmed(
        self,
        command: int,
        callback: Callable[[ControlAck], None] | None,
        *,
        require_connection: bool = True,
    ) -> int:
        self.start()
        with self._lock:
            if require_connection and not self._connected:
                raise RuntimeError("제어 연결 ACK를 먼저 확인하세요.")
            if self._pending is not None:
                raise RuntimeError("이전 확인 명령의 ACK를 기다리는 중입니다.")
            sequence, payload = self._make_packet_locked(command, (0, 0, 0, 0))
            now = self.monotonic()
            self._pending = _PendingCommand(
                command=command,
                sequence=sequence,
                payload=payload,
                deadline=now + self.ack_timeout,
                next_retry_at=(
                    now + self.hello_retry_seconds
                    if command == COMMAND_HELLO
                    else None
                ),
                callback=callback,
            )
            udp_socket = self._socket
        try:
            if udp_socket is None:
                raise RuntimeError("제어 UDP 소켓이 준비되지 않았습니다.")
            udp_socket.sendto(payload, self.target)
        except Exception:
            with self._lock:
                if self._pending is not None and self._pending.sequence == sequence:
                    self._pending = None
            raise
        self._notify_status(f"{ACK_NAMES[command]} ACK 대기 중")
        return sequence

    # ACK 없는 RC 계열 패킷을 증가한 sequence로 전송한다.
    def _send_unconfirmed(
        self,
        command: int,
        axes: tuple[int, int, int, int],
    ) -> int:
        with self._lock:
            if self._socket is None or self._closed:
                raise RuntimeError("제어 UDP 소켓이 준비되지 않았습니다.")
            sequence, payload = self._make_packet_locked(command, axes)
            udp_socket = self._socket
        udp_socket.sendto(payload, self.target)
        return sequence

    # 수신 ACK를 검증하고 연결·명령 상태를 갱신한다.
    def _handle_ack(self, payload: bytes) -> None:
        if len(payload) != ACK_PACKET.size:
            return
        magic, version, status, sequence, _, acknowledged = ACK_PACKET.unpack(payload)
        if (
            magic != CONTROL_MAGIC
            or version != CONTROL_VERSION
            or acknowledged != 1
        ):
            return
        with self._lock:
            pending = self._pending
            if pending is None or pending.sequence != sequence:
                return
            self._pending = None
            dispatched = (
                pending.command == COMMAND_EMERGENCY
                and status == ACK_STATUS_DISPATCHED
            )
            success = status == ACK_STATUS_OK or dispatched
            if pending.command == COMMAND_HELLO:
                self._connected = success
                if success and self._clear_latch_on_hello:
                    self._emergency_latched = False
                    self._clear_latch_on_hello = False
                if success and self._clear_unknown_on_hello:
                    self._command_outcome_unknown = False
                    self._clear_unknown_on_hello = False
        ack = ControlAck(
            command=pending.command,
            sequence=sequence,
            success=success,
            status=status,
            message=(
                (
                    "비상 정지 전송 완료 (기체 응답 미확인)"
                    if dispatched
                    else f"{ACK_NAMES[pending.command]} 성공"
                )
                if success
                else f"{ACK_NAMES[pending.command]} 실패(status={status})"
            ),
        )
        self._deliver_ack(pending, ack)

    # HELLO 재시도와 확인 명령 시간 초과를 처리한다.
    def _service_pending(self) -> None:
        now = self.monotonic()
        retry_payload: bytes | None = None
        expired: _PendingCommand | None = None
        with self._lock:
            pending = self._pending
            if pending is None:
                return
            if now >= pending.deadline:
                self._pending = None
                self._connected = False
                if pending.command != COMMAND_HELLO:
                    self._command_outcome_unknown = True
                expired = pending
            elif pending.next_retry_at is not None and now >= pending.next_retry_at:
                pending.next_retry_at = now + self.hello_retry_seconds
                retry_payload = pending.payload
            udp_socket = self._socket
        if retry_payload is not None and udp_socket is not None:
            try:
                udp_socket.sendto(retry_payload, self.target)
            except OSError:
                pass
        if expired is not None:
            message = (
                f"{ACK_NAMES[expired.command]} ACK 시간 초과"
                if expired.command == COMMAND_HELLO
                else (
                    f"{ACK_NAMES[expired.command]} ACK 시간 초과: "
                    "실제 결과 불명, 명시적 재연결 필요"
                )
            )
            ack = ControlAck(
                command=expired.command,
                sequence=expired.sequence,
                success=False,
                status=-1,
                message=message,
            )
            self._deliver_ack(expired, ack)

    # 누르고 있는 이동 축을 지정 간격으로 반복 전송한다.
    def _service_movement(self) -> None:
        now = self.monotonic()
        with self._lock:
            axes = self._movement_axes
            if axes is None or now < self._next_rc_at:
                return
            if not self._connected or self._emergency_latched:
                self._movement_axes = None
                return
            self._next_rc_at = now + self.rc_interval
        try:
            self._send_unconfirmed(COMMAND_RC, axes)
        except (OSError, RuntimeError):
            self._notify_status("반복 RC 전송에 실패했습니다.")

    # 프로토콜 범위에 맞춰 축 값을 정수로 제한한다.
    @staticmethod
    def _normalize_axes(axes: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
        if len(axes) != 4:
            raise ValueError("RC 축은 네 개여야 합니다.")
        return tuple(max(-100, min(100, int(value))) for value in axes)  # type: ignore[return-value]

    # 증가 sequence와 데스크톱 플래그를 포함한 패킷을 만든다.
    def _make_packet_locked(
        self,
        command: int,
        axes: tuple[int, int, int, int],
    ) -> tuple[int, bytes]:
        self._sequence = (self._sequence + 1) & 0xFFFFFFFFFFFFFFFF
        if self._sequence == 0:
            self._sequence = 1
        packet = ControlPacket(
            sequence=self._sequence,
            input_ticks=time.perf_counter_ns(),
            axes=axes,
            command=command,
            flags=DESKTOP_CONTROL_FLAG,
        )
        return self._sequence, encode_control_packet(packet)

    # 연결과 비상 잠금 상태가 수동 제어에 안전한지 확인한다.
    def _require_manual_control_locked(self) -> None:
        if not self._connected:
            raise RuntimeError("제어 연결 ACK를 먼저 확인하세요.")
        if self._emergency_latched:
            raise RuntimeError("비상 정지 상태입니다. 명시적으로 다시 연결하세요.")
        if self._pending is not None:
            raise RuntimeError("확인 명령의 ACK를 기다리는 동안 RC와 이륙을 사용할 수 없습니다.")
        if self._command_outcome_unknown:
            raise RuntimeError("이전 명령 결과가 불명입니다. 명시적으로 다시 연결하세요.")

    # 명령별 콜백과 공통 ACK 콜백을 안전하게 호출한다.
    def _deliver_ack(self, pending: _PendingCommand, ack: ControlAck) -> None:
        self._notify_status(ack.message)
        for callback in (pending.callback, self.on_ack):
            if callback is None:
                continue
            try:
                callback(ack)
            except Exception:
                pass

    # 상태 콜백 예외가 제어 스레드를 종료하지 않도록 보호한다.
    def _notify_status(self, message: str) -> None:
        if self.on_status is None:
            return
        try:
            self.on_status(message)
        except Exception:
            pass


class RuntimeSupervisorController:
    def __init__(
        self,
        root: Path,
        config_path: Path,
        on_status: Callable[[str], None] | None = None,
    ) -> None:
        self.root = root.resolve()
        self.config_path = config_path.resolve()
        self.on_status = on_status
        self._lock = threading.Lock()
        self._supervisor: WifiRuntimeSupervisor | None = None
        self._thread: threading.Thread | None = None
        self._last_error: str | None = None
        self._active_ssid: str | None = None

    # 선택된 설정으로 콘솔 없는 Wi-Fi 런타임 감시기를 시작한다.
    def start(self, modules: tuple[str, ...] | None = None) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("런타임 감시기가 이미 실행 중입니다.")
            config = load_config(self.config_path, self.root)
            if modules is not None:
                if not modules:
                    raise ValueError("모듈을 하나 이상 선택해야 합니다.")
                config = replace(config, modules=tuple(modules))
            self._supervisor = WifiRuntimeSupervisor(
                self.root,
                config,
                require_visible_console=False,
            )
            self._last_error = None
            self._active_ssid = config.ssid
            self._thread = threading.Thread(
                target=self._run,
                name="StrikerPCRuntimeSupervisor",
                daemon=False,
            )
            self._thread.start()
        self._emit("Wi-Fi 런타임 감시를 시작했습니다.")

    # 중립 RC와 기존 stop-file 절차로 런타임을 안전 종료한다.
    def stop(self, timeout: float = 30.0) -> None:
        with self._lock:
            supervisor = self._supervisor
            thread = self._thread
        if supervisor is None:
            return
        supervisor.exit_file.parent.mkdir(parents=True, exist_ok=True)
        deadline = time.monotonic() + timeout
        while thread is not None and thread.is_alive():
            supervisor.exit_file.write_text(
                "desktop control safe stop",
                encoding="utf-8",
            )
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("런타임 감시기가 제한 시간 안에 종료되지 않았습니다.")
            thread.join(timeout=min(0.1, remaining))
        with self._lock:
            self._supervisor = None
            self._thread = None
            self._active_ssid = None
        self._emit("런타임이 안전 종료되었습니다.")

    # 현재 감시기와 런타임 프로세스 상태를 짧게 반환한다.
    def status(self) -> str:
        with self._lock:
            if self._last_error:
                return f"오류: {self._last_error}"
            supervisor = self._supervisor
            thread = self._thread
            if supervisor is not None:
                process = supervisor.process
                if process is not None and process.poll() is None:
                    return "런타임 실행 중"
            if thread is not None and thread.is_alive():
                return "Wi-Fi 연결 대기 중"
        return "중지됨"

    @property
    # 감시기 스레드가 현재 실행 중인지 반환한다.
    def active(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    @property
    # 현재 감시기가 선택한 단일 드론 SSID를 반환한다.
    def active_ssid(self) -> str | None:
        with self._lock:
            return self._active_ssid

    # 감시기 예외를 상태로 보존한다.
    def _run(self) -> None:
        try:
            supervisor = self._supervisor
            if supervisor is not None:
                supervisor.run()
        except Exception as error:
            with self._lock:
                self._last_error = str(error)
            self._emit(f"런타임 감시 오류: {error}")

    # UI 큐로 보낼 상태 문자열을 전달한다.
    def _emit(self, message: str) -> None:
        if self.on_status is not None:
            self.on_status(message)


class StrikerPCControlApp:
    def __init__(
        self,
        window: tk.Tk,
        root: Path,
        config_path: Path,
        desktop_control_port: int,
        desktop_state_port: int,
    ) -> None:
        self.window = window
        self.root = root.resolve()
        self.config_path = config_path.resolve()
        self.desktop_control_port = desktop_control_port
        self.desktop_state_port = desktop_state_port
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.runtime = RuntimeSupervisorController(
            self.root,
            self.config_path,
            on_status=lambda message: self.events.put(("runtime", message)),
        )
        self.launcher = StrikerPCLauncher(self.root / "team-sources.json")
        self.module_entries: tuple[ModuleSelectionEntry, ...] = (
            self.launcher.module_selection_entries()
        )
        self.client: DesktopControlClient | None = None
        self.profiles: tuple[DroneProfile, ...] = ()
        self.module_variables: dict[str, tk.BooleanVar] = {}
        self._module_checkbuttons: list[ttk.Checkbutton] = []
        self._pressed_control_keys: set[str] = set()
        self._emergency_button: tk.Button | None = None
        self._closing = False
        self._stopping = False
        self._close_when_stopped = False
        self._next_auto_hello_at = 0.0
        self._resources_closed = False
        self._gateway_window: tk.Toplevel | None = None
        self._gateway_app: object | None = None
        self._gateway_receiver: object | None = None
        self.telemetry: DesktopTelemetryMonitor | None = None
        self._request_stop_path = desktop_gui_stop_path(self.root)
        if self._request_stop_path.exists():
            age = time.time() - self._request_stop_path.stat().st_mtime
            if age > 5.0:
                self._request_stop_path.unlink(missing_ok=True)

        self.profile_name = tk.StringVar()
        self.profile_ssid = tk.StringVar()
        self.runtime_status = tk.StringVar(value="중지됨")
        self.wifi_status = tk.StringVar(value="확인 중")
        self.ack_status = tk.StringVar(value="연결 안 됨")
        self.battery_status = tk.StringVar(value="수신 없음")
        self.temperature_status = tk.StringVar(value="수신 없음")
        self.signal_status = tk.StringVar(value="수신 없음")
        self.flight_status = tk.StringVar(value="확인 중")
        self.speed = tk.IntVar(value=DEFAULT_RC_SPEED)
        self.always_on_top = tk.BooleanVar(value=True)
        self.module_summary = tk.StringVar(value="선택 모듈 확인 중")

        self._configure_window()
        self._build_ui()
        self._reload_profiles()
        self._refresh_controls()
        self.telemetry = DesktopTelemetryMonitor(port=self.desktop_state_port)
        self.telemetry.start()
        self.window.after(100, self._pump_events)
        self.window.after(500, self._poll_status)

    # 창 제목, 크기와 기본 항상 위 상태를 설정한다.
    def _configure_window(self) -> None:
        self.window.title("StrikerPC 실행기")
        self.window.geometry("760x820")
        self.window.minsize(700, 760)
        self.window.attributes("-topmost", True)
        self.window.protocol("WM_DELETE_WINDOW", self._on_close)
        self.window.bind("<FocusOut>", self._on_focus_out)
        self.window.bind("<KeyPress>", self._on_key_press, add="+")
        self.window.bind("<KeyRelease>", self._on_key_release, add="+")

    # 프로필, 상태, 런타임과 비행 제어 UI를 구성한다.
    def _build_ui(self) -> None:
        outer = ttk.Frame(self.window, padding=14)
        outer.pack(fill="both", expand=True)

        title = ttk.Label(
            outer,
            text="StrikerPC 실행기",
            font=("Malgun Gothic", 17, "bold"),
        )
        title.pack(anchor="w", pady=(0, 10))

        profile_frame = ttk.LabelFrame(outer, text="드론 프로필", padding=10)
        profile_frame.pack(fill="x", pady=4)
        self.profile_combo = ttk.Combobox(profile_frame, state="readonly")
        self.profile_combo.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 6))
        self.profile_combo.bind("<<ComboboxSelected>>", self._on_profile_selected)
        ttk.Label(profile_frame, text="이름").grid(row=1, column=0, sticky="w")
        ttk.Entry(profile_frame, textvariable=self.profile_name).grid(
            row=1, column=1, sticky="ew", padx=6
        )
        ttk.Label(profile_frame, text="SSID").grid(row=2, column=0, sticky="w")
        ttk.Entry(profile_frame, textvariable=self.profile_ssid).grid(
            row=2, column=1, sticky="ew", padx=6, pady=4
        )
        ttk.Button(profile_frame, text="저장/선택", command=self._save_profile).grid(
            row=1, column=2, rowspan=2, sticky="nsew"
        )
        profile_frame.columnconfigure(1, weight=1)

        status_frame = ttk.LabelFrame(outer, text="상태", padding=10)
        status_frame.pack(fill="x", pady=4)
        for row, (label, variable) in enumerate(
            (
                ("런타임", self.runtime_status),
                ("Wi-Fi", self.wifi_status),
                ("제어 ACK", self.ack_status),
                ("배터리", self.battery_status),
                ("온도", self.temperature_status),
                ("연결 강도", self.signal_status),
                ("비행 상태", self.flight_status),
            )
        ):
            ttk.Label(status_frame, text=label, width=11).grid(row=row, column=0, sticky="w")
            ttk.Label(status_frame, textvariable=variable).grid(row=row, column=1, sticky="w")
        ttk.Checkbutton(
            status_frame,
            text="항상 위",
            variable=self.always_on_top,
            command=self._toggle_topmost,
        ).grid(row=0, column=2, rowspan=7, padx=(10, 0))
        status_frame.columnconfigure(1, weight=1)

        configured_modules = set(load_config(self.config_path, self.root).modules)
        module_frame = ttk.LabelFrame(outer, text="실행 모듈 선택", padding=10)
        module_frame.pack(fill="x", pady=4)
        for index, entry in enumerate(self.module_entries):
            variable = tk.BooleanVar(value=entry.key in configured_modules)
            self.module_variables[entry.key] = variable
            checkbox = ttk.Checkbutton(
                module_frame,
                text=entry.name,
                variable=variable,
                command=self._update_module_summary,
            )
            checkbox.grid(
                row=index // 3,
                column=index % 3,
                sticky="w",
                padx=(0, 12),
                pady=2,
            )
            self._module_checkbuttons.append(checkbox)
        for column in range(3):
            module_frame.columnconfigure(column, weight=1)
        ttk.Label(
            module_frame,
            textvariable=self.module_summary,
            foreground="#315b7d",
            wraplength=700,
        ).grid(
            row=(len(self.module_entries) + 2) // 3,
            column=0,
            columnspan=3,
            sticky="w",
            pady=(7, 0),
        )
        self._update_module_summary()

        runtime_frame = ttk.Frame(outer)
        runtime_frame.pack(fill="x", pady=8)
        ttk.Button(
            runtime_frame,
            text="선택 모듈 실행 / 제어 재연결",
            command=self._start,
        ).pack(
            side="left", fill="x", expand=True, padx=(0, 4)
        )
        ttk.Button(runtime_frame, text="안전 종료", command=self._safe_stop).pack(
            side="left", fill="x", expand=True, padx=4
        )
        ttk.Button(
            runtime_frame,
            text="게이트웨이 상태",
            command=self._open_gateway_dashboard,
        ).pack(
            side="left", fill="x", expand=True, padx=(4, 0)
        )

        keyboard_frame = ttk.LabelFrame(outer, text="키보드 비행 제어", padding=10)
        keyboard_frame.pack(fill="both", expand=True, pady=4)
        speed_frame = ttk.Frame(keyboard_frame)
        speed_frame.pack(fill="x", pady=(0, 8))
        ttk.Label(speed_frame, text="RC 속도").pack(side="left")
        ttk.Spinbox(
            speed_frame,
            from_=10,
            to=100,
            increment=5,
            textvariable=self.speed,
            width=7,
        ).pack(side="left", padx=8)
        ttk.Label(
            keyboard_frame,
            text=(
                "W / S  전진·후진     A / D  좌·우 이동     "
                "R / F  상승·하강     Q / E  좌·우 회전\n"
                "T  이륙     L  착륙     Space  중립     "
                "이동 키를 놓으면 즉시 중립"
            ),
            font=("Malgun Gothic", 11, "bold"),
            justify="left",
        ).pack(anchor="w", pady=4)
        ttk.Label(
            keyboard_frame,
            text=(
                "프로필 이름·SSID·속도 입력칸에 커서가 있으면 비행 키를 무시합니다. "
                "창이 포커스를 잃어도 중립 RC를 전송합니다."
            ),
            foreground="#5c6670",
            wraplength=700,
        ).pack(anchor="w", pady=(4, 0))

        command_frame = ttk.LabelFrame(outer, text="비상 명령", padding=10)
        command_frame.pack(fill="x", pady=6)
        self._emergency_button = tk.Button(
            command_frame,
            text="비상 정지",
            command=self._emergency,
            bg="#b00020",
            fg="white",
            activebackground="#7f0015",
            activeforeground="white",
            font=("Malgun Gothic", 13, "bold"),
            relief="raised",
        )
        self._emergency_button.pack(fill="x", expand=True, padx=3, ipady=7)

        ttk.Label(
            outer,
            text=(
                "비상 정지는 전송 즉시 RC와 이륙을 잠급니다. "
                "런타임을 완전히 종료하고 다시 시작하기 전에는 해제되지 않습니다."
            ),
            foreground="#8b0000",
            wraplength=620,
        ).pack(fill="x", pady=(6, 0))

    # 체크된 모듈을 런처 메뉴 순서대로 반환한다.
    def _selected_modules(self) -> tuple[str, ...]:
        return tuple(
            entry.key
            for entry in self.module_entries
            if self.module_variables[entry.key].get()
        )

    # 동일 프로세스의 보조 창으로 타격 게이트웨이 상태 화면을 연다.
    def _open_gateway_dashboard(self) -> None:
        current = self._gateway_window
        if current is not None:
            try:
                if current.winfo_exists():
                    current.deiconify()
                    current.lift()
                    current.focus_force()
                    return
            except tk.TclError:
                pass

        from strike_gateway_dashboard import (
            GatewayDashboardModel,
            SafetyStatusReceiver,
            StrikeGatewayDashboardApp,
            load_dashboard_port,
        )

        receiver: SafetyStatusReceiver | None = None
        gateway_window: tk.Toplevel | None = None
        try:
            receiver = SafetyStatusReceiver(load_dashboard_port(self.root))
            receiver.start()
            gateway_window = tk.Toplevel(self.window)
            gateway_app = StrikeGatewayDashboardApp(
                gateway_window,
                GatewayDashboardModel(self.root),
                receiver,
                1000,
                runtime_controller=self.runtime,
                owns_runtime_lifecycle=False,
            )
        except Exception as error:
            if receiver is not None:
                receiver.close()
            if gateway_window is not None:
                gateway_window.destroy()
            messagebox.showerror("게이트웨이 화면 시작 실패", str(error), parent=self.window)
            return

        self._gateway_window = gateway_window
        self._gateway_app = gateway_app
        self._gateway_receiver = receiver
        gateway_window.bind("<Destroy>", self._on_gateway_dashboard_destroyed, add="+")
        gateway_window.lift()
        gateway_window.focus_force()

    # 게이트웨이 보조 창이 닫히면 재실행을 위해 참조를 해제한다.
    def _on_gateway_dashboard_destroyed(self, event: object) -> None:
        if getattr(event, "widget", None) is not self._gateway_window:
            return
        self._gateway_window = None
        self._gateway_app = None
        self._gateway_receiver = None

    # 선택 모듈과 자동 포함되는 의존 모듈을 즉시 표시한다.
    def _update_module_summary(self) -> None:
        selected = self._selected_modules()
        if not selected:
            self.module_summary.set("모듈을 하나 이상 선택하세요.")
            return
        try:
            resolved = self.launcher.resolve_modules(selected)
        except (RuntimeError, ValueError) as error:
            self.module_summary.set(f"선택 오류: {error}")
            return
        automatic = tuple(key for key in resolved if key not in selected)
        summary = f"선택: {', '.join(selected)}"
        if automatic:
            summary += f"  ·  의존 모듈 자동 포함: {', '.join(automatic)}"
        self.module_summary.set(summary)

    # 프로필과 속도 입력 중에는 문자 키를 비행 입력으로 사용하지 않는다.
    @staticmethod
    def _is_editable_key_target(widget: object) -> bool:
        try:
            widget_class = str(widget.winfo_class())  # type: ignore[attr-defined]
        except (AttributeError, tk.TclError):
            return False
        return widget_class in {"Entry", "TEntry", "TCombobox", "TSpinbox", "Text"}

    # 키를 누르면 이동을 합성하거나 이륙·착륙·중립 명령을 한 번 실행한다.
    def _on_key_press(self, event: object) -> str | None:
        if self._is_editable_key_target(getattr(event, "widget", None)):
            return None
        key = str(getattr(event, "keysym", "")).lower()
        if key not in KEYBOARD_AXIS_DIRECTIONS and key not in KEYBOARD_COMMAND_KEYS:
            return None
        if key in self._pressed_control_keys:
            return "break"
        self._pressed_control_keys.add(key)
        if key in KEYBOARD_AXIS_DIRECTIONS:
            self._update_keyboard_movement()
        elif key == "t":
            self._takeoff()
        elif key == "l":
            self._land()
        else:
            self._neutral()
        return "break"

    # 이동 키를 놓으면 남은 키 조합으로 갱신하고 모두 놓였으면 중립을 보낸다.
    def _on_key_release(self, event: object) -> str | None:
        key = str(getattr(event, "keysym", "")).lower()
        if key not in KEYBOARD_AXIS_DIRECTIONS and key not in KEYBOARD_COMMAND_KEYS:
            return None
        self._pressed_control_keys.discard(key)
        if key in KEYBOARD_AXIS_DIRECTIONS:
            self._update_keyboard_movement()
        return "break"

    # 현재 눌린 이동 키를 지정 RC 속도의 단일 축 패킷으로 반영한다.
    def _update_keyboard_movement(self) -> None:
        if self.client is None:
            return
        try:
            speed = int(self.speed.get())
        except (TypeError, ValueError, tk.TclError):
            speed = DEFAULT_RC_SPEED
        axes = keyboard_axes(self._pressed_control_keys, speed)
        try:
            if axes == (0, 0, 0, 0):
                self.client.stop_movement()
            else:
                self.client.start_movement(axes)
        except Exception as error:
            self.ack_status.set(str(error))
        self._refresh_controls()

    # 설정 파일에서 프로필을 다시 읽고 첫 항목을 선택한다.
    def _reload_profiles(self, preferred: DroneProfile | None = None) -> None:
        try:
            self.profiles = load_drone_profiles(self.config_path)
        except ValueError as error:
            self.profiles = ()
            self.ack_status.set(str(error))
        names = [profile.name for profile in self.profiles]
        self.profile_combo.configure(values=names)
        if not self.profiles:
            return
        selected = preferred or select_config_profile(self.config_path, self.profiles)
        index = next(
            (i for i, profile in enumerate(self.profiles) if profile.name == selected.name),
            0,
        )
        self.profile_combo.current(index)
        self._apply_profile(self.profiles[index])

    # 선택된 프로필을 입력 필드에 표시한다.
    def _apply_profile(self, profile: DroneProfile) -> None:
        self.profile_name.set(profile.name)
        self.profile_ssid.set(profile.ssid)
        if self.telemetry is not None:
            self.telemetry.reset()

    # 콤보박스 선택을 편집 필드에 반영한다.
    def _on_profile_selected(self, _event: object = None) -> None:
        index = self.profile_combo.current()
        if 0 <= index < len(self.profiles):
            self._apply_profile(self.profiles[index])

    # 현재 이름과 SSID를 저장하고 기존 런타임 설정을 유지한다.
    def _save_profile(self) -> DroneProfile | None:
        try:
            profile = DroneProfile(self.profile_name.get(), self.profile_ssid.get())
            save_drone_profile(self.config_path, profile)
            self._reload_profiles(profile)
            self.ack_status.set(f"프로필 저장: {profile.name}")
            return profile
        except (OSError, ValueError) as error:
            messagebox.showerror("프로필 저장 실패", str(error), parent=self.window)
            return None

    # 로그인 시작 시 현재 ssid 감시와 제어 HELLO를 자동 시작한다.
    def _auto_start_runtime(self) -> None:
        if self._closing or self._stopping or self.runtime.active:
            return
        try:
            if self.telemetry is not None:
                self.telemetry.reset()
            self.runtime.start()
            self.runtime_status.set("저장된 SSID 자동 감시 시작")
            self.window.after(500, self._connect_control)
        except Exception as error:
            self.runtime_status.set(f"자동 시작 실패: {error}")

    # 선택 프로필을 저장하고 감시기와 제어 HELLO를 시작한다.
    def _start(self) -> None:
        if self.client is not None and (
            self.client.emergency_latched
            or self.client.command_outcome_unknown
        ):
            messagebox.showwarning(
                "런타임 재시작 필요",
                (
                    "비상 정지 또는 명령 결과 불명 상태입니다.\n"
                    "기체 상태를 직접 확인한 뒤 안전 종료하고 런타임을 다시 시작하세요."
                ),
                parent=self.window,
            )
            return
        selected_modules = self._selected_modules()
        try:
            self.launcher.resolve_modules(selected_modules)
            save_module_selection(self.config_path, selected_modules)
        except (OSError, RuntimeError, ValueError) as error:
            messagebox.showerror("모듈 선택 오류", str(error), parent=self.window)
            return
        profile = self._save_profile()
        if profile is None:
            return
        try:
            if self.runtime.active and self.runtime.active_ssid != profile.ssid:
                raise RuntimeError(
                    "다른 드론으로 전환하려면 현재 런타임을 안전 종료한 뒤 시작하세요."
                )
            if not self.runtime.active:
                if self.telemetry is not None:
                    self.telemetry.reset()
                self.runtime.start(selected_modules)
                self.runtime_status.set(
                    f"선택 모듈 실행 준비: {', '.join(selected_modules)}"
                )
        except Exception as error:
            self.runtime_status.set(f"시작 실패: {error}")
            return
        self.window.after(500, self._connect_control)

    # 새 클라이언트 또는 명시적 재연결로 HELLO ACK를 요청한다.
    def _connect_control(self) -> None:
        if self._closing or self._stopping or not self.runtime.active:
            return
        try:
            if self.client is None:
                self.client = DesktopControlClient(
                    ("127.0.0.1", self.desktop_control_port),
                    on_ack=lambda ack: self.events.put(("ack", ack)),
                    on_status=lambda status: self.events.put(("control", status)),
                )
                self.client.connect()
            else:
                self.client.reconnect()
            self.ack_status.set("연결 ACK 대기 중")
            self._next_auto_hello_at = time.monotonic() + 2.0
        except Exception as error:
            self.ack_status.set(f"제어 연결 실패: {error}")
        self._refresh_controls()

    # 지정 방향과 현재 속도로 반복 RC를 시작한다.
    def _movement_press(self, direction: tuple[int, int, int, int]) -> None:
        if self.client is None:
            return
        speed = max(10, min(100, int(self.speed.get())))
        axes = tuple(component * speed for component in direction)
        try:
            self.client.start_movement(axes)  # type: ignore[arg-type]
        except Exception as error:
            self.ack_status.set(str(error))
        self._refresh_controls()

    # 이동 버튼을 놓는 즉시 중립 RC를 전송한다.
    def _movement_release(self, _event: object = None) -> None:
        if self.client is None:
            return
        try:
            self.client.stop_movement()
        except Exception as error:
            self.ack_status.set(str(error))

    # 포커스를 잃으면 눌림 상태와 무관하게 중립 RC를 보낸다.
    def _on_focus_out(self, _event: object = None) -> None:
        self._pressed_control_keys.clear()
        if self.client is None:
            return
        try:
            self.client.stop_movement()
            self.client.send_neutral()
        except Exception:
            pass

    # 중립 버튼으로 현재 이동을 즉시 정지한다.
    def _neutral(self) -> None:
        if self.client is None:
            return
        try:
            self.client.stop_movement()
            self.client.send_neutral()
            self.ack_status.set("중립 RC 전송")
        except Exception as error:
            self.ack_status.set(str(error))

    # 사용자 확인 후 이륙 명령을 보낸다.
    def _takeoff(self) -> None:
        if self.client is None:
            return
        if not messagebox.askyesno(
            "이륙 확인",
            "비행 구역과 프로펠러 주변을 확인했습니까?",
            parent=self.window,
        ):
            return
        self._send_confirmed(self.client.send_takeoff)

    # 착륙 명령은 비상 잠금 중에도 전송한다.
    def _land(self) -> None:
        if self.client is not None:
            self._send_confirmed(self.client.send_land)

    # 확인창 없이 즉시 비상 정지를 보내고 UI를 잠근다.
    def _emergency(self) -> None:
        if self.client is None:
            return
        self._send_confirmed(self.client.send_emergency)
        self._refresh_controls()

    # 확인형 명령의 동기 전송 오류만 UI에 표시한다.
    def _send_confirmed(self, sender: Callable[[], int]) -> None:
        try:
            sender()
        except Exception as error:
            self.ack_status.set(str(error))

    # 제어 클라이언트와 런타임을 백그라운드에서 안전 종료한다.
    def _safe_stop(self) -> None:
        if self._shutdown_confirmation_required():
            if not messagebox.askyesno(
                "안전 종료 확인",
                (
                    "안전 종료는 착륙 명령이 아닙니다.\n"
                    "기체가 비행 중이면 먼저 L 키로 착륙시키세요.\n\n"
                    "중립 RC를 보낸 뒤 런타임을 종료하시겠습니까?"
                ),
                parent=self.window,
            ):
                return
        self._begin_safe_stop(close_after=False)

    # 중복 종료를 막고 중립 RC를 먼저 전송한다.
    def _begin_safe_stop(self, *, close_after: bool) -> None:
        if self._stopping:
            self._close_when_stopped = self._close_when_stopped or close_after
            return
        self._stopping = True
        self._close_when_stopped = close_after
        self._pressed_control_keys.clear()
        client = self.client
        self.client = None
        if client is not None:
            client.close()
        self._refresh_controls()

        def stop_worker() -> None:
            try:
                self.runtime.stop()
                result: object = None
            except Exception as error:
                result = error
            self.events.put(("stopped-close" if close_after else "stopped", result))

        threading.Thread(
            target=stop_worker,
            name="StrikerPCDesktopSafeStop",
            daemon=False,
        ).start()

    # 작업 스레드의 상태와 ACK를 Tk 메인 스레드에 반영한다.
    def _pump_events(self) -> None:
        while True:
            try:
                event, value = self.events.get_nowait()
            except queue.Empty:
                break
            if event == "ack" and isinstance(value, ControlAck):
                self.ack_status.set(value.message)
                if self.telemetry is not None:
                    command = {
                        COMMAND_TAKEOFF: "takeoff",
                        COMMAND_LAND: "land",
                        COMMAND_EMERGENCY: "emergency",
                    }.get(value.command)
                    if command is not None:
                        self.telemetry.record_action(command, value.success)
                self._refresh_controls()
                if (
                    value.command == COMMAND_EMERGENCY
                    and value.success
                    and not self._stopping
                ):
                    self.runtime_status.set("비상 정지 ACK 확인 - 런타임 안전 종료 중")
                    self._begin_safe_stop(close_after=False)
            elif event in {"runtime", "control"}:
                if event == "runtime":
                    self.runtime_status.set(str(value))
                else:
                    self.ack_status.set(str(value))
            elif event in {"stopped", "stopped-close"}:
                self._stopping = False
                if isinstance(value, Exception):
                    self.runtime_status.set(f"종료 오류: {value}")
                else:
                    self.runtime_status.set("중지됨")
                    self.ack_status.set("안전 종료 완료")
                if event == "stopped-close" or self._close_when_stopped:
                    self.close_resources()
                    self.window.destroy()
                    return
                self._refresh_controls()
        self.window.after(100, self._pump_events)

    # Wi-Fi, 런타임과 제어 연결 상태를 주기적으로 갱신한다.
    def _poll_status(self) -> None:
        if self._closing:
            return
        if self._request_stop_path.exists():
            self._request_stop_path.unlink(missing_ok=True)
            self._closing = True
            self._begin_safe_stop(close_after=True)
            return
        self.runtime_status.set(self.runtime.status())
        ssid = self.profile_ssid.get().strip()
        connected = read_windows_connected_ssids()
        if ssid and ssid in connected:
            self.wifi_status.set(f"연결됨: {ssid}")
        elif ssid:
            self.wifi_status.set(f"대기 중: {ssid}")
        else:
            self.wifi_status.set("SSID 미설정")
        self._update_telemetry_status()
        if self.client is not None and self.client.connected:
            if self.client.emergency_latched:
                self.ack_status.set("비상 정지 잠금 - 재연결 필요")
        now = time.monotonic()
        if (
            self.runtime.active
            and self.client is not None
            and not self.client.connected
            and not self.client.emergency_latched
            and not self.client.command_pending
            and not self.client.command_outcome_unknown
            and now >= self._next_auto_hello_at
        ):
            try:
                self.client.connect()
                self.ack_status.set("런타임 제어 HELLO 자동 재시도")
            except Exception as error:
                self.ack_status.set(f"자동 재연결 실패: {error}")
            self._next_auto_hello_at = now + 2.0
        self._refresh_controls()
        self.window.after(500, self._poll_status)

    # 최신 Tello state를 사람이 읽을 수 있는 표시 문자열로 갱신한다.
    def _update_telemetry_status(self) -> None:
        monitor = self.telemetry
        if monitor is None:
            return
        status = monitor.status()
        if not status.connected:
            self.battery_status.set("수신 없음")
            self.temperature_status.set("수신 없음")
            self.signal_status.set("수신 없음")
            self.flight_status.set("상태 수신 없음")
            return
        self.battery_status.set(
            "--" if status.battery_percent is None else f"{status.battery_percent}%"
        )
        self.temperature_status.set(
            "--" if status.temperature_c is None else f"{status.temperature_c:.1f} °C"
        )
        self.signal_status.set(
            "--"
            if status.wifi_signal_percent is None
            else f"{status.wifi_signal_percent}%"
        )
        self.flight_status.set(
            {
                FLIGHT_GROUNDED: "착륙 확인",
                FLIGHT_AIRBORNE: "비행 중",
                FLIGHT_LANDING: "착륙 중",
            }.get(status.flight_state, "확인 중")
        )

    # 현재 제어·텔레메트리 상태에서 종료 확인창이 필요한지 판정한다.
    def _shutdown_confirmation_required(self) -> bool:
        client = self.client
        telemetry_status = (
            self.telemetry.status()
            if self.telemetry is not None
            else DesktopTelemetryStatus(False, FLIGHT_UNKNOWN, False)
        )
        return shutdown_confirmation_required(
            runtime_active=self.runtime.active,
            control_connected=bool(client is not None and client.connected),
            command_pending=bool(client is not None and client.command_pending),
            command_outcome_unknown=bool(
                client is not None and client.command_outcome_unknown
            ),
            emergency_latched=bool(client is not None and client.emergency_latched),
            telemetry_connected=telemetry_status.connected,
            confirmed_grounded=telemetry_status.confirmed_grounded,
        )

    # 런타임과 제어 연결 상태에 따라 모듈 선택과 비상 버튼을 활성화한다.
    def _refresh_controls(self) -> None:
        module_state = "disabled" if self.runtime.active or self._stopping else "normal"
        for checkbox in self._module_checkbuttons:
            checkbox.configure(state=module_state)
        emergency_enabled = bool(
            self.client is not None
            and self.client.connected
            and not self.client.emergency_latched
            and not self._stopping
        )
        if self._emergency_button is not None:
            self._emergency_button.configure(
                state="normal" if emergency_enabled else "disabled"
            )

    # 체크박스 상태를 실제 always-on-top 속성에 반영한다.
    def _toggle_topmost(self) -> None:
        self.window.attributes("-topmost", bool(self.always_on_top.get()))

    # 창 닫기 전에 중립 RC와 런타임 안전 종료를 완료한다.
    def _on_close(self) -> None:
        if self._closing:
            return
        if self._shutdown_confirmation_required():
            if not messagebox.askyesno(
                "창 닫기 확인",
                (
                    "창 닫기는 착륙 명령이 아닙니다.\n"
                    "비행 중이면 취소하고 먼저 L 키로 착륙시키세요.\n\n"
                    "중립 RC와 런타임 안전 종료 후 창을 닫으시겠습니까?"
                ),
                parent=self.window,
            ):
                return
        self._closing = True
        self._begin_safe_stop(close_after=True)

    # GUI가 종료될 때 데스크톱 telemetry 수신 소켓을 한 번만 닫는다.
    def close_resources(self) -> None:
        if self._resources_closed:
            return
        self._resources_closed = True
        gateway_app = self._gateway_app
        if gateway_app is not None:
            close_monitor = getattr(gateway_app, "close_monitor", None)
            if callable(close_monitor):
                close_monitor()
        elif self._gateway_receiver is not None:
            close_receiver = getattr(self._gateway_receiver, "close", None)
            if callable(close_receiver):
                close_receiver()
        if self.telemetry is not None:
            self.telemetry.close()


class SingleInstanceMutex:
    def __init__(self, name: str = MUTEX_NAME) -> None:
        self.name = name
        self._handle: int | None = None
        self._kernel32 = None

    # Win32 named mutex를 만들고 중복 실행을 거부한다.
    def acquire(self) -> bool:
        if sys.platform != "win32":
            return True
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.CreateMutexW.argtypes = [ctypes.c_void_p, ctypes.c_bool, ctypes.c_wchar_p]
        kernel32.CreateMutexW.restype = ctypes.c_void_p
        kernel32.CloseHandle.argtypes = [ctypes.c_void_p]
        handle = kernel32.CreateMutexW(None, False, self.name)
        if not handle:
            return False
        if ctypes.get_last_error() == 183:
            kernel32.CloseHandle(handle)
            return False
        self._handle = int(handle)
        self._kernel32 = kernel32
        return True

    # 보유한 Win32 mutex handle을 닫는다.
    def close(self) -> None:
        if self._handle is None or sys.platform != "win32":
            return
        self._kernel32.CloseHandle(ctypes.c_void_p(self._handle))
        self._handle = None
        self._kernel32 = None


# 외부 저장소를 번들하지 않고 현재 StrikerPC 루트를 찾는다.
def find_default_root() -> Path:
    candidates: list[Path] = []
    configured = os.getenv("STRIKERPC_ROOT") or os.getenv("DRONEOPS_ROOT")
    if configured:
        candidates.append(Path(configured))
    candidates.append(Path.cwd())
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


# 외부 request-stop 명령과 GUI가 공유할 stop-file 경로를 만든다.
def desktop_gui_stop_path(root: Path) -> Path:
    return Path(root) / ".team-runtime" / GUI_STOP_NAME


# manifest에서 독립 데스크톱 제어 포트를 읽는다.
def load_default_desktop_port(root: Path) -> int:
    manifest_path = Path(root) / "team-sources.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        port = manifest["ports"]["desktopControl"]
        if isinstance(port, int) and 1 <= port <= 65535:
            return port
    except (KeyError, OSError, TypeError, json.JSONDecodeError):
        pass
    return DEFAULT_DESKTOP_CONTROL_PORT


# manifest에서 데스크톱 Tello state relay 포트를 읽는다.
def load_default_desktop_state_port(root: Path) -> int:
    manifest_path = Path(root) / "team-sources.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        port = manifest["ports"]["desktopStateRelay"]
        if isinstance(port, int) and 1 <= port <= 65535:
            return port
    except (KeyError, OSError, TypeError, json.JSONDecodeError):
        pass
    return DEFAULT_DESKTOP_STATE_PORT


# 명령행에서 저장소 루트, 설정과 데스크톱 제어·상태 포트를 받는다.
def parse_args() -> argparse.Namespace:
    default_root = find_default_root()
    parser = argparse.ArgumentParser(description="StrikerPC 실행기 GUI")
    parser.add_argument("--root", default=str(default_root))
    parser.add_argument("--config")
    parser.add_argument("--check-config", action="store_true")
    parser.add_argument("--request-stop", action="store_true")
    parser.add_argument(
        "--desktop-control-port",
        "--relay-port",
        dest="desktop_control_port",
        type=int,
        default=None,
    )
    parser.add_argument("--desktop-state-port", type=int, default=None)
    args = parser.parse_args()
    if args.desktop_control_port is None:
        args.desktop_control_port = load_default_desktop_port(Path(args.root))
    if args.desktop_state_port is None:
        args.desktop_state_port = load_default_desktop_state_port(Path(args.root))
    return args


# 단일 인스턴스 GUI를 시작한다.
def main() -> int:
    args = parse_args()
    root = Path(args.root).resolve()
    config_path = (
        Path(args.config).resolve()
        if args.config
        else root / "Gateway" / "wifi_runtime_autostart.json"
    )
    if args.check_config:
        try:
            config = load_config(config_path, root)
            profiles = load_drone_profiles(config_path)
            from strike_gateway_dashboard import (
                DEFAULT_DASHBOARD_STATE_PORT,
                load_dashboard_port,
            )

            gateway_state_port = (
                load_dashboard_port(root)
                if (root / "team-sources.json").is_file()
                else DEFAULT_DASHBOARD_STATE_PORT
            )
        except ValueError as error:
            print(f"[DesktopControl] 설정 오류: {error}", file=sys.stderr)
            return 2
        print(
            f"[DesktopControl] OK profiles={len(profiles)} "
            f"ssid={config.ssid} controlPort={args.desktop_control_port} "
            f"statePort={args.desktop_state_port} "
            f"gatewayStatePort={gateway_state_port}"
        )
        return 0
    if args.request_stop:
        gui_stop = desktop_gui_stop_path(root)
        launcher_stop, supervisor_exit = runtime_paths(root)
        for stop_path in (gui_stop, launcher_stop, supervisor_exit):
            stop_path.parent.mkdir(parents=True, exist_ok=True)
            stop_path.write_text("desktop control stop requested", encoding="utf-8")
        return 0

    mutex = SingleInstanceMutex()
    if not mutex.acquire():
        if sys.platform == "win32":
            ctypes.windll.user32.MessageBoxW(
                None,
                "StrikerPC 실행기가 이미 실행 중입니다.",
                "StrikerPC",
                0x30,
            )
        return 2

    window: tk.Tk | None = None
    app: StrikerPCControlApp | None = None
    try:
        window = tk.Tk()
        app = StrikerPCControlApp(
            window,
            root,
            config_path,
            args.desktop_control_port,
            args.desktop_state_port,
        )
        window.mainloop()
        return 0
    except Exception as error:
        if window is not None:
            messagebox.showerror("StrikerPC 시작 실패", str(error), parent=window)
        return 1
    finally:
        if app is not None:
            app.close_resources()
        mutex.close()


if __name__ == "__main__":
    raise SystemExit(main())
