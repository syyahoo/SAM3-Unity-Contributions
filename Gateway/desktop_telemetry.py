from __future__ import annotations

import json
import math
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable


FLIGHT_UNKNOWN = "unknown"
FLIGHT_GROUNDED = "grounded"
FLIGHT_AIRBORNE = "airborne"
FLIGHT_LANDING = "landing"
SAFETY_STATE_TYPE = "flight-safety-state"
SAFETY_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class DesktopTelemetrySnapshot:
    received_at: float
    battery_percent: int | None = None
    temperature_c: float | None = None
    wifi_signal_percent: int | None = None
    height_meters: float | None = None
    tof_meters: float | None = None

    # 마지막 Tello state가 지정 시간 안에 수신됐는지 확인한다.
    def is_fresh(self, now: float, maximum_age_seconds: float) -> bool:
        return 0.0 <= now - self.received_at <= maximum_age_seconds


@dataclass(frozen=True)
class DesktopTelemetryStatus:
    connected: bool
    flight_state: str
    confirmed_grounded: bool
    battery_percent: int | None = None
    temperature_c: float | None = None
    wifi_signal_percent: int | None = None


@dataclass(frozen=True)
class DesktopFlightSafetyStatus:
    received_at: float
    session_id: str
    sequence: int
    state: str
    airborne: bool
    grounded_confirmed: bool
    state_age_ms: int

    # 로컬 수신 지연을 더한 Tello state 나이가 안전 한도 안인지 확인한다.
    def is_fresh(
        self,
        now: float,
        *,
        maximum_local_age_seconds: float = 1.0,
        maximum_state_age_seconds: float = 0.5,
    ) -> bool:
        local_age = now - self.received_at
        effective_state_age = self.state_age_ms / 1000.0 + local_age
        return (
            0.0 <= local_age <= maximum_local_age_seconds
            and 0.0 <= effective_state_age <= maximum_state_age_seconds
        )


def _finite_number(values: dict[str, str], key: str) -> float | None:
    try:
        value = float(values[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _bounded_integer(
    values: dict[str, str],
    key: str,
    minimum: int,
    maximum: int,
) -> int | None:
    value = _finite_number(values, key)
    if value is None or value < minimum or value > maximum:
        return None
    return int(round(value))


# Position Sync가 전달한 Tello state에서 데스크톱 표시 필드만 안전하게 읽는다.
def parse_desktop_telemetry(
    payload: bytes,
    *,
    received_at: float | None = None,
) -> DesktopTelemetrySnapshot | None:
    try:
        text = payload.decode("ascii").strip()
    except UnicodeDecodeError:
        return None
    values: dict[str, str] = {}
    for item in text.split(";"):
        if ":" not in item:
            continue
        key, value = item.split(":", 1)
        if key:
            values[key] = value
    if not values:
        return None

    temperatures = [
        value
        for value in (
            _finite_number(values, "templ"),
            _finite_number(values, "temph"),
        )
        if value is not None
    ]
    if not temperatures:
        single_temperature = _finite_number(values, "temp")
        if single_temperature is not None:
            temperatures.append(single_temperature)
    height_cm = _finite_number(values, "h")
    tof_cm = _finite_number(values, "tof")
    return DesktopTelemetrySnapshot(
        received_at=time.monotonic() if received_at is None else received_at,
        battery_percent=_bounded_integer(values, "bat", 0, 100),
        temperature_c=(
            None
            if not temperatures
            else round(sum(temperatures) / len(temperatures), 1)
        ),
        wifi_signal_percent=_bounded_integer(values, "wifi", 0, 100),
        height_meters=(
            None if height_cm is None or height_cm < 0.0 else height_cm / 100.0
        ),
        tof_meters=None if tof_cm is None or tof_cm < 0.0 else tof_cm / 100.0,
    )


# Pilot 안전 상태에서 종료 경고 판정에 필요한 필드만 검증해 읽는다.
def parse_desktop_safety_status(
    payload: bytes,
    *,
    received_at: float | None = None,
) -> DesktopFlightSafetyStatus | None:
    try:
        message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(message, dict):
        return None

    schema_version = message.get("schemaVersion")
    session_id = message.get("sessionId")
    sequence = message.get("sequence")
    state = message.get("state")
    airborne = message.get("airborne")
    grounded_confirmed = message.get("groundedConfirmed")
    state_age_ms = message.get("stateAgeMs")
    if (
        isinstance(schema_version, bool)
        or schema_version != SAFETY_SCHEMA_VERSION
        or message.get("type") != SAFETY_STATE_TYPE
        or not isinstance(session_id, str)
        or not session_id
        or len(session_id) > 128
        or isinstance(sequence, bool)
        or not isinstance(sequence, int)
        or sequence < 0
        or not isinstance(state, str)
        or len(state) > 64
        or not isinstance(airborne, bool)
        or not isinstance(grounded_confirmed, bool)
        or isinstance(state_age_ms, bool)
        or not isinstance(state_age_ms, int)
        or state_age_ms < 0
    ):
        return None
    return DesktopFlightSafetyStatus(
        received_at=time.monotonic() if received_at is None else received_at,
        session_id=session_id,
        sequence=sequence,
        state=state,
        airborne=airborne,
        grounded_confirmed=grounded_confirmed,
        state_age_ms=state_age_ms,
    )


class DesktopTelemetryMonitor:
    """데스크톱 전용 Tello state relay를 읽고 표시·착륙 상태를 판정한다."""

    def __init__(
        self,
        *,
        port: int = 8893,
        freshness_seconds: float = 0.5,
        clock: Callable[[], float] = time.monotonic,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self._clock = clock
        self._freshness_seconds = freshness_seconds
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._snapshot: DesktopTelemetrySnapshot | None = None
        self._safety_status: DesktopFlightSafetyStatus | None = None
        self._flight_state = FLIGHT_UNKNOWN
        self._airborne_samples = 0
        self._grounded_samples = 0
        self._takeoff_grace_until = 0.0
        self._thread: threading.Thread | None = None
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind(("127.0.0.1", port))
            self._socket.settimeout(0.05)
        except Exception:
            self._socket.close()
            raise

    @property
    def address(self) -> tuple[str, int]:
        return self._socket.getsockname()

    # Tello state 수신 스레드를 한 번만 시작한다.
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="StrikerPCDesktopTelemetry",
            daemon=True,
        )
        self._thread.start()

    # 새 드론 세션에서 이전 기체의 착륙 판정을 재사용하지 않도록 초기화한다.
    def reset(self) -> None:
        with self._lock:
            self._snapshot = None
            self._safety_status = None
            self._flight_state = FLIGHT_UNKNOWN
            self._airborne_samples = 0
            self._grounded_samples = 0
            self._takeoff_grace_until = 0.0

    # 실제 확인 명령 ACK를 지면 판정의 보조 상태로 반영한다.
    def record_action(self, command: str, success: bool) -> None:
        if not success:
            return
        current = self._clock()
        with self._lock:
            if command == "takeoff":
                self._flight_state = FLIGHT_AIRBORNE
                self._takeoff_grace_until = current + 3.0
                self._airborne_samples = 3
                self._grounded_samples = 0
            elif command == "land":
                self._flight_state = FLIGHT_LANDING
                self._airborne_samples = 0
                self._grounded_samples = 0
            elif command == "emergency":
                self._flight_state = FLIGHT_UNKNOWN
                self._airborne_samples = 0
                self._grounded_samples = 0

    # UI가 사용할 최신 표시값과 보수적인 착륙 확인 상태를 반환한다.
    def status(self) -> DesktopTelemetryStatus:
        current = self._clock()
        with self._lock:
            snapshot = self._snapshot
            safety_status = self._safety_status
            fresh = snapshot is not None and snapshot.is_fresh(
                current,
                self._freshness_seconds,
            )
            flight_state = self._flight_state if fresh else FLIGHT_UNKNOWN
            safety_fresh = bool(
                safety_status is not None and safety_status.is_fresh(current)
            )
            confirmed_grounded = bool(
                fresh
                and flight_state == FLIGHT_GROUNDED
                and safety_fresh
                and safety_status is not None
                and safety_status.grounded_confirmed
                and not safety_status.airborne
            )
            if safety_fresh and safety_status is not None:
                if safety_status.airborne:
                    flight_state = FLIGHT_AIRBORNE
                elif safety_status.state == "landing":
                    flight_state = FLIGHT_LANDING
                elif confirmed_grounded:
                    flight_state = FLIGHT_GROUNDED
                elif flight_state == FLIGHT_GROUNDED:
                    flight_state = FLIGHT_UNKNOWN
            elif flight_state == FLIGHT_GROUNDED:
                flight_state = FLIGHT_UNKNOWN
            return DesktopTelemetryStatus(
                connected=fresh,
                flight_state=flight_state,
                confirmed_grounded=confirmed_grounded,
                battery_percent=(snapshot.battery_percent if fresh else None),
                temperature_c=(snapshot.temperature_c if fresh else None),
                wifi_signal_percent=(snapshot.wifi_signal_percent if fresh else None),
            )

    # 수신 스레드를 멈춘 뒤 소켓을 닫는다.
    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)
        self._socket.close()
        self._thread = None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                payload, _ = self._socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if payload.lstrip().startswith(b"{"):
                safety_status = parse_desktop_safety_status(
                    payload,
                    received_at=self._clock(),
                )
                if safety_status is not None:
                    self._record_safety_status(safety_status)
                continue
            snapshot = parse_desktop_telemetry(
                payload,
                received_at=self._clock(),
            )
            if snapshot is not None:
                self._record_snapshot(snapshot)

    # safety session과 증가 sequence를 확인한 최신 상태만 보존한다.
    def _record_safety_status(self, status: DesktopFlightSafetyStatus) -> None:
        with self._lock:
            current = self._safety_status
            if (
                current is not None
                and current.session_id == status.session_id
                and status.sequence <= current.sequence
            ):
                return
            self._safety_status = status

    # 높이와 TOF를 연속 확인해 일시적인 0값을 착륙으로 오인하지 않는다.
    def _record_snapshot(self, snapshot: DesktopTelemetrySnapshot) -> None:
        altitude_values = [
            value
            for value in (snapshot.height_meters, snapshot.tof_meters)
            if value is not None
        ]
        with self._lock:
            self._snapshot = snapshot
            if not altitude_values:
                self._airborne_samples = 0
                self._grounded_samples = 0
                self._flight_state = FLIGHT_UNKNOWN
                return
            altitude = max(altitude_values)
            if altitude >= 0.20:
                self._airborne_samples += 1
                self._grounded_samples = 0
                if self._flight_state == FLIGHT_GROUNDED:
                    self._flight_state = FLIGHT_UNKNOWN
                if self._airborne_samples >= 3:
                    self._flight_state = FLIGHT_AIRBORNE
            elif altitude <= 0.10 and snapshot.received_at >= self._takeoff_grace_until:
                self._grounded_samples += 1
                self._airborne_samples = 0
                if self._grounded_samples >= 5:
                    self._flight_state = FLIGHT_GROUNDED
            else:
                self._airborne_samples = 0
                self._grounded_samples = 0
                if self._flight_state == FLIGHT_GROUNDED:
                    self._flight_state = FLIGHT_UNKNOWN
