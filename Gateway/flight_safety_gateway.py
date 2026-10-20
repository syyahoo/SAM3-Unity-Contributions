from __future__ import annotations

import json
import logging
import math
import select
import socket
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Callable


log = logging.getLogger(__name__)

SCHEMA_VERSION = 1
HEARTBEAT_TYPE = "unity-heartbeat"
SAFETY_STATE_TYPE = "flight-safety-state"
STATUS_PUBLISH_INTERVAL_SECONDS = 0.2


@dataclass(frozen=True)
class TelloStateSnapshot:
    received_at: float
    battery_percent: int | None = None
    height_meters: float | None = None
    tof_meters: float | None = None
    temperature_min_c: float | None = None
    temperature_max_c: float | None = None
    flight_time_seconds: int | None = None
    wifi_signal_percent: int | None = None
    pitch_degrees: float | None = None
    roll_degrees: float | None = None
    yaw_degrees: float | None = None

    def is_fresh(self, now: float, maximum_age_seconds: float = 0.5) -> bool:
        return 0.0 <= now - self.received_at <= maximum_age_seconds

    def upstream_telemetry(self, now: float) -> dict[str, int | float | bool]:
        fresh = self.is_fresh(now)
        result: dict[str, int | float | bool] = {"connected": fresh}
        if not fresh:
            return result
        if self.battery_percent is not None:
            result["battery"] = self.battery_percent
        if self.height_meters is not None:
            result["height"] = round(self.height_meters * 100.0)
        temperatures = [
            value
            for value in (self.temperature_min_c, self.temperature_max_c)
            if value is not None
        ]
        if temperatures:
            result["temperature"] = round(sum(temperatures) / len(temperatures), 1)
        if self.flight_time_seconds is not None:
            result["flight_time"] = self.flight_time_seconds
        if self.wifi_signal_percent is not None:
            result["wifi"] = self.wifi_signal_percent
        return result


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


# Tello state UDP의 세미콜론 필드를 안전 계층용 단위로 변환한다.
def parse_tello_state_snapshot(
    payload: bytes,
    *,
    received_at: float | None = None,
) -> TelloStateSnapshot | None:
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

    height_cm = _finite_number(values, "h")
    tof_cm = _finite_number(values, "tof")
    return TelloStateSnapshot(
        received_at=time.monotonic() if received_at is None else received_at,
        battery_percent=_bounded_integer(values, "bat", 0, 100),
        height_meters=(
            None if height_cm is None or height_cm < 0.0 else height_cm / 100.0
        ),
        tof_meters=None if tof_cm is None or tof_cm < 0.0 else tof_cm / 100.0,
        temperature_min_c=_finite_number(values, "templ"),
        temperature_max_c=_finite_number(values, "temph"),
        flight_time_seconds=_bounded_integer(values, "time", 0, 86400),
        wifi_signal_percent=_bounded_integer(values, "wifi", 0, 100),
        pitch_degrees=_finite_number(values, "pitch"),
        roll_degrees=_finite_number(values, "roll"),
        yaw_degrees=_finite_number(values, "yaw"),
    )


class FlightSafetyController:
    """Heartbeat와 비행 상태를 순수 상태 기계로 판정한다."""

    def __init__(
        self,
        *,
        clock: Callable[[], float] = time.monotonic,
        neutral_after_seconds: float = 2.0,
        land_after_seconds: float = 30.0,
        recovery_heartbeats: int = 3,
    ) -> None:
        self._clock = clock
        self.neutral_after_seconds = neutral_after_seconds
        self.land_after_seconds = land_after_seconds
        self.recovery_heartbeats = recovery_heartbeats
        self.session_id = str(uuid.uuid4())
        self.airborne = False
        self.grounded_confirmed = False
        self.force_neutral = False
        self.control_connected = False
        self.heartbeat_seen = False
        self.heartbeat_required = True
        self.last_heartbeat_at: float | None = None
        self.last_healthy_at: float | None = None
        self.last_state: TelloStateSnapshot | None = None
        self.state_name = "waiting-heartbeat"
        self.land_attempts = 0
        self.last_error = ""
        self._heartbeat_session = ""
        self._heartbeat_sequence = -1
        self._healthy_streak = 0
        self._airborne_samples = 0
        self._grounded_samples = 0
        self._airborne_since: float | None = None
        self._heartbeat_required_since: float | None = None
        self._takeoff_grace_until = 0.0
        self._land_requested = False
        self._landing_acknowledged = False
        self._status_sequence = 0

    # Unity heartbeat의 session/sequence를 검사하고 liveness를 갱신한다.
    def record_heartbeat(self, message: dict, *, now: float | None = None) -> bool:
        current = self._clock() if now is None else now
        try:
            schema_version = message["schemaVersion"]
            session = message["sessionId"]
            sequence = message["sequence"]
            control_connected = message["controlConnected"]
        except KeyError:
            return False
        if (
            isinstance(schema_version, bool)
            or not isinstance(schema_version, int)
            or schema_version != SCHEMA_VERSION
            or message.get("type") != HEARTBEAT_TYPE
            or not isinstance(session, str)
            or not session
            or len(session) > 128
            or isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or sequence < 0
            or not isinstance(control_connected, bool)
        ):
            return False
        if session != self._heartbeat_session:
            self._heartbeat_session = session
            self._heartbeat_sequence = -1
            self._healthy_streak = 0
        if sequence <= self._heartbeat_sequence:
            return False

        self._heartbeat_sequence = sequence
        self.heartbeat_seen = True
        self.last_heartbeat_at = current
        self.last_healthy_at = current
        self.control_connected = control_connected
        self._healthy_streak += 1
        if (
            self.heartbeat_required
            and self.force_neutral
            and not self._land_requested
            and self._healthy_streak >= self.recovery_heartbeats
        ):
            self.force_neutral = False
            self.state_name = "healthy" if self.airborne else "grounded"
            self.last_error = ""
        return True

    # 현재 조종 권한에 따라 Unity heartbeat 착륙 감시를 켜거나 끈다.
    def set_heartbeat_required(
        self,
        required: bool,
        *,
        now: float | None = None,
    ) -> bool:
        if required == self.heartbeat_required:
            return False
        current = self._clock() if now is None else now
        self.heartbeat_required = required
        self._healthy_streak = 0
        if required:
            self._heartbeat_required_since = current
            if self.airborne and not self._land_requested:
                self.state_name = (
                    "healthy" if self._is_heartbeat_fresh(current) else "watchdog-grace"
                )
        else:
            self._heartbeat_required_since = None
            if not self._land_requested:
                self.force_neutral = False
                self.last_error = ""
                self.state_name = "healthy" if self.airborne else "grounded"
        return True

    # Tello 링크 복구 후 이전 단절 시간을 착륙 타이머로 이어 쓰지 않는다.
    def record_link_recovered(self, *, now: float | None = None) -> None:
        current = self._clock() if now is None else now
        self._healthy_streak = 0
        self._land_requested = False
        self._landing_acknowledged = False
        self._heartbeat_required_since = current
        if self.airborne:
            self.force_neutral = True
            self.state_name = "link-recovery-neutral"
        self.last_error = ""

    # Fresh state의 높이를 debounce하여 재시작 시 airborne 상태를 복구한다.
    def record_state(self, state: TelloStateSnapshot, *, now: float | None = None) -> None:
        current = self._clock() if now is None else now
        self.last_state = state
        if not state.is_fresh(current):
            self.grounded_confirmed = False
            return
        altitude = (
            state.height_meters
            if state.height_meters is not None
            else state.tof_meters
        )
        if altitude is None:
            self._airborne_samples = 0
            self._grounded_samples = 0
            self.grounded_confirmed = False
            return

        if self._landing_acknowledged:
            self.airborne = False
            self._airborne_samples = 0
            if altitude <= 0.10:
                self._grounded_samples += 1
                if self._grounded_samples >= 5:
                    self.grounded_confirmed = True
                    self.state_name = "grounded"
            else:
                self._grounded_samples = 0
                self.grounded_confirmed = False
            return

        if altitude > 0.10:
            self.grounded_confirmed = False
        if altitude >= 0.20:
            self._airborne_samples += 1
            self._grounded_samples = 0
            if self._airborne_samples >= 3:
                if not self.airborne:
                    self._airborne_since = current
                self.airborne = True
                if self.state_name in {"grounded", "waiting-heartbeat"}:
                    self.state_name = "healthy" if self._is_healthy(current) else "unhealthy"
        elif altitude <= 0.10 and current >= self._takeoff_grace_until:
            self._grounded_samples += 1
            self._airborne_samples = 0
            if self._grounded_samples >= 5:
                self.airborne = False
                self.grounded_confirmed = True
                self._airborne_since = None
                self.force_neutral = False
                self._land_requested = False
                self.state_name = "grounded"
        else:
            self._airborne_samples = 0
            self._grounded_samples = 0

    # 실제 Tello 명령 ACK를 airborne lifecycle에 반영한다.
    def record_action(self, command: str, success: bool, *, now: float | None = None) -> None:
        if not success:
            return
        current = self._clock() if now is None else now
        if command == "takeoff":
            self.airborne = True
            self.grounded_confirmed = False
            self._airborne_since = current
            self._takeoff_grace_until = current + 3.0
            self._landing_acknowledged = False
            self._land_requested = False
            self.force_neutral = False
            self._grounded_samples = 0
            self.land_attempts = 0
            self.last_error = ""
            self.state_name = "healthy" if self._is_healthy(current) else "unhealthy"
        elif command in {"land", "emergency"}:
            self.airborne = False
            self.grounded_confirmed = False
            self._airborne_since = None
            self._airborne_samples = 0
            self._grounded_samples = 0
            self.force_neutral = False
            self._land_requested = True
            self._landing_acknowledged = True
            self.state_name = "grounded" if command == "land" else "emergency"

    # 현재 시간의 safety action을 neutral/land 순서로 한 번씩 반환한다.
    def poll(self, *, now: float | None = None) -> tuple[str, ...]:
        current = self._clock() if now is None else now
        if not self.airborne:
            return ()
        if not self.heartbeat_required:
            if not self.force_neutral and not self._land_requested:
                self.state_name = "healthy"
            return ()
        if self._is_healthy(current):
            if not self.force_neutral and not self._land_requested:
                self.state_name = "healthy"
            return ()

        safety_anchors = [
            value
            for value in (
                self.last_healthy_at,
                self.last_heartbeat_at,
                self._airborne_since,
                self._heartbeat_required_since,
            )
            if value is not None
        ]
        last_healthy = max(safety_anchors) if safety_anchors else None
        unhealthy_for = current - last_healthy if last_healthy is not None else 0.0
        actions: list[str] = []
        if unhealthy_for >= self.neutral_after_seconds and not self.force_neutral:
            self.force_neutral = True
            self._healthy_streak = 0
            self.state_name = "neutral"
            actions.append("neutral")
        if unhealthy_for >= self.land_after_seconds and not self._land_requested:
            tello_link_fresh = bool(
                self.last_state is not None and self.last_state.is_fresh(current)
            )
            if tello_link_fresh:
                self._land_requested = True
                self.force_neutral = True
                self.state_name = "landing"
                actions.append("land")
            else:
                self.force_neutral = True
                self.state_name = "link-lost-neutral"
        return tuple(actions)

    def record_land_attempt(self, attempt: int, success: bool, error: str = "") -> None:
        self.land_attempts = attempt
        self.last_error = error
        if success:
            self.airborne = False
            self.grounded_confirmed = False
            self._airborne_since = None
            self._airborne_samples = 0
            self._grounded_samples = 0
            self.force_neutral = False
            self._land_requested = True
            self._landing_acknowledged = True
            self.state_name = "landed"
        elif attempt >= 3:
            self.force_neutral = True
            self.state_name = "land-failed"

    def status_message(self, *, now: float | None = None) -> dict:
        current = self._clock() if now is None else now
        self._status_sequence += 1
        heartbeat_age = (
            None
            if self.last_heartbeat_at is None
            else max(0, round((current - self.last_heartbeat_at) * 1000))
        )
        state_age = (
            None
            if self.last_state is None
            else max(0, round((current - self.last_state.received_at) * 1000))
        )
        grounded_confirmed = bool(
            self.grounded_confirmed
            and self.last_state is not None
            and self.last_state.is_fresh(current)
        )
        return {
            "schemaVersion": SCHEMA_VERSION,
            "type": SAFETY_STATE_TYPE,
            "sessionId": self.session_id,
            "sequence": self._status_sequence,
            "state": self.state_name,
            "airborne": self.airborne,
            "groundedConfirmed": grounded_confirmed,
            "forceNeutral": self.force_neutral,
            "controlConnected": self.control_connected,
            "heartbeatRequired": self.heartbeat_required,
            "heartbeatFresh": self._is_heartbeat_fresh(current),
            "heartbeatAgeMs": heartbeat_age,
            "heartbeatSessionId": self._heartbeat_session,
            "heartbeatSequence": self._heartbeat_sequence,
            "stateAgeMs": state_age,
            "landAttempts": self.land_attempts,
            "lastError": self.last_error,
            "timestamp": int(time.time() * 1000),
        }

    def telemetry(self, *, now: float | None = None) -> dict[str, int | float | bool]:
        current = self._clock() if now is None else now
        if self.last_state is None:
            return {"connected": False}
        return self.last_state.upstream_telemetry(current)

    def _is_healthy(self, now: float) -> bool:
        return not self.heartbeat_required or self._is_heartbeat_fresh(now)

    def _is_heartbeat_fresh(self, now: float) -> bool:
        return (
            self.last_heartbeat_at is not None
            and now - self.last_heartbeat_at <= self.neutral_after_seconds
        )


class FlightSafetyGateway:
    """Tello state와 Unity heartbeat를 Pilot 프로세스에서 감시한다."""

    def __init__(
        self,
        *,
        state_port: int = 8892,
        heartbeat_port: int = 49304,
        operation_event_port: int = 49301,
        desktop_state_port: int | None = None,
        additional_status_ports: tuple[int, ...] = (),
        drone_id: str = "tello-publisher",
        mission_id: str = "mission-alpha",
        controller: FlightSafetyController | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.controller = controller or FlightSafetyController(clock=clock)
        self._clock = clock
        self._drone_id = drone_id
        self._mission_id = mission_id
        status_ports = [operation_event_port]
        if desktop_state_port is not None:
            status_ports.append(desktop_state_port)
        status_ports.extend(additional_status_ports)
        if any(port <= 0 or port > 65535 for port in status_ports):
            raise ValueError("상태 전송 포트는 1~65535 범위여야 합니다.")
        self._state_socket = self._bind_udp(state_port)
        self._heartbeat_socket = self._bind_udp(heartbeat_port)
        self._status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._status_targets = tuple(
            ("127.0.0.1", port) for port in dict.fromkeys(status_ports)
        )
        self._adapter = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._land_thread: threading.Thread | None = None
        self._status_key: tuple | None = None
        self._last_status_at = 0.0

    @staticmethod
    def _bind_udp(port: int) -> socket.socket:
        current = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        current.bind(("127.0.0.1", port))
        current.setblocking(False)
        return current

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="flight-safety-gateway",
            daemon=True,
        )
        self._thread.start()

    def attach_adapter(self, adapter) -> None:
        self._adapter = adapter

    def telemetry(self) -> dict[str, int | float | bool]:
        return self.controller.telemetry(now=self._clock())

    def record_action(self, command: str, success: bool) -> None:
        self.controller.record_action(command, success, now=self._clock())
        self._publish_status(force=True)

    def set_unity_heartbeat_required(self, required: bool) -> None:
        if not self.controller.set_heartbeat_required(required, now=self._clock()):
            return
        log.info(
            "Unity heartbeat watchdog required=%s controlConnected=%s",
            required,
            self.controller.control_connected,
        )
        self._publish_status(force=True)

    # 명령 소켓 재연결 후 중립 복구 구간과 새 30초 착륙 유예를 시작한다.
    def record_link_recovered(self) -> None:
        self.controller.record_link_recovered(now=self._clock())
        log.warning("Tello link recovered; holding neutral until fresh control resumes")
        self._publish_status(force=True)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._land_thread is not None and self._land_thread.is_alive():
            self._land_thread.join(timeout=10.0)
        self._publish_status(force=True)
        self._state_socket.close()
        self._heartbeat_socket.close()
        self._status_socket.close()

    def controlled_land_now(self) -> bool:
        if not self.controller.airborne:
            return True
        land_thread = self._land_thread
        if land_thread is not None and land_thread.is_alive():
            if land_thread is not threading.current_thread():
                land_thread.join(timeout=10.0)
            return not self.controller.airborne
        self.controller.force_neutral = True
        self.controller.state_name = "landing"
        self._send_neutral()
        return self._perform_land_attempts()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                readable, _, _ = select.select(
                    [self._state_socket, self._heartbeat_socket],
                    [],
                    [],
                    0.05,
                )
            except (OSError, ValueError):
                break
            for current in readable:
                try:
                    payload, _ = current.recvfrom(65535)
                except (BlockingIOError, OSError):
                    continue
                if current is self._state_socket:
                    state = parse_tello_state_snapshot(payload, received_at=self._clock())
                    if state is not None:
                        self.controller.record_state(state, now=self._clock())
                else:
                    try:
                        message = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if isinstance(message, dict):
                        self.controller.record_heartbeat(message, now=self._clock())

            actions = self.controller.poll(now=self._clock())
            if "neutral" in actions:
                self._log_watchdog_transition(
                    "Unity heartbeat lost while airborne; forcing neutral RC"
                )
                self._send_neutral()
            if "land" in actions and (
                self._land_thread is None or not self._land_thread.is_alive()
            ):
                self._log_watchdog_transition(
                    "Unity heartbeat remained unavailable; starting controlled landing",
                    error=True,
                )
                self._land_thread = threading.Thread(
                    target=self._perform_land_attempts,
                    name="flight-safety-land",
                    daemon=True,
                )
                self._land_thread.start()
            self._publish_status()

    def _log_watchdog_transition(self, message: str, *, error: bool = False) -> None:
        status = self.controller.status_message(now=self._clock())
        state = self.controller.last_state
        logger = log.error if error else log.warning
        logger(
            "%s heartbeatAgeMs=%s controlConnected=%s heartbeatSession=%s "
            "heartbeatSequence=%s battery=%s heightM=%s tofM=%s stateAgeMs=%s",
            message,
            status["heartbeatAgeMs"],
            status["controlConnected"],
            status["heartbeatSessionId"],
            status["heartbeatSequence"],
            None if state is None else state.battery_percent,
            None if state is None else state.height_meters,
            None if state is None else state.tof_meters,
            status["stateAgeMs"],
        )

    def _perform_land_attempts(self) -> bool:
        adapter = self._adapter
        if adapter is None:
            self.controller.record_land_attempt(3, False, "Tello adapter unavailable")
            self._publish_status(force=True)
            return False
        for attempt in range(1, 4):
            if not self.controller.airborne:
                return True
            try:
                success = bool(adapter.strikerpc_safety_land(timeout_seconds=3.0))
                error = "" if success else "Tello land ACK failed"
            except Exception as exception:
                success = False
                error = str(exception)
            self.controller.record_land_attempt(attempt, success, error)
            self._publish_status(force=True)
            if success:
                log.warning("Controlled landing acknowledged on attempt %d", attempt)
                return True
            if attempt < 3:
                time.sleep(0.25)
        log.error("Controlled landing failed after 3 attempts; neutral RC remains forced")
        return False

    # Safety 상태 전환 시 다음 프레임을 기다리지 않고 즉시 중립 RC를 보낸다.
    def _send_neutral(self) -> bool:
        adapter = self._adapter
        if adapter is None:
            log.error("Cannot send safety neutral RC: Tello adapter unavailable")
            return False
        try:
            return bool(adapter.strikerpc_safety_neutral())
        except Exception as exception:
            log.warning("Tello safety neutral failed: %s", exception)
            return False

    def _publish_status(self, *, force: bool = False) -> None:
        now = self._clock()
        key = (
            self.controller.state_name,
            self.controller.airborne,
            self.controller.grounded_confirmed,
            self.controller.force_neutral,
            self.controller.control_connected,
            self.controller.heartbeat_required,
            self.controller.land_attempts,
            self.controller.last_error,
        )
        if (
            not force
            and key == self._status_key
            and now - self._last_status_at < STATUS_PUBLISH_INTERVAL_SECONDS
        ):
            return
        message = self.controller.status_message(now=now)
        message["missionId"] = self._mission_id
        message["droneId"] = self._drone_id
        payload = json.dumps(message, separators=(",", ":")).encode("utf-8")
        for target in self._status_targets:
            try:
                self._status_socket.sendto(payload, target)
            except OSError:
                pass
        self._status_key = key
        self._last_status_at = now
