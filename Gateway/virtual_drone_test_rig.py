from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
import math
import random
import socket
import sys
import threading
import time
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Iterable

from control_protocol import (
    ACK_STATUS_OK,
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
    COMMAND_LAND,
    COMMAND_TAKEOFF,
    ControlPacket,
    encode_ack,
)
from pose_protocol import (
    LOCAL_MAP_READY_FLAG,
    SESSION_MAP_FLAG,
    TVP2_PACKET,
    PosePacketV2,
    TrackingMode,
)


REPORT_SCHEMA_VERSION = 1
READY_TOKEN = "VIRTUAL_DRONE_TEST_RIG_READY"
DEFAULT_SEED = 20260811
DEFAULT_CHANNELS = ("state", "pose", "visual", "map")
SUPPORTED_CHANNELS = frozenset((*DEFAULT_CHANNELS, "measurement"))
MAP_REPEAT_SECONDS = 1.0
CONFIRMED_CONTROL_COMMANDS = frozenset(
    (COMMAND_HELLO, COMMAND_TAKEOFF, COMMAND_LAND, COMMAND_EMERGENCY)
)


# IPv4 loopback literal만 허용해 DNS와 실제 드론 주소 사용을 차단한다.
def normalize_loopback_host(host: str) -> str:
    value = str(host).strip().lower()
    if value == "localhost":
        return "127.0.0.1"
    try:
        address = ipaddress.ip_address(value)
    except ValueError as error:
        raise ValueError(
            "가상 테스트 장치는 localhost 또는 IPv4 loopback 주소만 허용합니다."
        ) from error
    if address.version != 4 or not address.is_loopback:
        raise ValueError(
            f"비-loopback 주소는 거부됩니다: {host}. 실제 Tello 주소로 송신할 수 없습니다."
        )
    return str(address)


# UDP 목적지 포트 범위를 검증한다.
def validate_target_port(port: int) -> int:
    value = int(port)
    if value < 1 or value > 65535:
        raise ValueError(f"UDP 목적지 포트가 올바르지 않습니다: {port}")
    return value


# UDP bind 포트 범위를 검증하며 0은 테스트용 자동 할당으로 허용한다.
def validate_bind_port(port: int) -> int:
    value = int(port)
    if value < 0 or value > 65535:
        raise ValueError(f"UDP bind 포트가 올바르지 않습니다: {port}")
    return value


# 수신 주소가 loopback인지 확인한다.
def is_loopback_sender(address: tuple[str, int]) -> bool:
    try:
        return ipaddress.ip_address(address[0]).is_loopback
    except ValueError:
        return False


@dataclass(frozen=True)
class LoopbackEndpoint:
    host: str
    port: int

    # endpoint 생성 시 실제 네트워크 주소를 거부한다.
    def __post_init__(self) -> None:
        object.__setattr__(self, "host", normalize_loopback_host(self.host))
        object.__setattr__(self, "port", validate_target_port(self.port))

    # socket.sendto가 사용할 튜플을 반환한다.
    def as_tuple(self) -> tuple[str, int]:
        return self.host, self.port


@dataclass(frozen=True)
class RigPorts:
    state: int = 8890
    pose: int = 49001
    measurement: int = 49003
    visual_pose: int = 49004
    map_command: int = 49300
    telemetry: int = 49301
    control_relay: int = 49200

    # 송신 포트와 bind 포트를 용도에 맞게 검증한다.
    def validate(self) -> None:
        for value in (
            self.state,
            self.pose,
            self.measurement,
            self.visual_pose,
            self.map_command,
        ):
            validate_target_port(value)
        validate_bind_port(self.telemetry)
        validate_bind_port(self.control_relay)


@dataclass(frozen=True)
class DropoutWindow:
    start_seconds: float
    end_seconds: float
    channels: frozenset[str]

    # 지정 채널이 현재 dropout 구간인지 확인한다.
    def contains(self, channel: str, elapsed_seconds: float) -> bool:
        return (
            channel in self.channels
            and self.start_seconds <= elapsed_seconds < self.end_seconds
        )


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    duration_seconds: float
    tick_hz: float
    dropout_windows: tuple[DropoutWindow, ...] = ()

    # 시나리오 값과 dropout 범위를 검증한다.
    def __post_init__(self) -> None:
        if self.duration_seconds <= 0.0 or self.tick_hz <= 0.0:
            raise ValueError("시나리오 duration과 tick rate는 양수여야 합니다.")
        for window in self.dropout_windows:
            if (
                window.start_seconds < 0.0
                or window.end_seconds <= window.start_seconds
                or window.end_seconds > self.duration_seconds
                or not window.channels <= SUPPORTED_CHANNELS
            ):
                raise ValueError("시나리오 dropout 범위가 올바르지 않습니다.")

    # 채널별 dropout 여부를 반환한다.
    def is_dropout(self, channel: str, elapsed_seconds: float) -> bool:
        return any(
            window.contains(channel, elapsed_seconds)
            for window in self.dropout_windows
        )

    # 실행할 고정 tick의 경과 시간을 생성한다.
    def timeline(self) -> tuple[float, ...]:
        count = max(1, int(math.ceil(self.duration_seconds * self.tick_hz)))
        return tuple(index / self.tick_hz for index in range(count))

    # JSON report용 시나리오 정보를 반환한다.
    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "durationSeconds": self.duration_seconds,
            "tickHz": self.tick_hz,
            "tickCount": len(self.timeline()),
            "dropoutWindows": [
                {
                    "startSeconds": item.start_seconds,
                    "endSeconds": item.end_seconds,
                    "channels": sorted(item.channels),
                }
                for item in self.dropout_windows
            ],
        }


# 이름과 선택적 override로 기본 시나리오를 만든다.
def build_scenario(
    name: str,
    *,
    duration_seconds: float | None = None,
    tick_hz: float = 10.0,
) -> ScenarioSpec:
    normalized = str(name).strip().lower()
    if normalized not in {"nominal", "dropout"}:
        raise ValueError(f"지원하지 않는 시나리오입니다: {name}")
    duration = float(duration_seconds) if duration_seconds is not None else (
        3.0 if normalized == "dropout" else 2.0
    )
    windows: tuple[DropoutWindow, ...] = ()
    if normalized == "dropout":
        windows = (
            DropoutWindow(
                duration / 3.0,
                duration * 2.0 / 3.0,
                frozenset(("state", "pose", "measurement", "visual")),
            ),
        )
    return ScenarioSpec(normalized, duration, float(tick_hz), windows)


class ScenarioClock:
    def __init__(
        self,
        *,
        monotonic: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
        counter_ns: Callable[[], int] = time.perf_counter_ns,
    ) -> None:
        self._monotonic = monotonic
        self._sleeper = sleeper
        self._counter_ns = counter_ns

    # 현재 monotonic 초를 반환한다.
    def now(self) -> float:
        return self._monotonic()

    # 지정 deadline까지 필요한 시간만 대기한다.
    def wait_until(self, deadline: float) -> None:
        remaining = deadline - self._monotonic()
        if remaining > 0.0:
            self._sleeper(remaining)

    # 동일 PC 계약에 사용할 monotonic nanosecond를 반환한다.
    def source_timestamp_ns(self) -> int:
        return max(1, int(self._counter_ns()))


@dataclass(frozen=True)
class VirtualSample:
    sequence: int
    source_timestamp_ns: int
    elapsed_seconds: float
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]
    velocity: tuple[float, float, float]
    pitch_degrees: float
    roll_degrees: float
    yaw_degrees: float
    height_centimeters: int
    tof_centimeters: int
    battery_percent: int
    flight_time_seconds: int

    # Tello SDK 상태 ASCII 패턴을 생성한다.
    def state_payload(self) -> bytes:
        velocity_cm = tuple(round(value * 100.0) for value in self.velocity)
        fields = (
            "mid:-1",
            "x:0",
            "y:0",
            "z:0",
            f"pitch:{self.pitch_degrees:.2f}",
            f"roll:{self.roll_degrees:.2f}",
            f"yaw:{self.yaw_degrees:.2f}",
            f"vgx:{velocity_cm[0]}",
            f"vgy:{velocity_cm[1]}",
            f"vgz:{velocity_cm[2]}",
            "templ:42",
            "temph:45",
            f"tof:{self.tof_centimeters}",
            f"h:{self.height_centimeters}",
            f"bat:{self.battery_percent}",
            f"baro:{self.position[1]:.2f}",
            f"time:{self.flight_time_seconds}",
            "agx:0.00",
            "agy:0.00",
            "agz:0.00",
        )
        return (";".join(fields) + ";").encode("ascii")

    # production PosePacketV2를 사용해 88-byte pose를 생성한다.
    def pose_packet(self) -> PosePacketV2:
        return PosePacketV2(
            sequence=self.sequence,
            source_timestamp_ns=self.source_timestamp_ns,
            position=self.position,
            rotation=self.rotation,
            velocity=self.velocity,
            position_stddev=(0.05, 0.05, 0.05),
            yaw_stddev=3.0,
            confidence=0.9,
            visual_anchor_count=64,
            detected_tag_count=0,
            mode=TrackingMode.VISUAL_TRACKED,
            flags=LOCAL_MAP_READY_FLAG | SESSION_MAP_FLAG,
        )

    # 49004 consumer가 읽는 visual pose JSON을 생성한다.
    def visual_pose_message(self) -> dict:
        return {
            "version": 1,
            "sequence": self.sequence,
            "sourceTimestampNs": self.source_timestamp_ns,
            "position": list(self.position),
            "rotation": list(self.rotation),
            "confidence": 0.9,
            "anchorCount": 64,
            "tagCount": 0,
            "reprojectionError": 0.8,
            "source": "anchor",
            "flags": LOCAL_MAP_READY_FLAG | SESSION_MAP_FLAG,
            "smoothed": True,
        }


class ScenarioGenerator:
    def __init__(
        self,
        seed: int,
        *,
        origin: tuple[float, float, float],
    ) -> None:
        self.seed = int(seed)
        self.origin = origin
        self._random = random.Random(self.seed)

    # seed 기반의 재현 가능한 정상 pose와 상태를 생성한다.
    def sample(
        self,
        sequence: int,
        elapsed_seconds: float,
        source_timestamp_ns: int,
    ) -> VirtualSample:
        jitter = self._random.uniform(-0.003, 0.003)
        yaw = elapsed_seconds * 8.0 + self._random.uniform(-0.1, 0.1)
        half_yaw = math.radians(yaw) * 0.5
        position = (
            self.origin[0] + elapsed_seconds * 0.15 + jitter,
            self.origin[1],
            self.origin[2] + math.sin(elapsed_seconds * 0.5) * 0.05,
        )
        return VirtualSample(
            sequence=sequence,
            source_timestamp_ns=source_timestamp_ns,
            elapsed_seconds=elapsed_seconds,
            position=position,
            rotation=(0.0, math.sin(half_yaw), 0.0, math.cos(half_yaw)),
            velocity=(0.15, 0.0, math.cos(elapsed_seconds * 0.5) * 0.025),
            pitch_degrees=math.sin(elapsed_seconds) * 2.0,
            roll_degrees=math.cos(elapsed_seconds) * 1.5,
            yaw_degrees=yaw,
            height_centimeters=5,
            tof_centimeters=5,
            battery_percent=max(0, 95 - int(elapsed_seconds // 30.0)),
            flight_time_seconds=int(elapsed_seconds),
        )


class LoopbackUdpEmitter:
    def __init__(
        self,
        endpoint: LoopbackEndpoint,
        *,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self.endpoint = endpoint
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        self.sent_count = 0
        self.sent_bytes = 0

    # payload을 loopback 목적지에만 송신한다.
    def send(self, payload: bytes) -> int:
        endpoint = LoopbackEndpoint(self.endpoint.host, self.endpoint.port)
        sent = self._socket.sendto(payload, endpoint.as_tuple())
        self.sent_count += 1
        self.sent_bytes += sent
        return sent

    # UDP socket을 닫는다.
    def close(self) -> None:
        self._socket.close()


class TelloStateEmitter:
    def __init__(self, endpoint: LoopbackEndpoint) -> None:
        self._emitter = LoopbackUdpEmitter(endpoint)

    # Tello ASCII 상태 한 프레임을 송신한다.
    def emit(self, sample: VirtualSample) -> int:
        return self._emitter.send(sample.state_payload())

    # 내부 송신 socket을 닫는다.
    def close(self) -> None:
        self._emitter.close()


class Tvp2PoseEmitter:
    def __init__(self, endpoint: LoopbackEndpoint) -> None:
        self._emitter = LoopbackUdpEmitter(endpoint)

    # production encoder로 TVP2 88-byte pose를 송신한다.
    def emit(self, sample: VirtualSample) -> int:
        payload = sample.pose_packet().encode()
        if len(payload) != TVP2_PACKET.size:
            raise RuntimeError("TVP2 packet 크기가 production 계약과 다릅니다.")
        return self._emitter.send(payload)

    # 내부 송신 socket을 닫는다.
    def close(self) -> None:
        self._emitter.close()


class VisualPoseEmitter:
    def __init__(self, endpoint: LoopbackEndpoint) -> None:
        self._emitter = LoopbackUdpEmitter(endpoint)

    # 49004용 UTF-8 visual pose JSON을 송신한다.
    def emit(self, sample: VirtualSample) -> int:
        payload = json.dumps(
            sample.visual_pose_message(),
            separators=(",", ":"),
        ).encode("utf-8")
        return self._emitter.send(payload)

    # 내부 송신 socket을 닫는다.
    def close(self) -> None:
        self._emitter.close()


# canonical map 파일과 이미지 해시를 포함한 map-state를 만든다.
def build_map_state(map_path: Path, public_base_url: str = "") -> dict:
    resolved = Path(map_path).resolve()
    try:
        data = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"venue map을 읽을 수 없습니다: {resolved}") from error
    required = ("mapId", "version", "scanId", "minX", "maxX", "minZ", "maxZ")
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError("venue map 필수 항목이 없습니다: " + ", ".join(missing))
    image_path = resolved.parent / Path(str(data.get("imagePath", "venue-map.png"))).name
    if not image_path.is_file():
        raise ValueError(f"venue map 이미지를 찾을 수 없습니다: {image_path}")
    base_url = public_base_url.rstrip("/")
    manifest_url = f"{base_url}/{resolved.name}" if base_url else f"/{resolved.name}"
    image_url = f"{base_url}/{image_path.name}" if base_url else f"/{image_path.name}"
    return {
        "type": "map-state",
        "mapId": str(data["mapId"]),
        "mapVersion": int(data["version"]),
        "scanId": str(data["scanId"]),
        "name": str(data.get("name", "")),
        "minX": float(data["minX"]),
        "maxX": float(data["maxX"]),
        "minZ": float(data["minZ"]),
        "maxZ": float(data["maxZ"]),
        "manifestUrl": manifest_url,
        "imageUrl": image_url,
        "manifestSha256": hashlib.sha256(resolved.read_bytes()).hexdigest(),
        "imageSha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
    }


class MapStateEmitter:
    def __init__(self, endpoint: LoopbackEndpoint, message: dict) -> None:
        self._emitter = LoopbackUdpEmitter(endpoint)
        self.message = dict(message)

    # Unity command 포트로 canonical map-state를 송신한다.
    def emit(self) -> int:
        payload = json.dumps(self.message, separators=(",", ":")).encode("utf-8")
        return self._emitter.send(payload)

    # 내부 송신 socket을 닫는다.
    def close(self) -> None:
        self._emitter.close()


class TelemetryCollector:
    def __init__(self, host: str = "127.0.0.1", port: int = 49301) -> None:
        self.host = normalize_loopback_host(host)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((self.host, validate_bind_port(port)))
        self._socket.settimeout(0.1)
        self.port = int(self._socket.getsockname()[1])
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._received = 0
        self._rejected = 0
        self._by_type: Counter[str] = Counter()
        self._last_message: dict | None = None

    # telemetry 수신 thread를 시작한다.
    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="VirtualDroneTelemetryCollector",
            daemon=True,
        )
        self._thread.start()

    # loopback JSON telemetry만 수집한다.
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                payload, sender = self._socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                message = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                message = None
            with self._lock:
                if not is_loopback_sender(sender) or not isinstance(message, dict):
                    self._rejected += 1
                    continue
                self._received += 1
                self._by_type[str(message.get("type") or "unknown")] += 1
                self._last_message = message

    # 현재 수집 결과의 복사본을 반환한다.
    def snapshot(self) -> dict:
        with self._lock:
            return {
                "enabled": True,
                "received": self._received,
                "rejected": self._rejected,
                "messagesByType": dict(sorted(self._by_type.items())),
                "lastMessage": self._last_message,
            }

    # telemetry socket과 thread를 종료한다.
    def close(self) -> None:
        self._stop.set()
        self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None


class UnityControlFakeRelay:
    def __init__(self, host: str = "127.0.0.1", port: int = 49200) -> None:
        self.host = normalize_loopback_host(host)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((self.host, validate_bind_port(port)))
        self._socket.settimeout(0.1)
        self.port = int(self._socket.getsockname()[1])
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._received = 0
        self._rejected = 0
        self._acks = 0
        self._last_packet: ControlPacket | None = None

    # Unity control 수신 thread를 시작한다.
    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="VirtualDroneControlRelay",
            daemon=True,
        )
        self._thread.start()

    # production control packet을 검증하고 confirmed 명령에 ACK한다.
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                payload, sender = self._socket.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                return
            packet = ControlPacket.decode(payload)
            valid = (
                is_loopback_sender(sender)
                and packet is not None
                and packet.flags == 0
                and 0 <= packet.command <= COMMAND_HELLO
            )
            if not valid:
                with self._lock:
                    self._rejected += 1
                continue
            with self._lock:
                self._received += 1
                self._last_packet = packet
            if packet.command in CONFIRMED_CONTROL_COMMANDS:
                self._send_ack(packet.sequence, sender)

    # ACK 목적지도 loopback인지 재검증한 뒤 응답한다.
    def _send_ack(self, sequence: int, sender: tuple[str, int]) -> None:
        if not is_loopback_sender(sender):
            raise ValueError("control ACK의 비-loopback 목적지는 거부됩니다.")
        payload = encode_ack(sequence, time.perf_counter_ns(), ACK_STATUS_OK)
        self._socket.sendto(payload, sender)
        with self._lock:
            self._acks += 1

    # 현재 control 수신 결과의 복사본을 반환한다.
    def snapshot(self) -> dict:
        with self._lock:
            packet = self._last_packet
            return {
                "enabled": True,
                "received": self._received,
                "rejected": self._rejected,
                "acksSent": self._acks,
                "lastPacket": (
                    None
                    if packet is None
                    else {
                        "sequence": packet.sequence,
                        "inputTicks": packet.input_ticks,
                        "axes": list(packet.axes),
                        "command": packet.command,
                        "flags": packet.flags,
                    }
                ),
            }

    # control socket과 thread를 종료한다.
    def close(self) -> None:
        self._stop.set()
        self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None


class VirtualDroneTestRig:
    def __init__(
        self,
        *,
        host: str,
        ports: RigPorts,
        scenario: ScenarioSpec,
        seed: int,
        map_path: Path,
        channels: Iterable[str] = DEFAULT_CHANNELS,
        enable_telemetry_collector: bool = True,
        enable_control_relay: bool = True,
        clock: ScenarioClock | None = None,
    ) -> None:
        self.host = normalize_loopback_host(host)
        ports.validate()
        self.ports = ports
        self.scenario = scenario
        self.seed = int(seed)
        self.map_path = Path(map_path).resolve()
        selected = tuple(dict.fromkeys(str(item).strip().lower() for item in channels))
        invalid = sorted(set(selected) - SUPPORTED_CHANNELS)
        if invalid or not selected:
            raise ValueError("지원하지 않는 channel입니다: " + ", ".join(invalid))
        self.channels = selected
        self.enable_telemetry_collector = bool(enable_telemetry_collector)
        self.enable_control_relay = bool(enable_control_relay)
        self.clock = clock or ScenarioClock()
        self.map_state = build_map_state(self.map_path)
        self._origin = (
            (self.map_state["minX"] + self.map_state["maxX"]) * 0.5,
            0.05,
            (self.map_state["minZ"] + self.map_state["maxZ"]) * 0.5,
        )

    # 포트·프로토콜·시나리오 구성을 소켓 없이 반환한다.
    def inspect(self) -> dict:
        return {
            "schemaVersion": REPORT_SCHEMA_VERSION,
            "readyToken": READY_TOKEN,
            "loopbackOnly": True,
            "host": self.host,
            "ports": {
                "state": self.ports.state,
                "pose": self.ports.pose,
                "measurement": self.ports.measurement,
                "visualPose": self.ports.visual_pose,
                "mapCommand": self.ports.map_command,
                "telemetry": self.ports.telemetry,
                "controlRelay": self.ports.control_relay,
            },
            "channels": list(self.channels),
            "scenario": self.scenario.to_dict(),
            "seed": self.seed,
            "map": {
                "path": str(self.map_path),
                "mapId": self.map_state["mapId"],
                "mapVersion": self.map_state["mapVersion"],
                "scanId": self.map_state["scanId"],
                "manifestSha256": self.map_state["manifestSha256"],
                "imageSha256": self.map_state["imageSha256"],
            },
            "protocols": {
                "state": "Tello ASCII key:value",
                "pose": f"TVP2/{TVP2_PACKET.size} bytes",
                "measurement": "UTF-8 JSON visual measurement v1",
                "visualPose": "UTF-8 JSON v1",
                "mapState": "UTF-8 JSON map-state",
                "control": "TCR1 v1 with confirmed-command ACK",
            },
            "telemetryCollectorEnabled": self.enable_telemetry_collector,
            "controlRelayEnabled": self.enable_control_relay,
        }

    # 실제 송신 또는 소켓 없는 dry-run을 수행하고 JSON report를 만든다.
    def run(self, *, dry_run: bool = False) -> dict:
        emitted: Counter[str] = Counter()
        dropped: Counter[str] = Counter()
        preview: list[dict] = []
        telemetry: TelemetryCollector | None = None
        control: UnityControlFakeRelay | None = None
        emitters: dict[str, object] = {}
        started_at = datetime.now(timezone.utc).isoformat()
        started_monotonic = self.clock.now()
        generator = ScenarioGenerator(self.seed, origin=self._origin)
        next_map_emit_seconds = MAP_REPEAT_SECONDS
        try:
            if not dry_run:
                telemetry, control, emitters = self._start_runtime_components()
                print(READY_TOKEN, file=sys.stderr, flush=True)
            if "map" in self.channels:
                if not dry_run:
                    map_emitter = emitters["map"]
                    assert isinstance(map_emitter, MapStateEmitter)
                    map_emitter.emit()
                emitted["map"] += 1

            for index, elapsed in enumerate(self.scenario.timeline(), start=1):
                if not dry_run:
                    self.clock.wait_until(started_monotonic + elapsed)
                    timestamp_ns = self.clock.source_timestamp_ns()
                else:
                    timestamp_ns = 1_000_000_000 + int(elapsed * 1_000_000_000)
                sample = generator.sample(index, elapsed, timestamp_ns)
                if "map" in self.channels and elapsed + 1e-9 >= next_map_emit_seconds:
                    if not dry_run:
                        map_emitter = emitters["map"]
                        assert isinstance(map_emitter, MapStateEmitter)
                        map_emitter.emit()
                    emitted["map"] += 1
                    while elapsed + 1e-9 >= next_map_emit_seconds:
                        next_map_emit_seconds += MAP_REPEAT_SECONDS
                if len(preview) < 3:
                    preview.append(
                        {
                            "sequence": sample.sequence,
                            "elapsedSeconds": sample.elapsed_seconds,
                            "position": list(sample.position),
                            "yawDegrees": sample.yaw_degrees,
                        }
                    )
                for channel in self.channels:
                    if channel == "map":
                        continue
                    if self.scenario.is_dropout(channel, elapsed):
                        dropped[channel] += 1
                        continue
                    if not dry_run:
                        emitter = emitters[channel]
                        emitter.emit(sample)
                    emitted[channel] += 1
            if not dry_run:
                time.sleep(0.05)
            telemetry_snapshot = (
                self._disabled_snapshot()
                if telemetry is None
                else telemetry.snapshot()
            )
            control_snapshot = (
                self._disabled_snapshot()
                if control is None
                else control.snapshot()
            )
            verification = self._build_verification(
                telemetry_snapshot,
                control_snapshot,
            )
            return {
                **self.inspect(),
                "success": True,
                "emissionComplete": True,
                "mode": "dry-run" if dry_run else "live",
                "startedAtUtc": started_at,
                "emitted": dict(sorted(emitted.items())),
                "dropped": dict(sorted(dropped.items())),
                "samplePreview": preview,
                "telemetry": telemetry_snapshot,
                "control": control_snapshot,
                "verification": verification,
            }
        finally:
            for emitter in emitters.values():
                emitter.close()
            if telemetry is not None:
                telemetry.close()
            if control is not None:
                control.close()

    # live 모드에서 필요한 수신기와 송신기를 시작한다.
    def _start_runtime_components(
        self,
    ) -> tuple[TelemetryCollector | None, UnityControlFakeRelay | None, dict[str, object]]:
        telemetry = (
            TelemetryCollector(self.host, self.ports.telemetry)
            if self.enable_telemetry_collector
            else None
        )
        control = (
            UnityControlFakeRelay(self.host, self.ports.control_relay)
            if self.enable_control_relay
            else None
        )
        emitters: dict[str, object] = {}
        try:
            if telemetry is not None:
                telemetry.start()
            if control is not None:
                control.start()
            if "state" in self.channels:
                emitters["state"] = TelloStateEmitter(
                    LoopbackEndpoint(self.host, self.ports.state)
                )
            if "pose" in self.channels:
                emitters["pose"] = Tvp2PoseEmitter(
                    LoopbackEndpoint(self.host, self.ports.pose)
                )
            if "measurement" in self.channels:
                emitters["measurement"] = VisualPoseEmitter(
                    LoopbackEndpoint(self.host, self.ports.measurement)
                )
            if "visual" in self.channels:
                emitters["visual"] = VisualPoseEmitter(
                    LoopbackEndpoint(self.host, self.ports.visual_pose)
                )
            if "map" in self.channels:
                emitters["map"] = MapStateEmitter(
                    LoopbackEndpoint(self.host, self.ports.map_command),
                    self.map_state,
                )
            return telemetry, control, emitters
        except Exception:
            for emitter in emitters.values():
                emitter.close()
            if telemetry is not None:
                telemetry.close()
            if control is not None:
                control.close()
            raise

    # 비활성 또는 dry-run 수집기의 공통 결과를 반환한다.
    @staticmethod
    def _disabled_snapshot() -> dict:
        return {
            "received": 0,
            "rejected": 0,
            "enabled": False,
        }

    # 요청된 Unity 회신 증거와 단순 송신 완료를 구분한다.
    @staticmethod
    def _build_verification(telemetry: dict, control: dict) -> dict:
        checks = {
            "telemetryReceived": (
                None if not telemetry.get("enabled") else telemetry.get("received", 0) > 0
            ),
            "controlReceived": (
                None if not control.get("enabled") else control.get("received", 0) > 0
            ),
        }
        requested = [value for value in checks.values() if value is not None]
        if not requested:
            return {
                "status": "not-requested",
                "passed": None,
                "checks": checks,
            }
        passed = all(requested)
        return {
            "status": "passed" if passed else "inconclusive",
            "passed": passed,
            "checks": checks,
        }


# 쉼표 구분 channel 목록을 검증해 반환한다.
def parse_channels(value: str) -> tuple[str, ...]:
    result = tuple(item.strip().lower() for item in value.split(",") if item.strip())
    invalid = sorted(set(result) - SUPPORTED_CHANNELS)
    if not result or invalid:
        raise argparse.ArgumentTypeError(
            "channels는 state,pose,measurement,visual,map 중 하나 이상이어야 합니다."
        )
    return tuple(dict.fromkeys(result))


# 양수 실수 CLI 값을 검증한다.
def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("0보다 큰 값이 필요합니다.")
    return parsed


# CLI 인수를 읽는다.
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description="StrikerPC loopback-only Virtual Drone Test Rig",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--scenario", choices=("nominal", "dropout"), default="nominal")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--duration", type=positive_float)
    parser.add_argument("--tick-hz", type=positive_float, default=10.0)
    parser.add_argument("--channels", type=parse_channels, default=DEFAULT_CHANNELS)
    parser.add_argument(
        "--map-path",
        type=Path,
        default=root / "Assets" / "Maps" / "current" / "venue-map.json",
    )
    parser.add_argument("--state-port", type=int, default=8890)
    parser.add_argument("--pose-port", type=int, default=49001)
    parser.add_argument("--measurement-port", type=int, default=49003)
    parser.add_argument("--visual-pose-port", type=int, default=49004)
    parser.add_argument("--map-command-port", type=int, default=49300)
    parser.add_argument("--telemetry-port", type=int, default=49301)
    parser.add_argument("--control-relay-port", type=int, default=49200)
    parser.add_argument(
        "--telemetry-collector",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--control-relay",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--inspect", action="store_true")
    parser.add_argument("--require-unity-evidence", action="store_true")
    parser.add_argument("--report", type=Path)
    return parser.parse_args(argv)


# report를 stdout과 선택적 JSON 파일에 기록한다.
def write_report(report: dict, path: Path | None) -> None:
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if path is not None:
        resolved = Path(path).resolve()
        resolved.parent.mkdir(parents=True, exist_ok=True)
        resolved.write_text(text, encoding="utf-8")
    print(text, end="")


# CLI 요청을 실행하고 성공 여부를 exit code로 반환한다.
def main(argv: list[str] | None = None) -> int:
    arguments = parse_args(argv)
    try:
        scenario = build_scenario(
            arguments.scenario,
            duration_seconds=arguments.duration,
            tick_hz=arguments.tick_hz,
        )
        rig = VirtualDroneTestRig(
            host=arguments.host,
            ports=RigPorts(
                state=arguments.state_port,
                pose=arguments.pose_port,
                measurement=arguments.measurement_port,
                visual_pose=arguments.visual_pose_port,
                map_command=arguments.map_command_port,
                telemetry=arguments.telemetry_port,
                control_relay=arguments.control_relay_port,
            ),
            scenario=scenario,
            seed=arguments.seed,
            map_path=arguments.map_path,
            channels=arguments.channels,
            enable_telemetry_collector=arguments.telemetry_collector,
            enable_control_relay=arguments.control_relay,
        )
        if arguments.inspect:
            report = {
                **rig.inspect(),
                "success": True,
                "mode": "inspect",
            }
        else:
            report = rig.run(dry_run=arguments.dry_run)
        write_report(report, arguments.report)
        if (
            arguments.require_unity_evidence
            and report.get("mode") == "live"
            and (report.get("verification") or {}).get("passed") is not True
        ):
            return 3
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        report = {
            "schemaVersion": REPORT_SCHEMA_VERSION,
            "success": False,
            "mode": "inspect" if arguments.inspect else "dry-run" if arguments.dry_run else "live",
            "error": str(error),
        }
        write_report(report, arguments.report)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
