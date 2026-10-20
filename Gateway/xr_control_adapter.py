from __future__ import annotations

import argparse
import ctypes
import logging
import select
import socket
import sys
import time

from control_protocol import (
    ACK_PACKET,
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
    COMMAND_LAND,
    COMMAND_NEUTRAL,
    COMMAND_RC,
    COMMAND_TAKEOFF,
    CONTROL_MAGIC,
    CONTROL_VERSION,
    INTERNAL_CONTROL_FLAG,
    ControlPacket,
    encode_ack,
    encode_control_packet,
)


logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

WINDOWS_UDP_CONNECTION_RESET = 10054

SUPPORTED_COMMANDS = {
    COMMAND_RC,
    COMMAND_NEUTRAL,
    COMMAND_TAKEOFF,
    COMMAND_LAND,
    COMMAND_EMERGENCY,
    COMMAND_HELLO,
}
CONFIRMED_COMMANDS = {
    COMMAND_TAKEOFF,
    COMMAND_LAND,
    COMMAND_EMERGENCY,
}


# Windows UDP의 닫힌 상대 포트 알림인지 판별한다.
def is_udp_connection_reset(error: OSError) -> bool:
    return (
        isinstance(error, ConnectionResetError)
        or getattr(error, "winerror", None) == WINDOWS_UDP_CONNECTION_RESET
    )


# Windows Stopwatch와 같은 성능 카운터 tick을 반환한다.
def query_performance_counter() -> int:
    if sys.platform == "win32":
        value = ctypes.c_longlong()
        ctypes.windll.kernel32.QueryPerformanceCounter(ctypes.byref(value))
        return value.value
    return time.perf_counter_ns()


# Windows Stopwatch와 같은 성능 카운터 초당 tick 수를 반환한다.
def query_performance_frequency() -> int:
    if sys.platform == "win32":
        value = ctypes.c_longlong()
        if not ctypes.windll.kernel32.QueryPerformanceFrequency(ctypes.byref(value)):
            raise OSError("QueryPerformanceFrequency failed")
        return value.value
    return 1_000_000_000


class XrControlAdapter:
    def __init__(
        self,
        *,
        relay_port: int = 49200,
        authority_port: int = 49199,
        pilot_port: int = 49210,
        watchdog_seconds: float = 0.2,
        pilot_timeout: float = 8.0,
    ) -> None:
        self._pilot_target = ("127.0.0.1", pilot_port)
        self._watchdog_seconds = watchdog_seconds
        self._pilot_timeout = pilot_timeout
        self._last_sequence = -1
        self._last_unity_packet_at = time.monotonic()
        self._watchdog_sent = True
        self._unity_sender: tuple[str, int] | None = None
        self._pending_confirmed: tuple[int, tuple[str, int], float] | None = None
        self._closed = False

        self._unity_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._unity_socket.bind(("127.0.0.1", relay_port))
        self._unity_socket.setblocking(False)

        self._pilot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pilot_socket.bind(("127.0.0.1", 0))
        self._pilot_socket.setblocking(False)

        self._authority_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._authority_socket.bind(("127.0.0.1", authority_port))
            self._authority_socket.listen(1)
        except Exception:
            self._unity_socket.close()
            self._pilot_socket.close()
            self._authority_socket.close()
            raise

    @property
    def unity_address(self) -> tuple[str, int]:
        return self._unity_socket.getsockname()

    @property
    def pilot_address(self) -> tuple[str, int]:
        return self._pilot_socket.getsockname()

    # Pilot 입력 호스트가 준비되었는지 HELLO/ACK로 확인한다.
    def connect_pilot(self) -> None:
        hello = encode_control_packet(
            ControlPacket(
                sequence=0,
                input_ticks=query_performance_counter(),
                axes=(0, 0, 0, 0),
                command=COMMAND_HELLO,
            )
        )
        deadline = time.monotonic() + self._pilot_timeout
        while time.monotonic() < deadline:
            self._pilot_socket.sendto(hello, self._pilot_target)
            remaining = max(0.0, min(0.25, deadline - time.monotonic()))
            readable, _, _ = select.select([self._pilot_socket], [], [], remaining)
            if not readable:
                continue
            try:
                payload, _ = self._pilot_socket.recvfrom(256)
            except OSError as error:
                if not is_udp_connection_reset(error):
                    raise
                logging.warning(
                    "Pilot UDP reset during handshake; retrying: %s",
                    error,
                )
                continue
            if self._valid_ack(payload, 0):
                logging.info("Pilot control IPC connected: %s", self._pilot_target)
                return
        raise TimeoutError(f"Pilot control IPC did not respond: {self._pilot_target}")

    # Unity와 Pilot 사이의 조종 패킷을 중계한다.
    def run(self) -> None:
        self.connect_pilot()
        print("STRIKERPC_READY XR_ADAPTER", flush=True)
        logging.info("XR Adapter ready. Start Unity Play now.")

        try:
            while True:
                readable, _, _ = select.select(
                    [self._unity_socket, self._pilot_socket],
                    [],
                    [],
                    0.02,
                )
                for current in readable:
                    received = self._receive_udp_packet(current)
                    if received is None:
                        continue
                    payload, sender = received
                    if current is self._unity_socket:
                        self.handle_unity_packet(payload, sender)
                    else:
                        self.handle_pilot_ack(payload)
                self.check_timeouts()
        finally:
            self.close()

    # Unity 패킷을 검증하고 Pilot으로 전달한다.
    def handle_unity_packet(self, payload: bytes, sender: tuple[str, int]) -> None:
        packet = ControlPacket.decode(payload)
        if (
            packet is None
            or packet.flags != 0
            or packet.command not in SUPPORTED_COMMANDS
        ):
            return

        if packet.command == COMMAND_HELLO:
            pending = self._pending_confirmed
            if pending is not None:
                self._unity_socket.sendto(
                    encode_ack(
                        pending[0],
                        query_performance_counter(),
                        status=1,
                    ),
                    pending[1],
                )
            self._unity_sender = sender
            self._last_sequence = -1
            self._pending_confirmed = (packet.sequence, sender, time.monotonic())
            self._pilot_socket.sendto(payload, self._pilot_target)
            return

        if self._unity_sender is None or sender != self._unity_sender:
            return
        if packet.sequence <= self._last_sequence:
            return

        self._last_sequence = packet.sequence
        self._last_unity_packet_at = time.monotonic()
        self._watchdog_sent = False

        if packet.command in CONFIRMED_COMMANDS:
            if self._pending_confirmed is not None:
                self._unity_socket.sendto(
                    encode_ack(packet.sequence, query_performance_counter(), status=1),
                    sender,
                )
                return
            self._pending_confirmed = (packet.sequence, sender, time.monotonic())

        self._pilot_socket.sendto(payload, self._pilot_target)

    # Pilot의 실제 명령 결과 ACK를 Unity로 전달한다.
    def handle_pilot_ack(self, payload: bytes) -> None:
        pending = self._pending_confirmed
        if pending is None or not self._valid_ack(payload, pending[0]):
            return
        self._pending_confirmed = None
        self._unity_socket.sendto(payload, pending[1])

    # 명령 응답과 Unity 입력 watchdog을 처리한다.
    def check_timeouts(self) -> None:
        now = time.monotonic()
        pending = self._pending_confirmed
        if pending is not None and now - pending[2] > self._pilot_timeout:
            self._pending_confirmed = None
            self._unity_socket.sendto(
                encode_ack(pending[0], query_performance_counter(), status=1),
                pending[1],
            )

        if (
            not self._watchdog_sent
            and now - self._last_unity_packet_at > self._watchdog_seconds
        ):
            self.send_internal_neutral()
            self._watchdog_sent = True

    # Pilot 전용 플래그로 중립 RC를 전달한다.
    def send_internal_neutral(self) -> None:
        packet = ControlPacket(
            sequence=0,
            input_ticks=query_performance_counter(),
            axes=(0, 0, 0, 0),
            command=COMMAND_NEUTRAL,
            flags=INTERNAL_CONTROL_FLAG,
        )
        self._pilot_socket.sendto(encode_control_packet(packet), self._pilot_target)

    # 종료된 UDP 상대가 남긴 일시적인 reset은 소비하고 다음 패킷을 계속 기다린다.
    def _receive_udp_packet(
        self,
        current: socket.socket,
    ) -> tuple[bytes, tuple[str, int]] | None:
        try:
            return current.recvfrom(256)
        except OSError as error:
            if current is not self._unity_socket or not is_udp_connection_reset(error):
                raise
            logging.warning(
                "Unity UDP peer reset ignored; waiting for the next packet: %s",
                error,
            )
            return None

    # 중립 RC를 전달하고 소켓을 닫는다.
    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.send_internal_neutral()
            time.sleep(0.1)
        except OSError:
            pass
        self._unity_socket.close()
        self._pilot_socket.close()
        self._authority_socket.close()

    @staticmethod
    def _valid_ack(payload: bytes, sequence: int) -> bool:
        if len(payload) != ACK_PACKET.size:
            return False
        values = ACK_PACKET.unpack(payload)
        return (
            values[0] == CONTROL_MAGIC
            and values[1] == CONTROL_VERSION
            and values[3] == sequence
            and values[5] == 1
        )


# 실행 옵션을 읽는다.
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Quest 3 Unity 입력을 Pilot-PC 제어 호스트로 전달합니다.",
    )
    parser.add_argument("--relay-port", type=int, default=49200)
    parser.add_argument("--authority-port", type=int, default=49199)
    parser.add_argument("--pilot-port", type=int, default=49210)
    parser.add_argument("--watchdog-seconds", type=float, default=0.2)
    parser.add_argument("--pilot-timeout", type=float, default=8.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    adapter = XrControlAdapter(
        relay_port=args.relay_port,
        authority_port=args.authority_port,
        pilot_port=args.pilot_port,
        watchdog_seconds=args.watchdog_seconds,
        pilot_timeout=args.pilot_timeout,
    )
    try:
        adapter.run()
    except KeyboardInterrupt:
        pass
    finally:
        adapter.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
