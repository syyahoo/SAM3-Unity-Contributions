from __future__ import annotations

import math
import socket
import struct
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np


FRAME_MAGIC = 0x31465644  # DVF1
FRAME_VERSION = 1
FRAME_PREFIX = struct.Struct("<I")
FRAME_HEADER = struct.Struct("<IHHQQIIII")
FRAME_BOX = struct.Struct("<4i")
MAX_DYNAMIC_BOXES = 8
MAX_PACKET_BYTES = 8 * 1024 * 1024
DEFAULT_VISION_STREAM_RATE_HZ = 7.5


@dataclass(frozen=True)
class TcpBgrFrame:
    sequence: int
    source_timestamp_ns: int
    frame: np.ndarray
    dynamic_boxes: tuple[tuple[int, int, int, int], ...]


class LatestFrameTcpPublisher:
    """최신 원본 프레임만 localhost TCP 구독자에게 전달한다."""

    def __init__(
        self,
        port: int = 49000,
        jpeg_quality: int = 90,
        max_rate_hz: float = DEFAULT_VISION_STREAM_RATE_HZ,
    ) -> None:
        self.port = port
        self.jpeg_quality = max(60, min(100, jpeg_quality))
        if not math.isfinite(max_rate_hz) or max_rate_hz <= 0.0:
            raise ValueError("Vision TCP 전송률은 0보다 큰 유한한 값이어야 합니다.")
        self.max_rate_hz = float(max_rate_hz)
        self._minimum_send_interval = 1.0 / self.max_rate_hz
        self._condition = threading.Condition()
        self._sequence = 0
        self._latest = None
        self._dynamic_boxes: tuple[tuple[int, int, int, int], ...] = ()
        self._closed = False
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", port))
        self.port = self._server.getsockname()[1]
        self._server.listen(1)
        self._server.settimeout(0.5)
        self._thread = threading.Thread(
            target=self._run,
            name="StrikerPCVisionTcpPublisher",
            daemon=True,
        )
        self._thread.start()
        print("STRIKERPC_STAGE VISION_TCP_READY", flush=True)

    def update_frame(self, frame, source_timestamp_ns: int | None = None) -> int:
        if frame is None or len(frame.shape) != 3 or frame.shape[2] != 3:
            raise ValueError("BGR24 프레임만 전달할 수 있습니다.")
        with self._condition:
            self._sequence += 1
            self._latest = (
                self._sequence,
                source_timestamp_ns or time.perf_counter_ns(),
                frame.copy(order="C"),
                self._dynamic_boxes,
            )
            self._condition.notify_all()
            return self._sequence

    def update_dynamic_boxes(
        self,
        boxes: list[tuple[int, int, int, int]],
    ) -> None:
        with self._condition:
            self._dynamic_boxes = tuple(boxes[:MAX_DYNAMIC_BOXES])

    def _run(self) -> None:
        while not self._closed:
            try:
                client, _ = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            client.settimeout(1.0)
            try:
                self._serve_client(client)
            except (ConnectionError, OSError):
                pass
            finally:
                client.close()

    def _serve_client(self, client: socket.socket) -> None:
        sent_sequence = 0
        next_send_at = 0.0
        while not self._closed:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._closed
                    or (
                        self._latest is not None
                        and self._latest[0] > sent_sequence
                    ),
                    timeout=0.5,
                )
                if self._closed:
                    return
                latest = self._latest
            if latest is None or latest[0] <= sent_sequence:
                continue

            remaining = next_send_at - time.monotonic()
            if remaining > 0.0:
                with self._condition:
                    self._condition.wait_for(
                        lambda: self._closed,
                        timeout=remaining,
                    )
                    if self._closed:
                        return
                    latest = self._latest
                if latest is None or latest[0] <= sent_sequence:
                    continue
            sequence, timestamp_ns, frame, boxes = latest
            success, encoded = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if not success:
                continue
            height, width = frame.shape[:2]
            jpeg = encoded.tobytes()
            header = FRAME_HEADER.pack(
                FRAME_MAGIC,
                FRAME_VERSION,
                FRAME_HEADER.size,
                sequence,
                timestamp_ns,
                width,
                height,
                len(jpeg),
                len(boxes),
            )
            body = header + b"".join(FRAME_BOX.pack(*box) for box in boxes) + jpeg
            client.sendall(FRAME_PREFIX.pack(len(body)) + body)
            sent_sequence = sequence
            next_send_at = time.monotonic() + self._minimum_send_interval

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        with self._condition:
            self._condition.notify_all()
        self._server.close()
        self._thread.join(timeout=2.0)


class LatestFrameTcpReader:
    """Pilot TCP stream을 읽고 손상되거나 역순인 프레임을 폐기한다."""

    def __init__(self, port: int = 49000) -> None:
        self.target = ("127.0.0.1", port)
        self._client: socket.socket | None = None
        self._last_sequence = 0
        self._last_source_timestamp_ns = 0
        self._session_reset_pending = False
        self._closed = False

    def read(self) -> TcpBgrFrame | None:
        if self._closed:
            return None
        try:
            if self._client is None:
                self._connect()
            packet_size = FRAME_PREFIX.unpack(
                self._receive_exact(FRAME_PREFIX.size)
            )[0]
            if packet_size < FRAME_HEADER.size or packet_size > MAX_PACKET_BYTES:
                raise ConnectionError("잘못된 Vision TCP packet 크기")
            payload = self._receive_exact(packet_size)
            values = FRAME_HEADER.unpack_from(payload)
            if (
                values[0] != FRAME_MAGIC
                or values[1] != FRAME_VERSION
                or values[2] != FRAME_HEADER.size
                or values[8] > MAX_DYNAMIC_BOXES
            ):
                return None
            sequence, timestamp_ns = values[3], values[4]
            width, height, jpeg_size, box_count = values[5:9]
            boxes_size = box_count * FRAME_BOX.size
            expected = FRAME_HEADER.size + boxes_size + jpeg_size
            new_session = (
                self._session_reset_pending
                and self._last_sequence > 0
                and 0 < sequence <= self._last_sequence
                and timestamp_ns > self._last_source_timestamp_ns
            )
            if (
                sequence <= 0
                or timestamp_ns <= 0
                or (sequence <= self._last_sequence and not new_session)
                or (
                    self._last_source_timestamp_ns > 0
                    and timestamp_ns <= self._last_source_timestamp_ns
                )
                or expected != len(payload)
            ):
                return None
            boxes = tuple(
                FRAME_BOX.unpack_from(
                    payload,
                    FRAME_HEADER.size + index * FRAME_BOX.size,
                )
                for index in range(box_count)
            )
            jpeg_offset = FRAME_HEADER.size + boxes_size
            frame = cv2.imdecode(
                np.frombuffer(payload[jpeg_offset:], np.uint8),
                cv2.IMREAD_COLOR,
            )
            if frame is None or frame.shape[1] != width or frame.shape[0] != height:
                return None
            self._last_sequence = sequence
            self._last_source_timestamp_ns = timestamp_ns
            self._session_reset_pending = False
            return TcpBgrFrame(sequence, timestamp_ns, frame, boxes)
        except (ConnectionError, OSError, struct.error):
            self._disconnect()
            time.sleep(0.1)
            return None

    def _connect(self) -> None:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(1.0)
        client.connect(self.target)
        self._client = client
        self._session_reset_pending = self._last_sequence > 0

    def _receive_exact(self, size: int) -> bytes:
        if self._client is None:
            raise ConnectionError("Vision TCP 연결 없음")
        chunks = bytearray()
        while len(chunks) < size:
            chunk = self._client.recv(size - len(chunks))
            if not chunk:
                raise ConnectionError("Vision TCP 연결 종료")
            chunks.extend(chunk)
        return bytes(chunks)

    def _disconnect(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None

    def close(self) -> None:
        self._closed = True
        self._disconnect()
