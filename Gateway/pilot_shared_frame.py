from __future__ import annotations

import mmap
import struct
import threading
import time

import numpy as np


FRAME_MAGIC = 0x31564654
FRAME_VERSION = 2
PIXEL_FORMAT_BGRA32 = 1
FRAME_HEADER = struct.Struct("<IIiiiiqQ")
FRAME_HEADER_SIZE = FRAME_HEADER.size


class PilotSharedFramePublisher:
    """Unity TelloPilotFrame_v2 계약으로 최신 BGRA 프레임을 게시한다."""

    def __init__(
        self,
        name: str = "TelloPilotFrame_v2",
        *,
        maximum_width: int = 960,
        maximum_height: int = 720,
    ) -> None:
        if maximum_width <= 0 or maximum_height <= 0:
            raise ValueError("공유 프레임 최대 해상도는 양수여야 합니다.")
        self.name = name
        self.maximum_width = maximum_width
        self.maximum_height = maximum_height
        self.mapping_size = FRAME_HEADER_SIZE + maximum_width * maximum_height * 4
        self._mapping = mmap.mmap(-1, self.mapping_size, tagname=name, access=mmap.ACCESS_WRITE)
        self._frame_id = 0
        self._lock = threading.Lock()
        self._bgra: np.ndarray | None = None

    # 음수 frame ID로 쓰기 시작을 표시하고 전체 복사 뒤 양수 ID로 확정한다.
    def update_frame(self, frame: np.ndarray, source_timestamp_ns: int) -> int:
        if (
            not isinstance(frame, np.ndarray)
            or frame.dtype != np.uint8
            or frame.ndim != 3
            or frame.shape[2] != 3
        ):
            raise ValueError("공유 프레임은 uint8 BGR 이미지여야 합니다.")
        height, width = frame.shape[:2]
        if width <= 0 or height <= 0 or width > self.maximum_width or height > self.maximum_height:
            raise ValueError("공유 프레임 해상도가 허용 범위를 벗어났습니다.")
        if source_timestamp_ns <= 0:
            raise ValueError("source timestamp는 양수여야 합니다.")

        stride = width * 4

        with self._lock:
            if self._bgra is None or self._bgra.shape[:2] != (height, width):
                self._bgra = np.empty((height, width, 4), dtype=np.uint8)
                self._bgra[:, :, 3] = 255
            self._bgra[:, :, :3] = frame

            self._frame_id += 1
            frame_id = self._frame_id
            writing_header = FRAME_HEADER.pack(
                FRAME_MAGIC,
                FRAME_VERSION,
                width,
                height,
                stride,
                PIXEL_FORMAT_BGRA32,
                -frame_id,
                source_timestamp_ns,
            )
            complete_header = FRAME_HEADER.pack(
                FRAME_MAGIC,
                FRAME_VERSION,
                width,
                height,
                stride,
                PIXEL_FORMAT_BGRA32,
                frame_id,
                source_timestamp_ns,
            )
            self._mapping.seek(0)
            self._mapping.write(writing_header)
            self._mapping.write(memoryview(self._bgra).cast("B"))
            self._mapping.seek(0)
            self._mapping.write(complete_header)
            return frame_id

    # 명명된 공유 메모리 핸들을 닫는다.
    def close(self) -> None:
        self._mapping.close()


class PilotSharedFrameReader:
    """LiveKit 같은 선택 소비자가 최신 로컬 FPV 프레임만 읽는다."""

    def __init__(
        self,
        name: str = "TelloPilotFrame_v2",
        *,
        maximum_width: int = 960,
        maximum_height: int = 720,
        poll_seconds: float = 0.005,
    ) -> None:
        if maximum_width <= 0 or maximum_height <= 0:
            raise ValueError("공유 프레임 최대 해상도는 양수여야 합니다.")
        self.name = name
        self.maximum_width = maximum_width
        self.maximum_height = maximum_height
        self.mapping_size = FRAME_HEADER_SIZE + maximum_width * maximum_height * 4
        self.poll_seconds = max(0.001, poll_seconds)
        self._mapping = mmap.mmap(
            -1,
            self.mapping_size,
            tagname=name,
            access=mmap.ACCESS_READ,
        )
        self._last_frame_id = 0
        self._last_source_timestamp_ns = 0
        self._reader_error: BaseException | None = None
        self._closed = False

    @property
    def has_failed(self) -> bool:
        return self._reader_error is not None

    @property
    def reader_error(self) -> BaseException | None:
        return self._reader_error

    @property
    def last_frame_id(self) -> int:
        return self._last_frame_id

    @property
    def last_source_timestamp_ns(self) -> int:
        return self._last_source_timestamp_ns

    # 완료된 양수 frame ID의 헤더와 payload가 같은 쓰기 세대인지 재확인한다.
    def _read_latest_frame(self) -> np.ndarray | None:
        self._mapping.seek(0)
        first_header = self._mapping.read(FRAME_HEADER_SIZE)
        if len(first_header) != FRAME_HEADER_SIZE:
            return None
        (
            magic,
            version,
            width,
            height,
            stride,
            pixel_format,
            frame_id,
            source_timestamp_ns,
        ) = FRAME_HEADER.unpack(first_header)
        if magic == 0 and version == 0:
            return None
        if magic != FRAME_MAGIC or version != FRAME_VERSION:
            raise RuntimeError("공유 프레임 헤더 형식이 올바르지 않습니다.")
        if frame_id <= self._last_frame_id:
            return None
        if frame_id <= 0:
            return None
        if (
            width <= 0
            or height <= 0
            or width > self.maximum_width
            or height > self.maximum_height
            or stride < width * 4
            or stride > self.maximum_width * 4
            or pixel_format != PIXEL_FORMAT_BGRA32
        ):
            raise RuntimeError("공유 프레임 크기 또는 픽셀 형식이 올바르지 않습니다.")

        payload_size = stride * height
        self._mapping.seek(FRAME_HEADER_SIZE)
        payload = self._mapping.read(payload_size)
        if len(payload) != payload_size:
            return None
        self._mapping.seek(0)
        if self._mapping.read(FRAME_HEADER_SIZE) != first_header:
            return None

        rows = np.frombuffer(payload, dtype=np.uint8).reshape(height, stride)
        bgra = rows[:, : width * 4].reshape(height, width, 4)
        self._last_frame_id = frame_id
        self._last_source_timestamp_ns = source_timestamp_ns
        return bgra[:, :, :3].copy()

    # 새 프레임이 생길 때까지 짧게 기다린 뒤 OpenCV VideoCapture 형식으로 반환한다.
    def read(self, timeout_seconds: float = 0.1):
        if self._closed:
            return False, None
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        try:
            while True:
                frame = self._read_latest_frame()
                if frame is not None:
                    return True, frame
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False, None
                time.sleep(min(self.poll_seconds, remaining))
        except Exception as error:
            self._reader_error = error
            raise

    # 명명된 공유 메모리 읽기 핸들을 닫는다.
    def release(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._mapping.close()

    close = release
