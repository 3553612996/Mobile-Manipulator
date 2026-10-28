#!/usr/bin/env python3
"""Serial transport for the host console.

pyserial runs in a plain worker thread with a short read timeout; decoded
frames are handed to the GUI thread through a Qt signal (queued connection).
The parser lives in the worker so a slow UI can never lose bytes.
"""

from __future__ import annotations

import queue
import threading
import time

import serial
from PySide6.QtCore import QObject, Signal
from serial.tools import list_ports

from .protocol import FrameParser

BAUDRATES = (115200, 230400, 460800, 921600)
DEFAULT_BAUD = 115200
# Any gap longer than this between two chunks means the previous frame is
# abandoned (protocol.md section 9 requires a 50 ms inter-byte reset).
BYTE_GAP_S = 0.050


def available_ports() -> list[tuple[str, str]]:
    ports = sorted(list_ports.comports(), key=lambda item: item.device)
    return [(port.device, port.description) for port in ports]


class SerialLink(QObject):
    """Owns one open serial port for the lifetime of the link."""

    connectedChanged = Signal(bool)
    frameReceived = Signal(object)
    errorOccurred = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._serial: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._out: queue.Queue[bytes] = queue.Queue()
        self._stop = threading.Event()
        self.rx_frames = 0
        self.crc_errors = 0

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def open(self, port: str, baud: int) -> None:
        self.close()
        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01,
            write_timeout=0.5,
        )
        self.rx_frames = 0
        self.crc_errors = 0
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="serial-link",
                                        daemon=True)
        self._thread.start()
        self.connectedChanged.emit(True)

    def close(self) -> None:
        was_open = self._serial is not None
        self._stop.set()
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
            self._serial = None
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        while not self._out.empty():
            try:
                self._out.get_nowait()
            except queue.Empty:
                break
        if was_open:
            self.connectedChanged.emit(False)

    def send(self, data: bytes) -> None:
        if self.is_open:
            self._out.put(data)

    # -- worker ------------------------------------------------------------

    def _run(self) -> None:
        port = self._serial
        assert port is not None
        parser = FrameParser()
        last_rx = 0.0

        while not self._stop.is_set():
            try:
                while True:
                    port.write(self._out.get_nowait())
            except queue.Empty:
                pass
            except (serial.SerialException, OSError) as exc:
                self.errorOccurred.emit(f"串口写入失败：{exc}")
                break

            try:
                waiting = port.in_waiting
                chunk = port.read(waiting if waiting else 1)
            except (serial.SerialException, OSError) as exc:
                if not self._stop.is_set():
                    self.errorOccurred.emit(f"串口读取失败：{exc}")
                break

            if not chunk:
                continue

            now = time.monotonic()
            if last_rx and now - last_rx > BYTE_GAP_S:
                parser.reset()
            last_rx = now

            for frame in parser.feed(chunk):
                self.rx_frames += 1
                self.frameReceived.emit(frame)
            self.crc_errors = parser.crc_errors

        # The port is gone or was closed: make sure the UI hears about it even
        # when the close came from the worker (cable pulled, adapter reset).
        if not self._stop.is_set():
            self._stop.set()
            self.connectedChanged.emit(False)
