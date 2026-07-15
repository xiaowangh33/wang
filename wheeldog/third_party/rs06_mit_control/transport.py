"""USB 转 CAN 串口传输层。"""

from __future__ import annotations

import threading
import time
from typing import Optional

import serial

from protocol import parse_adapter_buffer


class UsbCanTransport:
    """灵足官方 USB-CAN 模块串口封装。"""

    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._rx_buf = bytearray()
        self._rx_lock = threading.Lock()
        self._reader: Optional[threading.Thread] = None
        self._reader_stop = threading.Event()

    def open(self) -> None:
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            dsrdtr=False,
            rtscts=False,
        )
        # 部分 CH340 USB-CAN 模块需拉高 DTR 才打开 CAN 收发
        self._ser.dtr = True
        self._ser.rts = False
        time.sleep(0.05)
        self._ser.reset_input_buffer()
        with self._rx_lock:
            self._rx_buf.clear()
        self._reader_stop.clear()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._reader_stop.set()
        if self._reader and self._reader.is_alive():
            self._reader.join(timeout=0.5)
        self._reader = None
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def send(self, frame: bytes, description: str = "", verbose: bool = True) -> None:
        if not self.is_open:
            raise RuntimeError("串口未打开")
        self._ser.write(frame)
        self._ser.flush()
        if verbose:
            prefix = f"[TX] {description}: " if description else "[TX] "
            print(prefix + frame.hex())

    def read_available(self) -> bytes:
        with self._rx_lock:
            if not self._rx_buf:
                return b""
            data = bytes(self._rx_buf)
            self._rx_buf.clear()
            return data

    def drain_rx(self, wait_ms: float = 50.0) -> bytes:
        deadline = time.time() + wait_ms / 1000.0
        while time.time() < deadline:
            time.sleep(0.005)
        return self.read_available()

    def read_can_frame(self, wait_ms: float = 50.0) -> Optional[tuple[int, bytes]]:
        raw = self.drain_rx(wait_ms)
        if not raw:
            return None
        parsed = parse_adapter_buffer(raw)
        return parsed[-1] if parsed else None

    def _reader_loop(self) -> None:
        while not self._reader_stop.is_set():
            if not self.is_open:
                break
            try:
                waiting = self._ser.in_waiting
                if waiting > 0:
                    chunk = self._ser.read(waiting)
                    if chunk:
                        with self._rx_lock:
                            self._rx_buf.extend(chunk)
                else:
                    time.sleep(0.002)
            except (serial.SerialException, OSError):
                break

    def __enter__(self) -> "UsbCanTransport":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
