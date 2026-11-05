"""Thread-safe serial transport for talking to the TITAN-IMX over its USB virtual COM port."""
from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

import serial
from serial.tools import list_ports


@dataclass
class SerialSettings:
    port: str
    baudrate: int = 115200
    bytesize: int = serial.EIGHTBITS
    parity: str = serial.PARITY_NONE
    stopbits: float = serial.STOPBITS_ONE
    timeout: float = 1.0
    terminator: str = "\r\n"


def list_available_ports() -> list[str]:
    return [p.device for p in list_ports.comports()]


class SerialLink:
    """Wraps a pyserial connection with a background reader thread.

    Unsolicited lines (e.g. status pushed by the device) are delivered via
    `on_unsolicited_line`; direct command/response pairs use `send_command`.
    """

    def __init__(
        self,
        on_unsolicited_line: Optional[Callable[[str], None]] = None,
        on_tx: Optional[Callable[[str], None]] = None,
    ):
        self._ser: Optional[serial.Serial] = None
        self._settings: Optional[SerialSettings] = None
        self._lock = threading.Lock()
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._response_queue: "queue.Queue[str]" = queue.Queue()
        self._on_unsolicited_line = on_unsolicited_line
        self._on_tx = on_tx
        self._awaiting_response = threading.Event()

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect(self, settings: SerialSettings) -> None:
        self.disconnect()
        self._settings = settings
        self._ser = serial.Serial(
            port=settings.port,
            baudrate=settings.baudrate,
            bytesize=settings.bytesize,
            parity=settings.parity,
            stopbits=settings.stopbits,
            timeout=settings.timeout,
        )
        self._stop_event.clear()
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def disconnect(self) -> None:
        self._stop_event.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=2.0)
            self._reader_thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def send_command(self, command: str, response_timeout: float = 1.0) -> str:
        """Send a command and wait for the next line of response (blocking)."""
        if not self.is_connected or self._ser is None or self._settings is None:
            raise ConnectionError("Serial port is not connected")

        with self._lock:
            # Drain any stale responses before sending a new command.
            while not self._response_queue.empty():
                self._response_queue.get_nowait()

            self._awaiting_response.set()
            payload = (command + self._settings.terminator).encode("ascii")
            if self._on_tx is not None:
                self._on_tx(command)
            self._ser.write(payload)
            self._ser.flush()

            try:
                return self._response_queue.get(timeout=response_timeout)
            except queue.Empty:
                raise TimeoutError(f"No response to command: {command!r}")
            finally:
                self._awaiting_response.clear()

    def _read_loop(self) -> None:
        buffer = b""
        while not self._stop_event.is_set():
            if self._ser is None:
                break
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
            except serial.SerialException:
                break
            if not chunk:
                continue
            # Normalize CR/CR-LF/LF line endings so any terminator style splits cleanly.
            buffer += chunk.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
            while b"\n" in buffer:
                line, _, buffer = buffer.partition(b"\n")
                if not line:
                    continue
                text = line.decode("ascii", errors="replace")
                if self._awaiting_response.is_set():
                    self._response_queue.put(text)
                elif self._on_unsolicited_line is not None:
                    self._on_unsolicited_line(text)
