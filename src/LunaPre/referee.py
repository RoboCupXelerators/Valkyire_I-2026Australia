"""Referee input providers compatible with the RCJ communication module.

The official UART/USB wire contract is deliberately small: one ASCII line,
``PLAY`` or ``STOP``, is emitted when the state changes. Detailed simulator
state such as a penalty reason or countdown must not be inferred from it.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import os
import threading
import time
from typing import Callable, Protocol


PLAY = "PLAY"
STOP = "STOP"
UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class RefereeSnapshot:
    phase: str = STOP
    control_enabled: bool = False
    penalty_active: bool = False
    penalty_reason: str = UNKNOWN
    penalty_remaining_s: float = 0.0
    detail_valid: bool = False
    connected: bool = False
    source: str = "unconfigured"
    sequence: int = 0
    updated_at_ms: int = 0
    raw_line: str = ""


class RefereeProvider(Protocol):
    def snapshot(self) -> RefereeSnapshot: ...


class RefereeLineDecoder:
    """Thread-safe decoder for the strict PLAY/STOP line protocol."""

    def __init__(self, source: str = "uart"):
        self._source = source
        self._lock = threading.Lock()
        self._snapshot = RefereeSnapshot(source=source)

    def feed(self, line: bytes | str, *, now_ms: int | None = None) -> bool:
        if isinstance(line, bytes):
            text = line.decode("ascii", errors="ignore").strip().upper()
        else:
            text = str(line).strip().upper()
        if text not in {PLAY, STOP}:
            return False

        with self._lock:
            self._snapshot = RefereeSnapshot(
                phase=text,
                control_enabled=text == PLAY,
                connected=True,
                source=self._source,
                sequence=self._snapshot.sequence + 1,
                updated_at_ms=int(now_ms if now_ms is not None else time.monotonic() * 1000),
                raw_line=text,
            )
        return True

    def disconnect(self) -> None:
        with self._lock:
            self._snapshot = replace(
                self._snapshot,
                phase=STOP,
                control_enabled=False,
                connected=False,
                raw_line="",
            )

    def snapshot(self) -> RefereeSnapshot:
        with self._lock:
            return self._snapshot


class SerialRefereeProvider:
    """Background serial reader with fail-closed reconnect behavior."""

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = 115200,
        timeout_s: float = 0.2,
        reconnect_s: float = 1.0,
        opener: Callable[..., object] | None = None,
    ):
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.reconnect_s = float(reconnect_s)
        self._opener = opener
        self._decoder = RefereeLineDecoder(source=f"serial:{self.port}")
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial = None

    def start(self) -> "SerialRefereeProvider":
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="RCJ-Referee-UART", daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        serial_port = self._serial
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.timeout_s * 2.0))
        self._thread = None
        self._serial = None
        self._decoder.disconnect()

    def snapshot(self) -> RefereeSnapshot:
        return self._decoder.snapshot()

    def _open(self):
        if self._opener is not None:
            return self._opener(self.port, self.baudrate, self.timeout_s)
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required for a physical or PTY referee port") from exc
        return serial.Serial(self.port, baudrate=self.baudrate, timeout=self.timeout_s)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._serial = self._open()
                while not self._stop.is_set():
                    line = self._serial.readline()
                    if line:
                        self._decoder.feed(line)
            except Exception:
                self._decoder.disconnect()
                self._stop.wait(self.reconnect_s)
            finally:
                serial_port = self._serial
                self._serial = None
                if serial_port is not None:
                    try:
                        serial_port.close()
                    except Exception:
                        pass


class IoRefereeProvider:
    """Adapter for the existing Bits GPIO/unlock-pin contract."""

    def __init__(self, reader: Callable[[], bool], source: str = "bits-io"):
        self._reader = reader
        self._source = source
        self._sequence = 0
        self._last_enabled: bool | None = None

    def snapshot(self) -> RefereeSnapshot:
        try:
            enabled = bool(self._reader())
            connected = True
        except Exception:
            enabled = False
            connected = False
        if enabled != self._last_enabled:
            self._sequence += 1
            self._last_enabled = enabled
        return RefereeSnapshot(
            phase=PLAY if enabled else STOP,
            control_enabled=enabled,
            connected=connected,
            source=self._source,
            sequence=self._sequence,
            updated_at_ms=int(time.monotonic() * 1000),
            raw_line=PLAY if enabled else STOP,
        )


class VirtualRefereeSerial:
    """POSIX PTY that behaves like the official UART/USB serial output."""

    def __init__(self, *, initial_state: str = STOP):
        if os.name != "posix":
            raise OSError("VirtualRefereeSerial requires a POSIX PTY")
        import pty
        import tty

        self._master_fd, self._slave_fd = pty.openpty()
        tty.setraw(self._slave_fd)
        self.port = os.ttyname(self._slave_fd)
        self._closed = False
        self.send(initial_state)

    def send(self, state: str) -> None:
        normalized = str(state).strip().upper()
        if normalized not in {PLAY, STOP}:
            raise ValueError("official referee serial state must be PLAY or STOP")
        if self._closed:
            raise RuntimeError("virtual referee serial is closed")
        os.write(self._master_fd, f"{normalized}\n".encode("ascii"))

    def play(self) -> None:
        self.send(PLAY)

    def stop(self) -> None:
        self.send(STOP)

    def penalty(self) -> None:
        self.send(STOP)

    def close(self) -> None:
        if not self._closed:
            os.close(self._master_fd)
            os.close(self._slave_fd)
            self._closed = True

    def __enter__(self) -> "VirtualRefereeSerial":
        return self

    def __exit__(self, *_args) -> None:
        self.close()
