"""Transport abstraction between the Raspberry Pi host and the STM32.

Mac development uses MemoryTransport (in-process pair). The real robot uses
SerialTransport (device/baudrate from config/hardware.yaml — never hardcoded).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque


class TransportError(Exception):
    pass


class Transport(ABC):
    """Byte-pipe with send() / read()."""

    @abstractmethod
    def send(self, data: bytes) -> None:
        ...

    @abstractmethod
    def read(self, timeout: float = 0.0) -> bytes:
        """Return bytes received so far (may be empty). Non-blocking beyond
        `timeout` at most."""
        ...

    def close(self) -> None:  # optional
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class MemoryTransport(Transport):
    """One end of an in-memory byte pipe; peer.send() arrives here."""

    def __init__(self) -> None:
        self._inbox: deque[bytes] = deque()
        self._peer: "MemoryTransport | None" = None

    @staticmethod
    def create_pair() -> tuple["MemoryTransport", "MemoryTransport"]:
        a = MemoryTransport()
        b = MemoryTransport()
        a._peer = b
        b._peer = a
        return a, b

    def send(self, data: bytes) -> None:
        if self._peer is None:
            raise TransportError("MemoryTransport not paired")
        self._peer._inbox.append(bytes(data))

    def read(self, timeout: float = 0.0) -> bytes:
        chunks = []
        while self._inbox:
            chunks.append(self._inbox.popleft())
        return b"".join(chunks)


class SerialTransport(Transport):
    """pyserial-backed transport for the Raspberry Pi (lazy import)."""

    def __init__(self, device: str, baudrate: int = 115200,
                 timeout: float = 0.01) -> None:
        try:
            import serial  # pyserial
        except ImportError as e:  # pragma: no cover
            raise TransportError(
                "pyserial not installed (pip install pyserial)") from e
        self._serial = serial.Serial(device, baudrate, timeout=timeout)

    def send(self, data: bytes) -> None:
        self._serial.write(data)
        self._serial.flush()

    def read(self, timeout: float = 0.0) -> bytes:
        if timeout:
            self._serial.timeout = timeout
        size = self._serial.in_waiting
        if size:
            return bytes(self._serial.read(size))
        return b""

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()


def create_transport(config: dict) -> Transport:
    """Factory from the `mcu:` section of hardware.yaml."""
    kind = config.get("transport", "memory")
    if kind == "memory":
        return MemoryTransport.create_pair()[0]
    if kind == "serial":
        serial_cfg = config.get("serial", {})
        device = serial_cfg.get("device")
        baudrate = int(serial_cfg.get("baudrate", 115200))
        if not device or device == "TODO":
            raise TransportError(
                "serial transport selected but device is not configured "
                "(set mcu.serial.device in config/hardware.yaml)")
        return SerialTransport(device, baudrate)
    raise TransportError(f"unknown transport type: {kind}")
