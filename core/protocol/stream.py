"""Robust stream parser: half frames, sticky frames, bad CRC, garbage, and
unknown MSG_IDs are all handled without losing stream sync."""
from __future__ import annotations

from dataclasses import dataclass, field

from core.protocol.frames import Frame, ProtocolError, try_decode_frame
from core.protocol.messages import HEADER_SIZE, SOF1, SOF2


@dataclass
class ParserStats:
    frames_ok: int = 0
    crc_errors: int = 0
    garbage_bytes: int = 0
    unknown_msg_ids: int = 0
    version_errors: int = 0
    length_errors: int = 0


@dataclass
class StreamParser:
    """Feed arbitrary chunks; get complete, CRC-valid frames out."""
    max_buffer: int = 4096
    stats: ParserStats = field(default_factory=ParserStats)
    _buf: bytearray = field(default_factory=bytearray)

    # -------------------------------------------------------------- helpers
    def _find_sof(self, start: int) -> int:
        buf = self._buf
        i = start
        while i < len(buf) - 1:
            if buf[i] == SOF1 and buf[i + 1] == SOF2:
                return i
            i += 1
        return len(buf) if len(buf) >= 1 else 0

    def _skip_garbage(self, n: int = 1) -> None:
        del self._buf[:n]
        self.stats.garbage_bytes += n

    # -------------------------------------------------------------- main
    def feed(self, data: bytes) -> list[Frame]:
        self._buf.extend(data)
        frames: list[Frame] = []
        while True:
            # drop leading garbage before SOF
            if len(self._buf) >= 2:
                if not (self._buf[0] == SOF1 and self._buf[1] == SOF2):
                    self._skip_garbage(1)
                    continue
            elif len(self._buf) == 1 and self._buf[0] != SOF1:
                self._skip_garbage(1)
                continue

            if len(self._buf) < HEADER_SIZE:
                break  # header incomplete

            try:
                result = try_decode_frame(self._buf)
            except ProtocolError as e:
                reason = str(e)
                if "CRC" in reason:
                    self.stats.crc_errors += 1
                    # consume the whole bad frame to keep sync
                    length = self._buf[5] | (self._buf[6] << 8)
                    total = HEADER_SIZE + length + 2
                    del self._buf[:total]
                elif "version" in reason:
                    self.stats.version_errors += 1
                    self._skip_garbage(1)
                else:
                    self.stats.length_errors += 1
                    self._skip_garbage(2)  # false SOF; rescan
                continue

            if result is None:
                break  # need more bytes (half frame)

            frame, consumed = result
            from core.protocol.messages import spec_by_id
            if spec_by_id(frame.msg_id) is None:
                self.stats.unknown_msg_ids += 1
            else:
                frames.append(frame)
                self.stats.frames_ok += 1
            del self._buf[:consumed]

        if len(self._buf) > self.max_buffer:
            self._skip_garbage(len(self._buf) - self.max_buffer)
        return frames

    @property
    def pending_bytes(self) -> int:
        return len(self._buf)
