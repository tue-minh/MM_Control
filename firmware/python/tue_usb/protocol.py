"""
Streaming state-machine parser and packet encoding helpers for TUE USB Binary Protocol.

Matches the defensive byte-wise state machine in Core/Src/tue_usb_binary.c:
  ST_WAIT_SOF1 -> ST_WAIT_SOF2 -> ST_HEADER -> ST_PAYLOAD -> ST_CRC -> ST_EOF1 -> ST_EOF2
"""

import struct
from enum import Enum
from typing import List, Optional, Union

from .crc import crc8_sae_j1850
from .defs import (
    SOF1,
    SOF2,
    EOF1,
    EOF2,
    HEADER_SIZE,
    MAX_DLC,
    FrameType,
    CanId,
)
from .frame import Frame


class ParserState(Enum):
    ST_WAIT_SOF1 = 0
    ST_WAIT_SOF2 = 1
    ST_HEADER = 2
    ST_PAYLOAD = 3
    ST_CRC = 4
    ST_EOF1 = 5
    ST_EOF2 = 6


class FrameParser:
    """
    Streaming byte-by-byte frame parser.
    Tolerant of arbitrary fragmentation, stream concatenation, and noisy corrupted bytes.
    """

    def __init__(self):
        self._state: ParserState = ParserState.ST_WAIT_SOF1
        self._header_buf = bytearray()
        self._payload_buf = bytearray()
        self._frame_type: Optional[FrameType] = None
        self._can_id: int = 0
        self._dlc: int = 0
        self._seq: int = 0
        self._crc8: int = 0

    def reset(self):
        """Reset parser state back to looking for SOF1."""
        self._state = ParserState.ST_WAIT_SOF1
        self._header_buf.clear()
        self._payload_buf.clear()
        self._frame_type = None
        self._can_id = 0
        self._dlc = 0
        self._seq = 0
        self._crc8 = 0

    def feed_byte(self, byte: int) -> Optional[Frame]:
        """
        Process a single incoming byte. Returns a completed Frame if one finishes, or None.
        Matches Core/Src/tue_usb_binary.c usb_rx_byte() logic.
        """
        byte &= 0xFF

        if self._state == ParserState.ST_WAIT_SOF1:
            if byte == SOF1:
                self._state = ParserState.ST_WAIT_SOF2
            return None

        elif self._state == ParserState.ST_WAIT_SOF2:
            if byte == SOF2:
                self._header_buf.clear()
                self._payload_buf.clear()
                self._state = ParserState.ST_HEADER
            elif byte != SOF1:
                self._state = ParserState.ST_WAIT_SOF1
            # A repeated 0xAA keeps us waiting for 0x55
            return None

        elif self._state == ParserState.ST_HEADER:
            self._header_buf.append(byte)
            if len(self._header_buf) >= HEADER_SIZE:
                # Header layout: TYPE(1B), CAN_ID_LE(4B), DLC(1B), SEQ(1B)
                type_val, can_id, dlc, seq = struct.unpack("<BIBB", self._header_buf)
                if dlc > MAX_DLC:
                    self.reset()
                    return None

                try:
                    self._frame_type = FrameType(type_val)
                except ValueError:
                    self._frame_type = type_val  # unknown type

                self._can_id = can_id
                self._dlc = dlc
                self._seq = seq

                if self._dlc == 0:
                    self._state = ParserState.ST_CRC
                else:
                    self._state = ParserState.ST_PAYLOAD
            return None

        elif self._state == ParserState.ST_PAYLOAD:
            self._payload_buf.append(byte)
            if len(self._payload_buf) >= self._dlc:
                self._state = ParserState.ST_CRC
            return None

        elif self._state == ParserState.ST_CRC:
            # CRC is computed over bytes [2 .. 8+DLC] = HEADER + PAYLOAD
            expected_crc = crc8_sae_j1850(self._header_buf + self._payload_buf)
            if byte != expected_crc:
                # CRC mismatch: drop and resynchronize
                self.reset()
                return None
            self._crc8 = byte
            self._state = ParserState.ST_EOF1
            return None

        elif self._state == ParserState.ST_EOF1:
            if byte == EOF1:
                self._state = ParserState.ST_EOF2
            elif byte == SOF1:
                self._state = ParserState.ST_WAIT_SOF2  # candidate new frame
            else:
                self.reset()
            return None

        elif self._state == ParserState.ST_EOF2:
            if byte == EOF2:
                # Valid complete frame received!
                completed_frame = Frame(
                    frame_type=self._frame_type,
                    can_id=self._can_id,
                    payload=bytes(self._payload_buf),
                    seq=self._seq,
                    crc8=self._crc8
                )
                self.reset()
                return completed_frame
            else:
                self.reset()
            return None

        self.reset()
        return None

    def feed(self, chunk: Union[bytes, bytearray, memoryview]) -> List[Frame]:
        """Feed a sequence of incoming bytes and return all complete frames found."""
        frames: List[Frame] = []
        for byte in chunk:
            f = self.feed_byte(byte)
            if f is not None:
                frames.append(f)
        return frames


def encode_frame(
    frame_type: FrameType,
    can_id: Union[CanId, int],
    payload: bytes = b"",
    seq: int = 0
) -> bytes:
    """Convenience helper to encode a full wire frame."""
    f = Frame(frame_type=frame_type, can_id=int(can_id), payload=payload, seq=seq)
    return f.to_bytes()


def encode_request(
    can_id: Union[CanId, int],
    payload: bytes = b"",
    seq: int = 0
) -> bytes:
    """Convenience helper to encode a REQ (Host -> MCU) frame."""
    return encode_frame(FrameType.REQ, can_id, payload, seq)
