"""
Frame data structure and raw serialization/deserialization for TUE USB Binary Protocol.

Envelope:
  [SOF1(0xAA)][SOF2(0x55)][TYPE(1B)][CAN_ID_LE(4B)][DLC(1B)][SEQ(1B)][PAYLOAD(0..8B)][CRC8(1B)][EOF1(0x55)][EOF2(0xAA)]
"""

import struct
from dataclasses import dataclass, field
from typing import Optional, Union

from .crc import crc8_sae_j1850
from .defs import (
    SOF1,
    SOF2,
    EOF1,
    EOF2,
    HEADER_SIZE,
    FRAME_OVERHEAD,
    MAX_DLC,
    FrameType,
    ErrorCode,
    CanId,
    is_reserved_can_id,
)


@dataclass
class Frame:
    """Represents a validated binary CAN-over-USB protocol frame."""
    frame_type: FrameType
    can_id: int
    payload: bytes = b""
    seq: int = 0
    crc8: Optional[int] = None
    dlc: int = field(init=False)

    def __post_init__(self):
        if isinstance(self.frame_type, int) and not isinstance(self.frame_type, FrameType):
            self.frame_type = FrameType(self.frame_type)

        if not isinstance(self.payload, (bytes, bytearray)):
            self.payload = bytes(self.payload)

        self.dlc = len(self.payload)
        if self.dlc > MAX_DLC:
            raise ValueError(f"Payload length {self.dlc} exceeds maximum DLC of {MAX_DLC}")

        self.seq = self.seq & 0xFF
        self.can_id = self.can_id & 0xFFFFFFFF

        if self.crc8 is None:
            self.crc8 = self.compute_crc()

    def compute_crc(self) -> int:
        """Compute the SAE J1850 CRC-8 over header + payload: [TYPE, CAN_ID(4B LE), DLC, SEQ, PAYLOAD]."""
        header_bytes = struct.pack("<BIBB", int(self.frame_type), self.can_id, self.dlc, self.seq)
        return crc8_sae_j1850(header_bytes + self.payload)

    def is_crc_valid(self) -> bool:
        """Verify if current crc8 matches calculated CRC-8."""
        return self.crc8 == self.compute_crc()

    def to_bytes(self) -> bytes:
        """Serialize frame to wire byte array."""
        crc = self.compute_crc() if self.crc8 is None else self.crc8
        header = struct.pack("<BIBB", int(self.frame_type), self.can_id, self.dlc, self.seq)
        return bytes([SOF1, SOF2]) + header + self.payload + bytes([crc, EOF1, EOF2])

    @classmethod
    def from_bytes(cls, raw: Union[bytes, bytearray]) -> "Frame":
        """
        Deserialize and validate a raw frame byte sequence.
        Raises ValueError if preamble, postamble, length or CRC are invalid.
        """
        if len(raw) < FRAME_OVERHEAD:
            raise ValueError(f"Frame length {len(raw)} is shorter than minimum {FRAME_OVERHEAD} bytes")

        if raw[0] != SOF1 or raw[1] != SOF2:
            raise ValueError(f"Invalid SOF markers: 0x{raw[0]:02X} 0x{raw[1]:02X} (expected 0x{SOF1:02X} 0x{SOF2:02X})")

        frame_type_val, can_id, dlc, seq = struct.unpack("<BIBB", raw[2:2 + HEADER_SIZE])

        if dlc > MAX_DLC:
            raise ValueError(f"DLC {dlc} exceeds maximum {MAX_DLC}")

        expected_len = FRAME_OVERHEAD + dlc
        if len(raw) != expected_len:
            raise ValueError(f"Byte length {len(raw)} does not match expected {expected_len} for DLC={dlc}")

        payload = bytes(raw[2 + HEADER_SIZE : 2 + HEADER_SIZE + dlc])
        crc = raw[2 + HEADER_SIZE + dlc]
        eof1 = raw[2 + HEADER_SIZE + dlc + 1]
        eof2 = raw[2 + HEADER_SIZE + dlc + 2]

        if eof1 != EOF1 or eof2 != EOF2:
            raise ValueError(f"Invalid EOF markers: 0x{eof1:02X} 0x{eof2:02X} (expected 0x{EOF1:02X} 0x{EOF2:02X})")

        # Verify CRC
        expected_crc = crc8_sae_j1850(raw[2 : 2 + HEADER_SIZE + dlc])
        if crc != expected_crc:
            raise ValueError(f"CRC-8 verification failed: got 0x{crc:02X}, expected 0x{expected_crc:02X}")

        return cls(
            frame_type=FrameType(frame_type_val),
            can_id=can_id,
            payload=payload,
            seq=seq,
            crc8=crc
        )

    def is_ok(self) -> bool:
        """Check if frame indicates successful execution."""
        return self.frame_type == FrameType.RESP_OK

    def is_error(self) -> bool:
        """Check if frame indicates error execution."""
        return self.frame_type == FrameType.RESP_ERR

    def error_code(self) -> Optional[ErrorCode]:
        """Return the ErrorCode if frame is RESP_ERR, otherwise None."""
        if self.is_error() and len(self.payload) >= 1:
            try:
                return ErrorCode(self.payload[0])
            except ValueError:
                return ErrorCode.ERR_INTERNAL
        return None

    def can_id_name(self) -> str:
        """Human-readable symbol for CAN ID if mapped."""
        try:
            return CanId(self.can_id).name
        except ValueError:
            if is_reserved_can_id(self.can_id):
                return f"USER_CUSTOM_0x{self.can_id:X}"
            return f"UNKNOWN_0x{self.can_id:X}"

    def hex_payload(self) -> str:
        """Format payload as space-separated hex bytes."""
        return " ".join(f"{b:02X}" for b in self.payload)

    def hex_frame(self) -> str:
        """Format full wire packet as space-separated hex bytes."""
        return " ".join(f"{b:02X}" for b in self.to_bytes())

    def summary(self) -> str:
        """Brief single-line summary of the frame."""
        status_info = ""
        if self.is_error():
            err = self.error_code()
            status_info = f" [ERR: {err.name if err else 'UNKNOWN'}]"
        elif self.is_ok():
            status_info = " [OK]"
        elif self.frame_type == FrameType.REQ:
            status_info = " [REQ]"
        elif self.frame_type == FrameType.ASYNC:
            status_info = " [ASYNC]"

        return (
            f"Frame({self.frame_type.name}, ID=0x{self.can_id:03X} <{self.can_id_name()}>, "
            f"SEQ={self.seq}, DLC={self.dlc}, DATA=[{self.hex_payload()}]{status_info})"
        )
