"""
TUE USB Protocol Framing, Serialization, and Streaming Parser.

This module is completely independent of serial communication and can be reused
by ROS 2 nodes, GUIs, or simulation harnesses.
"""

from dataclasses import dataclass
import struct
from typing import Any, Dict, List, Optional, Tuple, Union

from .crc import crc16_ccitt
from .defs import (
    CRC_SIZE,
    HEADER_SIZE,
    MAX_PAYLOAD_SIZE,
    MIN_FRAME_SIZE,
    SOF1,
    SOF2,
    LedMode,
    StatusCode,
)
from .exceptions import TueCrcError, TueError


@dataclass
class TueFrame:
    """Represents a decoded TUE USB binary frame."""
    seq: int
    cmd: int
    status: int
    payload: bytes
    crc: int

    @property
    def is_ok(self) -> bool:
        """True if the frame status is StatusCode.OK."""
        return self.status == StatusCode.OK


def encode_frame(
    seq: int,
    cmd: int,
    status: int = StatusCode.OK,
    payload: bytes = b"",
) -> bytes:
    """Encode a protocol frame into binary wire format.

    Args:
        seq: Sequence number (0..255).
        cmd: Command ID (0..255).
        status: Status code (0 for requests, or status enum in responses).
        payload: Command payload bytes (max 256 bytes).

    Returns:
        Packed bytes including preamble, header, payload, and CRC-16.

    Raises:
        ValueError: If payload length exceeds MAX_PAYLOAD_SIZE.
    """
    payload_len = len(payload)
    if payload_len > MAX_PAYLOAD_SIZE:
        raise ValueError(
            f"Payload size {payload_len} exceeds maximum allowed {MAX_PAYLOAD_SIZE} bytes"
        )

    # Frame header: [SOF1, SOF2, SEQ, CMD, STATUS, LEN_L, LEN_H]
    header = struct.pack(
        "<BBBBBH",
        SOF1,
        SOF2,
        seq & 0xFF,
        cmd & 0xFF,
        status & 0xFF,
        payload_len & 0xFFFF,
    )

    # Calculate CRC-16 over SEQ through end of payload (everything except SOF1, SOF2)
    crc_data = header[2:] + payload
    crc = crc16_ccitt(crc_data)

    # Append little-endian CRC-16
    return header + payload + struct.pack("<H", crc)


def decode_frame(raw_bytes: bytes) -> TueFrame:
    """Decode a single complete frame from raw bytes.

    Args:
        raw_bytes: Buffer containing a complete frame.

    Returns:
        Decoded TueFrame object.

    Raises:
        TueError: If frame is truncated or SOF is invalid.
        TueCrcError: If CRC verification fails.
    """
    if len(raw_bytes) < MIN_FRAME_SIZE:
        raise TueError(f"Frame length {len(raw_bytes)} is shorter than minimum {MIN_FRAME_SIZE}")

    sof1, sof2, seq, cmd, status, payload_len = struct.unpack_from("<BBBBBH", raw_bytes, 0)
    if sof1 != SOF1 or sof2 != SOF2:
        raise TueError(f"Invalid frame start markers: 0x{sof1:02X} 0x{sof2:02X}")

    expected_total_len = HEADER_SIZE + payload_len + CRC_SIZE
    if len(raw_bytes) < expected_total_len:
        raise TueError(
            f"Truncated frame: expected {expected_total_len} bytes, got {len(raw_bytes)}"
        )

    payload = raw_bytes[HEADER_SIZE : HEADER_SIZE + payload_len]
    (received_crc,) = struct.unpack_from("<H", raw_bytes, HEADER_SIZE + payload_len)

    # Verify CRC
    crc_data = raw_bytes[2 : HEADER_SIZE + payload_len]
    computed_crc = crc16_ccitt(crc_data)
    if computed_crc != received_crc:
        raise TueCrcError(
            f"CRC mismatch: computed 0x{computed_crc:04X}, received 0x{received_crc:04X}"
        )

    return TueFrame(
        seq=seq,
        cmd=cmd,
        status=status,
        payload=payload,
        crc=received_crc,
    )


class TueFrameParser:
    """Streaming byte-oriented parser state machine.

    Correctly reassembles fragmented USB packets and unpacks concatenated frames.
    Recovers cleanly after noise or dropped bytes.
    """

    STATE_WAIT_SOF1 = 0
    STATE_WAIT_SOF2 = 1
    STATE_SEQ = 2
    STATE_CMD = 3
    STATE_STATUS = 4
    STATE_LEN_L = 5
    STATE_LEN_H = 6
    STATE_PAYLOAD = 7
    STATE_CRC_L = 8
    STATE_CRC_H = 9

    def __init__(self) -> None:
        self._state: int = self.STATE_WAIT_SOF1
        self._seq: int = 0
        self._cmd: int = 0
        self._status: int = 0
        self._len: int = 0
        self._payload: bytearray = bytearray()
        self._crc_l: int = 0
        self._crc: int = 0

    def reset(self) -> None:
        """Reset parser state machine to initial idle state."""
        self._state = self.STATE_WAIT_SOF1
        self._seq = 0
        self._cmd = 0
        self._status = 0
        self._len = 0
        self._payload = bytearray()
        self._crc_l = 0
        self._crc = 0

    def feed(self, data: bytes) -> List[TueFrame]:
        """Feed incoming byte chunks into the state machine.

        Args:
            data: Arbitrary chunk of incoming serial bytes.

        Returns:
            List of fully assembled and CRC-validated TueFrame instances.
        """
        frames: List[TueFrame] = []

        for b in data:
            if self._state == self.STATE_WAIT_SOF1:
                if b == SOF1:
                    self._state = self.STATE_WAIT_SOF2

            elif self._state == self.STATE_WAIT_SOF2:
                if b == SOF2:
                    self._state = self.STATE_SEQ
                elif b == SOF1:
                    # Consecutive 0xAA bytes
                    self._state = self.STATE_WAIT_SOF2
                else:
                    self._state = self.STATE_WAIT_SOF1

            elif self._state == self.STATE_SEQ:
                self._seq = b
                self._state = self.STATE_CMD

            elif self._state == self.STATE_CMD:
                self._cmd = b
                self._state = self.STATE_STATUS

            elif self._state == self.STATE_STATUS:
                self._status = b
                self._state = self.STATE_LEN_L

            elif self._state == self.STATE_LEN_L:
                self._len = b
                self._state = self.STATE_LEN_H

            elif self._state == self.STATE_LEN_H:
                self._len |= b << 8
                if self._len > MAX_PAYLOAD_SIZE:
                    # Invalid payload size, drop and resync
                    self._state = self.STATE_WAIT_SOF1
                elif self._len == 0:
                    self._payload = bytearray()
                    self._state = self.STATE_CRC_L
                else:
                    self._payload = bytearray()
                    self._state = self.STATE_PAYLOAD

            elif self._state == self.STATE_PAYLOAD:
                self._payload.append(b)
                if len(self._payload) >= self._len:
                    self._state = self.STATE_CRC_L

            elif self._state == self.STATE_CRC_L:
                self._crc_l = b
                self._state = self.STATE_CRC_H

            elif self._state == self.STATE_CRC_H:
                self._crc = self._crc_l | (b << 8)

                # Validate CRC
                header_data = struct.pack(
                    "<BBBH",
                    self._seq,
                    self._cmd,
                    self._status,
                    self._len,
                )
                computed_crc = crc16_ccitt(header_data + bytes(self._payload))

                if computed_crc == self._crc:
                    frames.append(
                        TueFrame(
                            seq=self._seq,
                            cmd=self._cmd,
                            status=self._status,
                            payload=bytes(self._payload),
                            crc=self._crc,
                        )
                    )

                self._state = self.STATE_WAIT_SOF1

            else:
                self._state = self.STATE_WAIT_SOF1

        return frames


# ==============================================================================
# Domain Payload Helpers
# ==============================================================================


def encode_led_cmd(
    led_index: int = 0,
    mode: Union[int, LedMode] = LedMode.TOGGLE,
    param: int = 0,
) -> bytes:
    """Encode payload for CMD_SYS_LED_CTRL (0x04).

    Args:
        led_index: Index of onboard LED (0 = LED_MCU).
        mode: LedMode enum (OFF, ON, TOGGLE, QUERY).
        param: Optional parameter (e.g. blink duration).

    Returns:
        Serialized payload bytes.
    """
    mode_val = int(mode) & 0xFF
    return struct.pack("<BBH", led_index & 0xFF, mode_val, param & 0xFFFF)


def decode_led_response(payload: bytes) -> Tuple[int, int]:
    """Decode response payload for CMD_SYS_LED_CTRL.

    Args:
        payload: Received response payload (2 bytes: led_index, current_state).

    Returns:
        Tuple of (led_index, current_state: 0=OFF, 1=ON).

    Raises:
        TueError: If payload length is less than 2 bytes.
    """
    if len(payload) < 2:
        raise TueError(f"LED response payload too short: expected >= 2 bytes, got {len(payload)}")
    led_index, current_state = struct.unpack_from("<BB", payload, 0)
    return led_index, current_state


def decode_info_response(payload: bytes) -> Dict[str, Any]:
    """Decode response payload for CMD_SYS_GET_INFO.

    Args:
        payload: Received response payload (19 bytes).

    Returns:
        Dictionary containing board_id, version, and mcu_uid.

    Raises:
        TueError: If payload length is less than 19 bytes.
    """
    if len(payload) < 19:
        raise TueError(f"Info response payload too short: expected 19 bytes, got {len(payload)}")

    board_id_bytes = payload[0:4]
    try:
        board_id = board_id_bytes.decode("ascii", errors="replace")
    except Exception:
        board_id = board_id_bytes.hex()

    fw_major, fw_minor, fw_patch = struct.unpack_from("<BBB", payload, 4)
    uid_bytes = payload[7:19]
    uid_hex = uid_bytes.hex().upper()

    return {
        "board_id": board_id,
        "fw_major": fw_major,
        "fw_minor": fw_minor,
        "fw_patch": fw_patch,
        "version_str": f"{fw_major}.{fw_minor}.{fw_patch}",
        "mcu_uid": uid_hex,
    }
