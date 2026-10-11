"""
TUE USB - Python Client Library for STM32 Motion Control Board.
"""

from .defs import (
    CommandId,
    LedMode,
    StatusCode,
    SOF1,
    SOF2,
    HEADER_SIZE,
    CRC_SIZE,
    MAX_PAYLOAD_SIZE,
)
from .crc import crc16_ccitt, verify_crc16
from .exceptions import (
    TueError,
    TueTimeoutError,
    TueCrcError,
    TueConnectionError,
    TueProtocolError,
)
from .protocol import (
    TueFrame,
    TueFrameParser,
    encode_frame,
    decode_frame,
    encode_led_cmd,
    decode_led_response,
    decode_info_response,
)
from .client import TueClient

__version__ = "1.0.0"

__all__ = [
    "TueClient",
    "TueFrame",
    "TueFrameParser",
    "encode_frame",
    "decode_frame",
    "encode_led_cmd",
    "decode_led_response",
    "decode_info_response",
    "crc16_ccitt",
    "verify_crc16",
    "CommandId",
    "LedMode",
    "StatusCode",
    "SOF1",
    "SOF2",
    "HEADER_SIZE",
    "CRC_SIZE",
    "MAX_PAYLOAD_SIZE",
    "TueError",
    "TueTimeoutError",
    "TueCrcError",
    "TueConnectionError",
    "TueProtocolError",
]
