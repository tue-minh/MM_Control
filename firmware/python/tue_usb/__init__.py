"""
TUE USB Binary Protocol Library for STM32 Motor Control.
"""

from .crc import crc8_sae_j1850, crc8_selftest
from .defs import (
    CanId,
    ErrorCode,
    FrameType,
    LedState,
    ServoAlarmBit,
    ServoMode,
    ServoTorque,
    is_reserved_can_id,
)
from .frame import Frame
from .protocol import FrameParser, encode_frame, encode_request
from .payloads import (
    Decoders,
    Encoders,
    PositionXYZ,
    ServoFeedback,
    ServoFeedbackDynamics,
    ServoFeedbackHealth,
    StepperConfig,
    StepperStatus,
    SysInfo,
    SysUid,
    mm_to_um,
    um_to_mm,
)
from .client import TueUsbClient, TueUsbError

__all__ = [
    # CRC
    "crc8_sae_j1850",
    "crc8_selftest",
    # Enums & Constants
    "CanId",
    "ErrorCode",
    "FrameType",
    "LedState",
    "ServoAlarmBit",
    "ServoMode",
    "ServoTorque",
    "is_reserved_can_id",
    # Frame & Protocol
    "Frame",
    "FrameParser",
    "encode_frame",
    "encode_request",
    # Payloads
    "Decoders",
    "Encoders",
    "PositionXYZ",
    "ServoFeedback",
    "ServoFeedbackDynamics",
    "ServoFeedbackHealth",
    "StepperConfig",
    "StepperStatus",
    "SysInfo",
    "SysUid",
    "mm_to_um",
    "um_to_mm",
    # Client
    "TueUsbClient",
    "TueUsbError",
]

__version__ = "1.0.0"
