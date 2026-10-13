"""TUE ACSI: host-side client for the STM32 MM_Control USB command protocol.

Field-oriented token frames with literal text payloads. See
``docs/ACSI_PROTOCOL.md`` for the wire specification.
"""

from __future__ import annotations

from .defs import (
    AXES,
    FEEDBACK_FIELDS,
    STEP_INFO_FIELDS,
    STEP_STATUS_FIELDS,
    UM_PER_MM,
    Code,
    Dir,
    LED_MCU,
    ServoBaud,
    ServoFault,
    ServoField,
    ServoMode,
    StepField,
    StepId,
    SysField,
    Target,
    mm_to_um,
    um_to_mm,
)
from .exceptions import (
    AcsiDeviceError,
    AcsiError,
    AcsiFormatError,
    AcsiTimeout,
)
from .protocol import (
    Response,
    build_body,
    decode,
    encode,
    encode_line,
)

__all__ = [
    "AXES",
    "AcsiClient",
    "AcsiDeviceError",
    "AcsiError",
    "AcsiFormatError",
    "AcsiTimeout",
    "Code",
    "Dir",
    "FEEDBACK_FIELDS",
    "LED_MCU",
    "Response",
    "ServoBaud",
    "ServoFault",
    "ServoField",
    "ServoMode",
    "STEP_INFO_FIELDS",
    "STEP_STATUS_FIELDS",
    "StepField",
    "StepId",
    "SysField",
    "Target",
    "UM_PER_MM",
    "build_body",
    "decode",
    "encode",
    "encode_line",
    "mm_to_um",
    "um_to_mm",
]

__version__ = "1.0.0"


def __getattr__(name: str):
    # AcsiClient is imported lazily so `import tue_acsi` works without pyserial.
    if name == "AcsiClient":
        from .client import AcsiClient

        return AcsiClient
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
