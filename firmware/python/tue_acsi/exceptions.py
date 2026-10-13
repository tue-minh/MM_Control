"""Exception types for the TUE ACSI protocol client."""

from __future__ import annotations

from .defs import Code


class AcsiError(Exception):
    """Base class for all ACSI client errors."""


class AcsiTimeout(AcsiError):
    """No complete response line arrived before the timeout."""


class AcsiFormatError(AcsiError):
    """A frame was malformed / could not be parsed."""


class AcsiDeviceError(AcsiError):
    """The board answered with a non-zero result code."""

    def __init__(self, code: int, raw: str = "") -> None:
        self.code = int(code)
        self.raw = raw
        try:
            self.name = Code(self.code).name
        except ValueError:
            self.name = f"UNKNOWN_{self.code}"
        super().__init__(f"{self.name} ({self.code}) {raw}".strip())
