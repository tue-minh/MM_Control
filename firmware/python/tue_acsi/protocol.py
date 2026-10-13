"""ACSI frame encoding/decoding (transport free).

Grammar::

    request : <TARGET> <ID> <DIR> <FIELD> [ARG ...]
    response: <TARGET> <ID> <CODE> [VALUE ...]

Header fields are numbers; value tokens are numbers, except that a response may
carry literal text (whitespace separated, e.g. the board name). There is no
checksum field.
"""

from __future__ import annotations

from dataclasses import dataclass, field as dc_field
from typing import List, Union

from .defs import Code, Dir, Target
from .exceptions import AcsiFormatError


def build_body(
    target: Union[Target, int],
    device_id: int,
    direction: Union[Dir, int],
    field: Union[int, object],
    *args: int,
) -> str:
    """Assemble a frame body (without line terminator)."""
    parts = [str(int(target)), str(int(device_id)), str(int(direction)), str(int(field))]
    parts.extend(str(int(a)) for a in args)
    return " ".join(parts)


def encode_line(
    target: Union[Target, int],
    device_id: int,
    direction: Union[Dir, int],
    field: Union[int, object],
    *args: int,
) -> str:
    """Return a complete request line (body only), without terminator."""
    return build_body(target, device_id, direction, field, *args)


def encode(
    target: Union[Target, int],
    device_id: int,
    direction: Union[Dir, int],
    field: Union[int, object],
    *args: int,
) -> bytes:
    """Return the bytes to write to the port, including the ``\\n`` terminator."""
    return (encode_line(target, device_id, direction, field, *args) + "\n").encode("ascii")


# -- responses --------------------------------------------------------------

@dataclass
class Response:
    """A decoded response frame (all fields are integers)."""

    target: int
    device_id: int
    code: int
    payload: List[Union[int, str]] = dc_field(default_factory=list)
    raw: str = ""

    @property
    def ok(self) -> bool:
        return self.code == int(Code.OK)

    @property
    def error_code(self) -> int:
        return self.code

    @property
    def error_name(self) -> str:
        try:
            return Code(self.code).name
        except ValueError:
            return f"UNKNOWN_{self.code}"

    @property
    def value(self) -> int:
        """First payload number."""
        if not self.payload:
            raise AcsiFormatError(f"no value in response {self.raw!r}")
        return self.payload[0]

    def __str__(self) -> str:  # pragma: no cover - cosmetic
        return self.raw


def decode(line: str) -> Response:
    """Parse a response line; numeric tokens become ints, others stay text."""
    body = line.rstrip("\r\n")
    tokens = body.split()
    if len(tokens) < 3:
        raise AcsiFormatError(f"too few fields in {body!r}")
    try:
        target, device_id, code = (int(tok, 10) for tok in tokens[:3])
    except ValueError as exc:
        raise AcsiFormatError(f"non-numeric header in {body!r}") from exc
    payload: List[Union[int, str]] = []
    for tok in tokens[3:]:
        try:
            payload.append(int(tok, 10))
        except ValueError:
            payload.append(tok)
    return Response(
        target=target,
        device_id=device_id,
        code=code,
        payload=payload,
        raw=body,
    )
