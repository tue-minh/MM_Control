"""Numeric protocol codes for the TUE ACSI protocol.

Everything on the wire is a number; these enums give the numbers names so host
code stays readable.
"""

from __future__ import annotations

from enum import IntEnum, IntFlag


class Target(IntEnum):
    """Device class of a frame (first number)."""

    SERVO = 1
    STEP = 2
    DC = 3
    SYS = 4


class Dir(IntEnum):
    """Direction code: 0 = GET (read data in), 1 = SET (write data out)."""

    GET = 0
    SET = 1


class ServoField(IntEnum):
    """Control field codes understood by the SERVO target."""

    PING = 1
    POS = 2
    VEL = 3
    ACC = 4
    TORQUE = 5
    TLIM = 6
    MODE = 7
    PWM = 8
    MEM = 9
    FB = 10
    ID = 11
    BAUD = 12
    LIMIT = 13
    TEMP = 14
    VOLT = 15
    LOAD = 16
    CUR = 17
    STATUS = 18
    MOVING = 19
    RESET = 20


class SysField(IntEnum):
    """Control field codes understood by the SYS target."""

    PING = 1
    ECHO = 2
    INFO = 3
    UID = 4
    UPTIME = 5
    LED = 6
    RESET = 7


class StepField(IntEnum):
    """Control field codes understood by the STEP target (``tue_stepper``)."""

    INFO = 1
    POS = 2
    TARGET = 3
    STATUS = 4
    ENABLE = 5
    MOVE = 6
    MOVE2D = 7
    MOVEZ = 8
    MOVEREL = 9
    JOG = 10
    JOGZ = 11
    STOP = 12
    ABORT = 13
    SETPOS = 14
    STEPS = 15
    RAW = 16
    WAIT = 17


class StepId(IntEnum):
    """The ``ID`` field of a STEP frame: 0 = tool, 1..3 = a single motor."""

    TOOL = 0  # the machine: X, Y, Z in CoreXY space
    M1 = 1    # CoreXY A = x + y   (STEP_1 / STEP_DIR1)
    M2 = 2    # CoreXY B = x - y   (STEP_2 / STEP_DIR2)
    M3 = 3    # Z                  (STEP_3 / STEP_DIR3)


class Code(IntEnum):
    """Result / error codes (response `CODE` field)."""

    OK = 0
    BAD_FORMAT = 2
    UNKNOWN_TARGET = 3
    UNKNOWN_FIELD = 4
    BAD_ARG = 5
    BUS_TIMEOUT = 6
    BUS_ERROR = 7
    NOT_SUPPORTED = 8
    BUSY = 9
    SERVO_FAULT = 10
    INTERNAL = 11
    QUEUE_FULL = 12
    TIMEOUT = 13


# Servo operating modes (STS3215 register 0x21)
class ServoMode(IntEnum):
    POSITION = 0
    SPEED = 1
    PWM = 2
    STEPPER = 3


# Servo baud-rate indices (STS3215 register 0x06)
class ServoBaud(IntEnum):
    BAUD_1000000 = 0
    BAUD_500000 = 1
    BAUD_250000 = 2
    BAUD_128000 = 3
    BAUD_115200 = 4
    BAUD_76800 = 5
    BAUD_57600 = 6
    BAUD_38400 = 7


# Servo hardware error-status bits (STS3215 register 0x41, SERVO STATUS / FB).
# Mask the integer returned by ``servo_status`` / ``servo_feedback``["status"].
class ServoFault(IntFlag):
    VOLTAGE = 1 << 0      # supply voltage out of range
    SENSOR = 1 << 1       # position sensor fault
    TEMPERATURE = 1 << 2  # over temperature
    CURRENT = 1 << 3      # overcurrent
    ANGLE = 1 << 4        # angle limit exceeded
    OVERLOAD = 1 << 5     # mechanical overload
    ALL = 0x3F            # all six fault bits


# Onboard LED index accepted by "4 0 1 6 <index> <state>"
LED_MCU = 0

# Ordered keys of the SERVO FB (field 10) payload.
FEEDBACK_FIELDS = ("pos", "spd", "load", "volt", "temp", "status", "moving", "cur")

# Ordered keys of the STEP STATUS payload.
STEP_STATUS_FIELDS = ("busy", "enabled")

# Ordered keys of the STEP INFO payload.
STEP_INFO_FIELDS = ("xy_steps_per_mm", "z_steps_per_mm", "min_rate", "max_rate")

# Ordered keys of the STEP POS / TARGET payload (millimetres).
AXES = ("x", "y", "z")


# -- STEP units -------------------------------------------------------------
#
# The wire is integer-only, so the STEP target carries distances in micrometres
# (1 um = 0.001 mm), speeds in um/s and accelerations in um/s^2. The client API
# works in millimetres and converts at the boundary.

UM_PER_MM = 1000


def mm_to_um(mm: float) -> int:
    """Millimetres -> micrometres as sent on the wire."""
    return int(round(float(mm) * UM_PER_MM))


def um_to_mm(um: int) -> float:
    """Micrometres received from the wire -> millimetres."""
    return int(um) / float(UM_PER_MM)
