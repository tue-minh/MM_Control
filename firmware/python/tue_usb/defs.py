"""
Protocol definitions, enums, constants, and ID mappings for TUE USB Binary Protocol.

Matches definitions in Core/Inc/tue_usb_binary.h and docs/USB_BINARY_PROTOCOL.md.
"""

from enum import IntEnum, IntFlag

# Framing Constants
SOF1: int = 0xAA
SOF2: int = 0x55
EOF1: int = 0x55
EOF2: int = 0xAA

HEADER_SIZE: int = 7       # TYPE(1) + ID(4) + DLC(1) + SEQ(1)
CRC_SIZE: int = 1          # SAE J1850 CRC-8
FRAME_OVERHEAD: int = 12   # SOF(2) + HEADER(7) + CRC(1) + EOF(2)
MAX_DLC: int = 8
MIN_FRAME_LEN: int = FRAME_OVERHEAD           # 12 bytes
MAX_FRAME_LEN: int = FRAME_OVERHEAD + MAX_DLC  # 20 bytes


class FrameType(IntEnum):
    """Envelope Frame Types."""
    REQ = 0x01         # Host -> MCU Command / Request
    RESP_OK = 0x02     # MCU -> Host Acknowledged Success (with optional data)
    RESP_ERR = 0x03    # MCU -> Host Execution Error (payload[0] = ErrorCode)
    ASYNC = 0x04       # MCU -> Host Unsolicited Event / Telemetry


class ErrorCode(IntEnum):
    """Protocol and subsystem execution error codes."""
    OK = 0x00               # Success
    ERR_CRC = 0x01          # Frame CRC-8 check failed
    ERR_BAD_FORMAT = 0x02   # Truncated frame, framing mismatch, or DLC > 8
    ERR_UNKNOWN_CAN_ID = 0x03 # CAN ID is unmapped or unsupported
    ERR_BAD_ARG = 0x04      # Payload parameter out of range or malformed
    ERR_BUS_TIMEOUT = 0x05  # Downstream bus transaction timed out (e.g. STS3215 UART)
    ERR_BUS_FAULT = 0x06    # Hardware framing / DMA error
    ERR_BUSY = 0x07         # Subsystem busy with prior transaction
    ERR_SERVO_FAULT = 0x08  # Servo reported hardware alarm bitmask
    ERR_QUEUE_FULL = 0x09   # Stepper motion planner queue saturated (16 segments max)
    ERR_TIMEOUT = 0x0A      # Stepper wait deadline expired
    ERR_NOT_SUPPORTED = 0x0B # Command or direction not implemented
    ERR_INTERNAL = 0x0C     # MCU driver internal fault

    def describe(self) -> str:
        descriptions = {
            self.OK: "Operation successful",
            self.ERR_CRC: "Frame CRC-8 integrity verification failed",
            self.ERR_BAD_FORMAT: "Malformed frame envelope or invalid DLC",
            self.ERR_UNKNOWN_CAN_ID: "Unknown or unmapped CAN ID",
            self.ERR_BAD_ARG: "Payload argument value out of range",
            self.ERR_BUS_TIMEOUT: "Downstream serial bus timeout",
            self.ERR_BUS_FAULT: "Hardware communication fault / framing error",
            self.ERR_BUSY: "Subsystem is currently busy",
            self.ERR_SERVO_FAULT: "Bus servo reported hardware alarm condition",
            self.ERR_QUEUE_FULL: "Motion planner queue full (16 segments max)",
            self.ERR_TIMEOUT: "Motion execution wait timeout expired",
            self.ERR_NOT_SUPPORTED: "Command or feature not supported",
            self.ERR_INTERNAL: "Internal firmware driver error",
        }
        return descriptions.get(self, f"Unknown error code {int(self)}")


class CanId(IntEnum):
    """CAN Identifier allocation map."""

    # --- System Management (0x100 - 0x107) ---
    SYS_PING = 0x100
    SYS_INFO = 0x101
    SYS_UID_PART1 = 0x102
    SYS_UID_PART2 = 0x103
    SYS_UPTIME = 0x104
    SYS_LED_CTRL = 0x105
    SYS_RESET = 0x106
    SYS_ECHO = 0x107

    # --- Stepper Motor Kinematics (CoreXY + Z) (0x200 - 0x21A) ---
    STEP_GET_INFO = 0x200
    STEP_GET_STATUS = 0x201
    STEP_ENABLE = 0x202
    STEP_GET_POS_XY = 0x203
    STEP_GET_POS_Z = 0x204
    STEP_GET_TGT_XY = 0x205
    STEP_GET_TGT_Z = 0x206
    STEP_SETPOS_XY = 0x207
    STEP_SETPOS_Z = 0x208
    STEP_MOVE_COORDS = 0x210
    STEP_MOVE_EXEC = 0x211
    STEP_MOVE_XY = 0x212
    STEP_MOVE_Z = 0x213
    STEP_MOVE_REL_XY = 0x214
    STEP_SET_SPEED = 0x215
    STEP_JOG_XY = 0x216
    STEP_JOG_Z = 0x217
    STEP_STOP = 0x218
    STEP_ABORT = 0x219
    STEP_WAIT_IDLE = 0x21A

    # --- Raw Stepper Motor Channels (0x280 - 0x281) ---
    STEP_RAW_GET_STEPS = 0x280
    STEP_RAW_MOVE = 0x281

    # --- STS3215 Serial Bus Servos (0x400 - 0x415) ---
    SERVO_PING = 0x400
    SERVO_SET_POS = 0x401
    SERVO_GET_POS = 0x402
    SERVO_SET_SPEED = 0x403
    SERVO_GET_SPEED = 0x404
    SERVO_TORQUE = 0x405
    SERVO_TLIM = 0x406
    SERVO_MODE = 0x407
    SERVO_PWM = 0x408
    SERVO_GET_FB = 0x410
    SERVO_FB_DYNAMICS = 0x410  # Telemetry Part 1
    SERVO_FB_HEALTH = 0x411    # Telemetry Part 2
    SERVO_MEM_READ = 0x412
    SERVO_MEM_WRITE = 0x413
    SERVO_SET_ID = 0x414
    SERVO_RESET = 0x415


class LedState(IntEnum):
    """LED control states."""
    OFF = 0
    ON = 1
    TOGGLE = 2
    QUERY = 0xFF


class ServoMode(IntEnum):
    """STS3215 Operating Modes."""
    POSITION = 0
    SPEED = 1
    PWM = 2
    STEP = 3
    QUERY = 0xFF


class ServoTorque(IntEnum):
    """Servo torque commands."""
    OFF = 0
    ON = 1
    CALIBRATE = 128
    QUERY = 0xFF


class ServoAlarmBit(IntFlag):
    """STS3215 Hardware Alarm Bitmask."""
    VOLTAGE = 0x01      # Input voltage out of safe bounds
    SENSOR = 0x02       # Magnetic encoder error
    TEMPERATURE = 0x04  # Overheating alarm
    CURRENT = 0x08      # Overcurrent alarm
    ANGLE = 0x10        # Target position beyond software limits
    OVERLOAD = 0x20     # Mechanical overload stall


def is_reserved_can_id(can_id: int) -> bool:
    """
    Check if a CAN ID collides with the user-reserved 0x3... address space.

    Firmware rule: CAN ID 0x3.. is strictly reserved for user custom communication.
    Standard 0x300-0x3FF, 16-bit 0x3000-0x3FFF and 29-bit 0x30000000-0x3FFFFFFF.
    """
    cid = int(can_id) & 0xFFFFFFFF
    return (
        (0x300 <= cid <= 0x3FF) or
        (0x3000 <= cid <= 0x3FFF) or
        (0x30000000 <= cid <= 0x3FFFFFFF)
    )
