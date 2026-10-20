"""
Typed payload encoders and decoders for all TUE USB Binary commands.

Provides high-level data models and pack/unpack functions for:
  - System Management (0x100 - 0x107)
  - Stepper Kinematics & Raw Channels (0x200 - 0x281)
  - STS3215 Serial Bus Servos (0x400 - 0x415)
"""

import struct
from dataclasses import dataclass
from typing import Optional, Tuple, Union

from .defs import CanId, LedState, ServoMode, ServoTorque, ServoAlarmBit


# ==============================================================================
# Unit Conversion Utilities
# ==============================================================================

def mm_to_um(mm: float) -> int:
    """Convert millimetres to integer micrometres."""
    return int(round(mm * 1000.0))


def um_to_mm(um: int) -> float:
    """Convert integer micrometres to millimetres."""
    return float(um) / 1000.0


# ==============================================================================
# System Data Models (0x100 - 0x107)
# ==============================================================================

@dataclass
class SysInfo:
    """System hardware and firmware build info."""
    hw_id: int
    hw_rev: int
    fw_major: int
    fw_minor: int
    build_flags: int

    @property
    def version_string(self) -> str:
        return f"v{self.fw_major}.{self.fw_minor}"

    def __str__(self) -> str:
        return (
            f"SysInfo(HW ID={self.hw_id}, Rev={self.hw_rev}, "
            f"Firmware={self.version_string}, Flags=0x{self.build_flags:08X})"
        )


@dataclass
class SysUid:
    """STM32 96-bit Unique Identifier."""
    uid0: int
    uid1: int
    uid2: int

    @property
    def hex_string(self) -> str:
        return f"{self.uid0:08X}-{self.uid1:08X}-{self.uid2:08X}"

    def __str__(self) -> str:
        return f"UID: {self.hex_string}"


# ==============================================================================
# Stepper Data Models (0x200 - 0x281)
# ==============================================================================

@dataclass
class StepperConfig:
    """Stepper configuration and motor rate limits."""
    xy_steps_per_mm: int
    z_steps_per_mm: int
    min_rate_steps_s: int
    max_rate_steps_s: int


@dataclass
class StepperStatus:
    """Real-time stepper subsystem operational status."""
    is_busy: bool
    is_enabled: bool
    queue_depth: int


@dataclass
class PositionXYZ:
    """Position representation in both millimetres and micrometres."""
    x_mm: float
    y_mm: float
    z_mm: float

    @property
    def x_um(self) -> int:
        return mm_to_um(self.x_mm)

    @property
    def y_um(self) -> int:
        return mm_to_um(self.y_mm)

    @property
    def z_um(self) -> int:
        return mm_to_um(self.z_mm)


# ==============================================================================
# Servo Data Models (0x400 - 0x415)
# ==============================================================================

@dataclass
class ServoFeedbackDynamics:
    """STS3215 Dynamic telemetry (Part 1 - CAN ID 0x410)."""
    servo_id: int
    position: int         # 0..4095 steps (0.088° per step)
    speed: int            # steps / sec
    load_raw: int         # internal units (-1000..1000)
    is_moving: bool

    @property
    def angle_deg(self) -> float:
        """Position converted to degrees (0°..360°)."""
        return (self.position * 360.0) / 4096.0

    @property
    def load_percent(self) -> float:
        """Torque load in percent (0.1% resolution)."""
        return self.load_raw / 10.0


@dataclass
class ServoFeedbackHealth:
    """STS3215 Health telemetry (Part 2 - CAN ID 0x411)."""
    servo_id: int
    voltage_v: float      # Bus voltage in Volts (from 0.1V raw)
    temperature_c: int    # Internal temperature in Celsius
    status_mask: int      # Alarm bitmask
    current_ma: int       # Motor drive current in mA

    @property
    def alarms(self) -> list[str]:
        """List of active alarm strings."""
        active = []
        for bit in ServoAlarmBit:
            if self.status_mask & bit:
                active.append(bit.name)
        return active


@dataclass
class ServoFeedback:
    """Complete combined STS3215 telemetry feedback."""
    dynamics: ServoFeedbackDynamics
    health: ServoFeedbackHealth


# ==============================================================================
# Encoders (Host -> Payload Bytes)
# ==============================================================================

class Encoders:
    """Payload serialization helpers."""

    # --- System ---
    @staticmethod
    def sys_ping(token: int = 0) -> bytes:
        return struct.pack("<I", token & 0xFFFFFFFF)

    @staticmethod
    def sys_led(led_idx: int = 0, state: Union[LedState, int] = LedState.QUERY) -> bytes:
        return struct.pack("<BB", led_idx & 0xFF, int(state) & 0xFF)

    @staticmethod
    def sys_reset(magic: int = 0xDEADBEEF) -> bytes:
        return struct.pack("<I", magic & 0xFFFFFFFF)

    @staticmethod
    def sys_echo(data: bytes) -> bytes:
        if len(data) > 8:
            raise ValueError("SYS_ECHO payload cannot exceed 8 bytes")
        return bytes(data)

    # --- Stepper Kinematics ---
    @staticmethod
    def step_enable(enable: Union[bool, int]) -> bytes:
        if isinstance(enable, bool):
            val = 1 if enable else 0
        else:
            val = int(enable) & 0xFF
        return struct.pack("<B", val)

    @staticmethod
    def step_setpos_xy(x_mm: float, y_mm: float) -> bytes:
        return struct.pack("<ii", mm_to_um(x_mm), mm_to_um(y_mm))

    @staticmethod
    def step_setpos_z(z_mm: float) -> bytes:
        return struct.pack("<i", mm_to_um(z_mm))

    @staticmethod
    def step_move_coords(x_mm: float, y_mm: float) -> bytes:
        """Latch X/Y coordinates before STEP_MOVE_EXEC."""
        return struct.pack("<ii", mm_to_um(x_mm), mm_to_um(y_mm))

    @staticmethod
    def step_move_exec(z_mm: float, feed_mm_s: float, accel_mm_s2: float) -> bytes:
        """Commit 3D move with previously latched X/Y coordinates."""
        return struct.pack("<iHH", mm_to_um(z_mm), int(feed_mm_s), int(accel_mm_s2))

    @staticmethod
    def step_move_xy(x_mm: float, y_mm: float) -> bytes:
        return struct.pack("<ii", mm_to_um(x_mm), mm_to_um(y_mm))

    @staticmethod
    def step_move_z(z_mm: float, feed_mm_s: float, accel_mm_s2: float) -> bytes:
        return struct.pack("<iHH", mm_to_um(z_mm), int(feed_mm_s), int(accel_mm_s2))

    @staticmethod
    def step_move_rel_xy(dx_mm: float, dy_mm: float) -> bytes:
        return struct.pack("<ii", mm_to_um(dx_mm), mm_to_um(dy_mm))

    @staticmethod
    def step_set_speed(default_feed_mm_s: float, default_accel_mm_s2: float) -> bytes:
        return struct.pack("<HH", int(default_feed_mm_s), int(default_accel_mm_s2))

    @staticmethod
    def step_jog_xy(vx_mm_s: float, vy_mm_s: float, accel_mm_s2: float) -> bytes:
        return struct.pack("<hhH", int(vx_mm_s), int(vy_mm_s), int(accel_mm_s2))

    @staticmethod
    def step_jog_z(vz_mm_s: float, accel_mm_s2: float) -> bytes:
        return struct.pack("<hH", int(vz_mm_s), int(accel_mm_s2))

    @staticmethod
    def step_wait_idle(timeout_ms: int = 0) -> bytes:
        return struct.pack("<I", timeout_ms & 0xFFFFFFFF)

    @staticmethod
    def step_raw_get_steps(motor_id: int) -> bytes:
        return struct.pack("<B", motor_id & 0xFF)

    @staticmethod
    def step_raw_move(motor_id: int, steps: int, feed_mm_s: int = 0) -> bytes:
        return struct.pack("<BBiH", motor_id & 0xFF, 0, int(steps), int(feed_mm_s) & 0xFFFF)

    # --- STS3215 Servos ---
    @staticmethod
    def servo_ping(servo_id: int) -> bytes:
        return struct.pack("<B", servo_id & 0xFF)

    @staticmethod
    def servo_set_pos(servo_id: int, pos: int, speed: int = 0, accel: int = 0) -> bytes:
        return struct.pack("<BBhh", servo_id & 0xFF, accel & 0xFF, int(pos), int(speed))

    @staticmethod
    def servo_get_pos(servo_id: int) -> bytes:
        return struct.pack("<B", servo_id & 0xFF)

    @staticmethod
    def servo_set_speed(servo_id: int, speed: int, accel: int = 0) -> bytes:
        return struct.pack("<BBh", servo_id & 0xFF, accel & 0xFF, int(speed))

    @staticmethod
    def servo_get_speed(servo_id: int) -> bytes:
        return struct.pack("<B", servo_id & 0xFF)

    @staticmethod
    def servo_torque(servo_id: int, state: Union[ServoTorque, int]) -> bytes:
        return struct.pack("<BB", servo_id & 0xFF, int(state) & 0xFF)

    @staticmethod
    def servo_tlim(servo_id: int, limit_0p1pct: int) -> bytes:
        return struct.pack("<BH", servo_id & 0xFF, limit_0p1pct & 0xFFFF)

    @staticmethod
    def servo_mode(servo_id: int, mode: Union[ServoMode, int]) -> bytes:
        return struct.pack("<BB", servo_id & 0xFF, int(mode) & 0xFF)

    @staticmethod
    def servo_pwm(servo_id: int, pwm: int) -> bytes:
        return struct.pack("<Bh", servo_id & 0xFF, int(pwm))

    @staticmethod
    def servo_get_fb(servo_id: int) -> bytes:
        return struct.pack("<B", servo_id & 0xFF)

    @staticmethod
    def servo_mem_read(servo_id: int, reg_addr: int, length: int) -> bytes:
        return struct.pack("<BBB", servo_id & 0xFF, reg_addr & 0xFF, length & 0xFF)

    @staticmethod
    def servo_mem_write(servo_id: int, reg_addr: int, data: bytes) -> bytes:
        if len(data) > 6:
            raise ValueError("SERVO_MEM_WRITE max write length is 6 bytes")
        return struct.pack("<BB", servo_id & 0xFF, reg_addr & 0xFF) + bytes(data)

    @staticmethod
    def servo_set_id(current_id: int, new_id: int) -> bytes:
        return struct.pack("<BB", current_id & 0xFF, new_id & 0xFF)

    @staticmethod
    def servo_reset(servo_id: int) -> bytes:
        return struct.pack("<B", servo_id & 0xFF)


# ==============================================================================
# Decoders (Payload Bytes -> High-Level Python Objects)
# ==============================================================================

class Decoders:
    """Payload deserialization helpers."""

    # --- System ---
    @staticmethod
    def sys_ping(payload: bytes) -> int:
        if len(payload) < 4:
            return 0
        return struct.unpack("<I", payload[:4])[0]

    @staticmethod
    def sys_info(payload: bytes) -> SysInfo:
        if len(payload) < 8:
            raise ValueError("SYS_INFO payload must be 8 bytes")
        hw_id, hw_rev, maj, min_, flags = struct.unpack("<BBBB I", payload[:8])
        return SysInfo(hw_id=hw_id, hw_rev=hw_rev, fw_major=maj, fw_minor=min_, build_flags=flags)

    @staticmethod
    def sys_uid_part1(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 8:
            raise ValueError("SYS_UID_PART1 payload must be 8 bytes")
        return struct.unpack("<II", payload[:8])

    @staticmethod
    def sys_uid_part2(payload: bytes) -> int:
        if len(payload) < 4:
            raise ValueError("SYS_UID_PART2 payload must be 4 bytes")
        return struct.unpack("<I", payload[:4])[0]

    @staticmethod
    def sys_uptime(payload: bytes) -> int:
        if len(payload) < 4:
            return 0
        return struct.unpack("<I", payload[:4])[0]

    @staticmethod
    def sys_led(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 2:
            return (0, 0)
        return struct.unpack("<BB", payload[:2])

    # --- Stepper ---
    @staticmethod
    def step_get_info(payload: bytes) -> StepperConfig:
        if len(payload) < 8:
            raise ValueError("STEP_GET_INFO payload must be 8 bytes")
        xy_spmm, z_spmm, min_rate, max_rate = struct.unpack("<HHHH", payload[:8])
        return StepperConfig(
            xy_steps_per_mm=xy_spmm,
            z_steps_per_mm=z_spmm,
            min_rate_steps_s=min_rate,
            max_rate_steps_s=max_rate
        )

    @staticmethod
    def step_get_status(payload: bytes) -> StepperStatus:
        if len(payload) < 3:
            raise ValueError("STEP_GET_STATUS payload must be at least 3 bytes")
        is_busy, is_enabled, q_depth = struct.unpack("<BBB", payload[:3])
        return StepperStatus(
            is_busy=bool(is_busy),
            is_enabled=bool(is_enabled),
            queue_depth=q_depth
        )

    @staticmethod
    def step_enable(payload: bytes) -> bool:
        if len(payload) < 1:
            return False
        return bool(payload[0])

    @staticmethod
    def step_pos_xy(payload: bytes) -> Tuple[float, float]:
        if len(payload) < 8:
            raise ValueError("STEP_POS_XY payload must be 8 bytes")
        x_um, y_um = struct.unpack("<ii", payload[:8])
        return um_to_mm(x_um), um_to_mm(y_um)

    @staticmethod
    def step_pos_z(payload: bytes) -> float:
        if len(payload) < 4:
            raise ValueError("STEP_POS_Z payload must be 4 bytes")
        z_um = struct.unpack("<i", payload[:4])[0]
        return um_to_mm(z_um)

    @staticmethod
    def step_move_coords(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 2:
            return (0, 0)
        ack, q_depth = struct.unpack("<BB", payload[:2])
        return ack, q_depth

    @staticmethod
    def step_set_speed(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 4:
            return (0, 0)
        feed, accel = struct.unpack("<HH", payload[:4])
        return feed, accel

    @staticmethod
    def step_jog_xy(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 4:
            return (0, 0)
        return struct.unpack("<hh", payload[:4])

    @staticmethod
    def step_jog_z(payload: bytes) -> int:
        if len(payload) < 2:
            return 0
        return struct.unpack("<h", payload[:2])[0]

    @staticmethod
    def step_stop(payload: bytes) -> bool:
        if len(payload) < 1:
            return False
        return bool(payload[0])

    @staticmethod
    def step_wait_idle(payload: bytes) -> int:
        if len(payload) < 4:
            return 0
        return struct.unpack("<I", payload[:4])[0]

    @staticmethod
    def step_raw_steps(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 5:
            raise ValueError("STEP_RAW payload must be 5 bytes")
        motor_id, steps = struct.unpack("<Bi", payload[:5])
        return motor_id, steps

    # --- STS3215 Servos ---
    @staticmethod
    def servo_ping(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 2:
            return (0, 0)
        return struct.unpack("<BB", payload[:2])

    @staticmethod
    def servo_pos(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 3:
            raise ValueError("SERVO_POS payload must be 3 bytes")
        s_id, pos = struct.unpack("<Bh", payload[:3])
        return s_id, pos

    @staticmethod
    def servo_speed(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 3:
            raise ValueError("SERVO_SPEED payload must be 3 bytes")
        s_id, spd = struct.unpack("<Bh", payload[:3])
        return s_id, spd

    @staticmethod
    def servo_torque(payload: bytes) -> Tuple[int, bool]:
        if len(payload) < 2:
            return (0, False)
        s_id, torque = struct.unpack("<BB", payload[:2])
        return s_id, bool(torque)

    @staticmethod
    def servo_tlim(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 3:
            return (0, 0)
        s_id, tlim = struct.unpack("<BH", payload[:3])
        return s_id, tlim

    @staticmethod
    def servo_mode(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 2:
            return (0, 0)
        return struct.unpack("<BB", payload[:2])

    @staticmethod
    def servo_pwm(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 3:
            return (0, 0)
        return struct.unpack("<Bh", payload[:3])

    @staticmethod
    def servo_fb_dynamics(payload: bytes) -> ServoFeedbackDynamics:
        if len(payload) < 8:
            raise ValueError("SERVO_FB_DYNAMICS payload must be 8 bytes")
        s_id, pos, spd, load, moving = struct.unpack("<BhhhB", payload[:8])
        return ServoFeedbackDynamics(
            servo_id=s_id,
            position=pos,
            speed=spd,
            load_raw=load,
            is_moving=bool(moving)
        )

    @staticmethod
    def servo_fb_health(payload: bytes) -> ServoFeedbackHealth:
        if len(payload) < 8:
            raise ValueError("SERVO_FB_HEALTH payload must be 8 bytes")
        s_id, volt_raw, temp_c, status, cur_ma, _res = struct.unpack("<BBbBhH", payload[:8])
        return ServoFeedbackHealth(
            servo_id=s_id,
            voltage_v=float(volt_raw) / 10.0,
            temperature_c=temp_c,
            status_mask=status,
            current_ma=cur_ma
        )

    @staticmethod
    def servo_mem_read(payload: bytes) -> Tuple[int, int, bytes]:
        if len(payload) < 2:
            raise ValueError("SERVO_MEM_READ payload too short")
        s_id, reg = struct.unpack("<BB", payload[:2])
        data = payload[2:]
        return s_id, reg, data

    @staticmethod
    def servo_mem_write(payload: bytes) -> Tuple[int, int, int]:
        if len(payload) < 3:
            return (0, 0, 0)
        return struct.unpack("<BBB", payload[:3])

    @staticmethod
    def servo_set_id(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 2:
            return (0, 0)
        return struct.unpack("<BB", payload[:2])

    @staticmethod
    def servo_reset(payload: bytes) -> Tuple[int, int]:
        if len(payload) < 2:
            return (0, 0)
        return struct.unpack("<BB", payload[:2])
