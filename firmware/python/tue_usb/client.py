"""
High-level TueUsbClient for host-to-STM32 binary communication.

Manages serial I/O, sequence generation, frame synchronization, thread-safe
transactions, and dual-frame telemetry reassembly. Also includes a built-in
Mock Engine for hardware-free simulation and testing.
"""

import sys
import time
import struct
import threading
from typing import Callable, List, Optional, Tuple, Union

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

from .defs import (
    CanId,
    ErrorCode,
    FrameType,
    LedState,
    ServoMode,
    ServoTorque,
    is_reserved_can_id,
)
from .frame import Frame
from .protocol import FrameParser
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


class TueUsbError(Exception):
    """Exception raised when MCU returns FRAME_TYPE_RESP_ERR."""
    def __init__(self, code: ErrorCode, message: str = ""):
        self.code = code
        super().__init__(f"MCU error [{code.name}] (0x{int(code):02X}): {message or code.describe()}")


class TueUsbClient:
    """
    Host client interface for the TUE USB Binary CAN-over-USB protocol.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        timeout: float = 0.5,
        mock: bool = False
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.mock = mock

        self._seq = 0
        self._lock = threading.RLock()
        self._parser = FrameParser()
        self._serial: Optional[object] = None
        self._is_connected = False

        # Callbacks for telemetry / packet logging
        self.tx_callbacks: List[Callable[[Frame], None]] = []
        self.rx_callbacks: List[Callable[[Frame], None]] = []
        self.async_callbacks: List[Callable[[Frame], None]] = []

        # Mock simulated state for offline testing
        self._mock_pos_x_um = 0
        self._mock_pos_y_um = 0
        self._mock_pos_z_um = 0
        self._mock_stepper_enabled = True
        self._mock_led_state = 0
        self._mock_uptime_start = time.time()
        self._mock_servo_pos = {1: 2048, 2: 2048}
        self._mock_servo_speed = {1: 0, 2: 0}
        self._mock_servo_torque = {1: 1, 2: 1}

    def __enter__(self) -> "TueUsbClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    @staticmethod
    def list_ports() -> List[str]:
        """List available serial communication ports."""
        if not HAS_SERIAL:
            return ["MOCK_PORT"]
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def connect(self, port: Optional[str] = None):
        """Open serial port connection or activate mock mode."""
        if port is not None:
            self.port = port

        with self._lock:
            if self._is_connected:
                return

            if self.mock or self.port == "MOCK_PORT":
                self.mock = True
                self._is_connected = True
                return

            if not HAS_SERIAL:
                raise RuntimeError("pyserial is required for hardware serial communication")

            if not self.port:
                raise ValueError("Serial port must be specified (e.g. 'COM3' or '/dev/ttyACM0')")

            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            self._parser.reset()
            self._is_connected = True

    def disconnect(self):
        """Close active serial port."""
        with self._lock:
            if not self._is_connected:
                return
            if self._serial is not None and HAS_SERIAL:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._is_connected = False

    def is_connected(self) -> bool:
        """Check if client is connected."""
        return self._is_connected

    def _next_seq(self) -> int:
        """Generate monotonic 8-bit sequence number."""
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq

    def _dispatch_tx(self, frame: Frame):
        for cb in self.tx_callbacks:
            try:
                cb(frame)
            except Exception:
                pass

    def _dispatch_rx(self, frame: Frame):
        for cb in self.rx_callbacks:
            try:
                cb(frame)
            except Exception:
                pass
        if frame.frame_type == FrameType.ASYNC:
            for cb in self.async_callbacks:
                try:
                    cb(frame)
                except Exception:
                    pass

    # ==========================================================================
    # Mock Simulation Engine
    # ==========================================================================

    def _simulate_mcu_response(self, req: Frame) -> List[Frame]:
        """Simulate MCU firmware responses when running in mock mode."""
        can_id = req.can_id
        seq = req.seq
        p = req.payload

        # 1. System Commands
        if can_id == CanId.SYS_PING:
            token = p if len(p) >= 4 else b"\x00\x00\x00\x00"
            return [Frame(FrameType.RESP_OK, can_id, token[:4], seq)]

        elif can_id == CanId.SYS_INFO:
            # hw_id=1, hw_rev=1, maj=1, min=0, flags=0
            payload = struct.pack("<BBBB I", 1, 1, 1, 0, 0x00000000)
            return [Frame(FrameType.RESP_OK, can_id, payload, seq)]

        elif can_id == CanId.SYS_UID_PART1:
            payload = struct.pack("<II", 0x11223344, 0x55667788)
            return [Frame(FrameType.RESP_OK, can_id, payload, seq)]

        elif can_id == CanId.SYS_UID_PART2:
            payload = struct.pack("<I", 0x99AABBCC)
            return [Frame(FrameType.RESP_OK, can_id, payload, seq)]

        elif can_id == CanId.SYS_UPTIME:
            uptime = int((time.time() - self._mock_uptime_start) * 1000)
            return [Frame(FrameType.RESP_OK, can_id, struct.pack("<I", uptime), seq)]

        elif can_id == CanId.SYS_LED_CTRL:
            led_idx, state = struct.unpack("<BB", p[:2])
            if state == 2:
                self._mock_led_state ^= 1
            elif state in (0, 1):
                self._mock_led_state = state
            return [Frame(FrameType.RESP_OK, can_id, struct.pack("<BB", led_idx, self._mock_led_state), seq)]

        elif can_id == CanId.SYS_RESET:
            return [Frame(FrameType.RESP_OK, can_id, b"\x00", seq)]

        elif can_id == CanId.SYS_ECHO:
            return [Frame(FrameType.RESP_OK, can_id, p, seq)]

        # 2. Stepper Commands
        elif can_id == CanId.STEP_GET_INFO:
            payload = struct.pack("<HHHH", 80, 400, 50, 4000)
            return [Frame(FrameType.RESP_OK, can_id, payload, seq)]

        elif can_id == CanId.STEP_GET_STATUS:
            payload = struct.pack("<BBBB", 0, 1 if self._mock_stepper_enabled else 0, 0, 0)
            return [Frame(FrameType.RESP_OK, can_id, payload, seq)]

        elif can_id == CanId.STEP_ENABLE:
            if p[0] != 0xFF:
                self._mock_stepper_enabled = (p[0] != 0)
            return [Frame(FrameType.RESP_OK, can_id, bytes([1 if self._mock_stepper_enabled else 0]), seq)]

        elif can_id == CanId.STEP_GET_POS_XY or can_id == CanId.STEP_GET_TGT_XY:
            payload = struct.pack("<ii", self._mock_pos_x_um, self._mock_pos_y_um)
            return [Frame(FrameType.RESP_OK, can_id, payload, seq)]

        elif can_id == CanId.STEP_GET_POS_Z or can_id == CanId.STEP_GET_TGT_Z:
            payload = struct.pack("<i", self._mock_pos_z_um)
            return [Frame(FrameType.RESP_OK, can_id, payload, seq)]

        elif can_id == CanId.STEP_SETPOS_XY:
            x, y = struct.unpack("<ii", p[:8])
            self._mock_pos_x_um, self._mock_pos_y_um = x, y
            return [Frame(FrameType.RESP_OK, can_id, p[:8], seq)]

        elif can_id == CanId.STEP_SETPOS_Z:
            z = struct.unpack("<i", p[:4])[0]
            self._mock_pos_z_um = z
            return [Frame(FrameType.RESP_OK, can_id, p[:4], seq)]

        elif can_id == CanId.STEP_MOVE_COORDS:
            x, y = struct.unpack("<ii", p[:8])
            self._mock_pos_x_um, self._mock_pos_y_um = x, y
            return [Frame(FrameType.RESP_OK, can_id, b"\x01\x00", seq)]

        elif can_id == CanId.STEP_MOVE_EXEC:
            z = struct.unpack("<i", p[:4])[0]
            self._mock_pos_z_um = z
            payload = struct.pack("<ii", self._mock_pos_x_um, self._mock_pos_y_um)
            return [Frame(FrameType.RESP_OK, can_id, payload, seq)]

        elif can_id == CanId.STEP_MOVE_XY:
            x, y = struct.unpack("<ii", p[:8])
            self._mock_pos_x_um, self._mock_pos_y_um = x, y
            return [Frame(FrameType.RESP_OK, can_id, p[:8], seq)]

        elif can_id == CanId.STEP_MOVE_Z:
            z = struct.unpack("<i", p[:4])[0]
            self._mock_pos_z_um = z
            return [Frame(FrameType.RESP_OK, can_id, p[:4], seq)]

        elif can_id == CanId.STEP_MOVE_REL_XY:
            dx, dy = struct.unpack("<ii", p[:8])
            self._mock_pos_x_um += dx
            self._mock_pos_y_um += dy
            return [Frame(FrameType.RESP_OK, can_id, p[:8], seq)]

        elif can_id == CanId.STEP_SET_SPEED:
            return [Frame(FrameType.RESP_OK, can_id, p[:4], seq)]

        elif can_id == CanId.STEP_JOG_XY:
            return [Frame(FrameType.RESP_OK, can_id, p[:4], seq)]

        elif can_id == CanId.STEP_JOG_Z:
            return [Frame(FrameType.RESP_OK, can_id, p[:2], seq)]

        elif can_id == CanId.STEP_STOP or can_id == CanId.STEP_ABORT:
            return [Frame(FrameType.RESP_OK, can_id, b"\x00", seq)]

        elif can_id == CanId.STEP_WAIT_IDLE:
            return [Frame(FrameType.RESP_OK, can_id, b"\x00\x00\x00\x00", seq)]

        elif can_id == CanId.STEP_RAW_GET_STEPS or can_id == CanId.STEP_RAW_MOVE:
            m_id = p[0]
            payload = struct.pack("<Bi", m_id, 1000)
            return [Frame(FrameType.RESP_OK, can_id, payload, seq)]

        # 3. STS3215 Servos
        elif can_id == CanId.SERVO_PING:
            return [Frame(FrameType.RESP_OK, can_id, bytes([p[0], 0]), seq)]

        elif can_id == CanId.SERVO_SET_POS:
            s_id, accel, pos, speed = struct.unpack("<BBhh", p[:6])
            self._mock_servo_pos[s_id] = pos
            return [Frame(FrameType.RESP_OK, can_id, struct.pack("<Bh", s_id, pos), seq)]

        elif can_id == CanId.SERVO_GET_POS:
            s_id = p[0]
            pos = self._mock_servo_pos.get(s_id, 2048)
            return [Frame(FrameType.RESP_OK, can_id, struct.pack("<Bh", s_id, pos), seq)]

        elif can_id == CanId.SERVO_SET_SPEED or can_id == CanId.SERVO_GET_SPEED:
            s_id = p[0]
            spd = self._mock_servo_speed.get(s_id, 500)
            return [Frame(FrameType.RESP_OK, can_id, struct.pack("<Bh", s_id, spd), seq)]

        elif can_id == CanId.SERVO_TORQUE:
            s_id, st = struct.unpack("<BB", p[:2])
            if st != 0xFF:
                self._mock_servo_torque[s_id] = 1 if st != 0 else 0
            cur = self._mock_servo_torque.get(s_id, 1)
            return [Frame(FrameType.RESP_OK, can_id, struct.pack("<BB", s_id, cur), seq)]

        elif can_id == CanId.SERVO_TLIM or can_id == CanId.SERVO_MODE or can_id == CanId.SERVO_PWM:
            return [Frame(FrameType.RESP_OK, can_id, p, seq)]

        elif can_id == CanId.SERVO_GET_FB:
            s_id = p[0]
            pos = self._mock_servo_pos.get(s_id, 2048)
            # Frame 1: Dynamics (0x410)
            f1 = Frame(
                FrameType.RESP_OK,
                CanId.SERVO_FB_DYNAMICS,
                struct.pack("<BhhhB", s_id, pos, 0, 50, 0),
                seq
            )
            # Frame 2: Health (0x411)
            f2 = Frame(
                FrameType.RESP_OK,
                CanId.SERVO_FB_HEALTH,
                struct.pack("<BBbBhH", s_id, 120, 32, 0, 150, 0),
                seq
            )
            return [f1, f2]

        elif can_id == CanId.SERVO_MEM_READ:
            s_id, reg, length = struct.unpack("<BBB", p[:3])
            return [Frame(FrameType.RESP_OK, can_id, bytes([s_id, reg]) + b"\x00" * length, seq)]

        elif can_id == CanId.SERVO_MEM_WRITE or can_id == CanId.SERVO_SET_ID or can_id == CanId.SERVO_RESET:
            return [Frame(FrameType.RESP_OK, can_id, bytes([p[0], 0]), seq)]

        return [Frame(FrameType.RESP_ERR, can_id, bytes([ErrorCode.ERR_UNKNOWN_CAN_ID]), seq)]

    # ==========================================================================
    # Transaction Engine
    # ==========================================================================

    def transact(
        self,
        can_id: Union[CanId, int],
        payload: bytes = b"",
        timeout: Optional[float] = None,
        raise_on_error: bool = True
    ) -> Frame:
        """
        Send a request frame and synchronously wait for matching response frame.
        """
        cid = int(can_id)
        if is_reserved_can_id(cid):
            raise ValueError(f"CAN ID 0x{cid:X} is strictly reserved for user custom communications!")

        t_timeout = timeout if timeout is not None else self.timeout

        with self._lock:
            if not self._is_connected:
                raise ConnectionError("Client is not connected to a device or mock")

            seq = self._next_seq()
            req_frame = Frame(frame_type=FrameType.REQ, can_id=cid, payload=payload, seq=seq)
            self._dispatch_tx(req_frame)

            # Mock simulation branch
            if self.mock:
                replies = self._simulate_mcu_response(req_frame)
                reply = replies[0]
                self._dispatch_rx(reply)
                if raise_on_error and reply.is_error():
                    err = reply.error_code() or ErrorCode.ERR_INTERNAL
                    raise TueUsbError(err, f"CAN ID 0x{cid:03X}")
                return reply

            # Real serial transmission
            raw_bytes = req_frame.to_bytes()
            self._serial.reset_input_buffer()
            self._serial.write(raw_bytes)
            self._serial.flush()

            start_t = time.time()
            while (time.time() - start_t) < t_timeout:
                n_waiting = self._serial.in_waiting
                if n_waiting > 0:
                    chunk = self._serial.read(n_waiting)
                    frames = self._parser.feed(chunk)
                    for f in frames:
                        self._dispatch_rx(f)
                        if f.seq == seq and (f.can_id == cid or f.is_error()):
                            if raise_on_error and f.is_error():
                                err = f.error_code() or ErrorCode.ERR_INTERNAL
                                raise TueUsbError(err, f"CAN ID 0x{cid:03X}")
                            return f
                else:
                    time.sleep(0.002)

            raise TimeoutError(f"Transaction timed out for CAN ID 0x{cid:03X} (SEQ={seq}) after {t_timeout}s")

    def transact_dual(
        self,
        can_id: Union[CanId, int],
        payload: bytes = b"",
        timeout: Optional[float] = None
    ) -> Tuple[Frame, Frame]:
        """
        Handle commands expecting two consecutive response frames (e.g. SERVO_GET_FB).
        """
        cid = int(can_id)
        t_timeout = timeout if timeout is not None else self.timeout

        with self._lock:
            if not self._is_connected:
                raise ConnectionError("Client is not connected")

            seq = self._next_seq()
            req_frame = Frame(frame_type=FrameType.REQ, can_id=cid, payload=payload, seq=seq)
            self._dispatch_tx(req_frame)

            if self.mock:
                replies = self._simulate_mcu_response(req_frame)
                for r in replies:
                    self._dispatch_rx(r)
                if len(replies) >= 2:
                    return replies[0], replies[1]
                raise RuntimeError("Mock failed to generate dual frame response")

            raw_bytes = req_frame.to_bytes()
            self._serial.reset_input_buffer()
            self._serial.write(raw_bytes)
            self._serial.flush()

            collected: List[Frame] = []
            start_t = time.time()
            while (time.time() - start_t) < t_timeout:
                n_waiting = self._serial.in_waiting
                if n_waiting > 0:
                    chunk = self._serial.read(n_waiting)
                    frames = self._parser.feed(chunk)
                    for f in frames:
                        self._dispatch_rx(f)
                        if f.seq == seq:
                            if f.is_error():
                                err = f.error_code() or ErrorCode.ERR_INTERNAL
                                raise TueUsbError(err, f"CAN ID 0x{cid:03X}")
                            collected.append(f)
                            if len(collected) == 2:
                                return collected[0], collected[1]
                else:
                    time.sleep(0.002)

            raise TimeoutError(f"Dual transaction timed out (got {len(collected)} frames)")

    # ==========================================================================
    # High-Level System API (0x100 - 0x107)
    # ==========================================================================

    def ping(self, token: int = 0x12345678) -> int:
        """Send SYS_PING with 32-bit token, return echoed token."""
        resp = self.transact(CanId.SYS_PING, Encoders.sys_ping(token))
        return Decoders.sys_ping(resp.payload)

    def get_info(self) -> SysInfo:
        """Query firmware version, hardware ID, and build flags."""
        resp = self.transact(CanId.SYS_INFO)
        return Decoders.sys_info(resp.payload)

    def get_uid(self) -> SysUid:
        """Query 96-bit STM32 unique chip identifier."""
        r1 = self.transact(CanId.SYS_UID_PART1)
        u0, u1 = Decoders.sys_uid_part1(r1.payload)
        r2 = self.transact(CanId.SYS_UID_PART2)
        u2 = Decoders.sys_uid_part2(r2.payload)
        return SysUid(uid0=u0, uid1=u1, uid2=u2)

    def get_uptime_ms(self) -> int:
        """Query MCU uptime in milliseconds."""
        resp = self.transact(CanId.SYS_UPTIME)
        return Decoders.sys_uptime(resp.payload)

    def set_led(self, led_idx: int = 0, state: Union[LedState, int] = LedState.TOGGLE) -> int:
        """Set or toggle on-board status LED, return current state."""
        resp = self.transact(CanId.SYS_LED_CTRL, Encoders.sys_led(led_idx, state))
        _idx, current_state = Decoders.sys_led(resp.payload)
        return current_state

    def reset_mcu(self, magic: int = 0xDEADBEEF):
        """Trigger MCU software reboot."""
        self.transact(CanId.SYS_RESET, Encoders.sys_reset(magic), timeout=0.1, raise_on_error=False)

    def echo(self, data: bytes) -> bytes:
        """Send loopback data and return echo payload."""
        resp = self.transact(CanId.SYS_ECHO, Encoders.sys_echo(data))
        return resp.payload

    # ==========================================================================
    # High-Level Stepper Motion API (0x200 - 0x281)
    # ==========================================================================

    def step_get_config(self) -> StepperConfig:
        """Read stepper motor calibration and rate limits."""
        resp = self.transact(CanId.STEP_GET_INFO)
        return Decoders.step_get_info(resp.payload)

    def step_get_status(self) -> StepperStatus:
        """Read real-time stepper busy, enabled, and queue depth flags."""
        resp = self.transact(CanId.STEP_GET_STATUS)
        return Decoders.step_get_status(resp.payload)

    def step_enable(self, enable: Union[bool, int] = True) -> bool:
        """Enable or disable stepper driver outputs."""
        resp = self.transact(CanId.STEP_ENABLE, Encoders.step_enable(enable))
        return Decoders.step_enable(resp.payload)

    def step_get_pos_xy(self) -> Tuple[float, float]:
        """Read current XY tool position in millimetres."""
        resp = self.transact(CanId.STEP_GET_POS_XY)
        return Decoders.step_pos_xy(resp.payload)

    def step_get_pos_z(self) -> float:
        """Read current Z tool position in millimetres."""
        resp = self.transact(CanId.STEP_GET_POS_Z)
        return Decoders.step_pos_z(resp.payload)

    def step_get_pos_xyz(self) -> PositionXYZ:
        """Read combined XYZ tool position in millimetres."""
        x, y = self.step_get_pos_xy()
        z = self.step_get_pos_z()
        return PositionXYZ(x_mm=x, y_mm=y, z_mm=z)

    def step_get_tgt_xy(self) -> Tuple[float, float]:
        """Read target XY position in millimetres."""
        resp = self.transact(CanId.STEP_GET_TGT_XY)
        return Decoders.step_pos_xy(resp.payload)

    def step_get_tgt_z(self) -> float:
        """Read target Z position in millimetres."""
        resp = self.transact(CanId.STEP_GET_TGT_Z)
        return Decoders.step_pos_z(resp.payload)

    def step_setpos_xy(self, x_mm: float, y_mm: float) -> Tuple[float, float]:
        """Set current XY software position without moving."""
        resp = self.transact(CanId.STEP_SETPOS_XY, Encoders.step_setpos_xy(x_mm, y_mm))
        return Decoders.step_pos_xy(resp.payload)

    def step_setpos_z(self, z_mm: float) -> float:
        """Set current Z software position without moving."""
        resp = self.transact(CanId.STEP_SETPOS_Z, Encoders.step_setpos_z(z_mm))
        return Decoders.step_pos_z(resp.payload)

    def step_move_xy(self, x_mm: float, y_mm: float) -> Tuple[float, float]:
        """Move absolute XY at default feed and acceleration."""
        resp = self.transact(CanId.STEP_MOVE_XY, Encoders.step_move_xy(x_mm, y_mm))
        return Decoders.step_pos_xy(resp.payload)

    def step_move_z(self, z_mm: float, feed_mm_s: float = 20.0, accel_mm_s2: float = 200.0) -> float:
        """Move absolute Z at specified feed and acceleration."""
        resp = self.transact(CanId.STEP_MOVE_Z, Encoders.step_move_z(z_mm, feed_mm_s, accel_mm_s2))
        return Decoders.step_pos_z(resp.payload)

    def step_move_xyz(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        feed_mm_s: float = 60.0,
        accel_mm_s2: float = 600.0
    ) -> Tuple[float, float]:
        """Coordinated 3D move: latch XY coordinates and commit with Z."""
        self.transact(CanId.STEP_MOVE_COORDS, Encoders.step_move_coords(x_mm, y_mm))
        resp = self.transact(CanId.STEP_MOVE_EXEC, Encoders.step_move_exec(z_mm, feed_mm_s, accel_mm_s2))
        return Decoders.step_pos_xy(resp.payload)

    def step_move_rel_xy(self, dx_mm: float, dy_mm: float) -> Tuple[float, float]:
        """Move relative delta XY."""
        resp = self.transact(CanId.STEP_MOVE_REL_XY, Encoders.step_move_rel_xy(dx_mm, dy_mm))
        return Decoders.step_pos_xy(resp.payload)

    def step_set_speed(self, feed_mm_s: float, accel_mm_s2: float) -> Tuple[int, int]:
        """Configure default feed rate (mm/s) and acceleration (mm/s²)."""
        resp = self.transact(CanId.STEP_SET_SPEED, Encoders.step_set_speed(feed_mm_s, accel_mm_s2))
        return Decoders.step_set_speed(resp.payload)

    def step_jog_xy(self, vx_mm_s: float, vy_mm_s: float, accel_mm_s2: float = 600.0) -> Tuple[int, int]:
        """Start constant velocity continuous jog on XY axes."""
        resp = self.transact(CanId.STEP_JOG_XY, Encoders.step_jog_xy(vx_mm_s, vy_mm_s, accel_mm_s2))
        return Decoders.step_jog_xy(resp.payload)

    def step_jog_z(self, vz_mm_s: float, accel_mm_s2: float = 200.0) -> int:
        """Start constant velocity continuous jog on Z axis."""
        resp = self.transact(CanId.STEP_JOG_Z, Encoders.step_jog_z(vz_mm_s, accel_mm_s2))
        return Decoders.step_jog_z(resp.payload)

    def step_stop(self) -> bool:
        """Controlled deceleration stop."""
        resp = self.transact(CanId.STEP_STOP)
        return Decoders.step_stop(resp.payload)

    def step_abort(self) -> int:
        """Immediate step clock emergency abort."""
        resp = self.transact(CanId.STEP_ABORT)
        return resp.payload[0] if len(resp.payload) > 0 else 0

    def step_wait_idle(self, timeout_ms: int = 5000) -> int:
        """Wait until motion planner queue is empty and stepper motion completes."""
        resp = self.transact(
            CanId.STEP_WAIT_IDLE,
            Encoders.step_wait_idle(timeout_ms),
            timeout=(timeout_ms / 1000.0) + 1.0
        )
        return Decoders.step_wait_idle(resp.payload)

    def step_raw_get_steps(self, motor_id: int) -> int:
        """Read raw cumulative pulse count for motor 1..3."""
        resp = self.transact(CanId.STEP_RAW_GET_STEPS, Encoders.step_raw_get_steps(motor_id))
        _m_id, steps = Decoders.step_raw_steps(resp.payload)
        return steps

    def step_raw_move(self, motor_id: int, steps: int, feed_mm_s: int = 0) -> int:
        """Command raw step pulses directly to motor 1..3."""
        resp = self.transact(CanId.STEP_RAW_MOVE, Encoders.step_raw_move(motor_id, steps, feed_mm_s))
        _m_id, steps_done = Decoders.step_raw_steps(resp.payload)
        return steps_done

    # ==========================================================================
    # High-Level STS3215 Servo API (0x400 - 0x415)
    # ==========================================================================

    def servo_ping(self, servo_id: int) -> bool:
        """Ping STS3215 bus servo by ID (1..254)."""
        resp = self.transact(CanId.SERVO_PING, Encoders.servo_ping(servo_id))
        _id, ack = Decoders.servo_ping(resp.payload)
        return ack == 0

    def servo_set_pos(self, servo_id: int, pos: int, speed: int = 0, accel: int = 0) -> int:
        """Command servo goal position (0..4095 steps), speed, and acceleration."""
        resp = self.transact(CanId.SERVO_SET_POS, Encoders.servo_set_pos(servo_id, pos, speed, accel))
        _id, set_pos = Decoders.servo_pos(resp.payload)
        return set_pos

    def servo_get_pos(self, servo_id: int) -> int:
        """Read current servo encoder position (0..4095)."""
        resp = self.transact(CanId.SERVO_GET_POS, Encoders.servo_get_pos(servo_id))
        _id, cur_pos = Decoders.servo_pos(resp.payload)
        return cur_pos

    def servo_set_speed(self, servo_id: int, speed: int, accel: int = 0) -> int:
        """Set servo running speed (steps/s) and acceleration."""
        resp = self.transact(CanId.SERVO_SET_SPEED, Encoders.servo_set_speed(servo_id, speed, accel))
        _id, spd = Decoders.servo_speed(resp.payload)
        return spd

    def servo_get_speed(self, servo_id: int) -> int:
        """Read current servo velocity (steps/s)."""
        resp = self.transact(CanId.SERVO_GET_SPEED, Encoders.servo_get_speed(servo_id))
        _id, spd = Decoders.servo_speed(resp.payload)
        return spd

    def servo_torque_enable(self, servo_id: int, enable: bool = True) -> bool:
        """Enable or disable servo torque output."""
        state = ServoTorque.ON if enable else ServoTorque.OFF
        resp = self.transact(CanId.SERVO_TORQUE, Encoders.servo_torque(servo_id, state))
        _id, is_on = Decoders.servo_torque(resp.payload)
        return is_on

    def servo_calibrate_2048(self, servo_id: int) -> bool:
        """Calibrate current servo physical position to middle 2048 steps."""
        resp = self.transact(CanId.SERVO_TORQUE, Encoders.servo_torque(servo_id, ServoTorque.CALIBRATE))
        _id, is_on = Decoders.servo_torque(resp.payload)
        return is_on

    def servo_set_torque_limit(self, servo_id: int, limit_0p1pct: int) -> int:
        """Set maximum torque limit in 0.1% units (0..1000)."""
        resp = self.transact(CanId.SERVO_TLIM, Encoders.servo_tlim(servo_id, limit_0p1pct))
        _id, limit = Decoders.servo_tlim(resp.payload)
        return limit

    def servo_set_mode(self, servo_id: int, mode: Union[ServoMode, int]) -> int:
        """Set servo operation mode (Position=0, Speed=1, PWM=2, Step=3)."""
        resp = self.transact(CanId.SERVO_MODE, Encoders.servo_mode(servo_id, mode))
        _id, cur_mode = Decoders.servo_mode(resp.payload)
        return cur_mode

    def servo_set_pwm(self, servo_id: int, pwm: int) -> int:
        """Direct PWM duty cycle control (-1000..1000)."""
        resp = self.transact(CanId.SERVO_PWM, Encoders.servo_pwm(servo_id, pwm))
        _id, cur_pwm = Decoders.servo_pwm(resp.payload)
        return cur_pwm

    def servo_get_feedback(self, servo_id: int) -> ServoFeedback:
        """Query complete dual-frame telemetry feedback (Dynamics + Health)."""
        f_dyn, f_hlt = self.transact_dual(CanId.SERVO_GET_FB, Encoders.servo_get_fb(servo_id))
        dyn = Decoders.servo_fb_dynamics(f_dyn.payload)
        hlt = Decoders.servo_fb_health(f_hlt.payload)
        return ServoFeedback(dynamics=dyn, health=hlt)

    def servo_mem_read(self, servo_id: int, reg_addr: int, length: int) -> bytes:
        """Read 1..6 bytes directly from servo internal EEPROM/SRAM registers."""
        resp = self.transact(CanId.SERVO_MEM_READ, Encoders.servo_mem_read(servo_id, reg_addr, length))
        _id, _reg, data = Decoders.servo_mem_read(resp.payload)
        return data

    def servo_mem_write(self, servo_id: int, reg_addr: int, data: bytes) -> int:
        """Write 1..6 bytes into servo internal registers."""
        resp = self.transact(CanId.SERVO_MEM_WRITE, Encoders.servo_mem_write(servo_id, reg_addr, data))
        _id, _reg, status = Decoders.servo_mem_write(resp.payload)
        return status

    def servo_set_id(self, current_id: int, new_id: int) -> int:
        """Change servo bus ID."""
        resp = self.transact(CanId.SERVO_SET_ID, Encoders.servo_set_id(current_id, new_id))
        _id, status = Decoders.servo_set_id(resp.payload)
        return status

    def servo_reset(self, servo_id: int) -> int:
        """Restore servo factory default parameters."""
        resp = self.transact(CanId.SERVO_RESET, Encoders.servo_reset(servo_id))
        _id, status = Decoders.servo_reset(resp.payload)
        return status
