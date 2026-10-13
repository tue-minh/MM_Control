"""Serial client for the TUE ACSI protocol over USB CDC (Virtual COM Port)."""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Sequence, Tuple

from . import protocol
from .defs import (
    AXES,
    FEEDBACK_FIELDS,
    LED_MCU,
    STEP_INFO_FIELDS,
    STEP_STATUS_FIELDS,
    Dir,
    ServoField,
    StepField,
    StepId,
    SysField,
    Target,
    mm_to_um,
    um_to_mm,
)
from .exceptions import AcsiDeviceError, AcsiFormatError, AcsiTimeout


class AcsiClient:
    """Blocking request/response client.

    ``pyserial`` is imported lazily so the protocol layer (and its tests) can be
    used without a serial stack installed.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 1.0,
        write_timeout: Optional[float] = None,
    ) -> None:
        import serial  # noqa: PLC0415 - lazy import keeps protocol usable w/o pyserial

        self.port = port
        self.timeout = float(timeout)
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=self.timeout,
            write_timeout=write_timeout if write_timeout is not None else self.timeout,
        )

    # -- lifecycle ----------------------------------------------------------
    def close(self) -> None:
        if getattr(self, "_serial", None) is not None:
            self._serial.close()

    def __enter__(self) -> "AcsiClient":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # -- low level ----------------------------------------------------------
    def _read_line(self, timeout: Optional[float] = None) -> str:
        if timeout is None:
            timeout = self.timeout
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._serial.read(1)
            if not chunk:
                continue
            if chunk == b"\n":
                return buf.decode("ascii", "replace")
            buf += chunk
            if len(buf) > 512:
                raise AcsiFormatError("response line too long")
        raise AcsiTimeout(f"no response from {self.port} within {timeout}s")

    def transact(
        self,
        target: Target,
        device_id: int,
        direction: Dir,
        field: object,
        *args: int,
        timeout: Optional[float] = None,
    ) -> protocol.Response:
        """Send one request and return the verified response.

        Raises :class:`AcsiDeviceError` when the board answers with a non-zero
        result code. ``timeout`` overrides the client's read deadline for this
        one exchange (used by long-running blocking commands such as
        :meth:`step_wait`).
        """
        self._serial.reset_input_buffer()
        self._serial.write(protocol.encode(target, device_id, direction, field, *args))
        self._serial.flush()
        response = protocol.decode(self._read_line(timeout))
        if not response.ok:
            raise AcsiDeviceError(response.code, raw=response.raw)
        return response

    def raw(self, body: str) -> protocol.Response:
        """Send a raw frame body (no checksum)."""
        self._serial.reset_input_buffer()
        self._serial.write((body + "\n").encode("ascii"))
        self._serial.flush()
        response = protocol.decode(self._read_line())
        if not response.ok:
            raise AcsiDeviceError(response.code, raw=response.raw)
        return response

    # -- SYS ----------------------------------------------------------------
    def sys_ping(self) -> int:
        return self.transact(Target.SYS, 0, Dir.GET, SysField.PING).value

    def sys_echo(self, value: int) -> int:
        return self.transact(Target.SYS, 0, Dir.SET, SysField.ECHO, int(value)).value

    def sys_info(self) -> Dict[str, str]:
        """Return ``{'board': ..., 'fw': ...}`` (literal text tokens)."""
        resp = self.transact(Target.SYS, 0, Dir.GET, SysField.INFO)
        tokens = [str(tok) for tok in resp.payload]
        return {
            "board": tokens[0] if len(tokens) > 0 else "",
            "fw": tokens[1] if len(tokens) > 1 else "",
        }

    def sys_uid(self) -> Tuple[int, int, int]:
        resp = self.transact(Target.SYS, 0, Dir.GET, SysField.UID)
        words = list(resp.payload) + [0, 0, 0]
        return words[0], words[1], words[2]

    def sys_uptime(self) -> int:
        return self.transact(Target.SYS, 0, Dir.GET, SysField.UPTIME).value

    def sys_get_led(self) -> int:
        return self.transact(Target.SYS, 0, Dir.GET, SysField.LED).value

    def sys_set_led(self, state: bool, index: int = LED_MCU) -> int:
        return self.transact(
            Target.SYS, 0, Dir.SET, SysField.LED, int(index), 1 if state else 0
        ).value

    def sys_reset(self) -> None:
        self.transact(Target.SYS, 0, Dir.SET, SysField.RESET)

    # -- SERVO --------------------------------------------------------------
    def servo_ping(self, servo_id: int = 1) -> int:
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.PING).value

    def servo_get_pos(self, servo_id: int = 1) -> int:
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.POS).value

    def servo_set_pos(
        self,
        pos: int,
        servo_id: int = 1,
        speed: Optional[int] = None,
        accel: Optional[int] = None,
    ) -> int:
        args: List[int] = [int(pos)]
        if speed is not None:
            args.append(int(speed))
        if accel is not None:
            args.append(int(accel))
        return self.transact(Target.SERVO, servo_id, Dir.SET, ServoField.POS, *args).value

    def servo_get_vel(self, servo_id: int = 1) -> int:
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.VEL).value

    def servo_set_vel(self, speed: int, servo_id: int = 1, accel: Optional[int] = None) -> int:
        args = [int(speed)] + ([] if accel is None else [int(accel)])
        return self.transact(Target.SERVO, servo_id, Dir.SET, ServoField.VEL, *args).value

    def servo_set_torque(self, enable: bool, servo_id: int = 1) -> int:
        return self.transact(
            Target.SERVO, servo_id, Dir.SET, ServoField.TORQUE, 1 if enable else 0
        ).value

    def servo_calibrate(self, servo_id: int = 1) -> int:
        """Correct the current position to 2048 (torque value 128)."""
        return self.transact(Target.SERVO, servo_id, Dir.SET, ServoField.TORQUE, 128).value

    def servo_set_mode(self, mode: int, servo_id: int = 1) -> int:
        return self.transact(Target.SERVO, servo_id, Dir.SET, ServoField.MODE, int(mode)).value

    def servo_read_mem(self, addr: int, length: int = 1, servo_id: int = 1) -> List[int]:
        return list(
            self.transact(
                Target.SERVO, servo_id, Dir.GET, ServoField.MEM, int(addr), int(length)
            ).payload
        )

    def servo_write_mem(self, addr: int, data: Sequence[int], servo_id: int = 1) -> int:
        return self.transact(
            Target.SERVO, servo_id, Dir.SET, ServoField.MEM, int(addr),
            *[int(b) for b in data]
        ).value

    def servo_feedback(self, servo_id: int = 1) -> Dict[str, int]:
        """Return the FB burst as a dict keyed by :data:`defs.FEEDBACK_FIELDS`."""
        resp = self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.FB)
        return {key: int(val) for key, val in zip(FEEDBACK_FIELDS, resp.payload)}

    def servo_status(self, servo_id: int = 1) -> int:
        """Servo hardware error bitmask (register 0x41). Mask with ServoFault."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.STATUS).value

    def servo_get_torque(self, servo_id: int = 1) -> int:
        """Torque enable state: 0 = off, 1 = on."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.TORQUE).value

    def servo_get_acc(self, servo_id: int = 1) -> int:
        """Acceleration/deceleration rate (100 steps/s^2 per LSB)."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.ACC).value

    def servo_set_acc(self, accel: int, servo_id: int = 1) -> int:
        return self.transact(Target.SERVO, servo_id, Dir.SET, ServoField.ACC, int(accel)).value

    def servo_get_torque_limit(self, servo_id: int = 1) -> int:
        """Dynamic torque limit (0.1 % units, 0..1000)."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.TLIM).value

    def servo_set_torque_limit(self, limit: int, servo_id: int = 1) -> int:
        return self.transact(Target.SERVO, servo_id, Dir.SET, ServoField.TLIM, int(limit)).value

    def servo_get_mode(self, servo_id: int = 1) -> int:
        """Operating mode: see :class:`defs.ServoMode` (0 pos … 3 stepper)."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.MODE).value

    def servo_get_pwm(self, servo_id: int = 1) -> int:
        """Open-loop runtime duty (-1000..1000)."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.PWM).value

    def servo_set_pwm(self, value: int, servo_id: int = 1) -> int:
        return self.transact(Target.SERVO, servo_id, Dir.SET, ServoField.PWM, int(value)).value

    def servo_get_id(self, servo_id: int = 1) -> int:
        """The servo's stored bus ID (register 0x05)."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.ID).value

    def servo_set_id(self, new_id: int, servo_id: int = 1) -> int:
        """Persist a new bus ID to EEPROM (register 0x05)."""
        return self.transact(Target.SERVO, servo_id, Dir.SET, ServoField.ID, int(new_id)).value

    def servo_get_baud(self, servo_id: int = 1) -> int:
        """Baud-rate index; see :class:`defs.ServoBaud`."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.BAUD).value

    def servo_set_baud(self, baud: int, servo_id: int = 1) -> int:
        return self.transact(Target.SERVO, servo_id, Dir.SET, ServoField.BAUD, int(baud)).value

    def servo_get_limits(self, servo_id: int = 1) -> Tuple[int, int]:
        """Travel angle limits as ``(min, max)`` in steps (registers 0x09/0x0B)."""
        payload = list(self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.LIMIT).payload)
        payload += [0, 0]
        return int(payload[0]), int(payload[1])

    def servo_set_limits(self, min_angle: int, max_angle: int, servo_id: int = 1) -> int:
        return self.transact(
            Target.SERVO, servo_id, Dir.SET, ServoField.LIMIT, int(min_angle), int(max_angle)
        ).value

    def servo_get_temp(self, servo_id: int = 1) -> int:
        """Internal temperature in degrees Celsius (register 0x3F)."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.TEMP).value

    def servo_get_volt(self, servo_id: int = 1) -> int:
        """Supply voltage in 0.1 V units (register 0x3E)."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.VOLT).value

    def servo_get_load(self, servo_id: int = 1) -> int:
        """Output load in 0.1 % units (register 0x3C)."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.LOAD).value

    def servo_get_current(self, servo_id: int = 1) -> int:
        """Motor current in 6.5 mA units (register 0x45)."""
        return self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.CUR).value

    def servo_is_moving(self, servo_id: int = 1) -> bool:
        """True while the servo is in motion (register 0x42)."""
        return bool(self.transact(Target.SERVO, servo_id, Dir.GET, ServoField.MOVING).value)

    def servo_reset(self, servo_id: int = 1) -> int:
        """Factory-reset the servo EEPROM/parameters (instruction 0x06)."""
        return self.transact(Target.SERVO, servo_id, Dir.SET, ServoField.RESET).value

    # -- STEP ---------------------------------------------------------------
    #
    # The STEP target drives three step/direction motors through the tue_stepper
    # library: motors 1+2 form a CoreXY pair, motor 3 is Z. Distances travel as
    # integer micrometres, so this API works in millimetres and converts at the
    # boundary (see ``defs.mm_to_um``).
    #
    # Moves are QUEUED on the board and answered immediately: a successful reply
    # means "accepted", not "finished". Use :meth:`step_wait` or poll
    # :meth:`step_status` to know when the motion is done.

    @staticmethod
    def _axes(payload: Sequence[int]) -> Dict[str, float]:
        return {key: um_to_mm(val) for key, val in zip(AXES, payload)}

    def step_info(self) -> Dict[str, int]:
        """Steps-per-mm of each axis plus the driver's min/max step rate."""
        resp = self.transact(Target.STEP, StepId.TOOL, Dir.GET, StepField.INFO)
        return {key: int(val) for key, val in zip(STEP_INFO_FIELDS, resp.payload)}

    def step_position(self) -> Dict[str, float]:
        """Current tool position in mm: ``{'x':…, 'y':…, 'z':…}``."""
        resp = self.transact(Target.STEP, StepId.TOOL, Dir.GET, StepField.POS)
        return self._axes(resp.payload)

    def step_target(self) -> Dict[str, float]:
        """End point of the queued motion in mm."""
        resp = self.transact(Target.STEP, StepId.TOOL, Dir.GET, StepField.TARGET)
        return self._axes(resp.payload)

    def step_status(self) -> Dict[str, int]:
        """``{'busy': 0/1, 'enabled': 0/1}``."""
        resp = self.transact(Target.STEP, StepId.TOOL, Dir.GET, StepField.STATUS)
        return {key: int(val) for key, val in zip(STEP_STATUS_FIELDS, resp.payload)}

    def step_is_busy(self) -> bool:
        return bool(self.step_status()["busy"])

    def step_get_enable(self) -> int:
        return self.transact(Target.STEP, StepId.TOOL, Dir.GET, StepField.ENABLE).value

    def step_enable(self, enable: bool = True) -> int:
        """Energise (True) or release (False) the shared driver enable line."""
        return self.transact(
            Target.STEP, StepId.TOOL, Dir.SET, StepField.ENABLE, 1 if enable else 0
        ).value

    def step_move(self, x: float, y: float, z: float, feed: float, accel: float) -> Dict[str, float]:
        """Move to an absolute CoreXY point ``(x, y, z)`` in mm.

        ``feed`` is the tool-path speed in mm/s and ``accel`` the path
        acceleration in mm/s^2; both must be > 0.
        """
        resp = self.transact(
            Target.STEP, StepId.TOOL, Dir.SET, StepField.MOVE,
            mm_to_um(x), mm_to_um(y), mm_to_um(z), mm_to_um(feed), mm_to_um(accel),
        )
        return self._axes(resp.payload)

    def step_move_xy(self, x: float, y: float, feed: float, accel: float) -> Dict[str, float]:
        """Absolute XY move in mm, Z unchanged."""
        resp = self.transact(
            Target.STEP, StepId.TOOL, Dir.SET, StepField.MOVE2D,
            mm_to_um(x), mm_to_um(y), mm_to_um(feed), mm_to_um(accel),
        )
        return self._axes(resp.payload)

    def step_move_z(self, z: float, feed: float, accel: float) -> float:
        """Absolute Z move in mm, X/Y untouched."""
        return um_to_mm(
            self.transact(
                Target.STEP, StepId.TOOL, Dir.SET, StepField.MOVEZ,
                mm_to_um(z), mm_to_um(feed), mm_to_um(accel),
            ).value
        )

    def step_move_rel(self, dx: float, dy: float, feed: float, accel: float) -> Dict[str, float]:
        """Relative XY move by ``(dx, dy)`` mm."""
        resp = self.transact(
            Target.STEP, StepId.TOOL, Dir.SET, StepField.MOVEREL,
            mm_to_um(dx), mm_to_um(dy), mm_to_um(feed), mm_to_um(accel),
        )
        return self._axes(resp.payload)

    def step_jog(self, vx: float, vy: float, accel: float) -> Dict[str, float]:
        """Start a continuous CoreXY jog at tool velocity ``(vx, vy)`` mm/s.

        Calling this again while a jog runs only changes the speed; use
        :meth:`step_stop` before reversing direction.
        """
        resp = self.transact(
            Target.STEP, StepId.TOOL, Dir.SET, StepField.JOG,
            mm_to_um(vx), mm_to_um(vy), mm_to_um(accel),
        )
        return self._axes(resp.payload)

    def step_jog_x(self, vx: float, accel: float) -> Dict[str, float]:
        return self.step_jog(vx, 0.0, accel)

    def step_jog_y(self, vy: float, accel: float) -> Dict[str, float]:
        return self.step_jog(0.0, vy, accel)

    def step_jog_z(self, vz: float, accel: float) -> float:
        """Continuous Z jog at ``vz`` mm/s (sign selects the direction)."""
        return um_to_mm(
            self.transact(
                Target.STEP, StepId.TOOL, Dir.SET, StepField.JOGZ,
                mm_to_um(vz), mm_to_um(accel),
            ).value
        )

    def step_stop(self) -> int:
        """Controlled stop: decelerate and drop everything queued behind it."""
        return self.transact(Target.STEP, StepId.TOOL, Dir.SET, StepField.STOP).value

    def step_abort(self) -> int:
        """Immediate stop: kill the step clock and empty the queue."""
        return self.transact(Target.STEP, StepId.TOOL, Dir.SET, StepField.ABORT).value

    def step_set_position(self, x: float, y: float, z: float) -> Dict[str, float]:
        """Redefine the current position (use after homing). Call while idle."""
        resp = self.transact(
            Target.STEP, StepId.TOOL, Dir.SET, StepField.SETPOS,
            mm_to_um(x), mm_to_um(y), mm_to_um(z),
        )
        return self._axes(resp.payload)

    def step_motor_steps(self, motor: int = StepId.M1) -> int:
        """Raw signed step counter of one motor (1 = A, 2 = B, 3 = Z)."""
        return self.transact(Target.STEP, int(motor), Dir.GET, StepField.STEPS).value

    def step_raw(
        self,
        motor: int,
        steps: int,
        feed: float,
        accel: float,
    ) -> int:
        """Move a single motor by ``steps``, bypassing the CoreXY kinematics.

        ``feed``/``accel`` are interpreted along that motor's own axis.
        """
        return self.transact(
            Target.STEP, int(motor), Dir.SET, StepField.RAW,
            int(steps), mm_to_um(feed), mm_to_um(accel),
        ).value

    def step_wait(self, timeout_ms: int = 0, read_timeout: Optional[float] = None) -> int:
        """Block until the motion queue drains; returns the elapsed time in ms.

        ``timeout_ms`` is the board-side deadline (0 = wait forever). The
        client's own read deadline is extended to match so that a long move
        does not raise :class:`~tue_acsi.exceptions.AcsiTimeout`; pass
        ``read_timeout`` (seconds) to override it.
        """
        if read_timeout is None:
            read_timeout = (timeout_ms / 1000.0 + 1.0) if timeout_ms > 0 else 600.0
        return self.transact(
            Target.STEP, StepId.TOOL, Dir.SET, StepField.WAIT,
            int(timeout_ms), timeout=read_timeout,
        ).value
