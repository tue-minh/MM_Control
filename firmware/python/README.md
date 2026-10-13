# TUE ACSI Python Client

Host-side Python library for the **MM_Control** STM32F407VET6 board over USB CDC
(Virtual COM Port), using the ACSI protocol
(`<TARGET> <ID> <DIR> <FIELD> [ARG ...]` — whitespace-separated tokens, no
checksum).

Full wire specification: [`docs/ACSI_PROTOCOL.md`](../docs/ACSI_PROTOCOL.md).

## Features

- **Token frames** — header fields are decimal numbers; the wire carries no
  checksum.
- **Literal text** — text payloads (e.g. the board name) are sent as plain tokens
  and surfaced directly by the client.
- **Typed errors** — device result codes surface as `AcsiDeviceError`.
- **Transport-independent core** — framing (`tue_acsi.protocol`) has no serial
  dependency.
- **No serial hardware needed for tests** — the suite runs against an in-memory
  fake port.

## Install

```bash
cd firmware/python
pip install -r requirements.txt        # pyserial
# or, editable:
pip install -e .
```

## Quick start

```python
from tue_acsi import AcsiClient

with AcsiClient("COM7") as dev:
    print(dev.sys_ping())                 # 1  (board alive)
    print(dev.sys_info())                 # {'board': 'MM_Control', 'fw': '1.0'}
    print(dev.sys_uid())                  # (uid0, uid1, uid2)

    print(dev.servo_ping(1))              # 1  (servo #1 on the bus)
    dev.servo_set_torque(True, 1)
    dev.servo_set_pos(2048, servo_id=1, speed=1000, accel=50)

    fb = dev.servo_feedback(1)
    print(fb["pos"], fb["spd"], fb["temp"], fb["status"])

    dev.sys_set_led(True)                 # onboard LED on
```

Stepper motors (distances in **mm**, converted to micrometres on the wire):

```python
from tue_acsi import AcsiClient, StepId

with AcsiClient("COM7") as dev:
    print(dev.step_info())                # {'xy_steps_per_mm': 320, 'z_steps_per_mm': 1600, …}
    print(dev.step_position())            # {'x': 0.0, 'y': 0.0, 'z': 0.0}

    dev.step_enable(True)
    dev.step_move(50.0, 30.0, 10.0, feed=60.0, accel=600.0)   # exact CoreXY point
    dev.step_wait(5000)                   # block until the queue drains

    dev.step_move_xy(20.0, 20.0, 50.0, 400.0)   # absolute XY
    dev.step_move_z(5.0, 10.0, 200.0)           # absolute Z
    dev.step_move_rel(-5.0, 0.0, 50.0, 400.0)   # relative XY

    dev.step_jog_x(30.0, accel=500.0)     # continuous, ramps up
    dev.step_jog_x(8.0, accel=500.0)      # live speed change
    dev.step_stop()                       # controlled ramp down

    print(dev.step_motor_steps(StepId.M3))     # raw Z step counter
    dev.step_raw(StepId.M1, 3200, 20.0, 300.0) # one motor, no kinematics
```

Runnable demos: [`examples/servo_demo.py`](examples/servo_demo.py) and
[`examples/stepper_demo.py`](examples/stepper_demo.py).

## Numeric codes

| Code | TARGET | | Code | DIR |
|:--|:--|:--|:--|:--|
| 1 | SERVO | | 0 | GET (read in) |
| 2 | STEP | | 1 | SET (write out) |
| 3 | DC | | | |
| 4 | SYS | | | |

SERVO fields: `1` PING, `2` POS, `3` VEL, `4` ACC, `5` TORQUE, `6` TLIM, `7` MODE,
`8` PWM, `9` MEM, `10` FB, `11` ID, `12` BAUD, `13` LIMIT, `14` TEMP, `15` VOLT,
`16` LOAD, `17` CUR, `18` STATUS, `19` MOVING, `20` RESET.

Every SERVO field is reachable from the client, e.g.:

```python
dev.servo_get_torque(1)                 # 0/1
dev.servo_set_acc(50, 1)                # acceleration (100 steps/s^2)
dev.servo_get_limits(1)                 # (min, max) in steps
dev.servo_is_moving(1)                  # bool
dev.servo_reset(1)                      # factory reset
```

Decode the hardware error bitmask (`SERVO STATUS` / `FB`) with `ServoFault`:

```python
from tue_acsi import ServoFault

status = dev.servo_status(1)
if status & ServoFault.OVERLOAD:
    print("servo is overloaded")
```

SYS fields: `1` PING, `2` ECHO, `3` INFO, `4` UID, `5` UPTIME, `6` LED, `7` RESET.

STEP fields: `1` INFO, `2` POS, `3` TARGET, `4` STATUS, `5` ENABLE, `6` MOVE,
`7` MOVE2D, `8` MOVEZ, `9` MOVEREL, `10` JOG, `11` JOGZ, `12` STOP, `13` ABORT,
`14` SETPOS, `15` STEPS, `16` RAW, `17` WAIT.

STEP distances travel as integer **micrometres** (`1 um = 0.001 mm`), speeds in
`um/s` and accelerations in `um/s^2`, so the frame stays numbers-only. The client
works in millimetres and converts at the boundary (`mm_to_um` / `um_to_mm`).
`StepId` selects the object: `0` = the tool (CoreXY X/Y + Z), `1..3` = one motor
(`M1` = CoreXY `x + y`, `M2` = `x - y`, `M3` = Z).

Moves are **queued** on the board and acknowledged immediately, so a successful
`step_move` means *accepted*: call `step_wait()` or poll `step_status()` before
assuming the machine has arrived. `CODE 12` is `QUEUE_FULL`, `CODE 13` is
`TIMEOUT` (a `step_wait` deadline that expired).

Response `CODE`: `0` = OK, otherwise an error code (see `tue_acsi.defs.Code`).

## Text payloads

Text is sent literally as whitespace-separated tokens, so SYS INFO responds with
the board name and firmware version as plain text:

```
4 0 0 MM_Control 1.0
```

`AcsiClient.sys_info()` returns `{'board': 'MM_Control', 'fw': '1.0'}`. A text
value cannot contain spaces.

## Building frames by hand

```python
from tue_acsi import encode_line

print(encode_line(1, 1, 0, 1))         # 1 1 0 1   (SERVO 1, GET, PING)
```

## API overview

`AcsiClient(port, baudrate=115200, timeout=1.0)`

| Category | Methods |
|:--|:--|
| System | `sys_ping`, `sys_echo`, `sys_info`, `sys_uid`, `sys_uptime`, `sys_get_led`, `sys_set_led`, `sys_reset` |
| Servo | `servo_ping`, `servo_get_pos`, `servo_set_pos`, `servo_get_vel`, `servo_set_vel`, `servo_get_acc`, `servo_set_acc`, `servo_get_torque`, `servo_set_torque`, `servo_calibrate`, `servo_get_torque_limit`, `servo_set_torque_limit`, `servo_get_mode`, `servo_set_mode`, `servo_get_pwm`, `servo_set_pwm`, `servo_get_id`, `servo_set_id`, `servo_get_baud`, `servo_set_baud`, `servo_get_limits`, `servo_set_limits`, `servo_read_mem`, `servo_write_mem`, `servo_feedback`, `servo_status`, `servo_get_temp`, `servo_get_volt`, `servo_get_load`, `servo_get_current`, `servo_is_moving`, `servo_reset` |
| Stepper | `step_info`, `step_position`, `step_target`, `step_status`, `step_is_busy`, `step_get_enable`, `step_enable`, `step_move`, `step_move_xy`, `step_move_z`, `step_move_rel`, `step_jog`, `step_jog_x`, `step_jog_y`, `step_jog_z`, `step_stop`, `step_abort`, `step_set_position`, `step_motor_steps`, `step_raw`, `step_wait` |
| Generic | `transact(target, id, dir, field, *args)`, `raw("1 1 0 1")` |

Exceptions: `AcsiError` (base), `AcsiTimeout`, `AcsiFormatError`,
`AcsiDeviceError` (`.code`, `.name`).

## Tests

```bash
cd firmware/python
python -m unittest discover -s tests -v
```
