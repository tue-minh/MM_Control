# TUE ACSI Protocol — Instruction Manual (SUPERSEDED)

> [!WARNING]
> **This document is superseded and kept for historical reference only.**
> The ASCII `tue_acsi` protocol and the `python/tue_acsi` host library have been
> replaced by the **binary CAN-over-USB protocol**. Use
> [`docs/USB_BINARY_PROTOCOL.md`](USB_BINARY_PROTOCOL.md) and
> [`python/tue_usb`](../python/tue_usb) instead. The legacy firmware sources may
> still exist on disk but are no longer built or wired into the application.

> **Platform**: STM32F407VET6 (`MM_Control`) over **USB CDC (Virtual COM Port)**
> **Firmware**: [`Core/Inc/tue_acsi.h`](../Core/Inc/tue_acsi.h), [`Core/Src/tue_acsi.c`](../Core/Src/tue_acsi.c) (legacy)
> **Host library**: `python/tue_acsi` (removed; see `python/tue_usb`)
> **Related**: [`docs/USB_BINARY_PROTOCOL.md`](USB_BINARY_PROTOCOL.md), [`docs/MEMORY_TABLE.md`](MEMORY_TABLE.md), [`docs/STS3215_INSTRUCTION.md`](STS3215_INSTRUCTION.md)

---

## 1. Overview

ACSI is a **field-oriented, whitespace-separated** command protocol for talking to
the board over USB. A frame is a short sequence of tokens:

| Field | Meaning | Values |
|:--|:--|:--|
| `TARGET` | which device class | `1` SERVO, `2` STEP, `3` DC, `4` SYS |
| `ID` | which device of that class (e.g. which servo on the bus) | `0`…`253` |
| `DIR` | direction — `0` = **GET** (read data in), `1` = **SET** (write data out) | `0`, `1` |
| `FIELD` | control field / type code from the motor library | see §5 |
| `ARG` | one or more values, usually the value to write | number, or (responses) text |

So `1 1 1 2 2048` reads as *"servo (#1), write, position field, 2048"*. Replies
reuse the same grammar, with a result code where `DIR` was.

The header fields are always numbers. Payload values are numbers too, except that
a response may carry **literal text** tokens (the board name in SYS INFO) — see
§4. There is **no checksum field**: the line is taken as-is.

---

## 2. Transport

| Setting | Value |
|:--|:--|
| Interface | USB CDC (Virtual COM Port) |
| Encoding | ASCII |
| Line ending | `LF` (`\n`); a preceding `CR` (`\r`) is ignored (so `CRLF` works) |
| Separator | one or more spaces |
| Max line length | 160 characters |

Exactly one command is processed per line. Blank lines are ignored.

---

## 3. Frame format

### Request (host → board)

```
<TARGET> <ID> <DIR> <FIELD> [ARG ...]
```

### Response (board → host)

```
<TARGET> <ID> <CODE> [VALUE ...]
```

- `TARGET` and `ID` are echoed from the request.
- `CODE` is `0` on success, otherwise an error code (§7).
- On `ERR` there is no value payload.

---

## 4. Text payloads

Payload values are normally decimal numbers. When a payload has to carry text
(e.g. the board name), the text is sent **literally**, as whitespace-separated
tokens. The receiver treats any token that is not an integer as text.

Example — the board name `"MM_Control"` and firmware `"1.0"` in the SYS INFO
response:

```
4 0 0 MM_Control 1.0
```

Firmware side: the response body is built directly with `snprintf()`. Python
side: `AcsiClient.sys_info()` returns `{'board': 'MM_Control', 'fw': '1.0'}`.

Because text tokens are whitespace-delimited, a text value cannot itself contain
spaces.

---

## 5. Codes

### TARGET

| Code | Target | Status |
|:--|:--|:--|
| 1 | `SERVO` | implemented |
| 2 | `STEP` | implemented |
| 3 | `DC` | reserved (`CODE 8`) |
| 4 | `SYS` | implemented |

### DIR

| Code | Direction |
|:--|:--|
| 0 | `GET` — read data in |
| 1 | `SET` — write data out |

### SERVO fields

| FIELD | Name | DIR | ARG | Response `VALUE` | Register |
|:--|:--|:--|:--|:--|:--|
| 1 | PING | GET | – | `<id>` | – |
| 2 | POS | GET | – | current position (steps) | 0x38 |
| 2 | POS | SET | `<pos> [speed] [acc]` | `<pos>` | 0x2A |
| 3 | VEL | GET | – | current speed (steps/s) | 0x3A |
| 3 | VEL | SET | `<speed> [acc]` | `<speed>` | 0x2E |
| 4 | ACC | GET/SET | `<acc>` | acceleration | 0x29 |
| 5 | TORQUE | GET/SET | `<0\|1\|128>` | `0`/`1` (`128` = calibrate to 2048) | 0x28 |
| 6 | TLIM | GET/SET | `<0..1000>` | torque limit (0.1 %) | 0x30 |
| 7 | MODE | GET/SET | `<0..3>` | 0 pos, 1 speed, 2 pwm, 3 stepper | 0x21 |
| 8 | PWM | GET/SET | `<value>` | open-loop runtime (-1000..1000) | 0x2C |
| 9 | MEM | GET | `<addr> [len]` | `b0 b1 ...` raw bytes (≤16) | any |
| 9 | MEM | SET | `<addr> <b0> [b1 ...]` | `<addr>` | any |
| 10 | FB | GET | – | `pos spd load volt temp status moving cur` | 0x38–0x46 |
| 11 | ID | GET/SET | `<newid>` | servo id | 0x05 |
| 12 | BAUD | GET/SET | `<0..7>` | baud index | 0x06 |
| 13 | LIMIT | GET | – | `min max` | 0x09 |
| 13 | LIMIT | SET | `<min> <max>` | `<min>` | 0x09 |
| 14 | TEMP | GET | – | temperature (°C) | 0x3F |
| 15 | VOLT | GET | – | voltage (0.1 V) | 0x3E |
| 16 | LOAD | GET | – | load (0.1 %) | 0x3C |
| 17 | CUR | GET | – | current (6.5 mA units) | 0x45 |
| 18 | STATUS | GET | – | status bitmask | 0x41 |
| 19 | MOVING | GET | – | `0`/`1` | 0x42 |
| 20 | RESET | SET | – | `<id>` (factory reset) | – |

`POS`/`VEL` accept optional extra arguments: `1 1 1 2 2048 1000 50` sets position
2048 with speed 1000 and acceleration 50 in one transaction.

### STEP fields

The `STEP` target drives the three step/direction motors through the
[`tue_stepper`](STEPPER_INSTRUCTION.md) library:

| Motor | Role | Step / Dir | CoreXY |
|:--|:--|:--|:--|
| 1 | belt pair (A) | PE9 / PC3 | `A = x + y` |
| 2 | belt pair (B) | PE11 / PC2 | `B = x - y` |
| 3 | Z | PE13 / PC1 | `z` |

#### Units

The wire carries integers only, so distances, speeds and accelerations are sent
scaled to **micrometres**:

| Quantity | Wire unit | Example |
|:--|:--|:--|
| distance | micrometre (`0.001 mm`) | `50.0 mm` -> `50000` |
| speed | um/s | `60 mm/s` -> `60000` |
| acceleration | um/s^2 | `600 mm/s^2` -> `600000` |

#### ID selects what the command acts on

| ID | Object |
|:--|:--|
| 0 | the tool / machine — X, Y, Z in CoreXY space |
| 1 | motor 1 (A) |
| 2 | motor 2 (B) |
| 3 | motor 3 (Z) |

#### Field table

| FIELD | Name | DIR | ARG | Response `VALUE` |
|:--|:--|:--|:--|:--|
| 1 | INFO | GET | – | `<xy steps/mm> <z steps/mm> <min rate> <max rate>` |
| 2 | POS | GET | – | `<x> <y> <z>` current tool position |
| 3 | TARGET | GET | – | `<x> <y> <z>` end of the queued motion |
| 4 | STATUS | GET | – | `<busy> <enabled>` |
| 5 | ENABLE | GET | – | `0`/`1` driver enable line |
| 5 | ENABLE | SET | `<0\|1>` | `<0\|1>` |
| 6 | MOVE | SET | `<x> <y> <z> <feed> <accel>` | `<x> <y> <z>` — absolute CoreXY + Z |
| 7 | MOVE2D | SET | `<x> <y> <feed> <accel>` | `<x> <y>` — absolute XY, Z unchanged |
| 8 | MOVEZ | SET | `<z> <feed> <accel>` | `<z>` |
| 9 | MOVEREL | SET | `<dx> <dy> <feed> <accel>` | `<dx> <dy>` |
| 10 | JOG | SET | `<vx> <vy> <accel>` | `<vx> <vy>` — continuous XY |
| 11 | JOGZ | SET | `<vz> <accel>` | `<vz>` — continuous Z |
| 12 | STOP | SET | – | `<busy>` — controlled stop (ramps down) |
| 13 | ABORT | SET | – | `0` — immediate stop, queue dropped |
| 14 | SETPOS | SET | `<x> <y> <z>` | `<x> <y> <z>` — redefine current position |
| 15 | STEPS | GET | ID `1..3` | `<signed steps>` raw counter of one motor |
| 16 | RAW | SET | ID `1..3` `<steps> <feed> <accel>` | `<steps>` — one motor, no kinematics |
| 17 | WAIT | SET | `<timeout_ms>` | `<elapsed_ms>` — returns once the queue drains |

Behaviour notes:

- **Moves are queued** (16 segments deep) and the reply is sent immediately, so
  a `MOVE` answer means *accepted*, not *finished*. Use `WAIT` or poll `STATUS`.
- A saturated queue answers `CODE 12` (`QUEUE_FULL`). Asking for a position move
  while a jog is running answers `CODE 9` (`BUSY`).
- `WAIT <0>` blocks until idle; with a timeout it answers `CODE 13`
  (`TIMEOUT`) if the deadline expires first. The reply carries the elapsed ms.
- `STOP` decelerates and discards everything queued behind the current segment;
  `ABORT` kills the step clock at once.
- `feed`/`accel` always describe the **tool path**, so a diagonal move keeps the
  same mm/s as an axis-aligned one.
- The drivers are energised automatically by the first accepted move.

### SYS fields

| FIELD | Name | DIR | ARG | Response `VALUE` |
|:--|:--|:--|:--|:--|
| 1 | PING | GET | – | `1` (liveness) |
| 2 | ECHO | SET | `<n>` | `<n>` (link integrity test) |
| 3 | INFO | GET | – | `<board> <fw>` (literal text) |
| 4 | UID | GET | – | `<uid0> <uid1> <uid2>` (96-bit id as 3 decimal words) |
| 5 | UPTIME | GET | – | milliseconds since boot |
| 6 | LED | GET | – | `0`/`1` |
| 6 | LED | SET | `<index> <state>` | `<state>` (index `0` = onboard LED PE5) |
| 7 | RESET | SET | – | `0`, then the MCU reboots |

---

## 6. Worked examples

```text
host  -> 1 1 0 1                        # servo 1, GET, PING
board -> 1 1 0 1

host  -> 1 1 1 5 1                      # servo 1, SET, TORQUE = 1
host  -> 1 1 1 2 2048                   # servo 1, SET, POS = 2048
board -> 1 1 0 2048

host  -> 1 1 1 2 2048 1000 50           # pos + speed + accel in one frame
board -> 1 1 0 2048

host  -> 1 1 0 10                       # servo 1, GET, FB
board -> 1 1 0 2048 0 0 120 30 0 0 0
#        ^ pos spd load volt temp status moving cur

host  -> 1 1 0 9 56 2                   # GET 2 bytes at 0x38 (position)
board -> 1 1 0 0 8                      # 0x00 0x08 -> 2048

host  -> 4 0 1 6 0 1                    # SYS: LED index 0 -> on
board -> 4 0 0 1

host  -> 4 0 0 3                        # SYS: GET INFO
board -> 4 0 0 MM_Control 1.0
#        decode -> board="MM_Control"  fw="1.0"

host  -> 2 0 0 1                        # STEP: GET INFO
board -> 2 0 0 320 1600 400 50000
#        ^ xy steps/mm, z steps/mm, min/max step rate

host  -> 2 0 1 6 50000 30000 10000 60000 600000   # MOVE to (50,30,10) mm
board -> 2 0 0 50000 30000 10000        # accepted, queued (not finished yet)

host  -> 2 0 1 17 5000                  # WAIT until the queue drains (max 5 s)
board -> 2 0 0 412                      # finished after 412 ms

host  -> 2 0 0 2                        # GET POS
board -> 2 0 0 50000 30000 10000

host  -> 2 0 1 9 5000 0 50000 400000    # MOVEREL +5 mm in X, 50 mm/s
board -> 2 0 0 5000 0                   # accepted, offset applied

host  -> 2 0 1 10 30000 0 500000        # JOG +X at 30 mm/s
board -> 2 0 0 30000 0                  # accepted, jogging
host  -> 2 0 1 12                       # STOP (ramps down)

host  -> 2 1 0 15                       # motor 1 step counter
board -> 2 1 0 -320

host  -> 2 1 1 16 3200 20000 200000     # motor 1: +3200 steps (10 mm)

host  -> 4 0 0                          # malformed / incomplete frame
board -> 4 0 2                          # CODE 2 = BAD_FORMAT
```

> Response values are examples.

---

## 7. Result / error codes (`CODE`)

| Code | Name | Meaning |
|:--|:--|:--|
| 0 | `OK` | success |
| 2 | `BAD_FORMAT` | malformed / truncated frame |
| 3 | `UNKNOWN_TARGET` | unknown `TARGET` code |
| 4 | `UNKNOWN_FIELD` | field not defined for this target |
| 5 | `BAD_ARG` | missing or out-of-range argument |
| 6 | `BUS_TIMEOUT` | servo did not answer on the bus |
| 7 | `BUS_ERROR` | servo bus framing / HAL error |
| 8 | `NOT_SUPPORTED` | field, direction or target not implemented yet |
| 9 | `BUSY` | servo bus busy |
| 10 | `SERVO_FAULT` | servo reported a hardware error status bit |
| 11 | `INTERNAL` | internal driver error |
| 12 | `QUEUE_FULL` | stepper motion queue saturated (`STEP` only) |
| 13 | `TIMEOUT` | `STEP WAIT` deadline expired |

Code `1` is unused (it was `BAD_CRC` before the checksum was removed).

`1 1 6` means servo #1 did not respond (bus timeout).

---

## 8. C API

```c
#include "tue_acsi.h"

void App_Init(void)
{
    tue_acsi_init();                        /* protocol + servo bus on USART1 */

    tue_sts3215_t *svc = tue_acsi_servo();   /* optional direct bus access */
    (void)svc;
}
```

The `STEP` target wraps the stepper library, so host commands and local code
share exactly the same motion queue. `tue_acsi_init()` also calls
`tue_stepper_init()` (idempotent), and the step clock is driven from
`tue_stepper_on_timer_elapsed()` in `HAL_TIM_PeriodElapsedCallback()`:

```c
tue_stepper_move_to_xyz_mm(50.0f, 30.0f, 10.0f, 60.0f, 600.0f);   /* 2 0 1 6 ... */
```

Integration points already wired in the project:

- `Core/Src/freertos.c` — calls `tue_acsi_init()` from the startup task.
- `USB_DEVICE/App/usbd_cdc_if.c` — `CDC_Receive_FS()` feeds bytes via
  `tue_acsi_on_cdc_rx()`.
- `CMakeLists.txt` — builds `Core/Src/tue_acsi.c`.

By design, USB reception only buffers bytes; a dedicated `tue_acsi` task does the
parsing, runs the (blocking) servo transactions and sends the reply.

---

## 9. Python client

```python
from tue_acsi import AcsiClient, ServoFault

with AcsiClient("COM7") as dev:
    print(dev.sys_ping())                 # 1
    print(dev.sys_info())                 # {'board': 'MM_Control', 'fw': '1.0'}
    print(dev.sys_uid())                  # (uid0, uid1, uid2)

    dev.servo_set_torque(True, 1)
    dev.servo_set_pos(2048, servo_id=1, speed=1000, accel=50)
    print(dev.servo_feedback(1))          # {'pos': 2048, 'spd': 0, ...}

    print(dev.servo_get_temp(1))          # temperature (°C)
    if dev.servo_status(1) & ServoFault.OVERLOAD:
        print("servo overloaded")
```

Stepper control (distances in mm — the client scales to micrometres on the wire):

```python
from tue_acsi import AcsiClient

with AcsiClient("COM7") as dev:
    dev.step_enable(True)
    print(dev.step_info())                # {'xy_steps_per_mm': 320, 'z_steps_per_mm': 1600, ...}

    dev.step_move(50.0, 30.0, 10.0, feed=60.0, accel=600.0)   # absolute point
    dev.step_wait(5000)                   # block until the queue drains
    print(dev.step_position())            # {'x': 50.0, 'y': 30.0, 'z': 10.0}

    dev.step_jog_x(30.0, accel=500.0)     # continuous, ramps up
    time.sleep(0.5)
    dev.step_stop()                       # ramps down

    print(dev.step_motor_steps(1))        # raw counter of motor 1
```

Building a frame by hand (no checksum is appended):

```python
from tue_acsi import encode_line

print(encode_line(1, 1, 0, 1))            # 1 1 0 1
```

See [`python/README.md`](../python/README.md) for the full API.
