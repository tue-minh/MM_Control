# TUE USB Binary Protocol — Specification

> **Platform**: STM32F407VET6 (`MM_Control`) over **USB CDC (Virtual COM Port)**
> **Firmware**: [`Core/Inc/tue_usb_binary.h`](../Core/Inc/tue_usb_binary.h), [`Core/Src/tue_usb_binary.c`](../Core/Src/tue_usb_binary.c)
> **Host library**: [`python/tue_usb`](../python/tue_usb)
> **Integrity**: SAE J1850 CRC-8 (poly `0x1D`, init `0xFF`, xorout `0xFF`)
> **Related**: [`docs/STEPPER_INSTRUCTION.md`](STEPPER_INSTRUCTION.md), [`docs/STS3215_INSTRUCTION.md`](STS3215_INSTRUCTION.md)

---

## 1. Overview

The binary protocol replaces the legacy ASCII `tue_acsi` protocol with a compact,
deterministic CAN-style frame carried over USB CDC. It removes all text
formatting/parsing from the Cortex-M4 and maps cleanly onto the onboard CAN 2.0B
controller (frame = `CAN ID` + `DLC` + `PAYLOAD`), so the USB port can act as a
host bridge to a CAN network.

Every transaction is a fixed envelope of **12 to 20 bytes**. Multi-byte fields are
**little-endian**.

```
+------------+------------+------------+---------------+------------+------------+------------------+------------+------------+------------+
|  SOF1      |  SOF2      | FRAME_TYPE | CAN_ID (4B)   | DLC        | SEQ        | PAYLOAD (0..8B)  | CRC8       | EOF1       | EOF2       |
|  0xAA      |  0x55      | uint8_t    | uint32_t (LE) | uint8_t    | uint8_t    | 0..8 Bytes       | SAE J1850  | 0x55       | 0xAA       |
+------------+------------+------------+---------------+------------+------------+------------------+------------+------------+------------+
|<-- 2B Preamble -------->|<----------------------- 7B Header -------------->|<--- 0..8B Data ->|<- 1B CRC ->|<-- 2B Postamble ------>|
```

### 1.1 Field layout

| Offset | Field | Type | Size | Description |
|:--|:--|:--|:--:|:--|
| `0` | `SOF1` | `u8` | 1 | Fixed `0xAA` |
| `1` | `SOF2` | `u8` | 1 | Fixed `0x55` |
| `2` | `FRAME_TYPE` | `u8` | 1 | See §2 |
| `3..6` | `CAN_ID` | `u32` | 4 | Little-endian identifier |
| `7` | `DLC` | `u8` | 1 | Payload length `0..8` |
| `8` | `SEQ` | `u8` | 1 | Sequence number (`0..255`), echoed in the response |
| `9..(9+DLC-1)` | `PAYLOAD` | `u8[]` | 0..8 | Command parameters / telemetry |
| `9+DLC` | `CRC8` | `u8` | 1 | SAE J1850 CRC-8 over bytes `2 .. 8+DLC` |
| `10+DLC` | `EOF1` | `u8` | 1 | Fixed `0x55` |
| `11+DLC` | `EOF2` | `u8` | 1 | Fixed `0xAA` |

* **Minimum frame**: 12 bytes (`DLC = 0`)
* **Maximum frame**: 20 bytes (`DLC = 8`)
* **Framing overhead**: 12 bytes

---

## 2. Frame types

| Value | Name | Direction | Semantics |
|:--:|:--|:--:|:--|
| `0x01` | `FRAME_TYPE_REQ` | Host → MCU | Command / data request |
| `0x02` | `FRAME_TYPE_RESP_OK` | MCU → Host | Acknowledged execution, optional return payload |
| `0x03` | `FRAME_TYPE_RESP_ERR` | MCU → Host | Error, `payload[0]` = error code |
| `0x04` | `FRAME_TYPE_ASYNC_EVENT` | MCU → Host | Unsolicited telemetry / fault event |

## 3. Status & error codes (`FRAME_TYPE_RESP_ERR`)

| Code | Symbol | Meaning |
|:--:|:--|:--|
| `0x00` | `ERR_OK` | Success (normally `RESP_OK`) |
| `0x01` | `ERR_CRC` | Frame CRC-8 check failed |
| `0x02` | `ERR_BAD_FORMAT` | Truncated frame / framing mismatch / `DLC > 8` |
| `0x03` | `ERR_UNKNOWN_CAN_ID` | CAN ID unmapped or unsupported |
| `0x04` | `ERR_BAD_ARG` | Payload parameter out of range or malformed |
| `0x05` | `ERR_BUS_TIMEOUT` | Downstream bus transaction timed out |
| `0x06` | `ERR_BUS_FAULT` | Hardware framing / DMA error |
| `0x07` | `ERR_BUSY` | Subsystem busy with a prior transaction |
| `0x08` | `ERR_SERVO_FAULT` | Servo reported a hardware alarm bitmask |
| `0x09` | `ERR_QUEUE_FULL` | Motion planner queue saturated (16 segments) |
| `0x0A` | `ERR_TIMEOUT` | Stepper wait deadline expired |
| `0x0B` | `ERR_NOT_SUPPORTED` | Command or direction not implemented |
| `0x0C` | `ERR_INTERNAL` | MCU driver internal fault |

## 4. SAE J1850 CRC-8

* **Polynomial**: `0x1D`, **init**: `0xFF`, **xorout**: `0xFF`
* **No input/output reflection** (MSB first)
* Computed over `FRAME_TYPE .. PAYLOAD` = `7 + DLC` bytes
* **Reference vector**: `CRC8("123456789") == 0x4B`

Both the firmware (`tue_usb_crc8`) and the host client
(`tue_usb.crc.crc8_sae_j1850`) implement this table-driven and are test-verified
against the reference vector.

---

## 5. CAN ID allocation

> [!WARNING]
> **CAN ID `0x3..` is STRICTLY RESERVED for user custom communication.**
> The standard `0x300–0x3FF`, 16-bit `0x3000–0x3FFF` and extended
> `0x30000000–0x3FFFFFFF` ranges must never be assigned to internal commands.
> The firmware enforces this with compile-time assertions
> (`TUE_USB_ASSERT_NOT_RESERVED`), and both implementations expose
> `is_reserved()` / `TUE_USB_CAN_ID_IS_RESERVED()`.

| ID range | Domain |
|:--|:--|
| `0x100 – 0x17F` | System management (SYS) |
| `0x200 – 0x27F` | CoreXY + Z tool motion (STEP) |
| `0x280 – 0x2FF` | Raw per-motor stepping (RAW) |
| `0x300 – 0x3FF` | **RESERVED — user custom** |
| `0x400 – 0x4FF` | STS3215 bus servos (SERVO) |
| `0x500 – 0x57F` | Sensors & inputs (reserved) |
| `0x600 – 0x6FF` | DC motors / actuators (reserved) |

---

## 6. Command map

Units: STEP positions are **signed int32 micrometres** (`1 mm = 1000 um`); feed
and acceleration are **`u16` mm/s** and **mm/s²**; jog velocities are **`i16`
mm/s**. Data in `[brackets]` is the return payload.

### 6.1 System (`0x100 – 0x107`)

| ID | Name | Req DLC | Request | Resp DLC | Response |
|:--:|:--|:--:|:--|:--:|:--|
| `0x100` | `SYS_PING` | 0..4 | `[token u32]` | 4 | `echo_token u32` |
| `0x101` | `SYS_INFO` | 0 | — | 8 | `hw_id u8, hw_rev u8, fw_maj u8, fw_min u8, build_flags u32` |
| `0x102` | `SYS_UID_PART1` | 0 | — | 8 | `uid0 u32, uid1 u32` |
| `0x103` | `SYS_UID_PART2` | 0 | — | 4 | `uid2 u32` |
| `0x104` | `SYS_UPTIME` | 0 | — | 4 | `uptime_ms u32` |
| `0x105` | `SYS_LED_CTRL` | 2 | `led_idx u8, state u8` | 2 | `led_idx u8, current_state u8` |
| `0x106` | `SYS_RESET` | 4 | `magic 0xDEADBEEF u32` | 1 | `ack u8` |
| `0x107` | `SYS_ECHO` | 1..8 | `data` | 1..8 | `data` (loopback) |

`SYS_LED_CTRL.state`: `0` off, `1` on, `2` toggle, `0xFF` query.

### 6.2 Stepper — tool motion (`0x200 – 0x21A`)

| ID | Name | Req DLC | Request | Resp DLC | Response |
|:--:|:--|:--:|:--|:--:|:--|
| `0x200` | `STEP_GET_INFO` | 0 | — | 8 | `xy_spmm u16, z_spmm u16, min_rate u16, max_rate u16` |
| `0x201` | `STEP_GET_STATUS` | 0 | — | 4 | `is_busy u8, is_enabled u8, queue_depth u8, reserved u8` |
| `0x202` | `STEP_ENABLE` | 1 | `enable u8 (0/1/0xFF=q)` | 1 | `state u8` |
| `0x203` | `STEP_GET_POS_XY` | 0 | — | 8 | `x_um i32, y_um i32` |
| `0x204` | `STEP_GET_POS_Z` | 0 | — | 4 | `z_um i32` |
| `0x205` | `STEP_GET_TGT_XY` | 0 | — | 8 | `x_um i32, y_um i32` |
| `0x206` | `STEP_GET_TGT_Z` | 0 | — | 4 | `z_um i32` |
| `0x207` | `STEP_SETPOS_XY` | 8 | `x_um i32, y_um i32` | 8 | `x_um i32, y_um i32` |
| `0x208` | `STEP_SETPOS_Z` | 4 | `z_um i32` | 4 | `z_um i32` |
| `0x210` | `STEP_MOVE_COORDS` | 8 | `x_um i32, y_um i32` (latch) | 2 | `ack u8, queue_depth u8` |
| `0x211` | `STEP_MOVE_EXEC` | 8 | `z_um i32, feed u16, accel u16` | 8 | `x_um i32, y_um i32` (committed) |
| `0x212` | `STEP_MOVE_XY` | 8 | `x_um i32, y_um i32` (default speed) | 8 | `x_um i32, y_um i32` |
| `0x213` | `STEP_MOVE_Z` | 8 | `z_um i32, feed u16, accel u16` | 4 | `z_um i32` |
| `0x214` | `STEP_MOVE_REL_XY` | 8 | `dx_um i32, dy_um i32` | 8 | `dx_um i32, dy_um i32` |
| `0x215` | `STEP_SET_SPEED` | 4 | `default_feed u16, default_accel u16` | 4 | `feed u16, accel u16` |
| `0x216` | `STEP_JOG_XY` | 6 | `vx i16, vy i16, accel u16` | 4 | `vx i16, vy i16` |
| `0x217` | `STEP_JOG_Z` | 4 | `vz i16, accel u16` | 2 | `vz i16` |
| `0x218` | `STEP_STOP` | 0 | — | 1 | `is_busy u8` |
| `0x219` | `STEP_ABORT` | 0 | — | 1 | `0 u8` |
| `0x21A` | `STEP_WAIT_IDLE` | 4 | `timeout_ms u32` (0 = forever) | 4 | `elapsed_ms u32` |

`STEP_MOVE_COORDS` + `STEP_MOVE_EXEC` latch an XYZ point then commit it as one
`tue_stepper_move_to_xyz_mm()` call. `STEP_MOVE_XY`/`STEP_MOVE_REL_XY` use the
speed set by `STEP_SET_SPEED` (defaults 60 mm/s, 600 mm/s²).

### 6.3 Stepper — raw channels (`0x280 – 0x281`)

| ID | Name | Req DLC | Request | Resp DLC | Response |
|:--:|:--|:--:|:--|:--:|:--|
| `0x280` | `STEP_RAW_GET_STEPS` | 1 | `motor_id u8 (1..3)` | 5 | `motor_id u8, steps i32` |
| `0x281` | `STEP_RAW_MOVE` | 8 | `motor_id u8, reserved u8, steps i32, feed u16` | 5 | `motor_id u8, steps i32` |

### 6.4 STS3215 servos (`0x400 – 0x415`)

| ID | Name | Req DLC | Request | Resp DLC | Response |
|:--:|:--|:--:|:--|:--:|:--|
| `0x400` | `SERVO_PING` | 1 | `servo_id u8` | 2 | `servo_id u8, ack u8` |
| `0x401` | `SERVO_SET_POS` | 6 | `id u8, accel u8, pos i16, speed i16` | 3 | `id u8, pos i16` |
| `0x402` | `SERVO_GET_POS` | 1 | `servo_id u8` | 3 | `id u8, pos i16` |
| `0x403` | `SERVO_SET_SPEED` | 4 | `id u8, accel u8, speed i16` | 3 | `id u8, speed i16` |
| `0x404` | `SERVO_GET_SPEED` | 1 | `servo_id u8` | 3 | `id u8, speed i16` |
| `0x405` | `SERVO_TORQUE` | 2 | `id u8, state u8` | 2 | `id u8, torque u8` |
| `0x406` | `SERVO_TLIM` | 3 | `id u8, limit u16 (0.1%)` | 3 | `id u8, limit u16` |
| `0x407` | `SERVO_MODE` | 2 | `id u8, mode u8 (0..3 / 0xFF=q)` | 2 | `id u8, active_mode u8` |
| `0x408` | `SERVO_PWM` | 3 | `id u8, pwm i16 (-1000..1000)` | 3 | `id u8, pwm i16` |
| `0x410` | `SERVO_GET_FB` | 1 | `servo_id u8` | dual | triggers `0x410` + `0x411` (see below) |
| `0x411` | `SERVO_FB_HEALTH` | — | *telemetry part 2* | 8 | `id u8, volt u8, temp i8, status u8, cur i16, res u16` |
| `0x412` | `SERVO_MEM_READ` | 3 | `id u8, reg u8, len u8 (1..6)` | 2..8 | `id u8, reg u8, val[len]` |
| `0x413` | `SERVO_MEM_WRITE` | 3..8 | `id u8, reg u8, val[..6]` | 3 | `id u8, reg u8, status u8` |
| `0x414` | `SERVO_SET_ID` | 2 | `current_id u8, new_id u8` | 2 | `new_id u8, status u8` |
| `0x415` | `SERVO_RESET` | 1 | `servo_id u8` | 2 | `id u8, status u8` |

`SERVO_TORQUE.state`: `0` off, `1` on, `128` calibrate, `0xFF` query.

**Dual-frame feedback.** `SERVO_GET_FB` answers with two 8-byte frames sharing
the request `SEQ`:

* `0x410 SERVO_FB_DYNAMICS`: `id u8, pos i16, spd i16, load i16, moving u8`
* `0x411 SERVO_FB_HEALTH`:   `id u8, volt u8, temp i8, status u8, cur i16, res u16`

The `status` byte carries the hardware fault bitmask (`0x01` voltage, `0x02`
sensor, `0x04` temperature, `0x08` current, `0x10` angle, `0x20` overload).

---

## 7. Host API (Python)

```python
from tue_usb import TueUsbClient, CanId, ErrorCode, FrameType

with TueUsbClient("COM7") as dev:
    print(dev.get_info())
    print(dev.step_get_position())       # {'x': .., 'y': .., 'z': ..} in mm
    dev.step_move_xyz(50, 30, 10, feed=60, accel=600)
    dev.step_wait(5000)
    dev.servo_torque_enable(True, 1)
    dev.servo_set_pos(2048, 1, accel=30, speed=800)
    print(dev.servo_get_feedback(1))
```

The transport-free core (`tue_usb.protocol.encode_frame`,
`tue_usb.protocol.FrameParser`, `tue_usb.crc`) can be used to build and parse
frames without a serial port.

## 8. Verification

| Check | Method |
|:--|:--|
| CRC-8 `0x4B` on `"123456789"` | C (`tue_usb_crc8_selftest`) + Python (`crc.self_test`) |
| No command in `0x3..` | Compile-time `_Static_assert` + Python `is_reserved` cross-check |
| Resync after corrupt/dropped bytes | `FrameParser` fuzz + bad-CRC/EOF tests |
| Enums match firmware | `tests/test_protocol.py` parses `Core/Inc/tue_usb_binary.h` |
| Round-trip latency | `python/examples/bench_test.py` (option `b`) |
