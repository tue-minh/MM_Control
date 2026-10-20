# TODO: STM32 USB Binary Protocol Migration Plan

> **Platform**: STM32F407VET6 (`MM_Control`) over **USB CDC (Virtual COM Port)**  
> **Frame Architecture**: CAN-over-USB Binary Protocol (Little-Endian, Fixed 12B Framing Overhead + 0..8B DLC)  
> **Integrity Check**: SAE J1850 CRC-8 (Polynomial `0x1D`, Init `0xFF`, XorOut `0xFF`)  
> **Target Subsystems**: System Management, CoreXY + Z Stepper Motion (`tue_stepper`), STS3215 Bus Servos (`tue_sts3215`), Sensors  
> **Critical Design Constraint**: **CAN ID range `0x3...` (`0x300`–`0x3FF`, `0x3000`–`0x3FFF`, or any ID with high nibble `0x3`) is STRICTLY RESERVED for user custom communications and MUST NOT be assigned to internal motor or system commands.**

---

## 1. Executive Summary & Architecture Overview

This document specifies the complete action plan to replace the legacy ASCII-based `tue_acsi` protocol with a high-throughput, deterministic, low-latency **Binary CAN Frame Protocol** encapsulated over USB CDC.

### 1.1 Motivation for Binary Protocol
1. **Zero Text-Formatting Overhead**: Eliminates all `snprintf()`, `strtol()`, `vsprintf()`, and token whitespace scanning on the STM32 Cortex-M4 core.
2. **Fixed Packet Boundaries**: Replaces variable-length newline-terminated strings with a fixed-delimiter framing envelope (`0xAA 0x55` ... `0x55 0xAA`).
3. **Direct CAN 2.0B Hardware Mapping**: The frame structure mirrors native CAN 2.0B standard/extended frames (`CAN ID` + `DLC (0..8)` + `Payload (0..8B)`). This enables the USB port to act directly as a host bridge to the onboard STM32 CAN1 controller or route transparently across internal CAN networks.
4. **Deterministic Latency & Hardware CRC**: Guaranteed 12 to 20 bytes per frame with hardware/table-accelerated SAE J1850 CRC-8 validation.

---

## 2. Binary Frame Specification

Every transaction over USB CDC conforms to the following byte envelope:

```
+------------+------------+------------+---------------+------------+------------+------------------+------------+------------+------------+
|  SOF1      |  SOF2      | FRAME_TYPE | CAN_ID (4B)   | DLC        | SEQ        | PAYLOAD (0..8B)  | CRC8       | EOF1       | EOF2       |
|  0xAA      |  0x55      | uint8_t    | uint32_t (LE) | uint8_t    | uint8_t    | 0..8 Bytes       | SAE J1850  | 0x55       | 0xAA       |
+------------+------------+------------+---------------+------------+------------+------------------+------------+------------+------------+
|<-- 2B Preamble -------->|<----------------------- 7B Header -------------->|<--- 0..8B Data ->|<- 1B CRC ->|<-- 2B Postamble ------>|
```

### 2.1 Field Layout & Byte Offsets

| Byte Offset | Field Name | Type | Size | Description / Valid Values |
|:---|:---|:---|:---:|:---|
| `0` | `SOF1` | `uint8_t` | 1 | Start of Frame byte 1: Fixed magic `0xAA` |
| `1` | `SOF2` | `uint8_t` | 1 | Start of Frame byte 2: Fixed magic `0x55` |
| `2` | `FRAME_TYPE` | `uint8_t` | 1 | Frame classification (see §2.2) |
| `3..6` | `CAN_ID` | `uint32_t` | 4 | Identifier selecting target device/command (Little-Endian) |
| `7` | `DLC` | `uint8_t` | 1 | Data Length Code: Payload byte count (`0` to `8`) |
| `8` | `SEQ` | `uint8_t` | 1 | Sequence number (`0x00`–`0xFF`). Echoed by MCU in response |
| `9..(9+DLC-1)`| `PAYLOAD` | `uint8_t[]`| 0..8 | Command-specific parameter or telemetry data |
| `9+DLC` | `CRC8` | `uint8_t` | 1 | SAE J1850 CRC-8 across bytes `[2 .. 8+DLC]` |
| `10+DLC` | `EOF1` | `uint8_t` | 1 | End of Frame byte 1: Fixed magic `0x55` |
| `11+DLC` | `EOF2` | `uint8_t` | 1 | End of Frame byte 2: Fixed magic `0xAA` |

- **Minimum Frame Length**: 12 bytes (`DLC = 0`)
- **Maximum Frame Length**: 20 bytes (`DLC = 8`)
- **Framing Overhead**: Fixed 12 bytes per packet

---

### 2.2 Frame Types (`FRAME_TYPE`)

| Value | Identifier | Direction | Semantics |
|:---:|:---|:---:|:---|
| `0x01` | `FRAME_TYPE_REQ` | Host $\rightarrow$ MCU | Command / Data Request sent by host |
| `0x02` | `FRAME_TYPE_RESP_OK` | MCU $\rightarrow$ Host | Acknowledged execution with optional return payload |
| `0x03` | `FRAME_TYPE_RESP_ERR` | MCU $\rightarrow$ Host | Execution error. `Payload[0]` = Error Status Code |
| `0x04` | `FRAME_TYPE_ASYNC_EVENT` | MCU $\rightarrow$ Host | Unsolicited periodic telemetry or emergency fault event |

---

### 2.3 Status & Error Codes (in `FRAME_TYPE_RESP_ERR`)

When `FRAME_TYPE == 0x03`, `DLC >= 1` and `Payload[0]` carries one of the following error codes:

| Code | Symbol | Meaning |
|:---:|:---|:---|
| `0x00` | `ERR_OK` | Success (normally returned with `FRAME_TYPE_RESP_OK`) |
| `0x01` | `ERR_CRC` | Frame CRC-8 check failed |
| `0x02` | `ERR_BAD_FORMAT` | Truncated frame, framing mismatch, or `DLC > 8` |
| `0x03` | `ERR_UNKNOWN_CAN_ID` | CAN ID is unmapped or unsupported |
| `0x04` | `ERR_BAD_ARG` | Payload parameter out of range or malformed |
| `0x05` | `ERR_BUS_TIMEOUT` | Downstream bus transaction timed out (e.g. STS3215 UART) |
| `0x06` | `ERR_BUS_FAULT` | Hardware framing/DMA error |
| `0x07` | `ERR_BUSY` | Subsystem busy with prior transaction |
| `0x08` | `ERR_SERVO_FAULT` | Servo reported hardware alarm bitmask |
| `0x09` | `ERR_QUEUE_FULL` | Stepper motion planner queue saturated (16 segments max) |
| `0x0A` | `ERR_TIMEOUT` | Stepper wait deadline expired |
| `0x0B` | `ERR_NOT_SUPPORTED` | Command or direction not implemented |
| `0x0C` | `ERR_INTERNAL` | MCU driver internal fault |

---

### 2.4 SAE J1850 CRC-8 Specification

The frame integrity is protected by **CRC-8/SAE J1850**:
- **Polynomial**: `0x1D` ($x^8 + x^4 + x^3 + x^2 + 1$)
- **Initial Value**: `0xFF`
- **Input Reflection**: False (MSB first)
- **Output Reflection**: False (MSB first)
- **XOR Output**: `0xFF`
- **Validation Constant**: For ASCII string `"123456789"` (`0x31..0x39`), the calculated CRC is `0x4B`.

**Protected Buffer Range**:
The CRC is computed over all bytes between the SOF and the CRC field:
$$\text{CRC} = \text{CRC8\_SAE\_J1850}(\text{FRAME\_TYPE}, \text{CAN\_ID}[0..3], \text{DLC}, \text{SEQ}, \text{PAYLOAD}[0..\text{DLC}-1])$$

Total bytes checked = $7 + \text{DLC}$ bytes.

---

## 3. CAN ID Allocation & Addressing Map

> [!WARNING]
> ### STRICT RESERVATION NOTICE
> **CAN ID range `0x3...` (`0x300` through `0x3FF`, `0x3000` through `0x3FFF`, or any extended 29-bit CAN ID matching `0x3XXXXXXX`) is STRICTLY RESERVED for future external communication protocols.**
> Under NO circumstances may any stepper, STS3215 servo, sensor, or system command be assigned to an ID beginning with `0x3`.

### 3.1 Primary CAN ID Partition Table

| ID Range | Functional Domain | Subsystem / Hardware |
|:---|:---|:---|
| `0x100` – `0x17F` | **System Management (SYS)** | Ping, Echo, Info, UID, Uptime, Status LED, MCU Reset |
| `0x200` – `0x27F` | **CoreXY + Z Stepper Motion (STEP)** | Tool kinematics, multi-axis moves, jogs, queue drain, stop |
| `0x280` – `0x2FF` | **Individual Stepper Motors (RAW)** | Direct per-motor step pulses (M1, M2, M3), homing, raw steps |
| `0x300` – `0x3FF` | **>>> RESERVED <<<** | **USER FUTURE COMMUNICATION (STRICTLY PROHIBITED)** |
| `0x400` – `0x4FF` | **STS3215 Bus Servos (SERVO)** | USART1 half-duplex bus: position, speed, torque, feedback, regs |
| `0x500` – `0x57F` | **Sensors & Inputs (SENSOR)** | Optical endstops, limit switches, ADC channels, IMU |
| `0x600` – `0x6FF` | **Actuators / DC Motors (DC)** | PWM channels, direction, encoders (future expansion) |

---

## 4. Complete Function & Payload Mapping

Because standard CAN frames limit the payload to 8 bytes (`DLC <= 8`), multi-parameter commands (such as 3D toolpath moves or comprehensive servo telemetry) are designed with clean 8-byte boundaries or paired ID sequences.

### 4.1 System Commands (`0x100` – `0x17F`)

| CAN ID | Command Name | Req DLC | Payload Format (Host $\rightarrow$ MCU) | Resp DLC | Payload Format (MCU $\rightarrow$ Host) |
|:---:|:---|:---:|:---|:---:|:---|
| `0x100` | `SYS_PING` | 0..4 | `[token (uint32)]` (optional) | 4 | `[echo_token (uint32)]` |
| `0x101` | `SYS_INFO` | 0 | None | 8 | `hw_id (u8), hw_rev (u8), fw_maj (u8), fw_min (u8), build_flags (u32)` |
| `0x102` | `SYS_UID_PART1` | 0 | None | 8 | `uid0 (uint32_t LE), uid1 (uint32_t LE)` |
| `0x103` | `SYS_UID_PART2` | 0 | None | 4 | `uid2 (uint32_t LE)` |
| `0x104` | `SYS_UPTIME` | 0 | None | 4 | `uptime_ms (uint32_t LE)` |
| `0x105` | `SYS_LED_CTRL` | 2 | `led_idx (u8), state (u8: 0=off, 1=on, 2=tgl, 0xFF=query)` | 2 | `led_idx (u8), current_state (u8)` |
| `0x106` | `SYS_RESET` | 4 | `magic_key (0xDEADBEEF uint32)` | 1 | `ack_code (0=rebooting)` |
| `0x107` | `SYS_ECHO` | 1..8 | `data[0..N-1]` | 1..8 | `data[0..N-1]` (exact loopback) |

---

### 4.2 Stepper Motor Controller Commands (`0x200` – `0x27F`)

Distances on the wire are encoded as **signed 32-bit micrometres** (`1 mm = 1000 um`); speeds are in **um/s** or **mm/s**; accelerations are in **mm/s²**.

| CAN ID | Command Name | Req DLC | Payload Format (Host $\rightarrow$ MCU) | Resp DLC | Payload Format (MCU $\rightarrow$ Host) |
|:---:|:---|:---:|:---|:---:|:---|
| `0x200` | `STEP_GET_INFO` | 0 | None | 8 | `xy_spmm (u16), z_spmm (u16), min_rate (u16), max_rate (u16)` |
| `0x201` | `STEP_GET_STATUS`| 0 | None | 4 | `is_busy (u8), is_enabled (u8), queue_depth (u8), reserved (u8)` |
| `0x202` | `STEP_ENABLE` | 1 | `enable (u8: 0=disable, 1=enable, 0xFF=query)` | 1 | `state (u8: 0=disabled, 1=enabled)` |
| `0x203` | `STEP_GET_POS_XY`| 0 | None | 8 | `x_um (int32_t LE), y_um (int32_t LE)` |
| `0x204` | `STEP_GET_POS_Z` | 0 | None | 4 | `z_um (int32_t LE)` |
| `0x205` | `STEP_GET_TGT_XY`| 0 | None | 8 | `target_x_um (int32_t LE), target_y_um (int32_t LE)` |
| `0x206` | `STEP_GET_TGT_Z` | 0 | None | 4 | `target_z_um (int32_t LE)` |
| `0x207` | `STEP_SETPOS_XY` | 8 | `x_um (int32_t LE), y_um (int32_t LE)` | 8 | `x_um (int32_t LE), y_um (int32_t LE)` |
| `0x208` | `STEP_SETPOS_Z` | 4 | `z_um (int32_t LE)` | 4 | `z_um (int32_t LE)` |
| `0x210` | `STEP_MOVE_COORDS`| 8 | `x_um (int32_t LE), y_um (int32_t LE)` (Arms XYZ point) | 2 | `ack (u8), queue_depth (u8)` |
| `0x211` | `STEP_MOVE_EXEC` | 8 | `z_um (int32 LE), feed_mm_s (u16), accel_mm_s2 (u16)` | 8 | `accepted_x_um (i32), accepted_y_um (i32)` (Moves committed) |
| `0x212` | `STEP_MOVE_XY` | 8 | `x_um (int32_t LE), y_um (int32_t LE)` (Uses default speed) | 8 | `x_um (int32_t LE), y_um (int32_t LE)` |
| `0x213` | `STEP_MOVE_Z` | 8 | `z_um (int32 LE), feed_mm_s (u16), accel_mm_s2 (u16)` | 4 | `z_um (int32_t LE)` |
| `0x214` | `STEP_MOVE_REL_XY`| 8 | `dx_um (int32_t LE), dy_um (int32_t LE)` | 8 | `dx_um (int32_t LE), dy_um (int32_t LE)` |
| `0x215` | `STEP_SET_SPEED` | 4 | `default_feed_mm_s (u16), default_accel_mm_s2 (u16)` | 4 | `feed (u16), accel (u16)` |
| `0x216` | `STEP_JOG_XY` | 6 | `vx_mm_s (int16_t), vy_mm_s (int16_t), accel_mm_s2 (u16)` | 4 | `vx_mm_s (int16_t), vy_mm_s (int16_t)` |
| `0x217` | `STEP_JOG_Z` | 4 | `vz_mm_s (int16_t), accel_mm_s2 (uint16_t)` | 2 | `vz_mm_s (int16_t)` |
| `0x218` | `STEP_STOP` | 0 | None (Controlled deceleration) | 1 | `is_busy (u8)` |
| `0x219` | `STEP_ABORT` | 0 | None (Immediate step clock halt) | 1 | `0 (u8)` |
| `0x21A` | `STEP_WAIT_IDLE` | 4 | `timeout_ms (uint32_t LE)` (`0` = wait forever) | 4 | `elapsed_ms (uint32_t LE)` |

---

### 4.3 Raw Stepper Channels (`0x280` – `0x29F`)

| CAN ID | Command Name | Req DLC | Payload Format (Host $\rightarrow$ MCU) | Resp DLC | Payload Format (MCU $\rightarrow$ Host) |
|:---:|:---|:---:|:---|:---:|:---|
| `0x280` | `STEP_RAW_GET_STEPS`| 1 | `motor_id (u8: 1=M1, 2=M2, 3=M3)` | 5 | `motor_id (u8), steps (int32_t LE)` |
| `0x281` | `STEP_RAW_MOVE` | 8 | `motor_id (u8), reserved (u8), steps (i32), feed_mm_s (u16)` | 5 | `motor_id (u8), steps (int32_t LE)` |

---

### 4.4 STS3215 Serial Bus Servos (`0x400` – `0x4FF`)

*(Notice: Strictly avoiding `0x3...` as required!)*

| CAN ID | Command Name | Req DLC | Payload Format (Host $\rightarrow$ MCU) | Resp DLC | Payload Format (MCU $\rightarrow$ Host) |
|:---:|:---|:---:|:---|:---:|:---|
| `0x400` | `SERVO_PING` | 1 | `servo_id (u8)` | 2 | `servo_id (u8), ack_status (u8)` |
| `0x401` | `SERVO_SET_POS` | 6 | `id (u8), accel (u8), pos_steps (int16 LE), speed (int16 LE)`| 3 | `servo_id (u8), echo_pos (int16_t LE)` |
| `0x402` | `SERVO_GET_POS` | 1 | `servo_id (u8)` | 3 | `servo_id (u8), pos_steps (int16_t LE)` |
| `0x403` | `SERVO_SET_SPEED`| 4 | `id (u8), accel (u8), speed_steps_s (int16_t LE)` | 3 | `servo_id (u8), echo_speed (int16_t LE)` |
| `0x404` | `SERVO_GET_SPEED`| 1 | `servo_id (u8)` | 3 | `servo_id (u8), speed_steps_s (int16_t LE)` |
| `0x405` | `SERVO_TORQUE` | 2 | `id (u8), state (u8: 0=off, 1=on, 128=calib, 0xFF=query)` | 2 | `servo_id (u8), current_torque (u8)` |
| `0x406` | `SERVO_TLIM` | 3 | `id (u8), limit_0p1pct (uint16_t LE: 0..1000)` | 3 | `servo_id (u8), limit_0p1pct (uint16_t LE)` |
| `0x407` | `SERVO_MODE` | 2 | `id (u8), mode (u8: 0=pos, 1=spd, 2=pwm, 3=step, 0xFF=q)` | 2 | `servo_id (u8), active_mode (u8)` |
| `0x408` | `SERVO_PWM` | 3 | `id (u8), pwm_duty (int16_t LE: -1000..1000)` | 3 | `servo_id (u8), echo_pwm (int16_t LE)` |
| `0x410` | `SERVO_GET_FB` | 1 | `servo_id (u8)` (Triggers Dual-Frame Feedback Response)| -- | *Triggers Frames `0x410` & `0x411` below* |
| `0x410` | `SERVO_FB_DYNAMICS`| -- | *Telemetry Response Part 1* | 8 | `id (u8), pos (i16), spd (i16), load_0p1pct (i16), moving (u8)` |
| `0x411` | `SERVO_FB_HEALTH` | -- | *Telemetry Response Part 2* | 8 | `id (u8), volt_0p1v (u8), temp_c (i8), status (u8), curr_ma (i16), res (u16)` |
| `0x412` | `SERVO_MEM_READ` | 3 | `servo_id (u8), reg_addr (u8), len (u8: 1..6)` | 2..8 | `id (u8), reg_addr (u8), val[0..len-1]` |
| `0x413` | `SERVO_MEM_WRITE`| 3..8 | `servo_id (u8), reg_addr (u8), val[0..N]` (up to 6B write)| 3 | `servo_id (u8), reg_addr (u8), status (u8)` |
| `0x414` | `SERVO_SET_ID` | 2 | `current_id (u8), new_id (u8)` | 2 | `new_id (u8), status (u8)` |
| `0x415` | `SERVO_RESET` | 1 | `servo_id (u8)` (Factory EEPROM restore) | 2 | `servo_id (u8), status (u8)` |

---

## 5. Implementation Roadmap & Action List

### Phase 1: Specifications & Protocol Module Definition (Firmware)
- [x] **Action 1.1**: Define header `Core/Inc/tue_usb_binary.h`:
  - Declare binary constants: `TUE_USB_SOF1 = 0xAA`, `TUE_USB_SOF2 = 0x55`, `TUE_USB_EOF1 = 0x55`, `TUE_USB_EOF2 = 0xAA`.
  - Declare enums for `FRAME_TYPE` (`REQ`, `RESP_OK`, `RESP_ERR`, `ASYNC_EVENT`).
  - Declare enums for Status/Error codes (`ERR_OK`, `ERR_CRC`, `ERR_BAD_FORMAT`, `ERR_UNKNOWN_CAN_ID`, etc.).
  - Declare CAN ID constants for System (`0x100`–`0x107`), Steppers (`0x200`–`0x281`), and STS3215 Servos (`0x400`–`0x415`).
  - Add explicit compile-time assertion or check verifying no command uses ID `0x3...`.
  - Declare typed packet structure:
    ```c
    typedef struct __attribute__((packed)) {
        uint8_t  frame_type;
        uint32_t can_id;
        uint8_t  dlc;
        uint8_t  seq;
        uint8_t  payload[8];
        uint8_t  crc8;
    } tue_usb_frame_t;
    ```
- [x] **Action 1.2**: Implement CRC-8 SAE J1850 engine in `Core/Src/tue_usb_binary.c`:
  - Embed 256-entry lookup table `s_crc8_table[256]` for single-cycle table lookup per byte.
  - Implement `uint8_t tue_usb_crc8(const uint8_t *data, size_t len)`.
  - Validate with known test vectors (`"123456789"` $\rightarrow$ `0x4B`).

---

### Phase 2: Frame Parsing & State Machine Engine (Firmware)
- [x] **Action 2.1**: Implement defensive streaming state machine in `Core/Src/tue_usb_binary.c`:
  ```
  STATE_WAIT_SOF1 -> STATE_WAIT_SOF2 -> STATE_HEADER -> STATE_PAYLOAD -> STATE_CRC -> STATE_EOF1 -> STATE_EOF2
  ```
  - State machine runs in FreeRTOS task context draining incoming ring buffer.
  - Tolerates arbitrary USB packet fragmentation, concatenation, and line noise.
  - Automatically resets to `STATE_WAIT_SOF1` if EOF bytes do not match `0x55 0xAA` or if CRC fails.
- [x] **Action 2.2**: Implement thread-safe USB transmit engine:
  - Allocate static transmit buffer for frame generation.
  - Serialize frame: `[0xAA, 0x55, TYPE, ID_LE(4), DLC, SEQ, PAYLOAD(DLC), CRC8, 0x55, 0xAA]`.
  - Non-blocking transmit with retry/poll on `hcdc->TxState == 0` and RTOS delay.
  - Helper functions: `tue_usb_send_ok(can_id, seq, payload, dlc)`, `tue_usb_send_err(can_id, seq, err_code)`.

---

### Phase 3: Subsystem Command Dispatchers (Firmware)
- [x] **Action 3.1**: System Command Dispatcher (`0x100`–`0x107`):
  - Handle `SYS_PING` (return echo token).
  - Handle `SYS_INFO` (return packed HW ID, version, capabilities).
  - Handle `SYS_UID_PART1` & `SYS_UID_PART2` (read `HAL_GetUIDw0..w2`).
  - Handle `SYS_UPTIME` (read `HAL_GetTick()`).
  - Handle `SYS_LED_CTRL` (read/write `LED_MCU_GPIO_Port`, `LED_MCU_Pin`).
  - Handle `SYS_RESET` (verify `0xDEADBEEF`, send response, delay 20ms, call `HAL_NVIC_SystemReset()`).
- [x] **Action 3.2**: Stepper Command Dispatcher (`0x200`–`0x281`):
  - Bind to `tue_stepper.h`:
    - `STEP_GET_INFO` $\rightarrow$ `tue_stepper_get_config()`.
    - `STEP_GET_STATUS` $\rightarrow$ `tue_stepper_is_busy()`, `tue_stepper_is_enabled()`.
    - `STEP_ENABLE` $\rightarrow$ `tue_stepper_enable()`.
    - `STEP_GET_POS_XY` / `Z` $\rightarrow$ `tue_stepper_get_position_mm()`.
    - `STEP_MOVE_COORDS` + `STEP_MOVE_EXEC` $\rightarrow$ coordinate latch followed by `tue_stepper_move_to_xyz_mm()`.
    - `STEP_MOVE_XY` $\rightarrow$ `tue_stepper_move_to_mm()`.
    - `STEP_MOVE_Z` $\rightarrow$ `tue_stepper_move_z_mm()`.
    - `STEP_MOVE_REL_XY` $\rightarrow$ `tue_stepper_move_by_mm()`.
    - `STEP_JOG_XY` $\rightarrow$ `tue_stepper_start_velocity()`.
    - `STEP_JOG_Z` $\rightarrow$ `tue_stepper_start_velocity_z()`.
    - `STEP_STOP` $\rightarrow$ `tue_stepper_stop()`.
    - `STEP_ABORT` $\rightarrow$ `tue_stepper_abort()`.
    - `STEP_WAIT_IDLE` $\rightarrow$ `tue_stepper_wait_idle()`.
    - `STEP_RAW_GET_STEPS` $\rightarrow$ `tue_stepper_get_motor_steps()`.
    - `STEP_RAW_MOVE` $\rightarrow$ `tue_stepper_move_motor_steps()`.
- [x] **Action 3.3**: STS3215 Servo Command Dispatcher (`0x400`–`0x415`):
  - Bind to `tue_sts3215.h`:
    - `SERVO_PING` $\rightarrow$ `tue_sts3215_ping()`.
    - `SERVO_SET_POS` $\rightarrow$ `tue_sts3215_set_goal_position_ex()`.
    - `SERVO_GET_POS` $\rightarrow$ `tue_sts3215_get_current_position()`.
    - `SERVO_SET_SPEED` $\rightarrow$ `tue_sts3215_set_running_speed()`.
    - `SERVO_GET_SPEED` $\rightarrow$ `tue_sts3215_get_current_speed()`.
    - `SERVO_TORQUE` $\rightarrow$ `tue_sts3215_enable_torque()` / calibrate.
    - `SERVO_TLIM` $\rightarrow$ `tue_sts3215_set_torque_limit()`.
    - `SERVO_MODE` $\rightarrow$ `tue_sts3215_set_operation_mode()`.
    - `SERVO_PWM` $\rightarrow$ `tue_sts3215_set_pwm_runtime()`.
    - `SERVO_GET_FB` $\rightarrow$ `tue_sts3215_get_feedback()`, followed by sending both Frame `0x410` (Dynamics) and Frame `0x411` (Health).
    - `SERVO_MEM_READ` $\rightarrow$ `tue_sts3215_read_raw()`.
    - `SERVO_MEM_WRITE` $\rightarrow$ `tue_sts3215_write_raw()`.
    - `SERVO_SET_ID` $\rightarrow$ `tue_sts3215_set_id()`.
    - `SERVO_RESET` $\rightarrow$ `tue_sts3215_factory_reset()`.

---

### Phase 4: Firmware Integration & Build System
- [x] **Action 4.1**: Connect CDC Receive Interrupt:
  - In `USB_DEVICE/App/usbd_cdc_if.c`: Replace `tue_acsi_on_cdc_rx(Buf, *Len);` with `tue_usb_on_rx(Buf, *Len);`.
- [x] **Action 4.2**: Replace Task Init in `Core/Src/freertos.c`:
  - In `StartDefaultTask()`: Replace `tue_acsi_init();` with `tue_usb_init();`.
- [x] **Action 4.3**: Update `CMakeLists.txt`:
  - Replace `Core/Src/tue_acsi.c` with `Core/Src/tue_usb_binary.c`.
  - Verify clean compilation with arm-none-eabi-gcc.

---

### Phase 5: Python API Library Rebuild (`python/`)
- [x] **Action 5.1**: Package Architecture:
  - Deprecate/rename `python/tue_acsi` to `python/tue_usb` (or provide unified `tue_usb` package).
  - Update `python/pyproject.toml` with project name `tue-usb` and entry points.
- [x] **Action 5.2**: Implement `python/tue_usb/crc.py`:
  - Implement fast table-driven CRC-8 SAE J1850 with `crc8_sae_j1850(data: bytes) -> int`.
  - Add self-test validating against known CRC vectors.
- [x] **Action 5.3**: Implement `python/tue_usb/defs.py`:
  - Define `FrameType(IntEnum)`: `REQ = 1`, `RESP_OK = 2`, `RESP_ERR = 3`, `ASYNC_EVENT = 4`.
  - Define `ErrorCode(IntEnum)`: `OK = 0`, `ERR_CRC = 1`, `ERR_BAD_FORMAT = 2`, etc.
  - Define `CanId(IntEnum)`: All allocated IDs for System (`0x100..`), Stepper (`0x200..`), Servo (`0x400..`).
  - Ensure CAN IDs `0x3...` are marked as `RESERVED_CUSTOM_COMM`.
- [x] **Action 5.4**: Implement `python/tue_usb/protocol.py`:
  - Implement `encode_frame(frame_type, can_id, dlc, seq, payload) -> bytes`.
  - Implement `FrameParser` class:
    - Buffers incoming serial byte streams.
    - Scans for `0xAA 0x55`.
    - Parses header, extracts payload, verifies SAE J1850 CRC-8, verifies `0x55 0xAA`.
    - Yields validated `Frame` objects.
- [x] **Action 5.5**: Implement `python/tue_usb/client.py` (`TueUsbClient`):
  - Serial connection management via `pyserial`.
  - Monotonic sequence number counter (`self._seq`).
  - Thread-safe or synchronous `transact(can_id, payload, timeout) -> Frame`.
  - High-level API methods:
    - **System**: `ping()`, `get_info()`, `get_uid()`, `get_uptime()`, `set_led()`, `reset_mcu()`.
    - **Stepper**: `step_enable()`, `step_get_status()`, `step_get_position()`, `step_move_xyz()`, `step_move_xy()`, `step_move_z()`, `step_move_rel()`, `step_jog_xy()`, `step_stop()`, `step_abort()`, `step_wait()`.
    - **STS3215 Servo**: `servo_ping()`, `servo_set_pos()`, `servo_get_pos()`, `servo_set_speed()`, `servo_torque_enable()`, `servo_get_feedback()`, `servo_read_reg()`, `servo_write_reg()`.
- [x] **Action 5.6**: Rebuild Examples & Demos:
  - Update `python/examples/bench_test.py` for binary packet roundtrip latency benchmarking.
  - Update `python/examples/stepper_demo.py` and `servo_demo.py`.
- [x] **Action 5.7**: Unit & Mock Test Suite:
  - Rebuild `python/tests/test_protocol.py` with 100% coverage of binary frame packing, parsing, CRC verification, and simulated serial exchanges.

---

### Phase 6: Documentation & Specification Update (`docs/`)
- [x] **Action 6.1**: Create `docs/USB_BINARY_PROTOCOL.md`:
  - Comprehensive specification of the binary CAN-over-USB protocol, framing, CRC-8, and payload definitions.
- [x] **Action 6.2**: Update `docs/STEPPER_INSTRUCTION.md`:
  - Update the host communication section to reference CAN IDs `0x200`–`0x281` and binary payload units.
- [x] **Action 6.3**: Update `docs/STS3215_INSTRUCTION.md`:
  - Update servo host communication section to reference CAN IDs `0x400`–`0x415` and 8-byte telemetry bursts.
- [x] **Action 6.4**: Update `python/README.md`:
  - Document installation, API usage, examples, and performance comparison against the legacy ASCII protocol.
- [x] **Action 6.5**: Archive / Deprecate `docs/ACSI_PROTOCOL.md`:
  - Mark `ACSI_PROTOCOL.md` as superseded by the new binary protocol.

---

## 6. Verification & Acceptance Criteria

| Subsystem | Test / Acceptance Criterion | Verification Method |
|:---|:---|:---|
| **CRC-8** | SAE J1850 implementation produces `0x4B` on test vector `"123456789"` | C unit test & Python pytest |
| **CAN ID Restriction** | Zero internal commands use ID prefix `0x3...` | Automated header linting check |
| **USB CDC Framing** | Frame synchronizer correctly recovers after arbitrary corrupted/dropped bytes | Stream fuzzing test in Python test suite |
| **Motion Execution** | Stepper CoreXY moves (XYZ, 2D, Jogs) match commanded distances within 1 um | Hardware test / logic analyzer on PE9, PE11, PE13 |
| **Servo Feedback** | STS3215 dual-frame feedback (`0x410` + `0x411`) returns full 15-byte telemetry | Hardware bus test on PA9 with oscilloscope |
| **Throughput Benchmark** | Round-trip request/response latency reduced by $\ge 60\%$ compared to ACSI ASCII | `bench_test.py` 1000-packet ping/pos benchmark |
