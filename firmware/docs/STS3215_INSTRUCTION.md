# STS3215 Serial Bus Servo Driver Instruction Manual

> **Target Platform**: STM32F407VET6 (ARM Cortex-M4F @ 168 MHz)  
> **Peripheral**: USART1 Half-Duplex Single-Wire Mode on `PA9` (`SERVO_TX`)  
> **DMA Configuration**: DMA2 Stream 7 (TX) & DMA2 Stream 2 (RX) in Normal Mode with Interrupts  
> **Protocol Reference**: [docs/MEMORY_TABLE.md](file:///D:/Project_Kicad/MM_Control/firmware/docs/MEMORY_TABLE.md)  
> **Library Files**:
> [`Core/Inc/tue_sts3215_regs.h`](file:///D:/Project_Kicad/MM_Control/firmware/Core/Inc/tue_sts3215_regs.h),
> [`Core/Inc/tue_sts3215_protocol.h`](file:///D:/Project_Kicad/MM_Control/firmware/Core/Inc/tue_sts3215_protocol.h) / [`Core/Src/tue_sts3215_protocol.c`](file:///D:/Project_Kicad/MM_Control/firmware/Core/Src/tue_sts3215_protocol.c),
> [`Core/Inc/tue_sts3215_hal.h`](file:///D:/Project_Kicad/MM_Control/firmware/Core/Inc/tue_sts3215_hal.h) / [`Core/Src/tue_sts3215_hal.c`](file:///D:/Project_Kicad/MM_Control/firmware/Core/Src/tue_sts3215_hal.c),
> [`Core/Inc/tue_sts3215.h`](file:///D:/Project_Kicad/MM_Control/firmware/Core/Inc/tue_sts3215.h) / [`Core/Src/tue_sts3215.c`](file:///D:/Project_Kicad/MM_Control/firmware/Core/Src/tue_sts3215.c)

---

## 1. Overview & Architecture

The **TUE STS3215 Driver** controls Feetech STS-series serial bus servos (such as the STS3215) on the STM32F407VET6. It is a three-layer port of the reference C library in `reference_for_sts325/`, wrapped by a blocking facade that stays API-compatible with the TUE binary servo command set.

### Layer Map
| Layer | Files | Depends on | Responsibility |
|:---|:---|:---|:---|
| Registers | `Core/Inc/tue_sts3215_regs.h` | — | Register map, protocol constants, enums, fault masks, defaults, unit conversions |
| Protocol | `Core/Inc/tue_sts3215_protocol.h`, `Core/Src/tue_sts3215_protocol.c` | `regs.h` + libc | Pure packet builders + reply parser (host-testable, zero STM32 dependency) |
| HAL | `Core/Inc/tue_sts3215_hal.h`, `Core/Src/tue_sts3215_hal.c` | `main.h` | Async half-duplex DMA state machine + weak HAL callback overrides |
| Facade | `Core/Inc/tue_sts3215.h`, `Core/Src/tue_sts3215.c` | all above + `cmsis_os2.h` | Typed, blocking API consumed by `tue_usb_binary.c` |

### Key Highlights
- **Three clean layers**: register/protocol knowledge is `STM32`-free and unit-testable on a host; the HAL owns the hardware; the facade offers the ergonomic typed API.
- **Async HAL + blocking facade**: the HAL is a non-blocking state machine (`IDLE → TX_BUSY → RX_BUSY`) with `on_reply`/`on_error` callbacks; the facade polls it under an RTOS delay until the reply arrives or the timeout expires, preserving the synchronous `tue_sts3215_*` API.
- **Hardware Half-Duplex DMA, adapted from the reference**: the reference library targets RS485 (DE pin). On this board USART1 runs single-wire half-duplex (`CR3_HDSEL`), so the transceiver direction is switched inside the TX-complete interrupt (`HAL_HalfDuplex_EnableReceiver`) before `HAL_UARTEx_ReceiveToIdle_DMA` arms reception on DMA2 Stream 2.
- **Zero-Latency Transceiver Turnaround**: directional switching occurs inside the UART Transmission Complete (`TC`) interrupt callback, guaranteeing reception even when servo Return Delay is set to minimum.
- **Multi-reply transactions**: the HAL accumulates and parses multiple reply frames per transaction, enabling broadcast `SYNC_READ` that collects one reply per servo.
- **100% Memory Table Coverage**: typed, dedicated API functions for every register (`0x00` through `0x45`) in [docs/MEMORY_TABLE.md](file:///D:/Project_Kicad/MM_Control/firmware/docs/MEMORY_TABLE.md), plus the full instruction set (`PING`, `READ`, `WRITE`, `REG_WRITE`, `ACTION`, `SYNC_WRITE`, `SYNC_READ`, `RESET`, `RESET_TURNS`).
- **Safe EEPROM Operations**: automatic unlocking of register `0x37` (`Lock Symbol`) during EEPROM parameter modifications so settings persist across power cycles.
- **High-Throughput Burst Feedback**: fetches all 15 telemetry bytes (Position, Speed, Load, Voltage, Temperature, Error Status, Moving Flag, Current) in a single bus transaction.

---

## 2. Hardware Wiring & Pinout

```
+-------------------+                      +-------------------------+
|  STM32F407VET6    |                      |   FEETECH STS3215       |
|                   |                      |   SERIAL BUS SERVO      |
|  PA9 (SERVO_TX)   |--------------------->| DATA (White/Yellow wire)|
|                   |      1-Wire UART     |                         |
|  GND              |--------------------->| GND  (Black/Brown wire) |
+-------------------+                      +-------------------------+
                                                       |
+-------------------+                                  |
| External Power    |                                  |
| 6.0V - 12.0V DC   |--------------------------------->| VDD  (Red wire)     |
| (Battery / PSU)   |                                  |                     |
| GND               |----------------------------------+                     |
+-------------------+                                  | (Common Ground)     |
```

> [!IMPORTANT]
> 1. **Common Ground**: Ensure the external power supply GND is firmly tied to STM32 GND.
> 2. **Pull-Up Resistor**: PA9 is configured as Open-Drain (`GPIO_MODE_AF_OD`). An external pull-up resistor (4.7 kΩ to 10 kΩ to 3.3V or 5V) should be installed on the bus line if not present in your servo interface board. PA9 is 5V-tolerant on STM32F407.

---

## 3. STM32CubeMX Peripheral Configuration

The firmware project is already configured with the required peripheral mappings:

| Parameter | Configuration | Details |
|:---|:---|:---|
| **Pin** | `PA9` (`SERVO_TX_Pin`) | Alternate Function `AF7` (USART1), Open-Drain (`GPIO_MODE_AF_OD`), Very High Speed |
| **Mode** | Single Wire (Half Duplex) | `HAL_HalfDuplex_Init(&huart1)` |
| **Baud Rate** | `1,000,000 bps` | 8 Data Bits, No Parity, 1 Stop Bit (8N1) |
| **TX DMA** | `DMA2_Stream7`, Channel 4 | Direction: Memory-to-Peripheral, Mode: **Normal**, Data width: Byte |
| **RX DMA** | `DMA2_Stream2`, Channel 4 | Direction: Peripheral-to-Memory, Mode: **Normal**, Data width: Byte |
| **Interrupts** | `USART1_IRQn`, `DMA2_Stream7_IRQn`, `DMA2_Stream2_IRQn` | Priority 5, Subpriority 0 |

---

## 4. Quick Start Guide

### 4.1 Initialization

Include the header and initialize the global servo bus handle:

```c
#include "tue_sts3215.h"

/* Global servo bus instance */
static tue_sts3215_t g_servo_bus;

void App_Servo_Init(void)
{
    /* Initialize driver with USART1 handle */
    tue_sts3215_status_t status = tue_sts3215_init(&g_servo_bus, &huart1);
    if (status != TUE_STS3215_OK) {
        // Initialization error handling
    }

    /* Optional: Configure response timeout (default is 20 ms) */
    tue_sts3215_set_timeout(&g_servo_bus, 15);
}
```

### 4.2 Ping & Bus Verification

```c
uint8_t servo_id = 1;

if (tue_sts3215_ping(&g_servo_bus, servo_id) == TUE_STS3215_OK) {
    // Servo 1 is online and communicating!
} else {
    // Servo not found or communication timed out
}
```

### 4.3 Motion Control Example

```c
/* 1. Enable Motor Torque */
tue_sts3215_enable_torque(&g_servo_bus, servo_id, true);

/* 2. Command Goal Position:
 *    - Position: 2048 (Middle position / 180 degrees)
 *    - Speed: 1500 steps/s
 *    - Acceleration: 50 (5,000 steps/s²)
 */
tue_sts3215_set_goal_position_ex(&g_servo_bus, servo_id, 2048, 1500, 50);

/* Wait for motion to complete */
bool moving = true;
while (moving) {
    tue_sts3215_is_moving(&g_servo_bus, servo_id, &moving);
    osDelay(20);
}
```

### 4.4 High-Speed Telemetry Burst

Instead of making multiple bus transactions, read all telemetry values simultaneously:

```c
tue_sts3215_feedback_t fb;

if (tue_sts3215_get_feedback(&g_servo_bus, servo_id, &fb) == TUE_STS3215_OK) {
    int16_t current_pos = fb.position;         // in steps (0-4095)
    int16_t current_spd = fb.speed;            // in steps/s
    float   voltage_v   = fb.voltage_v;        // e.g. 11.8 V
    int8_t  temp_c      = fb.temperature_degc; // e.g. 38 °C
    float   current_ma  = fb.current_ma;       // e.g. 240.5 mA
    uint8_t error_flags = fb.status_mask;      // 0 if healthy
}
```

### 4.5 Multi-Servo Synchronized Write (`SYNC_WRITE`)

Drive multiple servos in lockstep with a single broadcast frame:

```c
tue_sts3215_sync_pos_t targets[3] = {
    { .id = 1, .position = 1000, .speed = 2000, .accel = 50 },
    { .id = 2, .position = 2000, .speed = 2000, .accel = 50 },
    { .id = 3, .position = 3000, .speed = 2000, .accel = 50 },
};

/* Broadcast position commands simultaneously to IDs 1, 2, 3 */
tue_sts3215_sync_write_position(&g_servo_bus, targets, 3);
```

### 4.6 Multi-Servo Synchronized Read (`SYNC_READ`)

Read the same register block from several servos in one broadcast and collect
each reply (up to `STS3215_SYNC_MAX_SERVOS` = 16 frames per transaction):

```c
uint8_t ids[3] = { 1, 2, 3 };
tue_sts3215_reply_t replies[3];
uint8_t n = 0;

/* Read 2 bytes (current position) from each servo */
if (tue_sts3215_sync_read(&g_servo_bus, STS3215_REG_CURRENT_POS, 2, ids, 3, replies, &n)
        == TUE_STS3215_OK) {
    for (uint8_t i = 0; i < n; i++) {
        int16_t pos = tue_sts3215_unpack_s16_le(replies[i].data);
        /* replies[i].id holds the responding servo ID */
    }
}
```

### 4.7 Clearing the Multi-Turn Counter (`RESET_TURNS`)

```c
/* Clear the volatile multi-turn cycle counter (instruction 0x0A) */
tue_sts3215_reset_turns(&g_servo_bus, servo_id);
```

---

## 5. Complete Memory Table & API Reference

All registers listed in [docs/MEMORY_TABLE.md](file:///D:/Project_Kicad/MM_Control/firmware/docs/MEMORY_TABLE.md) have dedicated, typed functions:

### 5.1 EEPROM Area (Read-Only Version & Info)

| Address | Parameter | Size | Function Prototype |
|:---|:---|:---|:---|
| `0x00`–`0x01` | Firmware Major & Minor | 2B | `tue_sts3215_get_fw_version(dev, id, &major, &minor)` |
| `0x02` | END Marker | 1B | `tue_sts3215_get_end_marker(dev, id, &end_val)` |
| `0x03`–`0x04` | Servo Main & Sub Version| 2B | `tue_sts3215_get_servo_version(dev, id, &main_ver, &sub_ver)` |

### 5.2 EEPROM Area (Configuration & Tuning)

*Note: Functions automatically handle unlocking and relocking register `0x37`.*

| Address | Parameter | Size | Set Function | Get Function |
|:---|:---|:---|:---|:---|
| `0x05` | Servo Bus ID | 1B | `tue_sts3215_set_id(dev, old_id, new_id)` | `tue_sts3215_get_id(dev, id, &id)` |
| `0x06` | Baud Rate | 1B | `tue_sts3215_set_baud_rate(dev, id, baud_enum)` | `tue_sts3215_get_baud_rate(dev, id, &baud)` |
| `0x07` | Return Delay (2 µs) | 1B | `tue_sts3215_set_return_delay(dev, id, delay_2us)` | `tue_sts3215_get_return_delay(dev, id, &delay)` |
| `0x08` | Response Status Level | 1B | `tue_sts3215_set_response_level(dev, id, level)` | `tue_sts3215_get_response_level(dev, id, &lvl)` |
| `0x09`–`0x0B` | Travel Angle Limits | 4B | `tue_sts3215_set_angle_limits(dev, id, min, max)` | `tue_sts3215_get_angle_limits(dev, id, &min, &max)` |
| `0x0D` | Max Temperature Limit | 1B | `tue_sts3215_set_max_temperature_limit(dev, id, deg_c)` | `tue_sts3215_get_max_temperature_limit(dev, id, &c)` |
| `0x0E`–`0x0F` | Input Voltage Limits | 2B | `tue_sts3215_set_voltage_limits(dev, id, min_0_1v, max_0_1v)`| `tue_sts3215_get_voltage_limits(dev, id, &min, &max)`|
| `0x10` | Max Output Torque | 2B | `tue_sts3215_set_max_torque(dev, id, torque_0_1pct)` | `tue_sts3215_get_max_torque(dev, id, &torque)` |
| `0x12` | Phase Parameter | 1B | `tue_sts3215_set_phase(dev, id, phase)` | `tue_sts3215_get_phase(dev, id, &phase)` |
| `0x13` | Unload Protection Cond.| 1B | `tue_sts3215_set_unload_conditions(dev, id, mask)` | `tue_sts3215_get_unload_conditions(dev, id, &mask)`|
| `0x14` | LED Alarm Condition | 1B | `tue_sts3215_set_led_alarm_conditions(dev, id, mask)` | `tue_sts3215_get_led_alarm_conditions(dev, id, &mask)`|
| `0x15`–`0x17` | Position Loop PID (P,D,I)| 3B | `tue_sts3215_set_position_pid(dev, id, p, d, i)` | `tue_sts3215_get_position_pid(dev, id, &p, &d, &i)` |
| `0x18` | Min Starting Force | 1B | `tue_sts3215_set_min_starting_force(dev, id, force)` | `tue_sts3215_get_min_starting_force(dev, id, &force)`|
| `0x19` | Points Limit | 1B | `tue_sts3215_set_points_limit(dev, id, limit)` | `tue_sts3215_get_points_limit(dev, id, &limit)` |
| `0x1A`–`0x1B` | CW/CCW Deadband Area | 2B | `tue_sts3215_set_deadband(dev, id, cw, ccw)` | `tue_sts3215_get_deadband(dev, id, &cw, &ccw)` |
| `0x1C` | Protective Current | 2B | `tue_sts3215_set_protective_current(dev, id, curr_6_5ma)` | `tue_sts3215_get_protective_current(dev, id, &curr)` |
| `0x1E` | Angular Resolution | 1B | `tue_sts3215_set_angular_resolution(dev, id, res)` | `tue_sts3215_get_angular_resolution(dev, id, &res)` |
| `0x1F` | Position Correction | 2B | `tue_sts3215_set_position_correction(dev, id, offset_steps)` | `tue_sts3215_get_position_correction(dev, id, &offset)` |
| `0x21` | Operating Mode | 1B | `tue_sts3215_set_operation_mode(dev, id, mode_enum)` | `tue_sts3215_get_operation_mode(dev, id, &mode)` |
| `0x22`–`0x24` | Overload Protection | 3B | `tue_sts3215_set_overload_protection(dev, id, torq, time, ovl)` | `tue_sts3215_get_overload_protection(...)` |
| `0x25`–`0x27` | Speed Loop PID & Time | 3B | `tue_sts3215_set_speed_pid(dev, id, p, i)`<br>`tue_sts3215_set_overcurrent_time(dev, id, time)`| `tue_sts3215_get_speed_pid(...)`<br>`tue_sts3215_get_overcurrent_time(...)` |

### 5.3 SRAM Area (Real-Time Control & Commands)

| Address | Parameter | Size | Functions |
|:---|:---|:---|:---|
| `0x28` | Torque Switch & Calibration | 1B | `tue_sts3215_enable_torque(dev, id, bool)`<br>`tue_sts3215_calibrate_position_to_2048(dev, id)`<br>`tue_sts3215_get_torque_enable(dev, id, &enabled)` |
| `0x29` | Acceleration (100 steps/s²) | 1B | `tue_sts3215_set_acceleration(dev, id, accel)`<br>`tue_sts3215_get_acceleration(dev, id, &accel)` |
| `0x2A` | Target Location (Goal Pos) | 2B | `tue_sts3215_set_goal_position(dev, id, pos)`<br>`tue_sts3215_set_goal_position_ex(dev, id, pos, spd, acc)`<br>`tue_sts3215_reg_write_goal_position_ex(...)` |
| `0x2C` | Runtime / PWM Duty | 2B | `tue_sts3215_set_pwm_runtime(dev, id, signed_pwm)`<br>`tue_sts3215_get_pwm_runtime(dev, id, &signed_pwm)` |
| `0x2E` | Running Speed (steps/s) | 2B | `tue_sts3215_set_running_speed(dev, id, signed_speed)`<br>`tue_sts3215_get_running_speed(dev, id, &signed_speed)` |
| `0x30` | Torque Limit (0.1%) | 2B | `tue_sts3215_set_torque_limit(dev, id, limit)`<br>`tue_sts3215_get_torque_limit(dev, id, &limit)` |
| `0x37` | EEPROM Write Lock | 1B | `tue_sts3215_lock_eeprom(dev, id)`<br>`tue_sts3215_unlock_eeprom(dev, id)`<br>`tue_sts3215_get_eeprom_lock(dev, id, &is_locked)` |

### 5.4 SRAM Area (Real-Time Telemetry Feedback)

| Address | Parameter | Unit | Function Prototype |
|:---|:---|:---|:---|
| `0x38` | Current Location | steps | `tue_sts3215_get_current_position(dev, id, &pos)` |
| `0x3A` | Current Speed | steps/s | `tue_sts3215_get_current_speed(dev, id, &speed)` |
| `0x3C` | Current Output Load | 0.1% | `tue_sts3215_get_current_load(dev, id, &load)` |
| `0x3E` | Current Operating Voltage | 0.1V / V | `tue_sts3215_get_current_voltage(dev, id, &raw_0_1v)`<br>`tue_sts3215_get_voltage_volts(dev, id, &volts_float)` |
| `0x3F` | Current Temperature | °C | `tue_sts3215_get_current_temperature(dev, id, &temp_c)` |
| `0x40` | Async Write (REG) Flag | bool | `tue_sts3215_get_async_write_flag(dev, id, &flag)` |
| `0x41` | Servo Hardware Error Status | bitmask | `tue_sts3215_get_servo_status(dev, id, &mask)` |
| `0x42` | Moving Status Sign | bool | `tue_sts3215_is_moving(dev, id, &is_moving)` |
| `0x45` | Motor Current | 6.5mA / mA | `tue_sts3215_get_motor_current_raw(dev, id, &raw)`<br>`tue_sts3215_get_motor_current_ma(dev, id, &ma_float)` |
| `0x38`–`0x46` | **Complete Burst Feedback**| All | `tue_sts3215_get_feedback(dev, id, &feedback_struct)` |
| any | **Multi-Servo Read** (`SYNC_READ`) | N frames | `tue_sts3215_sync_read(dev, reg, len, ids, count, replies, &reply_count)` |
| — | **Clear Multi-Turn Counter** (`RESET_TURNS`) | — | `tue_sts3215_reset_turns(dev, id)` |

---

## 6. Return Codes & Error Diagnosis

All API functions return `tue_sts3215_status_t`:

| Return Code | Meaning | Action / Diagnosis |
|:---|:---|:---|
| `TUE_STS3215_OK` (0) | Success | Normal execution |
| `TUE_STS3215_ERR_TIMEOUT` (1) | Servo did not respond | Verify power supply, wiring, servo ID, and baud rate |
| `TUE_STS3215_ERR_CHECKSUM` (2) | Frame checksum mismatch | Line noise or baud rate clock drift |
| `TUE_STS3215_ERR_HEADER` (3) | Preamble `0xFF 0xFF` missing | Desynchronization or noise spike |
| `TUE_STS3215_ERR_LEN` (4) | Frame truncated | Insufficient bytes received |
| `TUE_STS3215_ERR_ID` (5) | Replying ID did not match | Multiple servos with duplicated IDs on bus |
| `TUE_STS3215_ERR_STATUS` (6) | Hardware error reported by servo | Inspect `dev->last_servo_status` against status mask |
| `TUE_STS3215_ERR_PARAM` (7) | Parameter out of bounds or NULL pointer | Check input arguments |
| `TUE_STS3215_ERR_HAL` (8) | STM32 HAL / DMA error | Check UART peripheral state |
| `TUE_STS3215_ERR_BUSY` (9) | HAL bus busy with another transfer | A previous transaction is still in flight; retry after it settles |

### Hardware Error Status Bitmask (`0x41`)
When `TUE_STS3215_ERR_STATUS` is returned, check `dev->last_servo_status`:
- `STS3215_STATUS_VOLTAGE_ERR` (`0x01`): Supply voltage out of specified min/max range.
- `STS3215_STATUS_SENSOR_ERR` (`0x02`): Magnetic encoder sensor malfunction.
- `STS3215_STATUS_TEMP_ERR` (`0x04`): Overheat protection threshold reached.
- `STS3215_STATUS_CURRENT_ERR` (`0x08`): Overcurrent threshold exceeded.
- `STS3215_STATUS_ANGLE_ERR` (`0x10`): Commanded angle outside travel limits.
- `STS3215_STATUS_OVERLOAD_ERR` (`0x20`): Overload torque threshold reached.
- `STS3215_STATUS_CHECKSUM_ERR` (`0x40`): Checksum error in received instruction.

---

## 7. Operating Modes Summary

Switch operating modes using `tue_sts3215_set_operation_mode(&g_servo_bus, id, mode)`:

1. **Position Mode (`STS3215_MODE_POSITION = 0`)**:
   Standard servo mode. Absolute positioning within travel angle limits (0 to 4095 steps = 0° to 360°).
2. **Constant Speed Mode (`STS3215_MODE_SPEED = 1`)**:
   Continuous 360° closed-loop speed control. Commanded in steps/second via `tue_sts3215_set_running_speed`.
3. **PWM Open-Loop Mode (`STS3215_MODE_PWM = 2`)**:
   Open-loop voltage drive. Duty cycle -1000 to +1000 via `tue_sts3215_set_pwm_runtime`.
4. **Stepper Servo Mode (`STS3215_MODE_STEPPER = 3`)**:
   Multi-turn continuous stepping with position tracking (-32766 to +32766 steps).

---

## 8. Host Communication (USB Binary Protocol)

The servo bus is exposed over USB CDC through the **binary CAN-over-USB
protocol** — see [`docs/USB_BINARY_PROTOCOL.md`](USB_BINARY_PROTOCOL.md) for the
full spec. The STS3215 command set uses CAN IDs `0x400`–`0x415`:

| CAN ID | Command | Facade call |
|:--|:--|:--|
| `0x400` | `SERVO_PING` | `tue_sts3215_ping` |
| `0x401` | `SERVO_SET_POS` | `tue_sts3215_set_goal_position_ex` |
| `0x402` | `SERVO_GET_POS` | `tue_sts3215_get_current_position` |
| `0x403` | `SERVO_SET_SPEED` | `tue_sts3215_set_acceleration` + `tue_sts3215_set_running_speed` |
| `0x404` | `SERVO_GET_SPEED` | `tue_sts3215_get_current_speed` |
| `0x405` | `SERVO_TORQUE` | `tue_sts3215_enable_torque` / `tue_sts3215_calibrate_position_to_2048` |
| `0x406` | `SERVO_TLIM` | `tue_sts3215_set_torque_limit` |
| `0x407` | `SERVO_MODE` | `tue_sts3215_set_operation_mode` / `_get_` |
| `0x408` | `SERVO_PWM` | `tue_sts3215_set_pwm_runtime` |
| `0x410` | `SERVO_GET_FB` | `tue_sts3215_get_feedback` → **dual 8-byte burst** |
| `0x412` | `SERVO_MEM_READ` | `tue_sts3215_read_raw` |
| `0x413` | `SERVO_MEM_WRITE` | `tue_sts3215_write_raw` |
| `0x414` | `SERVO_SET_ID` | `tue_sts3215_set_id` |
| `0x415` | `SERVO_RESET` | `tue_sts3215_reset` |

### Dual-frame telemetry burst (`SERVO_GET_FB`, `0x410`)

`SERVO_GET_FB` answers with **two 8-byte frames sharing the request `SEQ`**:

* `0x410 SERVO_FB_DYNAMICS` — `id u8, pos i16, spd i16, load i16, moving u8`
* `0x411 SERVO_FB_HEALTH` — `id u8, volt u8 (0.1 V), temp i8 (°C), status u8, cur i16, res u16`

The 15 bytes of actual telemetry (position, speed, load, voltage, temperature,
status, current) arrive across the two frames; the `status` byte is the hardware
error bitmask documented in §6. Host side, `TueUsbClient.servo_get_feedback()`
reassembles both frames into one dictionary.
