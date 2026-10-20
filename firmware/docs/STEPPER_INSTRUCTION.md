# TUE Stepper — Instruction Manual

> **Platform**: STM32F407VET6 (`MM_Control`)
> **Firmware**: [`Core/Inc/tue_stepper.h`](../Core/Inc/tue_stepper.h), [`Core/Src/tue_stepper.c`](../Core/Src/tue_stepper.c)
> **Timing source**: TIM1 update interrupt (1 µs tick)
> **Related**: [`docs/USB_BINARY_PROTOCOL.md`](USB_BINARY_PROTOCOL.md), [`docs/STS3215_INSTRUCTION.md`](STS3215_INSTRUCTION.md)

---

## 1. Overview

`tue_stepper` drives **three step/direction motor drivers**:

| Motor | Step | Direction | Role | Axis |
|:--|:--|:--|:--|:--|
| `TUE_STEPPER_M1` (A) | `PE9` | `PC3` (`STEP_DIR1`) | CoreXY belt pair | `A = x + y` |
| `TUE_STEPPER_M2` (B) | `PE11` | `PC2` (`STEP_DIR2`) | CoreXY belt pair | `B = x − y` |
| `TUE_STEPPER_M3` (C) | `PE13` | `PC1` (`STEP_DIR3`) | Z (lead screw) | `C = z` |
| common enable | — | `PC5` (`STEP_ENABLE`) | driver `EN`, active low | — |
| spare | `PE14` | `PC0` (`STEP_DIR4`) | unused 4th channel | — |

Everything is expressed in **millimetres** for X/Y/Z, or in **raw steps** per motor
when you need to bypass the kinematics.

Features:

* **Position control** — absolute or relative moves to an exact point.
* **Velocity control** — continuous jogs with a controlled acceleration ramp.
* **Acceleration control** — every move follows a trapezoidal
  (accelerate → cruise → decelerate) profile.
* **CoreXY kinematics** — `move_to(x, y, z)` is resolved into the two belt
  motors automatically, and the tool position is tracked back from the counters.
* **Motion queue** — up to 16 segments are buffered, so consecutive moves run
  back-to-back without stopping at each waypoint.
* **No CPU hogging** — pulses are generated in the TIM1 interrupt; application
  code only plans moves.

---

## 2. Wiring assumptions

* Motor 1 and motor 2 drive the **CoreXY** belt pair; motor 3 is the **Z** axis.
* All three drivers share the single `STEP_ENABLE` line on `PC5` (active low —
  set `TUE_STEPPER_EN_ACTIVE_LOW` to `0` for active-high drivers).
* Drivers are configured for **64 microsteps**.
* The library **owns TIM1** and the three STEP pins.

> **Important** — in CubeMX, `PE9 / PE11 / PE13 (/PE14)` are assigned to
> `TIM1_CH1..CH4` as PWM alternate functions. `tue_stepper_init()` re-configures
> those pins as **plain push-pull outputs** and re-initialises TIM1 as a 1 µs
> free-running time base. Do **not** also start `HAL_TIM_PWM_Start()` on TIM1.

---

## 3. Units, geometry and conversion

The GT2 belt, pulley and microstep setting are converted into steps/mm at
compile time (or at runtime through the configuration struct).

| Quantity | Symbol | Value |
|:--|:--|:--|
| GT2 belt pitch | — | `2 mm` |
| Motor full steps / turn | — | `200` (1.8°) |
| Microsteps | — | `64` |
| Steps per motor revolution | `TUE_STEPPER_STEPS_PER_REV` | `200 × 64 = 12800` |
| X/Y pulley teeth | `TUE_STEPPER_XY_TEETH` | `20` |
| X/Y travel per motor turn | `TUE_STEPPER_XY_MM_PER_REV` | `2 × 20 = 40 mm` |
| **X/Y resolution** | `TUE_STEPPER_XY_STEPS_PER_MM` | `12800 / 40 = ` **`320 steps/mm`** |
| X/Y step size | `TUE_STEPPER_XY_MM_PER_STEP` | `0.003125 mm` (3.125 µm) |
| Z lead per turn | `TUE_STEPPER_Z_LEAD_MM` | `8 mm` |
| **Z resolution** | `TUE_STEPPER_Z_STEPS_PER_MM` | `12800 / 8 = ` **`1600 steps/mm`** |
| Z step size | `TUE_STEPPER_Z_MM_PER_STEP` | `0.000625 mm` (0.625 µm) |

Helpers:

```c
int32_t tue_stepper_mm_to_steps_xy(float mm);   /*  10.0f -> 3200  */
int32_t tue_stepper_mm_to_steps_z(float mm);    /*  10.0f -> 16000 */
```

Different hardware? Either edit the macros in `Core/Inc/tue_stepper.h`, or
override at runtime **before** the first move:

```c
tue_stepper_config_t cfg = tue_stepper_default_config();
cfg.steps_per_mm[TUE_STEPPER_M3] = 400.0f;   /* 2 mm Z lead, for example */
cfg.invert[TUE_STEPPER_M1]       = true;     /* belt wired the other way  */
tue_stepper_set_config(&cfg);
```

---

## 4. How the stepping engine works

```
   application                TIM1 update ISR (1 µs tick)
  ┌────────────┐   push    ┌──────────────────────────────────────┐
  │ move_to /  │ ────────► │ load segment                         │
  │ jog / stop │  16-deep  │  ├─ Bresenham: which motors pulse?    │
  └────────────┘   ring    │  ├─ drive STEP high → U3 wait → low  │
                           │  └─ trapezoid: update TIM1 reload     │
                           └──────────────────────────────────────┘
```

1. **Master step.** Each TIM1 update interrupt emits one *master step*. The
   reload value is set to `1 µs × interval`, so the step rate equals
   `1 / interval`.
2. **Bresenham distribution.** A per-motor accumulator adds that motor's step
   count every master step and fires it when it reaches the largest axis count.
   All axes therefore finish on the same master step, i.e. in a straight line.
3. **Trapezoid ramp.** Before scheduling the next interval, the ISR updates the
   squared velocity: `v² += 2·a` while accelerating, `v² -= 2·a` while braking.
   The switch to braking happens when the remaining steps are less than the
   braking distance `v² / (2a)`. If the ramp never reaches the requested feed
   (short move), the profile degenerates to a triangle automatically.
4. **Queue.** Segments are pushed by the application and consumed by the ISR;
   the driver keeps stepping through the queue without pausing at segment
   boundaries.
5. **Position.** `x`/`y`/`z` are maintained by counting the pulses that were
   actually emitted, so `tue_stepper_get_position_mm()` always reflects the
   commanded step count (there is no encoder feedback).

### Planning in mm → steps

For a move of `L` mm along the tool path, the dominant motor needs `N` steps:

```
scale   = N / L                        [steps per mm along the path]
v_steps = feed_mm_s  × scale           [master steps/s]
a_steps = accel_mm_s² × scale          [master steps/s²]
```

so the requested **feed and acceleration always apply to the real tool path**,
in mm, whichever axis happens to do most of the work.

---

## 5. CoreXY kinematics

```
forward (motors → tool)          inverse (tool → motors)
    A = x + y                        x = (A + B) / 2
    B = x − y                        y = (A − B) / 2
    C = z                            z = C

A = M1 steps / 320   B = M2 steps / 320   C = M3 steps / 1600
```

Consequences worth remembering:

* A pure **X** move turns both belt motors by the same amount in the same
  direction (`A = B = 320 · x`).
* A pure **Y** move turns both belt motors in *opposite* directions.
* A move along the **diagonal `y = x`** turns only motor 1
  (`A = 2·320·x`, `B = 0`) — useful for a quick sanity check.
* `steps_per_mm` is assumed identical for M1 and M2. If you set them
  differently, the forward transform becomes approximate.

---

## 6. API reference

All functions live in `tue_stepper.h`. Status codes:

| Code | Meaning |
|:--|:--|
| `TUE_STEPPER_OK` | accepted / finished |
| `TUE_STEPPER_ERR_PARAM` | bad argument (zero feed/accel, unknown motor, …) |
| `TUE_STEPPER_ERR_QUEUE_FULL` | the 16-segment queue is saturated |
| `TUE_STEPPER_BUSY` | machine is already executing a different kind of motion |
| `TUE_STEPPER_ERR_TIMEOUT` | `wait_idle()` gave up |

### 6.1 Lifecycle

| Function | Notes |
|:--|:--|
| `void tue_stepper_init(void)` | Call once after `MX_GPIO_Init()` / `MX_TIM1_Init()`. Re-purposes the STEP pins, configures TIM1, releases the drivers. |
| `tue_stepper_config_t tue_stepper_default_config(void)` | GT2 / 64 microstep / 20T / 8 mm defaults. |
| `void tue_stepper_set_config(const tue_stepper_config_t *cfg)` | Change geometry or `invert[]` at runtime. |
| `void tue_stepper_get_config(tue_stepper_config_t *cfg)` | Read the active configuration. |
| `void tue_stepper_enable(bool enable)` | Energise / release the shared enable line. |
| `bool tue_stepper_is_enabled(void)` | Current enable state. |

### 6.2 Position

| Function | Notes |
|:--|:--|
| `void tue_stepper_set_position_mm(float x, float y, float z)` | Overwrite the logical position (use after homing). Call only while idle. |
| `void tue_stepper_get_position_mm(float *x, float *y, float *z)` | Current tool position; any pointer may be `NULL`. |
| `void tue_stepper_get_target_mm(float *x, float *y, float *z)` | End point of the motion currently queued/executing. |
| `int32_t tue_stepper_get_motor_steps(tue_stepper_motor_t motor)` | Raw signed step counter of one motor. |

### 6.3 Motion

| Function | Notes |
|:--|:--|
| `tue_stepper_move_to_xyz_mm(x, y, z, feed_mm_s, accel_mm_s2)` | Absolute 3-axis move to an exact point. |
| `tue_stepper_move_to_mm(x, y, feed_mm_s, accel_mm_s2)` | Absolute XY move in the current Z plane. |
| `tue_stepper_move_z_mm(z, feed_mm_s, accel_mm_s2)` | Absolute Z move, XY untouched. |
| `tue_stepper_move_by_mm(dx, dy, feed_mm_s, accel_mm_s2)` | Relative XY move. |
| `tue_stepper_move_motor_steps(motor, steps, feed_mm_s, accel_mm_s2)` | One raw motor, bypassing the kinematics. |

`feed_mm_s` and `accel_mm_s2` must both be `> 0`; they describe the **tool
path**, so a diagonal move keeps the same mm/s as an axis-aligned one.
The first successful move also energises the drivers.

### 6.4 Velocity (jog)

| Function | Notes |
|:--|:--|
| `tue_stepper_start_velocity(vx_mm_s, vy_mm_s, accel_mm_s2)` | Continuous CoreXY jog; ramps up to the requested tool velocity. Calling it again while the jog runs **only changes the speed** (the direction is kept). Use `stop()`/`abort()` before reversing. |
| `tue_stepper_start_velocity_z(vz_mm_s, accel_mm_s2)` | Continuous Z jog; the sign selects the direction. |

### 6.5 Stopping & status

| Function | Notes |
|:--|:--|
| `void tue_stepper_stop(void)` | **Controlled stop.** Recomputes the braking distance, shortens the current move accordingly and discards everything queued behind it. A jog ramps down to zero. |
| `void tue_stepper_abort(void)` | **Immediate stop.** Kills the step clock, empties the queue, clears pulses. The position counter is preserved. |
| `bool tue_stepper_is_busy(void)` | `true` while a segment is executing. |
| `tue_stepper_wait_idle(timeout_ms)` | Block until idle (`0` = wait forever). |

### 6.6 ISR hook

```c
void tue_stepper_on_timer_elapsed(TIM_HandleTypeDef *htim);
```

Already wired into `HAL_TIM_PeriodElapsedCallback()` in `Core/Src/main.c`.
It returns immediately for any timer other than TIM1, so it is safe to call
unconditionally.

---

## 7. Getting started

### 7.1 Integration (already done in this project)

```c
/* Core/Src/main.c */
#include "tue_stepper.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();          /* CubeMX time base for TIM1 */
    /* ... other peripherals ... */

    tue_stepper_init();      /* takes over TIM1 + PE9/PE11/PE13 */

    osKernelInitialize();
    MX_FREERTOS_Init();
    osKernelStart();
}

/* one master step per TIM1 update interrupt */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14) { HAL_IncTick(); }
    tue_stepper_on_timer_elapsed(htim);
}
```

### 7.2 Moving to an exact CoreXY point

```c
tue_stepper_enable(true);

/* go to x=50 mm, y=30 mm, z=10 mm at 60 mm/s with 600 mm/s^2 */
tue_stepper_move_to_xyz_mm(50.0f, 30.0f, 10.0f, 60.0f, 600.0f);
tue_stepper_wait_idle(5000);            /* 0 = wait forever */

float x, y, z;
tue_stepper_get_position_mm(&x, &y, &z);   /* -> 50.000, 30.000, 10.000 */
```

### 7.3 Chained moves (no stop at the waypoints)

```c
tue_stepper_move_to_mm(20.0f,  0.0f, 60.0f, 600.0f);
tue_stepper_move_to_mm(20.0f, 20.0f, 60.0f, 600.0f);   /* queued   */
tue_stepper_move_to_mm( 0.0f, 20.0f, 60.0f, 600.0f);   /* queued   */
tue_stepper_wait_idle(0);                              /* square    */
```

### 7.4 Jogging from a UI / pendant

```c
tue_stepper_start_velocity(25.0f, 0.0f, 500.0f);   /* +X at 25 mm/s   */
/* later, while it is still running */
tue_stepper_start_velocity(8.0f, 0.0f, 500.0f);    /* slow down        */
/* release the button */
tue_stepper_stop();                                /* ramps down       */
```

### 7.5 Homing without endstops

This library has no endstop/limit-switch logic (the `DC_F1..F4` inputs on
`PD8..PD11` are free if you want to add it). The usual manual procedure is:

```c
tue_stepper_start_velocity_z(-5.0f, 500.0f);   /* creep down        */
/* ... wait for the operator / a switch ...  */
tue_stepper_abort();
tue_stepper_set_position_mm(0.0f, 0.0f, 0.0f); /* this is now zero  */
```

---

## 8. Tuning and limits

| Constant | Default | Meaning |
|:--|:--|:--|
| `TUE_STEPPER_TIMER_HZ` | `1000000` | TIM1 tick (1 µs) |
| `TUE_STEPPER_MIN_INTERVAL_US` | `4` | shortest step period → hard ceiling of 250 k steps/s |
| `TUE_STEPPER_PULSE_US` | `3` | STEP pulse width |
| `TUE_STEPPER_MAX_RATE_STEPS_S` | `50000` | soft speed cap → **156 mm/s on XY, 31 mm/s on Z** |
| `TUE_STEPPER_MIN_RATE_STEPS_S` | `400` | start/end speed of the ramp → **1.25 mm/s on XY, 0.25 mm/s on Z** |
| `TUE_STEPPER_QUEUE_SIZE` | `16` | buffered motion segments |

Rules of thumb:

* **Feed** is limited by the mechanics: if the motor buzzes or stalls, lower
  `feed_mm_s` or raise `accel_mm_s2` is *not* the answer — reduce the feed first.
* **Acceleration** for a bare stepper on a light CoreXY gantry is typically
  300–1000 mm/s²; a screw-driven Z usually wants 100–500 mm/s².
* The braking distance is `v² / (2a)`; at 60 mm/s and 600 mm/s² that is 3 mm,
  so moves shorter than roughly 6 mm never reach the requested feed (they are
  triangular). That is normal and safe.
* Raising `TUE_STEPPER_MAX_RATE_STEPS_S` is only useful together with lowering
  `TUE_STEPPER_MIN_INTERVAL_US`; remember the ISR runs the whole ramp, the
  Bresenham update and the 3 µs pulse wait per step, so leave headroom.

---

## 9. Caveats

* **Open loop.** Positions are counted pulses, not measured ones. A skipped
  step is invisible; add endstops or an encoder if that matters.
* **No homing / limits / bed levelling.** See §7.5.
* **One motion model at a time.** A position move is rejected with
  `TUE_STEPPER_BUSY` while a jog is running, and a jog speed update is ignored
  if a position move is in progress. `stop()` before switching modes.
* **`s_head`/`s_tail` integrity.** `stop()` deliberately discards queued
  segments; `abort()` discards everything.
* **TIM1 is shared.** Nothing else in the firmware may use TIM1 once
  `tue_stepper_init()` has run.
* **M1/M2 must have the same `steps_per_mm`** for the forward transform to be
  exact.

---

## 10. Verification status

Checked on the host by compiling `Core/Src/tue_stepper.c` against a stubbed HAL
and a virtual 1 µs timer, then stepping the ISR:

| Check | Result |
|:--|:--|
| Geometry | 320 / 1600 steps-per-mm, 3.125 µm / 0.625 µm per step |
| Inverse kinematics | `(10, 5, 2) mm` → `M1 = 4800`, `M2 = 1600`, `M3 = 3200` |
| Forward kinematics | counts read back as `(10.000, 5.000, 2.000) mm` |
| Diagonal `y = x` | only motor 1 turns |
| Pure X | both belt motors, same direction |
| Direction signs | negative motor counted, `DIR1`/`DIR2` polarity correct |
| `invert[]` | swaps the DIR level, counter unaffected |
| Trapezoid | commanded 50 mm/s realised as 21 739 steps/s (peak measured), total time within 1 % of the analytic profile |
| All axes together | 25 600 / 6 400 / 16 000 steps, exact |
| Velocity jog | 30 mm/s for 1 s → 9 362 steps (analytic 9 312) |
| Controlled stop | decelerates and stops short of the target instead of overshooting |
| `stop()` / `abort()` | queued segments dropped, counters preserved |
| Queue saturation | reports `TUE_STEPPER_ERR_QUEUE_FULL` |
| Short moves | 1 mm at 50 mm/s → 84 ms; 10-step Z move → 20 ms |

Firmware build: clean (`-Wall -Wextra`), links with FLASH 77 216 B / RAM 30 920 B.

**Not verified:** anything electrical — actual pulse timing on the pins, driver
behaviour, motor torque and real motion. Those need the board.

---

## 11. Host communication (USB binary protocol)

The board exposes the stepper library over USB CDC using the **binary
CAN-over-USB protocol**; see [`docs/USB_BINARY_PROTOCOL.md`](USB_BINARY_PROTOCOL.md)
for the full wire specification. `tue_stepper` is reached through the tool-motion
CAN IDs `0x200–0x21A` and the raw-motor IDs `0x280–0x281`:

| CAN ID | Command | Units on the wire |
|:--|:--|:--|
| `0x200` | `STEP_GET_INFO` | steps/mm (`u16`), steps/s (`u16`) |
| `0x201` | `STEP_GET_STATUS` | busy/enabled/queue depth (`u8`) |
| `0x202` | `STEP_ENABLE` | `u8` (`0` off, `1` on, `0xFF` query) |
| `0x203` / `0x204` | `STEP_GET_POS_XY` / `_Z` | signed `i32` **micrometres** to/from `tue_stepper_get_position_mm` |
| `0x205` / `0x206` | `STEP_GET_TGT_XY` / `_Z` | signed `i32` um (`tue_stepper_get_target_mm`) |
| `0x207` / `0x208` | `STEP_SETPOS_XY` / `_Z` | signed `i32` um (`tue_stepper_set_position_mm`) |
| `0x210` + `0x211` | `STEP_MOVE_COORDS` + `STEP_MOVE_EXEC` | `x,y` um then `z` um, feed `u16` mm/s, accel `u16` mm/s² → `tue_stepper_move_to_xyz_mm` |
| `0x212` | `STEP_MOVE_XY` | signed `i32` um, default speed → `tue_stepper_move_to_mm` |
| `0x213` | `STEP_MOVE_Z` | signed `i32` um, feed/accel `u16` → `tue_stepper_move_z_mm` |
| `0x214` | `STEP_MOVE_REL_XY` | signed `i32` um → `tue_stepper_move_by_mm` |
| `0x215` | `STEP_SET_SPEED` | feed `u16` mm/s, accel `u16` mm/s² |
| `0x216` / `0x217` | `STEP_JOG_XY` / `_Z` | `i16` mm/s velocities, `u16` mm/s² accel → `tue_stepper_start_velocity(_z)` |
| `0x218` / `0x219` | `STEP_STOP` / `STEP_ABORT` | — (`tue_stepper_stop` / `_abort`) |
| `0x21A` | `STEP_WAIT_IDLE` | `u32` timeout ms → `tue_stepper_wait_idle` |
| `0x280` / `0x281` | `STEP_RAW_GET_STEPS` / `STEP_RAW_MOVE` | motor id `1..3`; `i32` steps, `u16` mm/s |

Units: distances travel as **signed 32-bit micrometres** (`1 mm = 1000 um`),
feed/accel as **`u16` mm/s and mm/s²**, jog velocities as **`i16` mm/s**. The host
library works in millimetres and converts at the boundary. Moves are queued and
acknowledged immediately — poll `STEP_GET_STATUS` or call `STEP_WAIT_IDLE` before
assuming arrival.
