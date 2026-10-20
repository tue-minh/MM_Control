/**
  ******************************************************************************
  * @file    tue_stepper.h
  * @brief   Public API of the 3-axis stepper library (CoreXY + Z).
  ******************************************************************************
  * @attention
  *
  *   TUE = "The Universal Engine" project libraries.
  *
  * Machine layout assumed by this library
  * -------------------------------------
  *   Motor 1 (A / PE9  / DIR PC3) ---+
  *                                   +-- CoreXY belt pair -> tool X / Y
  *   Motor 2 (B / PE11 / DIR PC2) ---+
  *   Motor 3 (C / PE13 / DIR PC1) ------- Z axis (screw / lead screw)
  *
  *   Common driver enable: PC5 (STEP_ENABLE).
  *
  *   CoreXY forward / inverse:
  *       A = x + y          x = (A + B) / 2
  *       B = x - y          y = (A - B) / 2
  *
  * Belt geometry (GT2, 64 microsteps)
  * ----------------------------------
  *   GT2 belt pitch ............... 2 mm
  *   XY pulley teeth .............. 20        -> 40 mm travel per motor turn
  *   Motor full steps ............. 200       -> 64 microsteps -> 12800 steps/turn
  *   XY resolution ................ 12800 / 40  = 320 steps/mm  (3.125 um)
  *   Z lead (single start) ........ 8 mm      -> 12800 / 8 = 1600 steps/mm (0.625 um)
  *
  *   Every motor is assumed to run the same 64 microstep setting.
  *   Change TUE_STEPPER_XY_TEETH / TUE_STEPPER_Z_LEAD_MM (or the runtime
  *   configuration) if your hardware differs.
  *
  * Stepping engine
  * ---------------
  *   TIM1 is used as a 1 us periodic interrupt. Each interrupt emits one
  *   "master step"; a Bresenham accumulator distributes pulses over the three
  *   motors so that all axes finish on the same master step. The reload value
  *   is recomputed every step from a trapezoidal velocity ramp
  *   (acceleration -> cruise -> deceleration).
  *
  *   !!! The STEP pins (PE9 / PE11 / PE13) are re-configured by
  *       tue_stepper_init() from TIM1 PWM alternate function to plain GPIO
  *       outputs. Do not also start HAL TIM1 PWM output on those channels.
  *
  * See docs/STEPPER_INSTRUCTION.md for the full manual.
  *
  ******************************************************************************
  */

#ifndef __TUE_STEPPER_H__
#define __TUE_STEPPER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "tim.h"

/* Exported constants --------------------------------------------------------*/

/* --- Mechanics ------------------------------------------------------------ */
#define TUE_STEPPER_GT2_PITCH_MM        2.0f    /* GT2 belt pitch              */
#define TUE_STEPPER_MICROSTEPS          16U     /* microstep setting per motor */
#define TUE_STEPPER_MOTOR_FULL_STEPS    200U    /* 1.8 deg motor               */
#define TUE_STEPPER_XY_TEETH            20U     /* GT2 pulley teeth (X/Y)      */
#define TUE_STEPPER_Z_LEAD_MM           8.0f    /* Z lead screw lead per turn  */

#define TUE_STEPPER_STEPS_PER_REV       ((float)(TUE_STEPPER_MOTOR_FULL_STEPS * TUE_STEPPER_MICROSTEPS))
#define TUE_STEPPER_XY_MM_PER_REV       (TUE_STEPPER_GT2_PITCH_MM * (float)TUE_STEPPER_XY_TEETH)

/* 64 * 200 / (2 * 20) = 320 steps per mm  (0.003125 mm per step) */
#define TUE_STEPPER_XY_STEPS_PER_MM     (TUE_STEPPER_STEPS_PER_REV / TUE_STEPPER_XY_MM_PER_REV)
#define TUE_STEPPER_XY_MM_PER_STEP      (TUE_STEPPER_XY_MM_PER_REV / TUE_STEPPER_STEPS_PER_REV)

/* 64 * 200 / 8 = 1600 steps per mm  (0.000625 mm per step) */
#define TUE_STEPPER_Z_STEPS_PER_MM      (TUE_STEPPER_STEPS_PER_REV / TUE_STEPPER_Z_LEAD_MM)
#define TUE_STEPPER_Z_MM_PER_STEP       (TUE_STEPPER_Z_LEAD_MM / TUE_STEPPER_STEPS_PER_REV)

/* --- Timing / limits ------------------------------------------------------ */
#define TUE_STEPPER_TIMER_HZ            1000000U  /* TIM1 tick = 1 us          */
#define TUE_STEPPER_MIN_INTERVAL_US     4U        /* shortest step period      */
#define TUE_STEPPER_PULSE_US            3U        /* STEP pulse width          */
#define TUE_STEPPER_MAX_RATE_STEPS_S    50000.0f  /* speed cap, master steps/s */
#define TUE_STEPPER_MIN_RATE_STEPS_S    400.0f    /* starting / ramp-end speed */
#define TUE_STEPPER_QUEUE_SIZE          16U       /* motion segments buffered  */
#define TUE_STEPPER_MOTOR_COUNT         3U

/* Set to 1 for the usual active-low driver enable (EN pin). */
#define TUE_STEPPER_EN_ACTIVE_LOW       1

/* Exported types ------------------------------------------------------------*/

/** Motor identifiers. M1/M2 form the CoreXY pair, M3 is Z. */
typedef enum {
    TUE_STEPPER_M1 = 0,   /* X+Y motor - PE9  step / PC3 dir */
    TUE_STEPPER_M2 = 1,   /* X-Y motor - PE11 step / PC2 dir */
    TUE_STEPPER_M3 = 2    /* Z motor   - PE13 step / PC1 dir */
} tue_stepper_motor_t;

/** Return codes. */
typedef enum {
    TUE_STEPPER_OK = 0,
    TUE_STEPPER_ERR_PARAM,
    TUE_STEPPER_ERR_QUEUE_FULL,
    TUE_STEPPER_BUSY,
    TUE_STEPPER_ERR_TIMEOUT
} tue_stepper_status_t;

/** Runtime geometry / wiring configuration. */
typedef struct {
    float steps_per_mm[TUE_STEPPER_MOTOR_COUNT]; /* per-motor resolution       */
    bool  invert[TUE_STEPPER_MOTOR_COUNT];       /* true = swap DIR polarity   */
} tue_stepper_config_t;

/* Exported functions --------------------------------------------------------*/

/* --- Lifecycle ------------------------------------------------------------ */

/** Default configuration (GT2, 64 microsteps, 20T pulleys, 8 mm Z lead). */
tue_stepper_config_t tue_stepper_default_config(void);

/** Replace the geometry / wiring configuration (does not restart motion). */
void tue_stepper_set_config(const tue_stepper_config_t *cfg);

/** Read the active configuration. */
void tue_stepper_get_config(tue_stepper_config_t *cfg);

/**
 * @brief  Initialise the library: re-purpose the STEP pins to GPIO,
 *         configure TIM1 as the 1 us step clock and release the drivers.
 * @note   Call once after MX_GPIO_Init() / MX_TIM1_Init().
 */
void tue_stepper_init(void);

/** Energise (true) or release (false) the driver enable line. */
void tue_stepper_enable(bool enable);

/** @return true when the drivers are energised. */
bool tue_stepper_is_enabled(void);

/** Number of motion segments currently pending in the planner queue. */
uint8_t tue_stepper_queue_depth(void);

/* --- Position ------------------------------------------------------------- */

/** Overwrite the logical position of the tool. Call only while idle. */
void tue_stepper_set_position_mm(float x, float y, float z);

/** Read the current (step-counted) tool position in mm. Any pointer may be NULL. */
void tue_stepper_get_position_mm(float *x, float *y, float *z);

/** Read the endpoint of the currently commanded motion in mm. NULL allowed. */
void tue_stepper_get_target_mm(float *x, float *y, float *z);

/** Step counter of one motor (logical, sign corrected). */
int32_t tue_stepper_get_motor_steps(tue_stepper_motor_t motor);

/** Millimetres -> steps for the X/Y (CoreXY) motors. */
int32_t tue_stepper_mm_to_steps_xy(float mm);

/** Millimetres -> steps for the Z motor. */
int32_t tue_stepper_mm_to_steps_z(float mm);

/* --- Motion --------------------------------------------------------------- */

/**
 * @brief  Move the tool to an absolute (x, y, z) point in mm.
 * @param  feed_mm_s   Cruising speed along the path (mm/s, > 0).
 * @param  accel_mm_s2 Path acceleration (mm/s^2, > 0).
 */
tue_stepper_status_t tue_stepper_move_to_xyz_mm(float x, float y, float z,
                                                float feed_mm_s, float accel_mm_s2);

/**
 * @brief  Move the tool to (x, y) in the current Z plane.
 */
tue_stepper_status_t tue_stepper_move_to_mm(float x, float y,
                                            float feed_mm_s, float accel_mm_s2);

/** Move Z to an absolute height, X/Y untouched. */
tue_stepper_status_t tue_stepper_move_z_mm(float z, float feed_mm_s, float accel_mm_s2);

/** Relative CoreXY move by (dx, dy) mm, Z untouched. */
tue_stepper_status_t tue_stepper_move_by_mm(float dx, float dy,
                                            float feed_mm_s, float accel_mm_s2);

/**
 * @brief  Move a single motor by a number of steps (bypasses CoreXY maths).
 * @note   feed_mm_s is interpreted along that motor's own axis.
 */
tue_stepper_status_t tue_stepper_move_motor_steps(tue_stepper_motor_t motor,
                                                  int32_t steps,
                                                  float feed_mm_s, float accel_mm_s2);

/* --- Velocity / jog ------------------------------------------------------- */

/**
 * @brief  Start a continuous CoreXY jog at tool velocity (vx, vy) mm/s.
 * @param  accel_mm_s2 Ramp-up acceleration (> 0).
 * @note   While a jog is running, calling this again only updates the speed
 *         (the direction stays as originally commanded). Use stop()/abort()
 *         to change direction or before starting a position move.
 */
tue_stepper_status_t tue_stepper_start_velocity(float vx_mm_s, float vy_mm_s,
                                                float accel_mm_s2);

/** Continuous Z jog at vz mm/s (sign selects the direction). */
tue_stepper_status_t tue_stepper_start_velocity_z(float vz_mm_s, float accel_mm_s2);

/* --- Stopping ------------------------------------------------------------- */

/** Controlled stop: decelerate the current move and drop queued segments. */
void tue_stepper_stop(void);

/** Immediate stop: kill the step clock and empty the queue. */
void tue_stepper_abort(void);

/* --- Status --------------------------------------------------------------- */

/** true while a motion segment is executing. */
bool tue_stepper_is_busy(void);

/**
 * @brief  Block until motion finishes.
 * @param  timeout_ms  0 = wait forever.
 */
tue_stepper_status_t tue_stepper_wait_idle(uint32_t timeout_ms);

typedef void (*tue_stepper_idle_cb_t)(void);

/**
 * @brief  Register callback invoked when motion stops (called in ISR or task context).
 */
void tue_stepper_set_idle_callback(tue_stepper_idle_cb_t cb);

/* --- ISR hook ------------------------------------------------------------- */

/**
 * @brief  Call from HAL_TIM_PeriodElapsedCallback().
 * @note   Executes the step generation in interrupt context.
 */
void tue_stepper_on_timer_elapsed(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __TUE_STEPPER_H__ */
