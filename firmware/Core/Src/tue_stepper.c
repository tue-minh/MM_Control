/**
  ******************************************************************************
  * @file           : tue_stepper.c
  * @brief          : 3-axis stepper driver (CoreXY + Z) for STM32F407.
  ******************************************************************************
  * @attention
  *
  * Stepping model
  * --------------
  * TIM1 is used as a periodic interrupt source (1 us tick). One interrupt
  * produces one "master step"; a Bresenham accumulator decides which of the
  * three motors actually receives a pulse on that master step, so all axes
  * finish together. The reload value is updated every step from a trapezoidal
  * velocity ramp, giving position / velocity / acceleration control.
  *
  * Directions are decided per motion segment and written once to the DIR pins.
  * Positions may be given in millimetres; the GT2/64-microstep geometry is
  * converted to steps (see tue_stepper.h).
  *
  * CoreXY kinematics (motor 1 = A, motor 2 = B):
  *     A = x + y
  *     B = x - y
  *     x = (A + B) / 2
  *     y = (A - B) / 2
  *
  * See docs/STEPPER_INSTRUCTION.md.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tue_stepper.h"

#include <math.h>
#include <string.h>
#include "cmsis_os2.h"

/* Private defines -----------------------------------------------------------*/
#define TUE_STEPPER_USE_DWT 1

/* Private types -------------------------------------------------------------*/

/* One queued motion segment. */
typedef struct {
    int32_t  num[TUE_STEPPER_MOTOR_COUNT];  /* Bresenham numerator |steps| or |rate| */
    int32_t  masters;                       /* Bresenham denominator (>= 1)          */
    int8_t   sign[TUE_STEPPER_MOTOR_COUNT]; /* logical direction (+1 / -1)           */
    float    v_peak;                        /* cruise speed, master steps/s          */
    float    accel;                         /* acceleration, master steps/s^2        */
    uint32_t total;                         /* master steps (0 = continuous jog)     */
} tue_stepper_segment_t;

/* Private variables ---------------------------------------------------------*/
static tue_stepper_config_t s_cfg;
static bool                 s_initialized = false;
static bool                 s_enabled = false;

/* Motion queue: producer = task, consumer = TIM1 ISR. */
static tue_stepper_segment_t s_queue[TUE_STEPPER_QUEUE_SIZE];
static volatile uint8_t      s_head = 0U;
static volatile uint8_t      s_tail = 0U;
static volatile bool         s_running = false;
static osSemaphoreId_t       s_idle_sem = NULL;
static tue_stepper_idle_cb_t s_idle_cb  = NULL;

/* Active segment state (ISR). */
static int32_t  s_num[TUE_STEPPER_MOTOR_COUNT];
static int32_t  s_acc[TUE_STEPPER_MOTOR_COUNT];
static int8_t   s_sign[TUE_STEPPER_MOTOR_COUNT];
static int32_t  s_masters = 0;
static float    s_v_peak = 0.0f;
static float    s_v_sq = 0.0f;
static float    s_v_min_sq = 0.0f;
static float    s_accel = 0.0f;
static uint32_t s_total = 0U;
static uint32_t s_idx = 0U;
static bool     s_continuous = false;
static bool     s_stopping = false;

/* Position feedback (ISR owned). */
static volatile int32_t s_pos_steps[TUE_STEPPER_MOTOR_COUNT];

/* Commanded endpoint (task owned, steps). */
static int32_t s_cmd_steps[TUE_STEPPER_MOTOR_COUNT];

/* Pin tables ----------------------------------------------------------------*/
static GPIO_TypeDef *const s_step_port[TUE_STEPPER_MOTOR_COUNT] = { GPIOE, GPIOE, GPIOE };
static const uint16_t      s_step_pin[TUE_STEPPER_MOTOR_COUNT]  = { STEP_1_Pin, STEP_2_Pin, STEP_3_Pin };

static GPIO_TypeDef *const s_dir_port[TUE_STEPPER_MOTOR_COUNT]  = { GPIOC, GPIOC, GPIOC };
static const uint16_t      s_dir_pin[TUE_STEPPER_MOTOR_COUNT]   = { STEP_DIR1_Pin, STEP_DIR2_Pin, STEP_DIR3_Pin };

/* Private helpers -----------------------------------------------------------*/
static int32_t stepper_abs32(int32_t v);
static void    stepper_delay_us(uint32_t us);
static void    stepper_write_dir(uint8_t motor, bool positive);
static void    stepper_apply_dirs(void);
static void    stepper_set_interval(void);
static void    stepper_load_segment(uint8_t index);
static void    stepper_advance(void);
static void    stepper_isr(void);
static void    stepper_emit(void);

static bool    stepper_queue_space(void);
static tue_stepper_status_t stepper_plan_enqueue(const int32_t dsteps[TUE_STEPPER_MOTOR_COUNT],
                                                 float feed_mm_s, float accel_mm_s2,
                                                 float path_mm);
static void    stepper_resync_if_idle(void);
static void    stepper_cmd_mm_from_steps(float *x, float *y, float *z);

/* Small helpers -------------------------------------------------------------*/

static int32_t stepper_abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static void stepper_delay_us(uint32_t us)
{
#if TUE_STEPPER_USE_DWT
    if (us == 0U) {
        return;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
        /* active wait: the STEP pulse width */
    }
#else
    (void)us;
#endif
}

static void stepper_write_dir(uint8_t motor, bool positive)
{
    bool level = positive;
    if (s_cfg.invert[motor]) {
        level = !level;
    }
    HAL_GPIO_WritePin(s_dir_port[motor], s_dir_pin[motor],
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stepper_apply_dirs(void)
{
    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        stepper_write_dir(i, (s_sign[i] > 0));
    }
}

/* Step generation -----------------------------------------------------------*/

static void stepper_emit(void)
{
    uint8_t pulse = 0U;

    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        if (s_num[i] == 0) {
            continue;
        }
        s_acc[i] += s_num[i];
        if (s_acc[i] >= s_masters) {
            s_acc[i] -= s_masters;
            pulse |= (uint8_t)(1U << i);
            s_pos_steps[i] += s_sign[i];
        }
    }

    if (pulse == 0U) {
        return;
    }

    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        if ((pulse & (uint8_t)(1U << i)) != 0U) {
            s_step_port[i]->BSRR = (uint32_t)s_step_pin[i];
        }
    }

    stepper_delay_us(TUE_STEPPER_PULSE_US);

    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        if ((pulse & (uint8_t)(1U << i)) != 0U) {
            s_step_port[i]->BSRR = (uint32_t)s_step_pin[i] << 16;
        }
    }
}

static void stepper_set_interval(void)
{
    float v = sqrtf(s_v_sq);
    if (v < TUE_STEPPER_MIN_RATE_STEPS_S) {
        v = TUE_STEPPER_MIN_RATE_STEPS_S;
    }
    float interval_us = (float)TUE_STEPPER_TIMER_HZ / v;
    if (interval_us < (float)TUE_STEPPER_MIN_INTERVAL_US) {
        interval_us = (float)TUE_STEPPER_MIN_INTERVAL_US;
    }
    if (interval_us > 65535.0f) {
        interval_us = 65535.0f;
    }

    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim1, (uint32_t)interval_us - 1U);
}

static void stepper_load_segment(uint8_t index)
{
    const tue_stepper_segment_t *seg = &s_queue[index];

    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        s_num[i]  = seg->num[i];
        s_acc[i]  = 0;
        s_sign[i] = seg->sign[i];
    }
    s_masters    = seg->masters;
    s_v_peak     = seg->v_peak;
    s_accel      = seg->accel;
    s_total      = seg->total;
    s_continuous = (seg->total == 0U);
    s_stopping   = false;
    s_idx        = 0U;
    s_v_sq       = s_v_min_sq;

    stepper_apply_dirs();
}

static void stepper_advance(void)
{
    /* Consume the finished segment so the ring really is empty when head == tail. */
    s_tail = (uint8_t)((s_tail + 1U) % TUE_STEPPER_QUEUE_SIZE);

    if (s_tail != s_head) {
        stepper_load_segment(s_tail);
        stepper_set_interval();
        return;
    }

    /* Nothing else queued: shut the step clock down. */
    s_running = false;
    HAL_TIM_Base_Stop_IT(&htim1);
    if (s_idle_sem != NULL) {
        (void)osSemaphoreRelease(s_idle_sem);
    }
    if (s_idle_cb != NULL) {
        s_idle_cb();
    }
}

static void stepper_isr(void)
{
    if (!s_running) {
        return;
    }

    stepper_emit();
    s_idx++;

    if (!s_continuous) {
        if (s_idx >= s_total) {
            stepper_advance();
            return;
        }
    } else if (s_stopping && (s_v_sq <= s_v_min_sq)) {
        stepper_advance();
        return;
    }

    /* Velocity ramp (trapezoid for position moves, hold for jogs). */
    float two_a = 2.0f * s_accel;

    if (s_continuous) {
        if (s_stopping) {
            s_v_sq -= two_a;
            if (s_v_sq < 0.0f) {
                s_v_sq = 0.0f;
            }
        } else if (s_v_sq < (s_v_peak * s_v_peak)) {
            s_v_sq += two_a;
        }
    } else {
        float peak_sq = s_v_peak * s_v_peak;
        float decel_dist = (s_accel > 0.0f) ? (s_v_sq / two_a) : 0.0f;
        float remaining = (float)(s_total - s_idx);

        if ((s_v_sq < peak_sq) && (remaining > decel_dist)) {
            s_v_sq += two_a;
            if (s_v_sq > peak_sq) {
                s_v_sq = peak_sq;
            }
        } else if ((s_v_sq > s_v_min_sq) && (remaining <= decel_dist)) {
            s_v_sq -= two_a;
            if (s_v_sq < 0.0f) {
                s_v_sq = 0.0f;
            }
        } else {
            /* cruise */
        }
    }

    stepper_set_interval();
}

/* Planning ------------------------------------------------------------------*/

static bool stepper_queue_space(void)
{
    uint8_t next = (uint8_t)((s_head + 1U) % TUE_STEPPER_QUEUE_SIZE);
    return (next != s_tail);
}

static void stepper_resync_if_idle(void)
{
    if (!s_running) {
        for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
            s_cmd_steps[i] = s_pos_steps[i];
        }
    }
}

static void stepper_cmd_mm_from_steps(float *x, float *y, float *z)
{
    float a = (float)s_cmd_steps[TUE_STEPPER_M1] / s_cfg.steps_per_mm[TUE_STEPPER_M1];
    float b = (float)s_cmd_steps[TUE_STEPPER_M2] / s_cfg.steps_per_mm[TUE_STEPPER_M2];
    float c = (float)s_cmd_steps[TUE_STEPPER_M3] / s_cfg.steps_per_mm[TUE_STEPPER_M3];
    if (x != NULL) { *x = (a + b) * 0.5f; }
    if (y != NULL) { *y = (a - b) * 0.5f; }
    if (z != NULL) { *z = c; }
}

static tue_stepper_status_t stepper_plan_enqueue(const int32_t dsteps[TUE_STEPPER_MOTOR_COUNT],
                                                 float feed_mm_s, float accel_mm_s2,
                                                 float path_mm)
{
    int32_t masters = 0;
    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        int32_t ad = stepper_abs32(dsteps[i]);
        if (ad > masters) {
            masters = ad;
        }
    }
    if (masters == 0) {
        return TUE_STEPPER_OK; /* nothing to do */
    }
    if ((feed_mm_s <= 0.0f) || (accel_mm_s2 <= 0.0f) || (path_mm <= 0.0f)) {
        return TUE_STEPPER_ERR_PARAM;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!stepper_queue_space()) {
        __set_PRIMASK(primask);
        return TUE_STEPPER_ERR_QUEUE_FULL;
    }

    tue_stepper_segment_t *seg = &s_queue[s_head];

    /* Scale the tool-path feed/accel onto the dominant (master) axis. */
    float scale = (float)masters / path_mm;
    float v = feed_mm_s * scale;
    float a = accel_mm_s2 * scale;
    float v_achievable = (float)TUE_STEPPER_TIMER_HZ / (float)TUE_STEPPER_MIN_INTERVAL_US;

    if (v > v_achievable)             { v = v_achievable; }
    if (v > TUE_STEPPER_MAX_RATE_STEPS_S) { v = TUE_STEPPER_MAX_RATE_STEPS_S; }
    if (v < TUE_STEPPER_MIN_RATE_STEPS_S) { v = TUE_STEPPER_MIN_RATE_STEPS_S; }

    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        seg->num[i]  = stepper_abs32(dsteps[i]);
        seg->sign[i] = (dsteps[i] < 0) ? (int8_t)-1 : (int8_t)+1;
    }
    seg->masters = masters;
    seg->v_peak  = v;
    seg->accel   = a;
    seg->total   = (uint32_t)masters;

    s_head = (uint8_t)((s_head + 1U) % TUE_STEPPER_QUEUE_SIZE);

    if (!s_running) {
        s_running = true;
        if (s_idle_sem != NULL) {
            (void)osSemaphoreAcquire(s_idle_sem, 0U);
        }
        stepper_load_segment(s_tail);
        stepper_set_interval();
        HAL_TIM_Base_Start_IT(&htim1);
    }

    __set_PRIMASK(primask);
    return TUE_STEPPER_OK;
}

/* Common move planner: absolute CoreXY/Z target in mm. ----------------------*/
static tue_stepper_status_t stepper_plan_move_to(float x, float y, float z,
                                                 float feed_mm_s, float accel_mm_s2)
{
    stepper_resync_if_idle();

    float cxs, cys, czs;
    stepper_cmd_mm_from_steps(&cxs, &cys, &czs);
    float dx = x - cxs;
    float dy = y - cys;
    float dz = z - czs;
    float path = sqrtf((dx * dx) + (dy * dy) + (dz * dz));

    int32_t target[TUE_STEPPER_MOTOR_COUNT];
    target[TUE_STEPPER_M1] = (int32_t)lroundf((x + y) * s_cfg.steps_per_mm[TUE_STEPPER_M1]);
    target[TUE_STEPPER_M2] = (int32_t)lroundf((x - y) * s_cfg.steps_per_mm[TUE_STEPPER_M2]);
    target[TUE_STEPPER_M3] = (int32_t)lroundf(z * s_cfg.steps_per_mm[TUE_STEPPER_M3]);

    int32_t d[TUE_STEPPER_MOTOR_COUNT];
    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        d[i] = target[i] - s_cmd_steps[i];
    }

    if (path <= 1e-6f) {
        bool any = false;
        for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
            if (d[i] != 0) { any = true; }
        }
        if (!any) {
            return TUE_STEPPER_OK;
        }
        return TUE_STEPPER_ERR_PARAM;
    }

    tue_stepper_status_t st = stepper_plan_enqueue(d, feed_mm_s, accel_mm_s2, path);
    if (st == TUE_STEPPER_OK) {
        for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
            s_cmd_steps[i] = target[i];
        }
        if (!s_enabled) {
            tue_stepper_enable(true);
        }
    }
    return st;
}

/* Public API ----------------------------------------------------------------*/

tue_stepper_config_t tue_stepper_default_config(void)
{
    tue_stepper_config_t cfg;
    cfg.steps_per_mm[TUE_STEPPER_M1] = TUE_STEPPER_XY_STEPS_PER_MM;
    cfg.steps_per_mm[TUE_STEPPER_M2] = TUE_STEPPER_XY_STEPS_PER_MM;
    cfg.steps_per_mm[TUE_STEPPER_M3] = TUE_STEPPER_Z_STEPS_PER_MM;
    cfg.invert[TUE_STEPPER_M1] = false;
    cfg.invert[TUE_STEPPER_M2] = false;
    cfg.invert[TUE_STEPPER_M3] = false;
    return cfg;
}

void tue_stepper_set_config(const tue_stepper_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memcpy(&s_cfg, cfg, sizeof(s_cfg));
}

void tue_stepper_get_config(tue_stepper_config_t *cfg)
{
    if (cfg != NULL) {
        memcpy(cfg, &s_cfg, sizeof(s_cfg));
    }
}

void tue_stepper_init(void)
{
    if (s_initialized) {
        return;
    }

    s_cfg = tue_stepper_default_config();
    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        s_pos_steps[i] = 0;
        s_cmd_steps[i] = 0;
    }
    s_v_min_sq = TUE_STEPPER_MIN_RATE_STEPS_S * TUE_STEPPER_MIN_RATE_STEPS_S;

#if TUE_STEPPER_USE_DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif

    /* STEP pins: re-purpose from TIM1 alternate function to plain outputs. */
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Pin   = (uint32_t)(STEP_1_Pin | STEP_2_Pin | STEP_3_Pin);
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &gpio);
    s_step_port[0]->BSRR = (uint32_t)s_step_pin[0] << 16;
    s_step_port[1]->BSRR = (uint32_t)s_step_pin[1] << 16;
    s_step_port[2]->BSRR = (uint32_t)s_step_pin[2] << 16;

    /* Direction pins (already outputs from MX_GPIO_Init). */
    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        HAL_GPIO_WritePin(s_dir_port[i], s_dir_pin[i], GPIO_PIN_RESET);
    }

    /* Drivers start released. */
    tue_stepper_enable(false);

    /* TIM1 as the step clock: 1 us tick, one update interrupt per step. */
    htim1.Init.Prescaler         = (uint32_t)((SystemCoreClock / TUE_STEPPER_TIMER_HZ) - 1U);
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 0xFFFFU;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0U;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    (void)HAL_TIM_Base_Init(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);

    if (s_idle_sem == NULL) {
        const osSemaphoreAttr_t sem_attr = { .name = "step_idle" };
        s_idle_sem = osSemaphoreNew(1U, 1U, &sem_attr);
    }

    s_initialized = true;
}

void tue_stepper_set_idle_callback(tue_stepper_idle_cb_t cb)
{
    s_idle_cb = cb;
}

void tue_stepper_enable(bool enable)
{
#if TUE_STEPPER_EN_ACTIVE_LOW
    GPIO_PinState level = enable ? GPIO_PIN_RESET : GPIO_PIN_SET;
#else
    GPIO_PinState level = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif
    HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, level);
    s_enabled = enable;
}

bool tue_stepper_is_enabled(void)
{
    return s_enabled;
}

/* Position ------------------------------------------------------------------*/

void tue_stepper_set_position_mm(float x, float y, float z)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_cmd_steps[TUE_STEPPER_M1] = (int32_t)lroundf((x + y) * s_cfg.steps_per_mm[TUE_STEPPER_M1]);
    s_cmd_steps[TUE_STEPPER_M2] = (int32_t)lroundf((x - y) * s_cfg.steps_per_mm[TUE_STEPPER_M2]);
    s_cmd_steps[TUE_STEPPER_M3] = (int32_t)lroundf(z * s_cfg.steps_per_mm[TUE_STEPPER_M3]);
    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        s_pos_steps[i] = s_cmd_steps[i];
    }
    __set_PRIMASK(primask);
}

void tue_stepper_get_position_mm(float *x, float *y, float *z)
{
    float a = (float)s_pos_steps[TUE_STEPPER_M1] / s_cfg.steps_per_mm[TUE_STEPPER_M1];
    float b = (float)s_pos_steps[TUE_STEPPER_M2] / s_cfg.steps_per_mm[TUE_STEPPER_M2];
    float c = (float)s_pos_steps[TUE_STEPPER_M3] / s_cfg.steps_per_mm[TUE_STEPPER_M3];
    if (x != NULL) { *x = (a + b) * 0.5f; }
    if (y != NULL) { *y = (a - b) * 0.5f; }
    if (z != NULL) { *z = c; }
}

void tue_stepper_get_target_mm(float *x, float *y, float *z)
{
    stepper_cmd_mm_from_steps(x, y, z);
}

int32_t tue_stepper_get_motor_steps(tue_stepper_motor_t motor)
{
    if (motor >= TUE_STEPPER_MOTOR_COUNT) {
        return 0;
    }
    return s_pos_steps[motor];
}

int32_t tue_stepper_mm_to_steps_xy(float mm)
{
    return (int32_t)lroundf(mm * s_cfg.steps_per_mm[TUE_STEPPER_M1]);
}

int32_t tue_stepper_mm_to_steps_z(float mm)
{
    return (int32_t)lroundf(mm * s_cfg.steps_per_mm[TUE_STEPPER_M3]);
}

/* Motion --------------------------------------------------------------------*/

tue_stepper_status_t tue_stepper_move_to_xyz_mm(float x, float y, float z,
                                                float feed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) {
        return TUE_STEPPER_ERR_PARAM;
    }
    return stepper_plan_move_to(x, y, z, feed_mm_s, accel_mm_s2);
}

tue_stepper_status_t tue_stepper_move_to_mm(float x, float y,
                                            float feed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) {
        return TUE_STEPPER_ERR_PARAM;
    }
    stepper_resync_if_idle();
    float cxs, cys, czs;
    stepper_cmd_mm_from_steps(&cxs, &cys, &czs);
    (void)cxs;
    (void)cys;
    return stepper_plan_move_to(x, y, czs, feed_mm_s, accel_mm_s2);
}

tue_stepper_status_t tue_stepper_move_z_mm(float z, float feed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) {
        return TUE_STEPPER_ERR_PARAM;
    }
    stepper_resync_if_idle();
    float cxs, cys, czs;
    stepper_cmd_mm_from_steps(&cxs, &cys, &czs);
    return stepper_plan_move_to(cxs, cys, z, feed_mm_s, accel_mm_s2);
}

tue_stepper_status_t tue_stepper_move_by_mm(float dx, float dy,
                                            float feed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) {
        return TUE_STEPPER_ERR_PARAM;
    }
    stepper_resync_if_idle();
    float cxs, cys, czs;
    stepper_cmd_mm_from_steps(&cxs, &cys, &czs);
    return stepper_plan_move_to(cxs + dx, cys + dy, czs, feed_mm_s, accel_mm_s2);
}

tue_stepper_status_t tue_stepper_move_motor_steps(tue_stepper_motor_t motor,
                                                  int32_t steps,
                                                  float feed_mm_s, float accel_mm_s2)
{
    if ((!s_initialized) || (motor >= TUE_STEPPER_MOTOR_COUNT)) {
        return TUE_STEPPER_ERR_PARAM;
    }
    stepper_resync_if_idle();

    int32_t d[TUE_STEPPER_MOTOR_COUNT] = { 0, 0, 0 };
    d[motor] = steps;
    float path = (float)stepper_abs32(steps) / s_cfg.steps_per_mm[motor];

    tue_stepper_status_t st = stepper_plan_enqueue(d, feed_mm_s, accel_mm_s2, path);
    if (st == TUE_STEPPER_OK) {
        s_cmd_steps[motor] += steps;
        if (!s_enabled) {
            tue_stepper_enable(true);
        }
    }
    return st;
}

/* Velocity jogs -------------------------------------------------------------*/

tue_stepper_status_t tue_stepper_start_velocity(float vx_mm_s, float vy_mm_s,
                                                float accel_mm_s2)
{
    if (!s_initialized) {
        return TUE_STEPPER_ERR_PARAM;
    }
    if (accel_mm_s2 <= 0.0f) {
        return TUE_STEPPER_ERR_PARAM;
    }

    float tool = sqrtf((vx_mm_s * vx_mm_s) + (vy_mm_s * vy_mm_s));
    if (tool <= 0.0f) {
        tue_stepper_stop();
        return TUE_STEPPER_OK;
    }

    int32_t rate_a = (int32_t)lroundf((vx_mm_s + vy_mm_s) * s_cfg.steps_per_mm[TUE_STEPPER_M1]);
    int32_t rate_b = (int32_t)lroundf((vx_mm_s - vy_mm_s) * s_cfg.steps_per_mm[TUE_STEPPER_M2]);
    int32_t masters = stepper_abs32(rate_a);
    if (stepper_abs32(rate_b) > masters) {
        masters = stepper_abs32(rate_b);
    }
    if (masters == 0) {
        return TUE_STEPPER_ERR_PARAM;
    }

    float v = (float)masters;
    float v_achievable = (float)TUE_STEPPER_TIMER_HZ / (float)TUE_STEPPER_MIN_INTERVAL_US;
    if (v > v_achievable) { v = v_achievable; }

    /* Live speed change while a jog is already running. */
    if (s_running && s_continuous) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_v_peak = v;
        __set_PRIMASK(primask);
        return TUE_STEPPER_OK;
    }
    if (s_running) {
        return TUE_STEPPER_BUSY;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!stepper_queue_space()) {
        __set_PRIMASK(primask);
        return TUE_STEPPER_ERR_QUEUE_FULL;
    }
    tue_stepper_segment_t *seg = &s_queue[s_head];
    seg->num[TUE_STEPPER_M1]  = stepper_abs32(rate_a);
    seg->num[TUE_STEPPER_M2]  = stepper_abs32(rate_b);
    seg->num[TUE_STEPPER_M3]  = 0;
    seg->sign[TUE_STEPPER_M1] = (rate_a < 0) ? (int8_t)-1 : (int8_t)+1;
    seg->sign[TUE_STEPPER_M2] = (rate_b < 0) ? (int8_t)-1 : (int8_t)+1;
    seg->sign[TUE_STEPPER_M3] = (int8_t)+1;
    seg->masters = masters;
    seg->v_peak  = v;
    seg->accel   = accel_mm_s2 * (float)masters / tool;
    seg->total   = 0U; /* continuous */

    s_head = (uint8_t)((s_head + 1U) % TUE_STEPPER_QUEUE_SIZE);

    s_running = true;
    stepper_load_segment(s_tail);
    stepper_set_interval();
    HAL_TIM_Base_Start_IT(&htim1);
    __set_PRIMASK(primask);

    if (!s_enabled) {
        tue_stepper_enable(true);
    }
    return TUE_STEPPER_OK;
}

tue_stepper_status_t tue_stepper_start_velocity_z(float vz_mm_s, float accel_mm_s2)
{
    if (!s_initialized) {
        return TUE_STEPPER_ERR_PARAM;
    }
    if (accel_mm_s2 <= 0.0f) {
        return TUE_STEPPER_ERR_PARAM;
    }

    float tool = fabsf(vz_mm_s);
    if (tool <= 0.0f) {
        tue_stepper_stop();
        return TUE_STEPPER_OK;
    }

    int32_t rate_c = (int32_t)lroundf(vz_mm_s * s_cfg.steps_per_mm[TUE_STEPPER_M3]);
    int32_t masters = stepper_abs32(rate_c);
    if (masters == 0) {
        return TUE_STEPPER_ERR_PARAM;
    }

    float v = (float)masters;
    float v_achievable = (float)TUE_STEPPER_TIMER_HZ / (float)TUE_STEPPER_MIN_INTERVAL_US;
    if (v > v_achievable) { v = v_achievable; }

    if (s_running && s_continuous) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_v_peak = v;
        __set_PRIMASK(primask);
        return TUE_STEPPER_OK;
    }
    if (s_running) {
        return TUE_STEPPER_BUSY;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!stepper_queue_space()) {
        __set_PRIMASK(primask);
        return TUE_STEPPER_ERR_QUEUE_FULL;
    }
    tue_stepper_segment_t *seg = &s_queue[s_head];
    seg->num[TUE_STEPPER_M1]  = 0;
    seg->num[TUE_STEPPER_M2]  = 0;
    seg->num[TUE_STEPPER_M3]  = masters;
    seg->sign[TUE_STEPPER_M1] = (int8_t)+1;
    seg->sign[TUE_STEPPER_M2] = (int8_t)+1;
    seg->sign[TUE_STEPPER_M3] = (rate_c < 0) ? (int8_t)-1 : (int8_t)+1;
    seg->masters = masters;
    seg->v_peak  = v;
    seg->accel   = accel_mm_s2 * (float)masters / tool;
    seg->total   = 0U;

    s_head = (uint8_t)((s_head + 1U) % TUE_STEPPER_QUEUE_SIZE);

    s_running = true;
    stepper_load_segment(s_tail);
    stepper_set_interval();
    HAL_TIM_Base_Start_IT(&htim1);
    __set_PRIMASK(primask);

    if (!s_enabled) {
        tue_stepper_enable(true);
    }
    return TUE_STEPPER_OK;
}

void tue_stepper_stop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_running) {
        if (s_continuous) {
            s_stopping = true;
        } else {
            float decel_dist = (s_accel > 0.0f) ? (s_v_sq / (2.0f * s_accel)) : 0.0f;
            uint32_t new_total = s_idx + (uint32_t)decel_dist + 1U;
            if (new_total < s_total) {
                s_total = new_total;
            }
        }
        /* Drop everything queued after the current segment. */
        s_head = (uint8_t)((s_tail + 1U) % TUE_STEPPER_QUEUE_SIZE);
    }

    __set_PRIMASK(primask);
}

void tue_stepper_abort(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    HAL_TIM_Base_Stop_IT(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    s_running = false;
    s_head = 0U;
    s_tail = 0U;
    s_continuous = false;
    s_stopping = false;
    s_masters = 0;
    for (uint8_t i = 0U; i < TUE_STEPPER_MOTOR_COUNT; i++) {
        s_num[i] = 0;
        s_acc[i] = 0;
        s_step_port[i]->BSRR = (uint32_t)s_step_pin[i] << 16;
        s_cmd_steps[i] = s_pos_steps[i];
    }

    if (s_idle_sem != NULL) {
        (void)osSemaphoreRelease(s_idle_sem);
    }
    if (s_idle_cb != NULL) {
        s_idle_cb();
    }

    __set_PRIMASK(primask);
}

/* Status --------------------------------------------------------------------*/

bool tue_stepper_is_busy(void)
{
    return s_running;
}

uint8_t tue_stepper_queue_depth(void)
{
    return (uint8_t)((s_head + TUE_STEPPER_QUEUE_SIZE - s_tail) % TUE_STEPPER_QUEUE_SIZE);
}

tue_stepper_status_t tue_stepper_wait_idle(uint32_t timeout_ms)
{
    if (!s_running) {
        return TUE_STEPPER_OK;
    }
    if (s_idle_sem != NULL && osKernelGetState() == osKernelRunning) {
        uint32_t wait = (timeout_ms == 0U) ? osWaitForever : timeout_ms;
        if (osSemaphoreAcquire(s_idle_sem, wait) != osOK) {
            return TUE_STEPPER_ERR_TIMEOUT;
        }
        (void)osSemaphoreRelease(s_idle_sem);
        return TUE_STEPPER_OK;
    }
    return TUE_STEPPER_OK;
}

/* ISR hook ------------------------------------------------------------------*/

void tue_stepper_on_timer_elapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) {
        return;
    }
    stepper_isr();
}
