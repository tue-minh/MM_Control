/**
  ******************************************************************************
  * @file    tue_sts3215.h
  * @brief   Feetech STS3215 serial bus servo - high-level blocking driver.
  ******************************************************************************
  * @attention
  *
  * Architecture (three layers, ported from reference_for_sts325):
  *   - tue_sts3215_regs.h     : register map & protocol constants (no deps).
  *   - tue_sts3215_protocol   : pure packet builders + reply parser (no STM32).
  *   - tue_sts3215_hal        : half-duplex USART1 + DMA state machine (async).
  *   - tue_sts3215.h/.c       : this file - typed, *blocking* facade over the
  *                              layers above, kept API-compatible with the
  *                              tue_acsi servo command set.
  *
  * Hardware: USART1 single-wire half-duplex on PA9 (SERVO_TX), DMA2 Stream 7
  * (TX) and DMA2 Stream 2 (RX), 1 Mbaud 8N1.
  ******************************************************************************
  */

#ifndef TUE_STS3215_H_
#define TUE_STS3215_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "tue_sts3215_regs.h"
#include "tue_sts3215_protocol.h"
#include "tue_sts3215_hal.h"
#include "cmsis_os2.h"

/* Configuration -------------------------------------------------------------*/
#define STS3215_DEFAULT_TIMEOUT_MS      20U
#define TUE_STS3215_MAX_REPLIES         STS3215_SYNC_MAX_SERVOS

/* Driver return status codes ------------------------------------------------*/
typedef enum {
    TUE_STS3215_OK              = 0,  /* Success */
    TUE_STS3215_ERR_TIMEOUT     = 1,  /* No response within timeout */
    TUE_STS3215_ERR_CHECKSUM    = 2,  /* Response checksum mismatch */
    TUE_STS3215_ERR_HEADER      = 3,  /* Response frame header (0xFF 0xFF) invalid */
    TUE_STS3215_ERR_LEN         = 4,  /* Frame length incorrect */
    TUE_STS3215_ERR_ID          = 5,  /* Response ID did not match request */
    TUE_STS3215_ERR_STATUS      = 6,  /* Servo reported hardware error bit in status byte */
    TUE_STS3215_ERR_PARAM       = 7,  /* Out of range or NULL parameter */
    TUE_STS3215_ERR_HAL         = 8,  /* Underlying STM32 HAL / DMA error */
    TUE_STS3215_ERR_BUSY        = 9   /* Bus is busy with another transfer */
} tue_sts3215_status_t;

/* Comprehensive telemetry feedback structure --------------------------------*/
typedef struct {
    uint8_t  id;                /* Responding servo ID */
    int16_t  position;          /* Current angular position (steps) */
    int16_t  speed;             /* Current angular velocity (steps/s) */
    int16_t  load;              /* Motor output drive duty (0.1%) */
    uint8_t  voltage_raw;       /* Raw voltage register value (0.1V) */
    float    voltage_v;         /* Supply voltage in Volts */
    int8_t   temperature_degc;  /* Internal motor temperature in °C */
    uint8_t  async_flag;        /* Reg write pending flag */
    uint8_t  status_mask;       /* Hardware status error flags */
    uint8_t  is_moving;         /* 1: in motion, 0: stationary */
    int16_t  current_raw;       /* Raw motor current (6.5mA units) */
    float    current_ma;        /* Motor current in mA */
} tue_sts3215_feedback_t;

/* Sync write entry for position control -------------------------------------*/
typedef struct {
    uint8_t  id;
    int16_t  position;
    int16_t  speed;
    uint8_t  accel;
} tue_sts3215_sync_pos_t;

/* Sync write entry for velocity control -------------------------------------*/
typedef struct {
    uint8_t  id;
    int16_t  speed;
    uint8_t  accel;
} tue_sts3215_sync_speed_t;

/* Driver handle structure ---------------------------------------------------*/
typedef struct {
    tue_sts3215_hal_t    hal;               /* Embedded async HAL instance */
    uint32_t             timeout_ms;        /* Blocking reply timeout */

    /* Capture state filled by the HAL callbacks (IRQ context). A single
     * transaction may collect several replies (SYNC_READ). */
    volatile bool        xfer_done;
    volatile uint8_t     reply_count;
    tue_sts3215_reply_t  replies[TUE_STS3215_MAX_REPLIES];
    tue_sts3215_status_t xfer_status;
    uint8_t              last_servo_status;
    osSemaphoreId_t      sem;
} tue_sts3215_t;

/* Core lifecycle & configuration --------------------------------------------*/

tue_sts3215_status_t tue_sts3215_init(tue_sts3215_t *dev, UART_HandleTypeDef *huart);
tue_sts3215_t*       tue_sts3215_get_default(void);
void                 tue_sts3215_set_timeout(tue_sts3215_t *dev, uint32_t timeout_ms);

/* Low-level protocol commands -----------------------------------------------*/

tue_sts3215_status_t tue_sts3215_ping(tue_sts3215_t *dev, uint8_t id);
tue_sts3215_status_t tue_sts3215_read_raw(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr,
                                          uint8_t *data, uint8_t len);
tue_sts3215_status_t tue_sts3215_write_raw(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr,
                                           const uint8_t *data, uint8_t len);
tue_sts3215_status_t tue_sts3215_write_raw_no_ack(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr,
                                                  const uint8_t *data, uint8_t len);
tue_sts3215_status_t tue_sts3215_reg_write_raw(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr,
                                               const uint8_t *data, uint8_t len);
tue_sts3215_status_t tue_sts3215_action(tue_sts3215_t *dev, uint8_t id);
tue_sts3215_status_t tue_sts3215_reset(tue_sts3215_t *dev, uint8_t id);
tue_sts3215_status_t tue_sts3215_reset_turns(tue_sts3215_t *dev, uint8_t id);
tue_sts3215_status_t tue_sts3215_sync_write_raw(tue_sts3215_t *dev, uint8_t start_addr,
                                                uint8_t data_len, const uint8_t *param_data,
                                                uint16_t total_param_bytes);

/**
  * @brief  Broadcast SYNC_READ (0x82) and collect one reply per servo.
  * @param  dev          Driver handle.
  * @param  reg          First register address.
  * @param  len          Bytes to read per servo.
  * @param  ids          Array of servo IDs (count entries).
  * @param  count        Number of servos (<= STS3215_SYNC_MAX_SERVOS).
  * @param  replies      Output array receiving each reply (may be NULL).
  * @param  reply_count  Output: number of replies actually collected.
  * @retval TUE_STS3215_OK on success (all replies received).
  */
tue_sts3215_status_t tue_sts3215_sync_read(tue_sts3215_t *dev, uint8_t reg, uint8_t len,
                                           const uint8_t *ids, uint8_t count,
                                           tue_sts3215_reply_t *replies, uint8_t *reply_count);

/* Primitive read/write helpers ----------------------------------------------*/
tue_sts3215_status_t tue_sts3215_read_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t *val);
tue_sts3215_status_t tue_sts3215_write_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t val);
tue_sts3215_status_t tue_sts3215_read_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t *val);
tue_sts3215_status_t tue_sts3215_write_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t val);
tue_sts3215_status_t tue_sts3215_read_i16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, int16_t *val);
tue_sts3215_status_t tue_sts3215_write_i16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, int16_t val);

/* EEPROM safe write helpers (handles unlock 0x37 automatically) -------------*/
tue_sts3215_status_t tue_sts3215_write_eeprom_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t val);
tue_sts3215_status_t tue_sts3215_write_eeprom_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t val);

/* High-level register access ------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_lock_eeprom(tue_sts3215_t *dev, uint8_t id);
tue_sts3215_status_t tue_sts3215_unlock_eeprom(tue_sts3215_t *dev, uint8_t id);
tue_sts3215_status_t tue_sts3215_get_eeprom_lock(tue_sts3215_t *dev, uint8_t id, uint8_t *is_locked);

tue_sts3215_status_t tue_sts3215_get_fw_version(tue_sts3215_t *dev, uint8_t id, uint8_t *major, uint8_t *minor);
tue_sts3215_status_t tue_sts3215_get_end_marker(tue_sts3215_t *dev, uint8_t id, uint8_t *end_val);
tue_sts3215_status_t tue_sts3215_get_servo_version(tue_sts3215_t *dev, uint8_t id, uint8_t *main_ver, uint8_t *sub_ver);

tue_sts3215_status_t tue_sts3215_set_id(tue_sts3215_t *dev, uint8_t old_id, uint8_t new_id);
tue_sts3215_status_t tue_sts3215_get_id(tue_sts3215_t *dev, uint8_t id, uint8_t *current_id);
tue_sts3215_status_t tue_sts3215_set_baud_rate(tue_sts3215_t *dev, uint8_t id, tue_sts3215_baud_t baud);
tue_sts3215_status_t tue_sts3215_get_baud_rate(tue_sts3215_t *dev, uint8_t id, tue_sts3215_baud_t *baud);

tue_sts3215_status_t tue_sts3215_set_return_delay(tue_sts3215_t *dev, uint8_t id, uint8_t delay_2us);
tue_sts3215_status_t tue_sts3215_get_return_delay(tue_sts3215_t *dev, uint8_t id, uint8_t *delay_2us);
tue_sts3215_status_t tue_sts3215_set_response_level(tue_sts3215_t *dev, uint8_t id, uint8_t level);
tue_sts3215_status_t tue_sts3215_get_response_level(tue_sts3215_t *dev, uint8_t id, uint8_t *level);

tue_sts3215_status_t tue_sts3215_set_angle_limits(tue_sts3215_t *dev, uint8_t id, uint16_t min_angle, uint16_t max_angle);
tue_sts3215_status_t tue_sts3215_get_angle_limits(tue_sts3215_t *dev, uint8_t id, uint16_t *min_angle, uint16_t *max_angle);

tue_sts3215_status_t tue_sts3215_set_max_temperature_limit(tue_sts3215_t *dev, uint8_t id, uint8_t max_temp_c);
tue_sts3215_status_t tue_sts3215_get_max_temperature_limit(tue_sts3215_t *dev, uint8_t id, uint8_t *max_temp_c);
tue_sts3215_status_t tue_sts3215_set_voltage_limits(tue_sts3215_t *dev, uint8_t id, uint8_t min_0_1v, uint8_t max_0_1v);
tue_sts3215_status_t tue_sts3215_get_voltage_limits(tue_sts3215_t *dev, uint8_t id, uint8_t *min_0_1v, uint8_t *max_0_1v);

tue_sts3215_status_t tue_sts3215_set_max_torque(tue_sts3215_t *dev, uint8_t id, uint16_t max_torque_0_1pct);
tue_sts3215_status_t tue_sts3215_get_max_torque(tue_sts3215_t *dev, uint8_t id, uint16_t *max_torque_0_1pct);
tue_sts3215_status_t tue_sts3215_set_phase(tue_sts3215_t *dev, uint8_t id, uint8_t phase);
tue_sts3215_status_t tue_sts3215_get_phase(tue_sts3215_t *dev, uint8_t id, uint8_t *phase);

tue_sts3215_status_t tue_sts3215_set_unload_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t conditions_mask);
tue_sts3215_status_t tue_sts3215_get_unload_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t *conditions_mask);
tue_sts3215_status_t tue_sts3215_set_led_alarm_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t conditions_mask);
tue_sts3215_status_t tue_sts3215_get_led_alarm_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t *conditions_mask);

tue_sts3215_status_t tue_sts3215_set_position_pid(tue_sts3215_t *dev, uint8_t id, uint8_t p, uint8_t d, uint8_t i);
tue_sts3215_status_t tue_sts3215_get_position_pid(tue_sts3215_t *dev, uint8_t id, uint8_t *p, uint8_t *d, uint8_t *i);
tue_sts3215_status_t tue_sts3215_set_min_starting_force(tue_sts3215_t *dev, uint8_t id, uint8_t force_0_1pct);
tue_sts3215_status_t tue_sts3215_get_min_starting_force(tue_sts3215_t *dev, uint8_t id, uint8_t *force_0_1pct);

tue_sts3215_status_t tue_sts3215_set_points_limit(tue_sts3215_t *dev, uint8_t id, uint8_t limit);
tue_sts3215_status_t tue_sts3215_get_points_limit(tue_sts3215_t *dev, uint8_t id, uint8_t *limit);
tue_sts3215_status_t tue_sts3215_set_deadband(tue_sts3215_t *dev, uint8_t id, uint8_t cw_deadband, uint8_t ccw_deadband);
tue_sts3215_status_t tue_sts3215_get_deadband(tue_sts3215_t *dev, uint8_t id, uint8_t *cw_deadband, uint8_t *ccw_deadband);

tue_sts3215_status_t tue_sts3215_set_protective_current(tue_sts3215_t *dev, uint8_t id, uint16_t current_6_5ma);
tue_sts3215_status_t tue_sts3215_get_protective_current(tue_sts3215_t *dev, uint8_t id, uint16_t *current_6_5ma);
tue_sts3215_status_t tue_sts3215_set_angular_resolution(tue_sts3215_t *dev, uint8_t id, uint8_t resolution);
tue_sts3215_status_t tue_sts3215_get_angular_resolution(tue_sts3215_t *dev, uint8_t id, uint8_t *resolution);
tue_sts3215_status_t tue_sts3215_set_position_correction(tue_sts3215_t *dev, uint8_t id, int16_t offset_steps);
tue_sts3215_status_t tue_sts3215_get_position_correction(tue_sts3215_t *dev, uint8_t id, int16_t *offset_steps);

tue_sts3215_status_t tue_sts3215_set_operation_mode(tue_sts3215_t *dev, uint8_t id, tue_sts3215_mode_t mode);
tue_sts3215_status_t tue_sts3215_get_operation_mode(tue_sts3215_t *dev, uint8_t id, tue_sts3215_mode_t *mode);

tue_sts3215_status_t tue_sts3215_set_overload_protection(tue_sts3215_t *dev, uint8_t id,
                                                         uint8_t protective_torque_pct,
                                                         uint8_t protection_time_10ms,
                                                         uint8_t overload_torque_pct);
tue_sts3215_status_t tue_sts3215_get_overload_protection(tue_sts3215_t *dev, uint8_t id,
                                                         uint8_t *protective_torque_pct,
                                                         uint8_t *protection_time_10ms,
                                                         uint8_t *overload_torque_pct);

tue_sts3215_status_t tue_sts3215_set_speed_pid(tue_sts3215_t *dev, uint8_t id, uint8_t speed_p, uint8_t speed_i);
tue_sts3215_status_t tue_sts3215_get_speed_pid(tue_sts3215_t *dev, uint8_t id, uint8_t *speed_p, uint8_t *speed_i);
tue_sts3215_status_t tue_sts3215_set_overcurrent_time(tue_sts3215_t *dev, uint8_t id, uint8_t time_10ms);
tue_sts3215_status_t tue_sts3215_get_overcurrent_time(tue_sts3215_t *dev, uint8_t id, uint8_t *time_10ms);

tue_sts3215_status_t tue_sts3215_enable_torque(tue_sts3215_t *dev, uint8_t id, bool enable);
tue_sts3215_status_t tue_sts3215_calibrate_position_to_2048(tue_sts3215_t *dev, uint8_t id);
tue_sts3215_status_t tue_sts3215_get_torque_enable(tue_sts3215_t *dev, uint8_t id, bool *enabled);

tue_sts3215_status_t tue_sts3215_set_acceleration(tue_sts3215_t *dev, uint8_t id, uint8_t accel_100steps_s2);
tue_sts3215_status_t tue_sts3215_get_acceleration(tue_sts3215_t *dev, uint8_t id, uint8_t *accel_100steps_s2);
tue_sts3215_status_t tue_sts3215_set_goal_position(tue_sts3215_t *dev, uint8_t id, int16_t position);
tue_sts3215_status_t tue_sts3215_set_goal_position_ex(tue_sts3215_t *dev, uint8_t id, int16_t position,
                                                      int16_t speed, uint8_t accel);
tue_sts3215_status_t tue_sts3215_reg_write_goal_position_ex(tue_sts3215_t *dev, uint8_t id, int16_t position,
                                                            int16_t speed, uint8_t accel);
tue_sts3215_status_t tue_sts3215_set_pwm_runtime(tue_sts3215_t *dev, uint8_t id, int16_t pwm_per_thousand);
tue_sts3215_status_t tue_sts3215_get_pwm_runtime(tue_sts3215_t *dev, uint8_t id, int16_t *pwm_per_thousand);
tue_sts3215_status_t tue_sts3215_set_running_speed(tue_sts3215_t *dev, uint8_t id, int16_t speed_steps_s);
tue_sts3215_status_t tue_sts3215_get_running_speed(tue_sts3215_t *dev, uint8_t id, int16_t *speed_steps_s);
tue_sts3215_status_t tue_sts3215_set_torque_limit(tue_sts3215_t *dev, uint8_t id, uint16_t torque_0_1pct);
tue_sts3215_status_t tue_sts3215_get_torque_limit(tue_sts3215_t *dev, uint8_t id, uint16_t *torque_0_1pct);
tue_sts3215_status_t tue_sts3215_stop(tue_sts3215_t *dev, uint8_t id);

tue_sts3215_status_t tue_sts3215_get_current_position(tue_sts3215_t *dev, uint8_t id, int16_t *position);
tue_sts3215_status_t tue_sts3215_get_current_speed(tue_sts3215_t *dev, uint8_t id, int16_t *speed);
tue_sts3215_status_t tue_sts3215_get_current_load(tue_sts3215_t *dev, uint8_t id, int16_t *load_0_1pct);
tue_sts3215_status_t tue_sts3215_get_current_voltage(tue_sts3215_t *dev, uint8_t id, uint8_t *voltage_0_1v);
tue_sts3215_status_t tue_sts3215_get_voltage_volts(tue_sts3215_t *dev, uint8_t id, float *volts);
tue_sts3215_status_t tue_sts3215_get_current_temperature(tue_sts3215_t *dev, uint8_t id, int8_t *temp_c);
tue_sts3215_status_t tue_sts3215_get_async_write_flag(tue_sts3215_t *dev, uint8_t id, uint8_t *flag);
tue_sts3215_status_t tue_sts3215_get_servo_status(tue_sts3215_t *dev, uint8_t id, uint8_t *status_mask);
tue_sts3215_status_t tue_sts3215_is_moving(tue_sts3215_t *dev, uint8_t id, bool *is_moving);
tue_sts3215_status_t tue_sts3215_get_motor_current_raw(tue_sts3215_t *dev, uint8_t id, int16_t *current_6_5ma);
tue_sts3215_status_t tue_sts3215_get_motor_current_ma(tue_sts3215_t *dev, uint8_t id, float *current_ma);
tue_sts3215_status_t tue_sts3215_get_feedback(tue_sts3215_t *dev, uint8_t id, tue_sts3215_feedback_t *feedback);

/* Multi-servo synchronized commands -----------------------------------------*/
tue_sts3215_status_t tue_sts3215_sync_write_position(tue_sts3215_t *dev, const tue_sts3215_sync_pos_t *targets, uint8_t count);
tue_sts3215_status_t tue_sts3215_sync_write_speed(tue_sts3215_t *dev, const tue_sts3215_sync_speed_t *targets, uint8_t count);
tue_sts3215_status_t tue_sts3215_sync_write_torque_enable(tue_sts3215_t *dev, const uint8_t *ids, const uint8_t *enables, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* TUE_STS3215_H_ */
