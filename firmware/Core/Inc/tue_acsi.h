/**
  ******************************************************************************
  * @file           : tue_acsi.h
  * @brief          : TUE ACSI command protocol carried over USB CDC (VCP).
  ******************************************************************************
  * @attention
  *
  * ACSI is a numeric, field-oriented command protocol carried as whitespace-
  * separated tokens. Header fields are decimal numbers; payload values are
  * numbers, except that a response may carry literal text (e.g. the board name
  * in SYS INFO). There is no checksum field.
  *
  *   <TARGET> <ID> <DIR> <FIELD> [ARG ...]
  *
  *   TARGET : device class code     -> 1 SERVO | 2 STEP | 3 DC | 4 SYS
  *   ID     : actuator / device id  (which servo on the bus, ...)
  *   DIR    : direction code        -> 0 GET (read in) | 1 SET (write out)
  *   FIELD  : control field code    (see the enums below)
  *   ARG    : one or more numbers   (usually the value to write)
  *
  * Replies reuse the same grammar, with the result code where DIR was:
  *
  *   <TARGET> <ID> <CODE> [VALUE ...]
  *
  *   CODE   : 0 = OK, otherwise an error code (see tue_acsi_code_t)
  *
  * Text payloads (used e.g. by SYS INFO) are sent as literal whitespace-
  * separated tokens, e.g. "MM_Control 1.0".
  *
  * Examples:
  *   1 1 1 2 2048   -> servo 1, SET, POS = 2048
  *   1 1 0 2 2048   -> servo 1, OK, position = 2048
  *
  * See docs/ACSI_PROTOCOL.md for the complete specification.
  *
  ******************************************************************************
  */

#ifndef TUE_ACSI_H_
#define TUE_ACSI_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "tue_sts3215.h"
#include "tue_stepper.h"

/* Board identity (sent as text-as-numbers by SYS INFO) ----------------------*/
#define TUE_ACSI_BOARD_NAME       "MM_Control"
#define TUE_ACSI_FW_VERSION_MAJOR 1U
#define TUE_ACSI_FW_VERSION_MINOR 0U

/* Frame limits --------------------------------------------------------------*/
#define TUE_ACSI_MAX_LINE         160U  /* longest accepted command line (chars) */
#define TUE_ACSI_MAX_TOKENS       32U   /* max whitespace separated numbers      */

/* TARGET codes --------------------------------------------------------------*/
typedef enum {
    TUE_ACSI_TARGET_SERVO = 1,  /* Feetech STS3215 serial bus servo */
    TUE_ACSI_TARGET_STEP  = 2,  /* stepper motors (tue_stepper)     */
    TUE_ACSI_TARGET_DC    = 3,  /* DC motors (reserved)             */
    TUE_ACSI_TARGET_SYS   = 4   /* board system / debug             */
} tue_acsi_target_t;

/* DIR codes -----------------------------------------------------------------*/
typedef enum {
    TUE_ACSI_DIR_GET = 0,  /* read data in  */
    TUE_ACSI_DIR_SET = 1   /* write data out */
} tue_acsi_dir_t;

/* SERVO field codes (register map in docs/MEMORY_TABLE.md) ------------------*/
typedef enum {
    TUE_ACSI_SERVO_PING   = 1,   /* GET  -> <id>                              */
    TUE_ACSI_SERVO_POS    = 2,   /* GET current / SET goal position (steps)   */
    TUE_ACSI_SERVO_VEL    = 3,   /* GET current / SET goal speed (steps/s)    */
    TUE_ACSI_SERVO_ACC    = 4,   /* GET/SET acceleration                      */
    TUE_ACSI_SERVO_TORQUE = 5,   /* GET/SET torque enable (0/1/128)           */
    TUE_ACSI_SERVO_TLIM   = 6,   /* GET/SET torque limit                      */
    TUE_ACSI_SERVO_MODE   = 7,   /* GET/SET operating mode (0..3)             */
    TUE_ACSI_SERVO_PWM    = 8,   /* GET/SET open-loop runtime (-1000..1000)   */
    TUE_ACSI_SERVO_MEM    = 9,   /* GET <addr> [len] / SET <addr> <b0> [b1..] */
    TUE_ACSI_SERVO_FB     = 10,  /* GET full telemetry burst                  */
    TUE_ACSI_SERVO_ID     = 11,  /* GET/SET servo id                          */
    TUE_ACSI_SERVO_BAUD   = 12,  /* GET/SET baud index (0..7)                 */
    TUE_ACSI_SERVO_LIMIT  = 13,  /* GET min/max / SET <min> <max>             */
    TUE_ACSI_SERVO_TEMP   = 14,  /* GET temperature (degC)                    */
    TUE_ACSI_SERVO_VOLT   = 15,  /* GET voltage (0.1 V)                       */
    TUE_ACSI_SERVO_LOAD   = 16,  /* GET load (0.1 %)                          */
    TUE_ACSI_SERVO_CUR    = 17,  /* GET current (6.5 mA units)                */
    TUE_ACSI_SERVO_STATUS = 18,  /* GET status bitmask                        */
    TUE_ACSI_SERVO_MOVING = 19,  /* GET 0/1                                   */
    TUE_ACSI_SERVO_RESET  = 20   /* SET factory reset                         */
} tue_acsi_servo_field_t;

/* STEP field codes (the tue_stepper library: CoreXY motors 1+2, Z motor 3) --*/
/*
 * UNIT CONVENTION - the wire stays integer-only, so all distances are
 * micrometres (1 um = 0.001 mm), speeds um/s and accelerations um/s^2.
 *   50.0 mm     -> 50000       60 mm/s  -> 60000       600 mm/s^2 -> 600000
 *
 * ID selects what the command acts on:
 *   0 = the tool / machine  (X, Y, Z in CoreXY space)  - use for MOVE/JOG/...
 *   1 = motor 1 (A = x + y)
 *   2 = motor 2 (B = x - y)
 *   3 = motor 3 (Z)
 *
 * See docs/STEPPER_INSTRUCTION.md for the mechanics and geometry.
 */
typedef enum {
    TUE_ACSI_STEP_INFO    = 1,  /* GET  -> <xy_spmm> <z_spmm> <min_rate> <max_rate> */
    TUE_ACSI_STEP_POS     = 2,  /* GET  -> <x> <y> <z>  current tool position (um)  */
    TUE_ACSI_STEP_TARGET  = 3,  /* GET  -> <x> <y> <z>  end of the queued motion    */
    TUE_ACSI_STEP_STATUS  = 4,  /* GET  -> <busy> <enabled>                         */
    TUE_ACSI_STEP_ENABLE  = 5,  /* GET 0/1 | SET <0/1>  driver enable line          */
    TUE_ACSI_STEP_MOVE    = 6,  /* SET  <x> <y> <z> <feed> <accel>  absolute XYZ    */
    TUE_ACSI_STEP_MOVE2D  = 7,  /* SET  <x> <y> <feed> <accel>      absolute XY     */
    TUE_ACSI_STEP_MOVEZ   = 8,  /* SET  <z> <feed> <accel>          absolute Z      */
    TUE_ACSI_STEP_MOVEREL = 9,  /* SET  <dx> <dy> <feed> <accel>    relative XY     */
    TUE_ACSI_STEP_JOG     = 10, /* SET  <vx> <vy> <accel>           continuous XY   */
    TUE_ACSI_STEP_JOGZ    = 11, /* SET  <vz> <accel>                continuous Z    */
    TUE_ACSI_STEP_STOP    = 12, /* SET  -> controlled stop (ramp down)              */
    TUE_ACSI_STEP_ABORT   = 13, /* SET  -> immediate stop, drop the queue           */
    TUE_ACSI_STEP_SETPOS  = 14, /* SET  <x> <y> <z>  redefine the current position   */
    TUE_ACSI_STEP_STEPS   = 15, /* GET  id 1..3 -> <signed steps>                   */
    TUE_ACSI_STEP_RAW     = 16, /* SET  id 1..3 <steps> <feed> <accel>  raw motor    */
    TUE_ACSI_STEP_WAIT    = 17  /* SET  <timeout_ms> -> <elapsed_ms>                */
} tue_acsi_step_field_t;

/* SYS field codes -----------------------------------------------------------*/
typedef enum {
    TUE_ACSI_SYS_PING   = 1,  /* GET  -> 1 (liveness)                          */
    TUE_ACSI_SYS_ECHO   = 2,  /* SET <n> -> <n> (link integrity test)          */
    TUE_ACSI_SYS_INFO   = 3,  /* GET  -> <board> <fw>  (literal text)          */
    TUE_ACSI_SYS_UID    = 4,  /* GET  -> <uid0> <uid1> <uid2> (decimal words)  */
    TUE_ACSI_SYS_UPTIME = 5,  /* GET  -> milliseconds since boot               */
    TUE_ACSI_SYS_LED    = 6,  /* GET 0/1 / SET <index> <state>                 */
    TUE_ACSI_SYS_RESET  = 7   /* SET reboot the MCU                            */
} tue_acsi_sys_field_t;

/* Result / error codes (first payload number of a response) -----------------*/
typedef enum {
    TUE_ACSI_OK                 = 0,  /* success                               */
    TUE_ACSI_ERR_BAD_FORMAT     = 2,  /* malformed / incomplete frame          */
    TUE_ACSI_ERR_UNKNOWN_TARGET = 3,  /* unknown TARGET code                   */
    TUE_ACSI_ERR_UNKNOWN_FIELD  = 4,  /* unknown FIELD code for this target    */
    TUE_ACSI_ERR_BAD_ARG        = 5,  /* missing / out-of-range argument       */
    TUE_ACSI_ERR_BUS_TIMEOUT    = 6,  /* servo did not answer on the bus       */
    TUE_ACSI_ERR_BUS_ERROR      = 7,  /* servo bus framing / HAL error         */
    TUE_ACSI_ERR_NOT_SUPPORTED  = 8,  /* field/dir not implemented yet         */
    TUE_ACSI_ERR_BUSY           = 9,  /* servo bus busy                        */
    TUE_ACSI_ERR_SERVO_FAULT    = 10, /* servo reported a hardware error bit   */
    TUE_ACSI_ERR_INTERNAL       = 11, /* internal driver error                 */
    TUE_ACSI_ERR_QUEUE_FULL     = 12, /* motion queue saturated (STEP)         */
    TUE_ACSI_ERR_TIMEOUT        = 13  /* wait deadline expired (STEP)          */
} tue_acsi_code_t;

/* Lifecycle -----------------------------------------------------------------*/

/**
  * @brief  Initialise the ACSI USB protocol: brings up the servo bus on USART1
  *         and starts the command processing task. Call once after the RTOS
  *         scheduler is running (e.g. from the default task).
  */
void tue_acsi_init(void);

/**
  * @brief  Feed raw USB CDC bytes into the protocol parser.
  *         Call from CDC_Receive_FS() - runs in interrupt context and only
  *         buffers the bytes; parsing/execution happens in the ACSI task.
  */
void tue_acsi_on_cdc_rx(const uint8_t *data, uint32_t len);

/**
  * @brief  Access the Feetech STS3215 bus instance used by the SERVO target,
  *         so application code can share the same bus.
  */
tue_sts3215_t *tue_acsi_servo(void);

#ifdef __cplusplus
}
#endif

#endif /* TUE_ACSI_H_ */
