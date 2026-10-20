/**
  ******************************************************************************
  * @file           : tue_usb_binary.h
  * @brief          : TUE binary CAN-over-USB protocol carried over USB CDC (VCP).
  ******************************************************************************
  * @attention
  *
  * This module replaces the legacy ASCII tue_acsi protocol with a compact,
  * deterministic binary frame protocol that mirrors a native CAN 2.0B frame.
  * Every transaction over the Virtual COM Port conforms to the following
  * fixed envelope (all multi-byte fields little-endian):
  *
  *   +--------+--------+------------+--------------+-----+-----+---------+------+------+------+
  *   | SOF1   | SOF2   | FRAME_TYPE | CAN_ID (4B)  | DLC | SEQ | PAYLOAD | CRC8 | EOF1 | EOF2 |
  *   | 0xAA   | 0x55   | uint8_t    | uint32_t LE  | u8  | u8  | 0..8 B  | J1850| 0x55 | 0xAA |
  *   +--------+--------+------------+--------------+-----+-----+---------+------+------+------+
  *
  *   Framing overhead is a fixed 12 bytes (minimum frame, DLC = 0) and the
  *   maximum frame is 20 bytes (DLC = 8). Integrity is protected by an
  *   SAE J1850 CRC-8 computed over bytes [FRAME_TYPE .. PAYLOAD] (7 + DLC bytes).
  *
  * CAN ID map (see TODO_NEW_USB_BINARY_PROTOCOL.md section 3):
  *   0x100-0x17F  System management
  *   0x200-0x27F  CoreXY + Z tool motion
  *   0x280-0x2FF  Raw per-motor stepping
  *   0x300-0x3FF  >>> RESERVED for user custom communication <<<
  *   0x300        Unified tactile haptic telemetry (forwarded to UART slave)
  *   0x400-0x4FF  STS3215 bus servos
  *   0x500-0x57F  Sensors / inputs (reserved)
  *   0x600-0x6FF  DC motors / actuators (reserved)
  *
  * CRITICAL: no internal command may ever use a CAN ID whose bits [11:8] are
  * 0x3 (i.e. 0x3.. ranges). This is enforced at compile time below.
  *
  ******************************************************************************
  */

#ifndef TUE_USB_BINARY_H_
#define TUE_USB_BINARY_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "tue_sts3215.h"
#include "tue_stepper.h"

/* Board identity ------------------------------------------------------------*/
#define TUE_USB_BOARD_NAME        "MM_Control"
#define TUE_USB_HW_ID             1U
#define TUE_USB_HW_REV            1U
#define TUE_USB_FW_VERSION_MAJOR  1U
#define TUE_USB_FW_VERSION_MINOR  0U
#define TUE_USB_BUILD_FLAGS       0U

/* Framing constants ---------------------------------------------------------*/
#define TUE_USB_SOF1              0xAAU
#define TUE_USB_SOF2              0x55U
#define TUE_USB_EOF1              0x55U
#define TUE_USB_EOF2              0xAAU

#define TUE_USB_HEADER_SIZE       7U    /* TYPE + ID(4) + DLC + SEQ            */
#define TUE_USB_CRC_SIZE          1U
#define TUE_USB_FRAME_OVERHEAD    12U   /* SOF(2)+hdr(7)+CRC(1)+EOF(2)         */
#define TUE_USB_MAX_DLC           8U
#define TUE_USB_MAX_FRAME         (TUE_USB_FRAME_OVERHEAD + TUE_USB_MAX_DLC)
#define TUE_USB_MIN_FRAME         TUE_USB_FRAME_OVERHEAD

/* Frame types ---------------------------------------------------------------*/
typedef enum {
    TUE_USB_FRAME_REQ        = 0x01U, /* Host -> MCU command / request         */
    TUE_USB_FRAME_RESP_OK    = 0x02U, /* MCU -> Host acknowledged execution    */
    TUE_USB_FRAME_RESP_ERR   = 0x03U, /* MCU -> Host error, payload[0] = code  */
    TUE_USB_FRAME_ASYNC      = 0x04U  /* MCU -> Host unsolicited event         */
} tue_usb_frame_type_t;

/* Status / error codes ------------------------------------------------------*/
typedef enum {
    TUE_USB_ERR_OK             = 0x00U,
    TUE_USB_ERR_CRC            = 0x01U,
    TUE_USB_ERR_BAD_FORMAT     = 0x02U,
    TUE_USB_ERR_UNKNOWN_CAN_ID = 0x03U,
    TUE_USB_ERR_BAD_ARG        = 0x04U,
    TUE_USB_ERR_BUS_TIMEOUT    = 0x05U,
    TUE_USB_ERR_BUS_FAULT      = 0x06U,
    TUE_USB_ERR_BUSY           = 0x07U,
    TUE_USB_ERR_SERVO_FAULT    = 0x08U,
    TUE_USB_ERR_QUEUE_FULL     = 0x09U,
    TUE_USB_ERR_TIMEOUT        = 0x0AU,
    TUE_USB_ERR_NOT_SUPPORTED  = 0x0BU,
    TUE_USB_ERR_INTERNAL       = 0x0CU
} tue_usb_error_t;

/* CAN ID allocation ---------------------------------------------------------*/

/* --- System management (0x100 - 0x107) ------------------------------------ */
#define TUE_USB_CAN_SYS_PING        0x100U
#define TUE_USB_CAN_SYS_INFO        0x101U
#define TUE_USB_CAN_SYS_UID_PART1   0x102U
#define TUE_USB_CAN_SYS_UID_PART2   0x103U
#define TUE_USB_CAN_SYS_UPTIME      0x104U
#define TUE_USB_CAN_SYS_LED_CTRL    0x105U
#define TUE_USB_CAN_SYS_RESET       0x106U
#define TUE_USB_CAN_SYS_ECHO        0x107U

/* --- CoreXY + Z tool motion (0x200 - 0x21A) ------------------------------- */
#define TUE_USB_CAN_STEP_GET_INFO    0x200U
#define TUE_USB_CAN_STEP_GET_STATUS  0x201U
#define TUE_USB_CAN_STEP_ENABLE      0x202U
#define TUE_USB_CAN_STEP_GET_POS_XY  0x203U
#define TUE_USB_CAN_STEP_GET_POS_Z   0x204U
#define TUE_USB_CAN_STEP_GET_TGT_XY  0x205U
#define TUE_USB_CAN_STEP_GET_TGT_Z   0x206U
#define TUE_USB_CAN_STEP_SETPOS_XY   0x207U
#define TUE_USB_CAN_STEP_SETPOS_Z    0x208U
#define TUE_USB_CAN_STEP_MOVE_COORDS 0x210U
#define TUE_USB_CAN_STEP_MOVE_EXEC   0x211U
#define TUE_USB_CAN_STEP_MOVE_XY     0x212U
#define TUE_USB_CAN_STEP_MOVE_Z      0x213U
#define TUE_USB_CAN_STEP_MOVE_REL_XY 0x214U
#define TUE_USB_CAN_STEP_SET_SPEED   0x215U
#define TUE_USB_CAN_STEP_JOG_XY      0x216U
#define TUE_USB_CAN_STEP_JOG_Z       0x217U
#define TUE_USB_CAN_STEP_STOP        0x218U
#define TUE_USB_CAN_STEP_ABORT       0x219U
#define TUE_USB_CAN_STEP_WAIT_IDLE   0x21AU

/* --- Raw per-motor stepping (0x280 - 0x281) ------------------------------- */
#define TUE_USB_CAN_STEP_RAW_GET_STEPS 0x280U
#define TUE_USB_CAN_STEP_RAW_MOVE      0x281U

/* --- STS3215 bus servos (0x400 - 0x415) ----------------------------------- */
#define TUE_USB_CAN_SERVO_PING       0x400U
#define TUE_USB_CAN_SERVO_SET_POS    0x401U
#define TUE_USB_CAN_SERVO_GET_POS    0x402U
#define TUE_USB_CAN_SERVO_SET_SPEED  0x403U
#define TUE_USB_CAN_SERVO_GET_SPEED  0x404U
#define TUE_USB_CAN_SERVO_TORQUE     0x405U
#define TUE_USB_CAN_SERVO_TLIM       0x406U
#define TUE_USB_CAN_SERVO_MODE       0x407U
#define TUE_USB_CAN_SERVO_PWM        0x408U
#define TUE_USB_CAN_SERVO_GET_FB     0x410U
#define TUE_USB_CAN_SERVO_FB_DYNAMICS 0x410U
#define TUE_USB_CAN_SERVO_FB_HEALTH  0x411U
#define TUE_USB_CAN_SERVO_MEM_READ   0x412U
#define TUE_USB_CAN_SERVO_MEM_WRITE  0x413U
#define TUE_USB_CAN_SERVO_SET_ID     0x414U
#define TUE_USB_CAN_SERVO_RESET      0x415U

/* --- Reserved for user custom communication (MUST NOT be used internally) - */
#define TUE_USB_CAN_RESERVED_BASE    0x300U
#define TUE_USB_CAN_RESERVED_LAST    0x3FFU

/* --- User custom stream allocated inside the reserved range -------------- *
 * The host streams the unified 20-byte haptic frame (DLC = 8) on 0x300 at
 * 100 Hz. It is cached locally and forwarded verbatim to the ESP32 slave on
 * USART2 by the haptic bridge (Core/Src/tue_haptic_bridge.c). This is the
 * only ID in the 0x3.. range the firmware interprets. */
#define TUE_USB_CAN_HAPTIC_TELEMETRY 0x300U

/* Reserved-range guard ------------------------------------------------------*/
/* True for the reserved user-custom ranges: standard 0x300-0x3FF, 16-bit
 * 0x3000-0x3FFF and extended 0x30000000-0x3FFFFFFF. */
#define TUE_USB_CAN_ID_IS_RESERVED(id)                                     \
    ( ((((uint32_t)(id)) >= 0x300U) && (((uint32_t)(id)) <= 0x3FFU)) ||    \
      ((((uint32_t)(id)) >= 0x3000U) && (((uint32_t)(id)) <= 0x3FFFU)) ||  \
      ((((uint32_t)(id)) >= 0x30000000U) && (((uint32_t)(id)) <= 0x3FFFFFFFU)) )

#define TUE_USB_ASSERT_NOT_RESERVED(id) \
    _Static_assert(!TUE_USB_CAN_ID_IS_RESERVED(id), \
                   "CAN ID " #id " collides with the reserved 0x3.. range")

/* Compile-time proof that no allocated command uses the reserved range. */
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SYS_PING);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SYS_INFO);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SYS_UID_PART1);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SYS_UID_PART2);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SYS_UPTIME);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SYS_LED_CTRL);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SYS_RESET);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SYS_ECHO);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_GET_INFO);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_GET_STATUS);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_ENABLE);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_GET_POS_XY);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_GET_POS_Z);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_GET_TGT_XY);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_GET_TGT_Z);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_SETPOS_XY);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_SETPOS_Z);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_MOVE_COORDS);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_MOVE_EXEC);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_MOVE_XY);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_MOVE_Z);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_MOVE_REL_XY);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_SET_SPEED);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_JOG_XY);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_JOG_Z);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_STOP);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_ABORT);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_WAIT_IDLE);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_RAW_GET_STEPS);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_STEP_RAW_MOVE);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_PING);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_SET_POS);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_GET_POS);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_SET_SPEED);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_GET_SPEED);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_TORQUE);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_TLIM);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_MODE);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_PWM);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_GET_FB);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_FB_HEALTH);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_MEM_READ);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_MEM_WRITE);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_SET_ID);
TUE_USB_ASSERT_NOT_RESERVED(TUE_USB_CAN_SERVO_RESET);

/* Decoded frame -------------------------------------------------------------*/
typedef struct __attribute__((packed)) {
    uint8_t  frame_type;
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  seq;
    uint8_t  payload[8];
    uint8_t  crc8;
} tue_usb_frame_t;

/* Lifecycle -----------------------------------------------------------------*/

/**
  * @brief  Initialise the binary USB protocol: brings up the STS3215 servo bus
  *         on USART1 (idempotent stepper init) and starts the parser task.
  *         Call once after the RTOS scheduler is running.
  */
void tue_usb_init(void);

/**
  * @brief  Feed raw USB CDC bytes into the protocol parser.
  *         Call from CDC_Receive_FS() - runs in interrupt context and only
  *         buffers the bytes; parsing/execution happens in the protocol task.
  */
void tue_usb_on_rx(const uint8_t *data, uint32_t len);

/**
  * @brief  Called from CDC_TransmitCplt_FS() when a USB IN packet has been transmitted.
  */
void tue_usb_on_tx_cplt(void);

/**
  * @brief  Access the Feetech STS3215 bus instance used by the SERVO commands,
  *         so application code can share the same bus.
  */
tue_sts3215_t *tue_usb_servo(void);

/**
  * @brief  SAE J1850 CRC-8 (poly 0x1D, init 0xFF, xorout 0xFF, no reflection).
  */
uint8_t tue_usb_crc8(const uint8_t *data, size_t len);

/** Serialize and transmit a RESP_OK frame: [SOF][TYPE=RESP_OK][ID][DLC][SEQ][payload][CRC][EOF]. */
void tue_usb_send_ok(uint32_t can_id, uint8_t seq, const uint8_t *payload, uint8_t dlc);

/** Serialize and transmit a RESP_ERR frame with a single status byte. */
void tue_usb_send_err(uint32_t can_id, uint8_t seq, tue_usb_error_t err_code);

/** Serialize and transmit an asynchronous event frame. */
void tue_usb_send_async(uint32_t can_id, uint8_t seq, const uint8_t *payload, uint8_t dlc);

/**
  * @brief  Validate the SAE J1850 CRC-8 implementation against its reference
  *         vectors. Exposed for host-side unit tests.
  * @retval true when all vectors match.
  */
bool tue_usb_crc8_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* TUE_USB_BINARY_H_ */
