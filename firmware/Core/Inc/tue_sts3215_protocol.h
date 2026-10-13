/**
  ******************************************************************************
  * @file    tue_sts3215_protocol.h
  * @brief   STS3215 pure-C protocol layer (packet builders + reply parser).
  ******************************************************************************
  * Ported from the reference C library (reference_for_sts325/stm32-sts3215-lib).
  * This layer has zero STM32 dependency: it only touches tue_sts3215_regs.h and
  * the C standard library, so it can be unit-tested on a host machine.
  ******************************************************************************
  */

#ifndef TUE_STS3215_PROTOCOL_H
#define TUE_STS3215_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "tue_sts3215_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Buffer sizing
 * ========================================================================= */

/** Maximum servos addressed in a single SYNC frame. */
#define STS3215_SYNC_MAX_SERVOS     (16U)

/** Maximum data bytes per servo in a SYNC_WRITE. */
#define STS3215_SYNC_DATA_MAX_LEN   (8U)

/**
 * Universal TX buffer size (sized for worst-case SYNC_WRITE):
 *   header(2) + id(1) + len(1) + instr(1) + reg(1) + dlen(1)
 *   + N * (id(1) + data(L)) + checksum(1)
 */
#define STS3215_TX_BUF_SIZE (7U + STS3215_SYNC_MAX_SERVOS * (1U + STS3215_SYNC_DATA_MAX_LEN))

/** RX buffer for a single maximal reply frame. */
#define STS3215_RX_BUF_SIZE (STS3215_REPLY_OVERHEAD + STS3215_REPLY_DATA_MAX)

/* =========================================================================
 * Status / error codes (published by builders and parser)
 * ========================================================================= */

typedef enum {
    TUE_STS3215_PROTO_OK                = 0,
    TUE_STS3215_PROTO_ERR_NULL_PTR      = -1,
    TUE_STS3215_PROTO_ERR_INVALID_PARAM = -2,
    TUE_STS3215_PROTO_ERR_BUFFER_SMALL  = -3,
    TUE_STS3215_PROTO_ERR_BAD_HEADER    = -4,
    TUE_STS3215_PROTO_ERR_BAD_CHECKSUM  = -5,
    TUE_STS3215_PROTO_ERR_SERVO_FAULT   = -6,
    TUE_STS3215_PROTO_ERR_FRAME_SHORT   = -7
} tue_sts3215_proto_status_t;

/* =========================================================================
 * Data structures
 * ========================================================================= */

/** Parsed representation of a servo reply packet. */
typedef struct {
    uint8_t id;
    uint8_t error;
    uint8_t data[STS3215_REPLY_DATA_MAX];
    uint8_t data_len;
} tue_sts3215_reply_t;

/** Atomic motion command parameters (WRITE / REG_WRITE). */
typedef struct {
    uint8_t  acceleration;
    int16_t  target_pos;
    uint16_t running_time;
    uint16_t running_speed;
} tue_sts3215_motion_cmd_t;

/** One servo entry for SYNC_WRITE operations. */
typedef struct {
    uint8_t id;
    uint8_t data[STS3215_SYNC_DATA_MAX_LEN];
} tue_sts3215_sync_entry_t;

/* =========================================================================
 * Packet builders
 *
 * Every builder returns the total frame length (> 0) on success, or a
 * tue_sts3215_proto_status_t value (< 0) on error.
 * ========================================================================= */

int16_t tue_sts3215_build_ping(uint8_t *buf, uint16_t cap, uint8_t id);

int16_t tue_sts3215_build_read(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, uint8_t len);

int16_t tue_sts3215_build_write1b(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, uint8_t val);

int16_t tue_sts3215_build_write2b(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, uint16_t val);

int16_t tue_sts3215_build_write_raw(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg,
                                    const uint8_t *data, uint8_t data_len);

int16_t tue_sts3215_build_motion_cmd(uint8_t *buf, uint16_t cap, uint8_t id,
                                     const tue_sts3215_motion_cmd_t *cmd);

int16_t tue_sts3215_build_reg_write(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg,
                                    const uint8_t *data, uint8_t data_len);

int16_t tue_sts3215_build_reg_write_motion_cmd(uint8_t *buf, uint16_t cap, uint8_t id,
                                               const tue_sts3215_motion_cmd_t *cmd);

int16_t tue_sts3215_build_action(uint8_t *buf, uint16_t cap, uint8_t id);

int16_t tue_sts3215_build_sync_write(uint8_t *buf, uint16_t cap, uint8_t reg, uint8_t data_len,
                                     const tue_sts3215_sync_entry_t *entries, uint8_t n);

int16_t tue_sts3215_build_sync_read(uint8_t *buf, uint16_t cap, uint8_t reg, uint8_t data_len,
                                    const uint8_t *ids, uint8_t n);

int16_t tue_sts3215_build_enable_torque(uint8_t *buf, uint16_t cap, uint8_t id);
int16_t tue_sts3215_build_disable_torque(uint8_t *buf, uint16_t cap, uint8_t id);
int16_t tue_sts3215_build_center_servo(uint8_t *buf, uint16_t cap, uint8_t id);

int16_t tue_sts3215_build_unlock_eeprom(uint8_t *buf, uint16_t cap, uint8_t id);
int16_t tue_sts3215_build_lock_eeprom(uint8_t *buf, uint16_t cap, uint8_t id);

int16_t tue_sts3215_build_reset(uint8_t *buf, uint16_t cap, uint8_t id);
int16_t tue_sts3215_build_reset_turns(uint8_t *buf, uint16_t cap, uint8_t id);

/* =========================================================================
 * Reply parser
 * ========================================================================= */

/**
  * @brief  Parse a raw RX buffer into a tue_sts3215_reply_t.
  *
  * Validates header, frame length consistency and checksum. On
  * TUE_STS3215_PROTO_ERR_SERVO_FAULT the reply struct is still fully populated.
  */
tue_sts3215_proto_status_t tue_sts3215_parse_reply(const uint8_t *buf, uint16_t buf_len,
                                                    tue_sts3215_reply_t *reply);

/* =========================================================================
 * Utility / unit conversion
 * ========================================================================= */

uint16_t tue_sts3215_unpack_u16_le(const uint8_t *buf);
int16_t  tue_sts3215_unpack_s16_le(const uint8_t *buf);
int16_t  tue_sts3215_unpack_signmag16_le(const uint8_t *buf, uint8_t sign_bit);

float   tue_sts3215_steps_to_deg(int16_t steps);
float   tue_sts3215_steps_to_rad(int16_t steps);
int16_t tue_sts3215_deg_to_steps(float deg);
float   tue_sts3215_raw_to_milliamps(uint16_t raw);
float   tue_sts3215_raw_to_volts(uint8_t raw);

#ifdef __cplusplus
}
#endif

#endif /* TUE_STS3215_PROTOCOL_H */
