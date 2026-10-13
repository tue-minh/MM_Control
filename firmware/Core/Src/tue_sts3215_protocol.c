/**
  ******************************************************************************
  * @file    tue_sts3215_protocol.c
  * @brief   STS3215 pure-C protocol layer implementation.
  ******************************************************************************
  * Ported from the reference C library (reference_for_sts325/stm32-sts3215-lib).
  ******************************************************************************
  */

#include "tue_sts3215_protocol.h"
#include <string.h>

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

static uint8_t prv_checksum(uint8_t id, uint8_t length, uint8_t instr,
                            const uint8_t *params, uint8_t n_params)
{
    uint32_t sum = (uint32_t)id + (uint32_t)length + (uint32_t)instr;
    for (uint8_t i = 0U; i < n_params; i++) {
        sum += (uint32_t)params[i];
    }
    return (uint8_t)(~sum & 0xFFU);
}

static uint16_t prv_pack_frame(uint8_t *buf, uint8_t id, uint8_t instr,
                               const uint8_t *params, uint8_t n_params)
{
    uint8_t length   = (uint8_t)(n_params + 2U); /* instr byte + checksum byte */
    uint8_t checksum = prv_checksum(id, length, instr, params, n_params);

    buf[0] = STS3215_HEADER_BYTE;
    buf[1] = STS3215_HEADER_BYTE;
    buf[2] = id;
    buf[3] = length;
    buf[4] = instr;

    if ((n_params > 0U) && (params != NULL)) {
        memcpy(&buf[5], params, n_params);
    }
    buf[5U + n_params] = checksum;

    return (uint16_t)(6U + n_params);
}

static bool prv_cap_ok(uint16_t cap, uint8_t n_params)
{
    return cap >= (uint16_t)(6U + (uint16_t)n_params);
}

/* =========================================================================
 * Packet builders - single instruction
 * ========================================================================= */

int16_t tue_sts3215_build_ping(uint8_t *buf, uint16_t cap, uint8_t id)
{
    if (buf == NULL)           { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 0U))  { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_PING, NULL, 0U);
}

int16_t tue_sts3215_build_read(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, uint8_t len)
{
    if (buf == NULL)           { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (len == 0U)             { return TUE_STS3215_PROTO_ERR_INVALID_PARAM; }
    if (!prv_cap_ok(cap, 2U))  { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    uint8_t params[2] = { reg, len };
    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_READ_DATA, params, 2U);
}

int16_t tue_sts3215_build_write1b(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, uint8_t val)
{
    if (buf == NULL)           { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 2U))  { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    uint8_t params[2] = { reg, val };
    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, 2U);
}

int16_t tue_sts3215_build_write2b(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, uint16_t val)
{
    if (buf == NULL)           { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 3U))  { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    uint8_t params[3] = {
        reg,
        (uint8_t)(val & 0xFFU),
        (uint8_t)((val >> 8U) & 0xFFU)
    };
    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, 3U);
}

int16_t tue_sts3215_build_write_raw(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg,
                                    const uint8_t *data, uint8_t data_len)
{
    if (buf == NULL || data == NULL) { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (data_len == 0U)              { return TUE_STS3215_PROTO_ERR_INVALID_PARAM; }

    uint8_t n_params = (uint8_t)(data_len + 1U);

    if (data_len > (uint8_t)STS3215_PARAM_MAX) { return TUE_STS3215_PROTO_ERR_INVALID_PARAM; }
    if (!prv_cap_ok(cap, n_params))            { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    uint8_t params[STS3215_PARAM_MAX + 1U];
    params[0] = reg;
    memcpy(&params[1], data, data_len);

    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, n_params);
}

int16_t tue_sts3215_build_motion_cmd(uint8_t *buf, uint16_t cap, uint8_t id,
                                     const tue_sts3215_motion_cmd_t *cmd)
{
    if (buf == NULL || cmd == NULL) { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 8U))       { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    uint16_t pos = (uint16_t)cmd->target_pos;

    uint8_t params[8] = {
        STS3215_REG_ACCELERATION,
        cmd->acceleration,
        (uint8_t)(pos & 0xFFU),
        (uint8_t)((pos >> 8U) & 0xFFU),
        (uint8_t)(cmd->running_time & 0xFFU),
        (uint8_t)((cmd->running_time >> 8U) & 0xFFU),
        (uint8_t)(cmd->running_speed & 0xFFU),
        (uint8_t)((cmd->running_speed >> 8U) & 0xFFU)
    };
    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, 8U);
}

int16_t tue_sts3215_build_reg_write(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg,
                                    const uint8_t *data, uint8_t data_len)
{
    if (buf == NULL || data == NULL) { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (data_len == 0U)              { return TUE_STS3215_PROTO_ERR_INVALID_PARAM; }
    if (data_len > (uint8_t)STS3215_PARAM_MAX) { return TUE_STS3215_PROTO_ERR_INVALID_PARAM; }

    uint8_t n_params = (uint8_t)(data_len + 1U);
    if (!prv_cap_ok(cap, n_params))  { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    uint8_t params[STS3215_PARAM_MAX + 1U];
    params[0] = reg;
    memcpy(&params[1], data, data_len);

    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_REG_WRITE_DATA, params, n_params);
}

int16_t tue_sts3215_build_reg_write_motion_cmd(uint8_t *buf, uint16_t cap, uint8_t id,
                                               const tue_sts3215_motion_cmd_t *cmd)
{
    if (buf == NULL || cmd == NULL) { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 8U))       { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    uint16_t pos = (uint16_t)cmd->target_pos;

    uint8_t params[8] = {
        STS3215_REG_ACCELERATION,
        cmd->acceleration,
        (uint8_t)(pos & 0xFFU),
        (uint8_t)((pos >> 8U) & 0xFFU),
        (uint8_t)(cmd->running_time & 0xFFU),
        (uint8_t)((cmd->running_time >> 8U) & 0xFFU),
        (uint8_t)(cmd->running_speed & 0xFFU),
        (uint8_t)((cmd->running_speed >> 8U) & 0xFFU)
    };
    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_REG_WRITE_DATA, params, 8U);
}

int16_t tue_sts3215_build_action(uint8_t *buf, uint16_t cap, uint8_t id)
{
    if (buf == NULL)           { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 0U))  { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_ACTION, NULL, 0U);
}

int16_t tue_sts3215_build_sync_write(uint8_t *buf, uint16_t cap, uint8_t reg, uint8_t data_len,
                                     const tue_sts3215_sync_entry_t *entries, uint8_t n)
{
    if (buf == NULL || entries == NULL)              { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (n == 0U || n > STS3215_SYNC_MAX_SERVOS)      { return TUE_STS3215_PROTO_ERR_INVALID_PARAM; }
    if (data_len == 0U || data_len > STS3215_SYNC_DATA_MAX_LEN) {
        return TUE_STS3215_PROTO_ERR_INVALID_PARAM;
    }

    uint8_t n_params = (uint8_t)(2U + (uint8_t)(n * (1U + data_len)));
    if (!prv_cap_ok(cap, n_params)) { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    uint8_t params[2U + STS3215_SYNC_MAX_SERVOS * (1U + STS3215_SYNC_DATA_MAX_LEN)];

    params[0] = reg;
    params[1] = data_len;

    uint8_t idx = 2U;
    for (uint8_t i = 0U; i < n; i++) {
        params[idx++] = entries[i].id;
        memcpy(&params[idx], entries[i].data, data_len);
        idx = (uint8_t)(idx + data_len);
    }

    return (int16_t)prv_pack_frame(buf, STS3215_BROADCAST_ID,
                                   STS3215_INSTR_SYNC_WRITE_DATA, params, n_params);
}

int16_t tue_sts3215_build_sync_read(uint8_t *buf, uint16_t cap, uint8_t reg, uint8_t data_len,
                                    const uint8_t *ids, uint8_t n)
{
    if (buf == NULL || ids == NULL)             { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (n == 0U || n > STS3215_SYNC_MAX_SERVOS) { return TUE_STS3215_PROTO_ERR_INVALID_PARAM; }
    if (data_len == 0U)                         { return TUE_STS3215_PROTO_ERR_INVALID_PARAM; }

    uint8_t n_params = (uint8_t)(2U + n);
    if (!prv_cap_ok(cap, n_params)) { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    uint8_t params[2U + STS3215_SYNC_MAX_SERVOS];
    params[0] = reg;
    params[1] = data_len;
    memcpy(&params[2], ids, n);

    return (int16_t)prv_pack_frame(buf, STS3215_BROADCAST_ID,
                                   STS3215_INSTR_SYNC_READ_DATA, params, n_params);
}

/* =========================================================================
 * Packet builders - control helpers
 * ========================================================================= */

int16_t tue_sts3215_build_enable_torque(uint8_t *buf, uint16_t cap, uint8_t id)
{
    return tue_sts3215_build_write1b(buf, cap, id, STS3215_REG_TORQUE_SWITCH,
                                     (uint8_t)STS3215_TORQUE_ON);
}

int16_t tue_sts3215_build_disable_torque(uint8_t *buf, uint16_t cap, uint8_t id)
{
    return tue_sts3215_build_write1b(buf, cap, id, STS3215_REG_TORQUE_SWITCH,
                                     (uint8_t)STS3215_TORQUE_OFF);
}

int16_t tue_sts3215_build_center_servo(uint8_t *buf, uint16_t cap, uint8_t id)
{
    return tue_sts3215_build_write1b(buf, cap, id, STS3215_REG_TORQUE_SWITCH,
                                     (uint8_t)STS3215_TORQUE_CENTER);
}

int16_t tue_sts3215_build_unlock_eeprom(uint8_t *buf, uint16_t cap, uint8_t id)
{
    return tue_sts3215_build_write1b(buf, cap, id, STS3215_REG_LOCK_MARK,
                                     (uint8_t)STS3215_LOCK_OPEN);
}

int16_t tue_sts3215_build_lock_eeprom(uint8_t *buf, uint16_t cap, uint8_t id)
{
    return tue_sts3215_build_write1b(buf, cap, id, STS3215_REG_LOCK_MARK,
                                     (uint8_t)STS3215_LOCK_CLOSED);
}

int16_t tue_sts3215_build_reset(uint8_t *buf, uint16_t cap, uint8_t id)
{
    if (buf == NULL)           { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 0U))  { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_RESET, NULL, 0U);
}

int16_t tue_sts3215_build_reset_turns(uint8_t *buf, uint16_t cap, uint8_t id)
{
    if (buf == NULL)           { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 0U))  { return TUE_STS3215_PROTO_ERR_BUFFER_SMALL; }

    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_RESET_TURNS, NULL, 0U);
}

/* =========================================================================
 * Reply parser
 * ========================================================================= */

tue_sts3215_proto_status_t tue_sts3215_parse_reply(const uint8_t *buf, uint16_t buf_len,
                                                    tue_sts3215_reply_t *reply)
{
    if (buf == NULL || reply == NULL) { return TUE_STS3215_PROTO_ERR_NULL_PTR; }
    if (buf_len < 6U)                 { return TUE_STS3215_PROTO_ERR_FRAME_SHORT; }
    if (buf[STS3215_REPLY_OFF_HEADER0] != STS3215_HEADER_BYTE ||
        buf[STS3215_REPLY_OFF_HEADER1] != STS3215_HEADER_BYTE) {
        return TUE_STS3215_PROTO_ERR_BAD_HEADER;
    }

    uint8_t id     = buf[STS3215_REPLY_OFF_ID];
    uint8_t length = buf[STS3215_REPLY_OFF_LENGTH];
    uint8_t error  = buf[STS3215_REPLY_OFF_ERROR];

    if (length < 2U) { return TUE_STS3215_PROTO_ERR_FRAME_SHORT; }
    uint16_t expected_total = (uint16_t)length + 4U;
    if (buf_len < expected_total) { return TUE_STS3215_PROTO_ERR_FRAME_SHORT; }

    uint8_t  n_data = (uint8_t)(length - 2U);
    uint32_t sum = (uint32_t)id + (uint32_t)length + (uint32_t)error;

    for (uint8_t i = 0U; i < n_data; i++) {
        sum += (uint32_t)buf[STS3215_REPLY_OFF_DATA + i];
    }

    uint8_t expected_chk = (uint8_t)(~sum & 0xFFU);
    uint8_t actual_chk   = buf[STS3215_REPLY_OFF_DATA + n_data];

    if (actual_chk != expected_chk) { return TUE_STS3215_PROTO_ERR_BAD_CHECKSUM; }

    reply->id = id;
    reply->error = error;
    reply->data_len = n_data;

    if (n_data > 0U) {
        uint8_t copy_len = (n_data <= (uint8_t)STS3215_REPLY_DATA_MAX)
                           ? n_data : (uint8_t)STS3215_REPLY_DATA_MAX;
        memcpy(reply->data, &buf[STS3215_REPLY_OFF_DATA], copy_len);
    }

    return (error != 0U) ? TUE_STS3215_PROTO_ERR_SERVO_FAULT : TUE_STS3215_PROTO_OK;
}

/* =========================================================================
 * Utility / unit conversion
 * ========================================================================= */

uint16_t tue_sts3215_unpack_u16_le(const uint8_t *buf)
{
    return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
}

int16_t tue_sts3215_unpack_s16_le(const uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
}

int16_t tue_sts3215_unpack_signmag16_le(const uint8_t *buf, uint8_t sign_bit)
{
    uint16_t raw = (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
    uint16_t mag = (uint16_t)(raw & (uint16_t)((1U << sign_bit) - 1U));
    return (raw & (uint16_t)(1U << sign_bit)) ? (int16_t)(-(int16_t)mag) : (int16_t)mag;
}

float tue_sts3215_steps_to_deg(int16_t steps)
{
    return (float)steps * STS3215_DEG_PER_STEP_F;
}

float tue_sts3215_steps_to_rad(int16_t steps)
{
    return (float)steps * STS3215_RAD_PER_STEP_F;
}

int16_t tue_sts3215_deg_to_steps(float deg)
{
    return (int16_t)(deg / STS3215_DEG_PER_STEP_F);
}

float tue_sts3215_raw_to_milliamps(uint16_t raw)
{
    return (float)raw * STS3215_CURRENT_LSB_MA_F;
}

float tue_sts3215_raw_to_volts(uint8_t raw)
{
    return (float)raw * STS3215_VOLTAGE_LSB_V_F;
}
