/**
  ******************************************************************************
  * @file    tue_sts3215.c
  * @brief   Feetech STS3215 serial bus servo - high-level blocking driver.
  ******************************************************************************
  * @attention
  *
  * This file is the *facade* over the reference-style layers:
  *   tue_sts3215_protocol.c (pure builders/parser) and tue_sts3215_hal.c
  *   (async half-duplex DMA state machine).
  *
  * A blocking transaction is built from the protocol layer, handed to the HAL,
  * then this task polls tue_sts3215_hal_process() until the reply/error
  * callback fires or the timeout elapses. This preserves the synchronous
  * tue_sts3215_* API that tue_acsi.c and the Python host client depend on.
  ******************************************************************************
  */

#include "tue_sts3215.h"

#include <string.h>
#include "cmsis_os2.h"

/* Private variables ---------------------------------------------------------*/
static tue_sts3215_t *s_default_dev = NULL;

/* HAL -> blocking capture callbacks -----------------------------------------*/

static void sts_on_reply(const tue_sts3215_reply_t *reply, uint8_t reply_idx,
                         tue_sts3215_proto_status_t status, void *ctx)
{
    tue_sts3215_t *dev = (tue_sts3215_t *)ctx;

    if (dev == NULL || reply == NULL || reply_idx >= TUE_STS3215_MAX_REPLIES) {
        return;
    }

    dev->replies[reply_idx] = *reply;
    if ((uint8_t)(reply_idx + 1U) > dev->reply_count) {
        dev->reply_count = (uint8_t)(reply_idx + 1U);
    }
    dev->last_servo_status = reply->error;
    if (status == TUE_STS3215_PROTO_ERR_SERVO_FAULT) {
        dev->xfer_status = TUE_STS3215_ERR_STATUS;
    }

    if (dev->sem != NULL) {
        (void)osSemaphoreRelease(dev->sem);
    }
}

static void sts_on_error(tue_sts3215_hal_error_t err, void *ctx)
{
    tue_sts3215_t *dev = (tue_sts3215_t *)ctx;

    if (dev == NULL) {
        return;
    }

    switch (err) {
        case TUE_STS3215_HAL_ERR_TIMEOUT:
        case TUE_STS3215_HAL_ERR_TX_TIMEOUT:
            dev->xfer_status = TUE_STS3215_ERR_TIMEOUT;
            break;
        case TUE_STS3215_HAL_ERR_BUSY:
            dev->xfer_status = TUE_STS3215_ERR_BUSY;
            break;
        case TUE_STS3215_HAL_ERR_PARSE:
            dev->xfer_status = TUE_STS3215_ERR_CHECKSUM;
            break;
        default:
            dev->xfer_status = TUE_STS3215_ERR_HAL;
            break;
    }
    dev->xfer_done = true;

    if (dev->sem != NULL) {
        (void)osSemaphoreRelease(dev->sem);
    }
}

/* Blocking transaction core -------------------------------------------------*/

static tue_sts3215_status_t sts_wait(tue_sts3215_t *dev, bool expect_reply,
                                     uint8_t expected_replies, uint32_t timeout_ms)
{
    if (!expect_reply) {
        uint32_t start = HAL_GetTick();
        while (!tue_sts3215_hal_is_idle(&dev->hal)) {
            if ((HAL_GetTick() - start) >= (timeout_ms + 1U)) {
                tue_sts3215_hal_abort(&dev->hal);
                return TUE_STS3215_ERR_TIMEOUT;
            }
            if (dev->sem != NULL) {
                (void)osSemaphoreAcquire(dev->sem, 1U);
            }
        }
        return TUE_STS3215_OK;
    }

    /* Event-driven: wait for DMA RX complete interrupt to release the semaphore.
     * ZERO polling, ZERO osDelay: task is suspended by RTOS until reply arrives. */
    while ((dev->reply_count < expected_replies) && !dev->xfer_done) {
        if (dev->sem != NULL) {
            if (osSemaphoreAcquire(dev->sem, timeout_ms) != osOK) {
                tue_sts3215_hal_abort(&dev->hal);
                return TUE_STS3215_ERR_TIMEOUT;
            }
        } else {
            tue_sts3215_hal_process(&dev->hal);
        }
    }

    return dev->xfer_status;
}

static tue_sts3215_status_t sts_xact(tue_sts3215_t *dev,
                                     const uint8_t *frame, uint16_t len,
                                     bool expect_reply, uint8_t expected_replies)
{
    if (dev == NULL || frame == NULL) {
        return TUE_STS3215_ERR_PARAM;
    }

    /* Clear any stale transfer that never completed. */
    if (!tue_sts3215_hal_is_idle(&dev->hal)) {
        tue_sts3215_hal_abort(&dev->hal);
    }

    dev->xfer_done  = false;
    dev->reply_count = 0U;
    dev->xfer_status = TUE_STS3215_OK;
    dev->last_servo_status = 0U;

    if (tue_sts3215_hal_send_frame(&dev->hal, frame, len, !expect_reply,
                                   expect_reply ? expected_replies : 0U) != TUE_STS3215_PROTO_OK) {
        return TUE_STS3215_ERR_PARAM;
    }

    return sts_wait(dev, expect_reply, expected_replies, dev->timeout_ms);
}

/* Register read/write helpers -----------------------------------------------*/

static tue_sts3215_status_t sts_read(tue_sts3215_t *dev, uint8_t id, uint8_t reg,
                                     uint8_t *out, uint8_t len)
{
    uint8_t frame[STS3215_TX_BUF_SIZE];
    int16_t n = tue_sts3215_build_read(frame, sizeof(frame), id, reg, len);
    if (n <= 0) {
        return TUE_STS3215_ERR_PARAM;
    }

    tue_sts3215_status_t st = sts_xact(dev, frame, (uint16_t)n, true, 1U);

    if ((st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) &&
        out != NULL && dev->reply_count > 0U) {
        uint8_t got = dev->replies[0].data_len;
        uint8_t copy = (got < len) ? got : len;
        memcpy(out, dev->replies[0].data, copy);
        if (st == TUE_STS3215_OK && got < len) {
            st = TUE_STS3215_ERR_LEN;
        }
    }
    return st;
}

static tue_sts3215_status_t sts_write(tue_sts3215_t *dev, uint8_t id, uint8_t reg,
                                      const uint8_t *data, uint8_t len,
                                      bool ack, bool deferred)
{
    uint8_t frame[STS3215_TX_BUF_SIZE];
    int16_t n = deferred
                ? tue_sts3215_build_reg_write(frame, sizeof(frame), id, reg, data, len)
                : tue_sts3215_build_write_raw(frame, sizeof(frame), id, reg, data, len);
    if (n <= 0) {
        return TUE_STS3215_ERR_PARAM;
    }

    bool expect = ack && (id != STS3215_BROADCAST_ID);
    return sts_xact(dev, frame, (uint16_t)n, expect, 1U);
}

static tue_sts3215_status_t sts_sync_write(tue_sts3215_t *dev, uint8_t reg, uint8_t data_len,
                                           const tue_sts3215_sync_entry_t *entries, uint8_t count)
{
    uint8_t frame[STS3215_TX_BUF_SIZE];
    int16_t n = tue_sts3215_build_sync_write(frame, sizeof(frame), reg, data_len, entries, count);
    if (n <= 0) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_xact(dev, frame, (uint16_t)n, false, 0U);
}

/* Sign-magnitude / PWM encoding helpers -------------------------------------*/

static uint16_t sts_pwm_to_raw(int16_t pwm)
{
    int16_t mag = pwm;
    if (mag < 0) {
        int32_t m = -(int32_t)pwm;
        if (m > 1000) m = 1000;
        return (uint16_t)((1U << 10) | ((uint16_t)m & 0x03FFU));
    }
    if (mag > 1000) mag = 1000;
    return (uint16_t)((uint16_t)mag & 0x03FFU);
}

static int16_t sts_raw_to_pwm(uint16_t raw)
{
    int16_t mag = (int16_t)(raw & 0x03FFU);
    return (raw & (1U << 10)) ? (int16_t)(-mag) : mag;
}

static uint16_t sts_offset_to_raw(int16_t offset)
{
    int32_t off = offset;
    if (off < 0) {
        int32_t mag = -off;
        if (mag > 2047) mag = 2047;
        return (uint16_t)((1U << 11) | ((uint16_t)mag & 0x07FFU));
    }
    if (off > 2047) off = 2047;
    return (uint16_t)((uint16_t)off & 0x07FFU);
}

static int16_t sts_raw_to_offset(uint16_t raw)
{
    int16_t val = (int16_t)(raw & 0x07FFU);
    return (raw & (1U << 11)) ? (int16_t)(-val) : val;
}

/* Lifecycle -----------------------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_init(tue_sts3215_t *dev, UART_HandleTypeDef *huart)
{
    if (dev == NULL || huart == NULL) {
        return TUE_STS3215_ERR_PARAM;
    }

    memset(dev, 0, sizeof(tue_sts3215_t));
    dev->timeout_ms = STS3215_DEFAULT_TIMEOUT_MS;
    dev->xfer_status = TUE_STS3215_ERR_HAL;

    tue_sts3215_hal_init(&dev->hal, huart, dev->timeout_ms,
                         sts_on_reply, sts_on_error, dev);
    tue_sts3215_hal_register_instance(&dev->hal);

    const osSemaphoreAttr_t sem_attr = { .name = "sts_sem" };
    dev->sem = osSemaphoreNew(1U, 0U, &sem_attr);

    if (s_default_dev == NULL) {
        s_default_dev = dev;
    }

    return TUE_STS3215_OK;
}

tue_sts3215_t *tue_sts3215_get_default(void)
{
    return s_default_dev;
}

void tue_sts3215_set_timeout(tue_sts3215_t *dev, uint32_t timeout_ms)
{
    if (dev != NULL && timeout_ms > 0U) {
        dev->timeout_ms = timeout_ms;
        dev->hal.reply_timeout_ms = timeout_ms;
    }
}

/* Low-level protocol commands -----------------------------------------------*/

tue_sts3215_status_t tue_sts3215_ping(tue_sts3215_t *dev, uint8_t id)
{
    uint8_t frame[STS3215_TX_BUF_SIZE];
    int16_t n = tue_sts3215_build_ping(frame, sizeof(frame), id);
    if (n <= 0) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_xact(dev, frame, (uint16_t)n, id != STS3215_BROADCAST_ID, 1U);
}

tue_sts3215_status_t tue_sts3215_read_raw(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr,
                                          uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0U) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_read(dev, id, reg_addr, data, len);
}

tue_sts3215_status_t tue_sts3215_write_raw(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr,
                                           const uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0U) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_write(dev, id, reg_addr, data, len, true, false);
}

tue_sts3215_status_t tue_sts3215_write_raw_no_ack(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr,
                                                  const uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0U) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_write(dev, id, reg_addr, data, len, false, false);
}

tue_sts3215_status_t tue_sts3215_reg_write_raw(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr,
                                               const uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0U) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_write(dev, id, reg_addr, data, len, true, true);
}

tue_sts3215_status_t tue_sts3215_action(tue_sts3215_t *dev, uint8_t id)
{
    uint8_t frame[STS3215_TX_BUF_SIZE];
    int16_t n = tue_sts3215_build_action(frame, sizeof(frame), id);
    if (n <= 0) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_xact(dev, frame, (uint16_t)n, id != STS3215_BROADCAST_ID, 1U);
}

tue_sts3215_status_t tue_sts3215_reset(tue_sts3215_t *dev, uint8_t id)
{
    uint8_t frame[STS3215_TX_BUF_SIZE];
    int16_t n = tue_sts3215_build_reset(frame, sizeof(frame), id);
    if (n <= 0) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_xact(dev, frame, (uint16_t)n, id != STS3215_BROADCAST_ID, 1U);
}

tue_sts3215_status_t tue_sts3215_reset_turns(tue_sts3215_t *dev, uint8_t id)
{
    uint8_t frame[STS3215_TX_BUF_SIZE];
    int16_t n = tue_sts3215_build_reset_turns(frame, sizeof(frame), id);
    if (n <= 0) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_xact(dev, frame, (uint16_t)n, id != STS3215_BROADCAST_ID, 1U);
}

tue_sts3215_status_t tue_sts3215_sync_write_raw(tue_sts3215_t *dev, uint8_t start_addr,
                                                uint8_t data_len, const uint8_t *param_data,
                                                uint16_t total_param_bytes)
{
    if (param_data == NULL || data_len == 0U ||
        data_len > STS3215_SYNC_DATA_MAX_LEN) {
        return TUE_STS3215_ERR_PARAM;
    }

    uint16_t stride = (uint16_t)(data_len + 1U);
    if (total_param_bytes == 0U || (total_param_bytes % stride) != 0U) {
        return TUE_STS3215_ERR_PARAM;
    }

    uint16_t count = (uint16_t)(total_param_bytes / stride);
    if (count == 0U || count > STS3215_SYNC_MAX_SERVOS) {
        return TUE_STS3215_ERR_PARAM;
    }

    tue_sts3215_sync_entry_t entries[STS3215_SYNC_MAX_SERVOS];
    for (uint16_t i = 0U; i < count; i++) {
        const uint8_t *src = &param_data[i * stride];
        entries[i].id = src[0];
        memcpy(entries[i].data, &src[1], data_len);
    }

    return sts_sync_write(dev, start_addr, data_len, entries, (uint8_t)count);
}

tue_sts3215_status_t tue_sts3215_sync_read(tue_sts3215_t *dev, uint8_t reg, uint8_t len,
                                           const uint8_t *ids, uint8_t count,
                                           tue_sts3215_reply_t *replies, uint8_t *reply_count)
{
    if (ids == NULL || len == 0U || count == 0U || count > STS3215_SYNC_MAX_SERVOS) {
        return TUE_STS3215_ERR_PARAM;
    }

    uint8_t frame[STS3215_TX_BUF_SIZE];
    int16_t n = tue_sts3215_build_sync_read(frame, sizeof(frame), reg, len, ids, count);
    if (n <= 0) {
        return TUE_STS3215_ERR_PARAM;
    }

    tue_sts3215_status_t st = sts_xact(dev, frame, (uint16_t)n, true, count);
    if (st != TUE_STS3215_OK && st != TUE_STS3215_ERR_STATUS) {
        return st;
    }

    uint8_t got = dev->reply_count;
    if (replies != NULL) {
        for (uint8_t i = 0U; i < got; i++) {
            replies[i] = dev->replies[i];
        }
    }
    if (reply_count != NULL) {
        *reply_count = got;
    }

    return (got >= count) ? st : TUE_STS3215_ERR_LEN;
}

/* Primitive typed helpers ---------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_read_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t *val)
{
    if (val == NULL) return TUE_STS3215_ERR_PARAM;
    return sts_read(dev, id, reg_addr, val, 1U);
}

tue_sts3215_status_t tue_sts3215_write_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t val)
{
    return sts_write(dev, id, reg_addr, &val, 1U, true, false);
}

tue_sts3215_status_t tue_sts3215_read_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t *val)
{
    if (val == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = sts_read(dev, id, reg_addr, buf, 2U);
    if (st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) {
        *val = tue_sts3215_unpack_u16_le(buf);
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_write_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val & 0xFFU);
    buf[1] = (uint8_t)((val >> 8U) & 0xFFU);
    return sts_write(dev, id, reg_addr, buf, 2U, true, false);
}

tue_sts3215_status_t tue_sts3215_read_i16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, int16_t *val)
{
    if (val == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = sts_read(dev, id, reg_addr, buf, 2U);
    if (st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) {
        *val = tue_sts3215_unpack_s16_le(buf);
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_write_i16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, int16_t val)
{
    return tue_sts3215_write_u16(dev, id, reg_addr, (uint16_t)val);
}

/* EEPROM safe write helpers -------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_write_eeprom_u8(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint8_t val)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    st = tue_sts3215_write_u8(dev, id, reg_addr, val);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_write_eeprom_u16(tue_sts3215_t *dev, uint8_t id, uint8_t reg_addr, uint16_t val)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    st = tue_sts3215_write_u16(dev, id, reg_addr, val);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

/* EEPROM lock ---------------------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_lock_eeprom(tue_sts3215_t *dev, uint8_t id)
{
    return tue_sts3215_write_u8(dev, id, STS3215_REG_LOCK_MARK, (uint8_t)STS3215_LOCK_CLOSED);
}

tue_sts3215_status_t tue_sts3215_unlock_eeprom(tue_sts3215_t *dev, uint8_t id)
{
    return tue_sts3215_write_u8(dev, id, STS3215_REG_LOCK_MARK, (uint8_t)STS3215_LOCK_OPEN);
}

tue_sts3215_status_t tue_sts3215_get_eeprom_lock(tue_sts3215_t *dev, uint8_t id, uint8_t *is_locked)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_LOCK_MARK, is_locked);
}

/* Versions & device markers -------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_get_fw_version(tue_sts3215_t *dev, uint8_t id, uint8_t *major, uint8_t *minor)
{
    if (major == NULL || minor == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = sts_read(dev, id, STS3215_REG_FW_MAJOR, buf, 2U);
    if (st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) {
        *major = buf[0];
        *minor = buf[1];
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_get_end_marker(tue_sts3215_t *dev, uint8_t id, uint8_t *end_val)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_END, end_val);
}

tue_sts3215_status_t tue_sts3215_get_servo_version(tue_sts3215_t *dev, uint8_t id, uint8_t *main_ver, uint8_t *sub_ver)
{
    if (main_ver == NULL || sub_ver == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = sts_read(dev, id, STS3215_REG_SERVO_MAIN_VER, buf, 2U);
    if (st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) {
        *main_ver = buf[0];
        *sub_ver  = buf[1];
    }
    return st;
}

/* Bus ID & baud rate --------------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_id(tue_sts3215_t *dev, uint8_t old_id, uint8_t new_id)
{
    if (new_id > STS3215_ID_MAX) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_write_eeprom_u8(dev, old_id, STS3215_REG_ID, new_id);
}

tue_sts3215_status_t tue_sts3215_get_id(tue_sts3215_t *dev, uint8_t id, uint8_t *current_id)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_ID, current_id);
}

tue_sts3215_status_t tue_sts3215_set_baud_rate(tue_sts3215_t *dev, uint8_t id, tue_sts3215_baud_t baud)
{
    if (baud > STS3215_BAUD_38400) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_BAUD_RATE, (uint8_t)baud);
}

tue_sts3215_status_t tue_sts3215_get_baud_rate(tue_sts3215_t *dev, uint8_t id, tue_sts3215_baud_t *baud)
{
    if (baud == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t val = 0U;
    tue_sts3215_status_t st = tue_sts3215_read_u8(dev, id, STS3215_REG_BAUD_RATE, &val);
    if (st == TUE_STS3215_OK) {
        *baud = (tue_sts3215_baud_t)val;
    }
    return st;
}

/* Return delay & response level ---------------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_return_delay(tue_sts3215_t *dev, uint8_t id, uint8_t delay_2us)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_RETURN_DELAY, delay_2us);
}

tue_sts3215_status_t tue_sts3215_get_return_delay(tue_sts3215_t *dev, uint8_t id, uint8_t *delay_2us)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_RETURN_DELAY, delay_2us);
}

tue_sts3215_status_t tue_sts3215_set_response_level(tue_sts3215_t *dev, uint8_t id, uint8_t level)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_RESP_LEVEL, (level > 0U) ? 1U : 0U);
}

tue_sts3215_status_t tue_sts3215_get_response_level(tue_sts3215_t *dev, uint8_t id, uint8_t *level)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_RESP_LEVEL, level);
}

/* Motion travel angle limits ------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_angle_limits(tue_sts3215_t *dev, uint8_t id, uint16_t min_angle, uint16_t max_angle)
{
    if (min_angle >= max_angle || max_angle > 4095U) return TUE_STS3215_ERR_PARAM;

    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    uint8_t buf[4];
    buf[0] = (uint8_t)(min_angle & 0xFFU);
    buf[1] = (uint8_t)((min_angle >> 8U) & 0xFFU);
    buf[2] = (uint8_t)(max_angle & 0xFFU);
    buf[3] = (uint8_t)((max_angle >> 8U) & 0xFFU);

    st = sts_write(dev, id, STS3215_REG_MIN_ANGLE, buf, 4U, true, false);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_angle_limits(tue_sts3215_t *dev, uint8_t id, uint16_t *min_angle, uint16_t *max_angle)
{
    if (min_angle == NULL || max_angle == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[4];
    tue_sts3215_status_t st = sts_read(dev, id, STS3215_REG_MIN_ANGLE, buf, 4U);
    if (st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) {
        *min_angle = tue_sts3215_unpack_u16_le(&buf[0]);
        *max_angle = tue_sts3215_unpack_u16_le(&buf[2]);
    }
    return st;
}

/* Temperature & voltage limits ----------------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_max_temperature_limit(tue_sts3215_t *dev, uint8_t id, uint8_t max_temp_c)
{
    if (max_temp_c > 100U) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_MAX_TEMP, max_temp_c);
}

tue_sts3215_status_t tue_sts3215_get_max_temperature_limit(tue_sts3215_t *dev, uint8_t id, uint8_t *max_temp_c)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_MAX_TEMP, max_temp_c);
}

tue_sts3215_status_t tue_sts3215_set_voltage_limits(tue_sts3215_t *dev, uint8_t id, uint8_t min_0_1v, uint8_t max_0_1v)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    uint8_t buf[2] = { max_0_1v, min_0_1v };
    st = sts_write(dev, id, STS3215_REG_MAX_VOLTAGE, buf, 2U, true, false);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_voltage_limits(tue_sts3215_t *dev, uint8_t id, uint8_t *min_0_1v, uint8_t *max_0_1v)
{
    if (min_0_1v == NULL || max_0_1v == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = sts_read(dev, id, STS3215_REG_MAX_VOLTAGE, buf, 2U);
    if (st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) {
        *max_0_1v = buf[0];
        *min_0_1v = buf[1];
    }
    return st;
}

/* Max torque & phase --------------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_max_torque(tue_sts3215_t *dev, uint8_t id, uint16_t max_torque_0_1pct)
{
    if (max_torque_0_1pct > 1000U) max_torque_0_1pct = 1000U;
    return tue_sts3215_write_eeprom_u16(dev, id, STS3215_REG_MAX_TORQUE, max_torque_0_1pct);
}

tue_sts3215_status_t tue_sts3215_get_max_torque(tue_sts3215_t *dev, uint8_t id, uint16_t *max_torque_0_1pct)
{
    return tue_sts3215_read_u16(dev, id, STS3215_REG_MAX_TORQUE, max_torque_0_1pct);
}

tue_sts3215_status_t tue_sts3215_set_phase(tue_sts3215_t *dev, uint8_t id, uint8_t phase)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_PHASE, phase);
}

tue_sts3215_status_t tue_sts3215_get_phase(tue_sts3215_t *dev, uint8_t id, uint8_t *phase)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_PHASE, phase);
}

/* Unload & LED alarm conditions ---------------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_unload_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t conditions_mask)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_UNLOAD_COND, conditions_mask);
}

tue_sts3215_status_t tue_sts3215_get_unload_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t *conditions_mask)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_UNLOAD_COND, conditions_mask);
}

tue_sts3215_status_t tue_sts3215_set_led_alarm_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t conditions_mask)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_LED_ALARM, conditions_mask);
}

tue_sts3215_status_t tue_sts3215_get_led_alarm_conditions(tue_sts3215_t *dev, uint8_t id, uint8_t *conditions_mask)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_LED_ALARM, conditions_mask);
}

/* Position loop PID & starting force ----------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_position_pid(tue_sts3215_t *dev, uint8_t id, uint8_t p, uint8_t d, uint8_t i)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    uint8_t buf[3] = { p, d, i };
    st = sts_write(dev, id, STS3215_REG_P_COEFF, buf, 3U, true, false);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_position_pid(tue_sts3215_t *dev, uint8_t id, uint8_t *p, uint8_t *d, uint8_t *i)
{
    if (p == NULL || d == NULL || i == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[3];
    tue_sts3215_status_t st = sts_read(dev, id, STS3215_REG_P_COEFF, buf, 3U);
    if (st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) {
        *p = buf[0];
        *d = buf[1];
        *i = buf[2];
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_set_min_starting_force(tue_sts3215_t *dev, uint8_t id, uint8_t force_0_1pct)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_MIN_STARTUP, force_0_1pct);
}

tue_sts3215_status_t tue_sts3215_get_min_starting_force(tue_sts3215_t *dev, uint8_t id, uint8_t *force_0_1pct)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_MIN_STARTUP, force_0_1pct);
}

/* Points limit & deadband ---------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_points_limit(tue_sts3215_t *dev, uint8_t id, uint8_t limit)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_POINTS_LIMIT, limit);
}

tue_sts3215_status_t tue_sts3215_get_points_limit(tue_sts3215_t *dev, uint8_t id, uint8_t *limit)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_POINTS_LIMIT, limit);
}

tue_sts3215_status_t tue_sts3215_set_deadband(tue_sts3215_t *dev, uint8_t id, uint8_t cw_deadband, uint8_t ccw_deadband)
{
    if (cw_deadband > 32U || ccw_deadband > 32U) return TUE_STS3215_ERR_PARAM;

    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    uint8_t buf[2] = { cw_deadband, ccw_deadband };
    st = sts_write(dev, id, STS3215_REG_CW_DEADBAND, buf, 2U, true, false);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_deadband(tue_sts3215_t *dev, uint8_t id, uint8_t *cw_deadband, uint8_t *ccw_deadband)
{
    if (cw_deadband == NULL || ccw_deadband == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t buf[2];
    tue_sts3215_status_t st = sts_read(dev, id, STS3215_REG_CW_DEADBAND, buf, 2U);
    if (st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) {
        *cw_deadband  = buf[0];
        *ccw_deadband = buf[1];
    }
    return st;
}

/* Protective current, resolution & position correction ----------------------*/

tue_sts3215_status_t tue_sts3215_set_protective_current(tue_sts3215_t *dev, uint8_t id, uint16_t current_6_5ma)
{
    if (current_6_5ma > 511U) current_6_5ma = 511U;
    return tue_sts3215_write_eeprom_u16(dev, id, STS3215_REG_PROT_CURRENT, current_6_5ma);
}

tue_sts3215_status_t tue_sts3215_get_protective_current(tue_sts3215_t *dev, uint8_t id, uint16_t *current_6_5ma)
{
    return tue_sts3215_read_u16(dev, id, STS3215_REG_PROT_CURRENT, current_6_5ma);
}

tue_sts3215_status_t tue_sts3215_set_angular_resolution(tue_sts3215_t *dev, uint8_t id, uint8_t resolution)
{
    if (resolution < 1U || resolution > 3U) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_ANGULAR_RES, resolution);
}

tue_sts3215_status_t tue_sts3215_get_angular_resolution(tue_sts3215_t *dev, uint8_t id, uint8_t *resolution)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_ANGULAR_RES, resolution);
}

tue_sts3215_status_t tue_sts3215_set_position_correction(tue_sts3215_t *dev, uint8_t id, int16_t offset_steps)
{
    uint16_t raw = sts_offset_to_raw(offset_steps);
    return tue_sts3215_write_eeprom_u16(dev, id, STS3215_REG_POS_CORRECTION, raw);
}

tue_sts3215_status_t tue_sts3215_get_position_correction(tue_sts3215_t *dev, uint8_t id, int16_t *offset_steps)
{
    if (offset_steps == NULL) return TUE_STS3215_ERR_PARAM;
    uint16_t raw = 0U;
    tue_sts3215_status_t st = tue_sts3215_read_u16(dev, id, STS3215_REG_POS_CORRECTION, &raw);
    if (st == TUE_STS3215_OK) {
        *offset_steps = sts_raw_to_offset(raw);
    }
    return st;
}

/* Operating mode ------------------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_operation_mode(tue_sts3215_t *dev, uint8_t id, tue_sts3215_mode_t mode)
{
    if (mode > STS3215_MODE_STEPPER) return TUE_STS3215_ERR_PARAM;
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_OPERATION_MODE, (uint8_t)mode);
}

tue_sts3215_status_t tue_sts3215_get_operation_mode(tue_sts3215_t *dev, uint8_t id, tue_sts3215_mode_t *mode)
{
    if (mode == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t val = 0U;
    tue_sts3215_status_t st = tue_sts3215_read_u8(dev, id, STS3215_REG_OPERATION_MODE, &val);
    if (st == TUE_STS3215_OK) {
        *mode = (tue_sts3215_mode_t)val;
    }
    return st;
}

/* Overload protection -------------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_overload_protection(tue_sts3215_t *dev, uint8_t id,
                                                         uint8_t protective_torque_pct,
                                                         uint8_t protection_time_10ms,
                                                         uint8_t overload_torque_pct)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    uint8_t buf[3] = { protective_torque_pct, protection_time_10ms, overload_torque_pct };
    st = sts_write(dev, id, STS3215_REG_PROT_TORQUE, buf, 3U, true, false);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_overload_protection(tue_sts3215_t *dev, uint8_t id,
                                                         uint8_t *protective_torque_pct,
                                                         uint8_t *protection_time_10ms,
                                                         uint8_t *overload_torque_pct)
{
    if (protective_torque_pct == NULL || protection_time_10ms == NULL || overload_torque_pct == NULL) {
        return TUE_STS3215_ERR_PARAM;
    }
    uint8_t buf[3];
    tue_sts3215_status_t st = sts_read(dev, id, STS3215_REG_PROT_TORQUE, buf, 3U);
    if (st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) {
        *protective_torque_pct = buf[0];
        *protection_time_10ms  = buf[1];
        *overload_torque_pct   = buf[2];
    }
    return st;
}

/* Speed loop PID & overcurrent time -----------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_speed_pid(tue_sts3215_t *dev, uint8_t id, uint8_t speed_p, uint8_t speed_i)
{
    tue_sts3215_status_t st = tue_sts3215_unlock_eeprom(dev, id);
    if (st != TUE_STS3215_OK) return st;

    st = tue_sts3215_write_u8(dev, id, STS3215_REG_SPEED_P, speed_p);
    if (st != TUE_STS3215_OK) return st;

    st = tue_sts3215_write_u8(dev, id, STS3215_REG_SPEED_I, speed_i);
    if (st != TUE_STS3215_OK) return st;

    return tue_sts3215_lock_eeprom(dev, id);
}

tue_sts3215_status_t tue_sts3215_get_speed_pid(tue_sts3215_t *dev, uint8_t id, uint8_t *speed_p, uint8_t *speed_i)
{
    if (speed_p == NULL || speed_i == NULL) return TUE_STS3215_ERR_PARAM;
    tue_sts3215_status_t st = tue_sts3215_read_u8(dev, id, STS3215_REG_SPEED_P, speed_p);
    if (st != TUE_STS3215_OK) return st;
    return tue_sts3215_read_u8(dev, id, STS3215_REG_SPEED_I, speed_i);
}

tue_sts3215_status_t tue_sts3215_set_overcurrent_time(tue_sts3215_t *dev, uint8_t id, uint8_t time_10ms)
{
    return tue_sts3215_write_eeprom_u8(dev, id, STS3215_REG_OC_PROT_TIME, time_10ms);
}

tue_sts3215_status_t tue_sts3215_get_overcurrent_time(tue_sts3215_t *dev, uint8_t id, uint8_t *time_10ms)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_OC_PROT_TIME, time_10ms);
}

/* Torque enable & zero-point calibration ------------------------------------*/

tue_sts3215_status_t tue_sts3215_enable_torque(tue_sts3215_t *dev, uint8_t id, bool enable)
{
    return tue_sts3215_write_u8(dev, id, STS3215_REG_TORQUE_SWITCH,
                                enable ? (uint8_t)STS3215_TORQUE_ON : (uint8_t)STS3215_TORQUE_OFF);
}

tue_sts3215_status_t tue_sts3215_calibrate_position_to_2048(tue_sts3215_t *dev, uint8_t id)
{
    return tue_sts3215_write_u8(dev, id, STS3215_REG_TORQUE_SWITCH,
                                (uint8_t)STS3215_TORQUE_CENTER);
}

tue_sts3215_status_t tue_sts3215_get_torque_enable(tue_sts3215_t *dev, uint8_t id, bool *enabled)
{
    if (enabled == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t val = 0U;
    tue_sts3215_status_t st = tue_sts3215_read_u8(dev, id, STS3215_REG_TORQUE_SWITCH, &val);
    if (st == TUE_STS3215_OK) {
        *enabled = (val == 1U);
    }
    return st;
}

/* Acceleration, position & speed control ------------------------------------*/

tue_sts3215_status_t tue_sts3215_set_acceleration(tue_sts3215_t *dev, uint8_t id, uint8_t accel_100steps_s2)
{
    return tue_sts3215_write_u8(dev, id, STS3215_REG_ACCELERATION, accel_100steps_s2);
}

tue_sts3215_status_t tue_sts3215_get_acceleration(tue_sts3215_t *dev, uint8_t id, uint8_t *accel_100steps_s2)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_ACCELERATION, accel_100steps_s2);
}

tue_sts3215_status_t tue_sts3215_set_goal_position(tue_sts3215_t *dev, uint8_t id, int16_t position)
{
    return tue_sts3215_write_i16(dev, id, STS3215_REG_TARGET_POS, position);
}

tue_sts3215_status_t tue_sts3215_set_goal_position_ex(tue_sts3215_t *dev, uint8_t id, int16_t position,
                                                      int16_t speed, uint8_t accel)
{
    uint8_t frame[STS3215_TX_BUF_SIZE];
    tue_sts3215_motion_cmd_t cmd = {
        .acceleration  = accel,
        .target_pos    = position,
        .running_time  = 0U,
        .running_speed = (uint16_t)speed
    };
    int16_t n = tue_sts3215_build_motion_cmd(frame, sizeof(frame), id, &cmd);
    if (n <= 0) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_xact(dev, frame, (uint16_t)n, id != STS3215_BROADCAST_ID, 1U);
}

tue_sts3215_status_t tue_sts3215_reg_write_goal_position_ex(tue_sts3215_t *dev, uint8_t id, int16_t position,
                                                            int16_t speed, uint8_t accel)
{
    uint8_t frame[STS3215_TX_BUF_SIZE];
    tue_sts3215_motion_cmd_t cmd = {
        .acceleration  = accel,
        .target_pos    = position,
        .running_time  = 0U,
        .running_speed = (uint16_t)speed
    };
    int16_t n = tue_sts3215_build_reg_write_motion_cmd(frame, sizeof(frame), id, &cmd);
    if (n <= 0) {
        return TUE_STS3215_ERR_PARAM;
    }
    return sts_xact(dev, frame, (uint16_t)n, id != STS3215_BROADCAST_ID, 1U);
}

tue_sts3215_status_t tue_sts3215_set_pwm_runtime(tue_sts3215_t *dev, uint8_t id, int16_t pwm_per_thousand)
{
    return tue_sts3215_write_u16(dev, id, STS3215_REG_RUNNING_TIME, sts_pwm_to_raw(pwm_per_thousand));
}

tue_sts3215_status_t tue_sts3215_get_pwm_runtime(tue_sts3215_t *dev, uint8_t id, int16_t *pwm_per_thousand)
{
    if (pwm_per_thousand == NULL) return TUE_STS3215_ERR_PARAM;
    uint16_t raw = 0U;
    tue_sts3215_status_t st = tue_sts3215_read_u16(dev, id, STS3215_REG_RUNNING_TIME, &raw);
    if (st == TUE_STS3215_OK) {
        *pwm_per_thousand = sts_raw_to_pwm(raw);
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_set_running_speed(tue_sts3215_t *dev, uint8_t id, int16_t speed_steps_s)
{
    return tue_sts3215_write_i16(dev, id, STS3215_REG_RUNNING_SPEED, speed_steps_s);
}

tue_sts3215_status_t tue_sts3215_get_running_speed(tue_sts3215_t *dev, uint8_t id, int16_t *speed_steps_s)
{
    return tue_sts3215_read_i16(dev, id, STS3215_REG_RUNNING_SPEED, speed_steps_s);
}

tue_sts3215_status_t tue_sts3215_set_torque_limit(tue_sts3215_t *dev, uint8_t id, uint16_t torque_0_1pct)
{
    if (torque_0_1pct > 1000U) torque_0_1pct = 1000U;
    return tue_sts3215_write_u16(dev, id, STS3215_REG_TORQUE_LIMIT, torque_0_1pct);
}

tue_sts3215_status_t tue_sts3215_get_torque_limit(tue_sts3215_t *dev, uint8_t id, uint16_t *torque_0_1pct)
{
    return tue_sts3215_read_u16(dev, id, STS3215_REG_TORQUE_LIMIT, torque_0_1pct);
}

tue_sts3215_status_t tue_sts3215_stop(tue_sts3215_t *dev, uint8_t id)
{
    return tue_sts3215_set_running_speed(dev, id, 0);
}

/* Feedback & telemetry ------------------------------------------------------*/

tue_sts3215_status_t tue_sts3215_get_current_position(tue_sts3215_t *dev, uint8_t id, int16_t *position)
{
    return tue_sts3215_read_i16(dev, id, STS3215_REG_CURRENT_POS, position);
}

tue_sts3215_status_t tue_sts3215_get_current_speed(tue_sts3215_t *dev, uint8_t id, int16_t *speed)
{
    return tue_sts3215_read_i16(dev, id, STS3215_REG_CURRENT_SPEED, speed);
}

tue_sts3215_status_t tue_sts3215_get_current_load(tue_sts3215_t *dev, uint8_t id, int16_t *load_0_1pct)
{
    return tue_sts3215_read_i16(dev, id, STS3215_REG_CURRENT_LOAD, load_0_1pct);
}

tue_sts3215_status_t tue_sts3215_get_current_voltage(tue_sts3215_t *dev, uint8_t id, uint8_t *voltage_0_1v)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_CURRENT_VOLTAGE, voltage_0_1v);
}

tue_sts3215_status_t tue_sts3215_get_voltage_volts(tue_sts3215_t *dev, uint8_t id, float *volts)
{
    if (volts == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t raw = 0U;
    tue_sts3215_status_t st = tue_sts3215_get_current_voltage(dev, id, &raw);
    if (st == TUE_STS3215_OK) {
        *volts = tue_sts3215_raw_to_volts(raw);
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_get_current_temperature(tue_sts3215_t *dev, uint8_t id, int8_t *temp_c)
{
    if (temp_c == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t raw = 0U;
    tue_sts3215_status_t st = sts_read(dev, id, STS3215_REG_CURRENT_TEMP, &raw, 1U);
    if (st == TUE_STS3215_OK || st == TUE_STS3215_ERR_STATUS) {
        *temp_c = (int8_t)raw;
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_get_async_write_flag(tue_sts3215_t *dev, uint8_t id, uint8_t *flag)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_ASYNC_FLAG, flag);
}

tue_sts3215_status_t tue_sts3215_get_servo_status(tue_sts3215_t *dev, uint8_t id, uint8_t *status_mask)
{
    return tue_sts3215_read_u8(dev, id, STS3215_REG_SERVO_STATUS, status_mask);
}

tue_sts3215_status_t tue_sts3215_is_moving(tue_sts3215_t *dev, uint8_t id, bool *is_moving)
{
    if (is_moving == NULL) return TUE_STS3215_ERR_PARAM;
    uint8_t val = 0U;
    tue_sts3215_status_t st = tue_sts3215_read_u8(dev, id, STS3215_REG_MOVING, &val);
    if (st == TUE_STS3215_OK) {
        *is_moving = (val != 0U);
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_get_motor_current_raw(tue_sts3215_t *dev, uint8_t id, int16_t *current_6_5ma)
{
    return tue_sts3215_read_i16(dev, id, STS3215_REG_CURRENT_CURRENT, current_6_5ma);
}

tue_sts3215_status_t tue_sts3215_get_motor_current_ma(tue_sts3215_t *dev, uint8_t id, float *current_ma)
{
    if (current_ma == NULL) return TUE_STS3215_ERR_PARAM;
    int16_t raw = 0;
    tue_sts3215_status_t st = tue_sts3215_get_motor_current_raw(dev, id, &raw);
    if (st == TUE_STS3215_OK) {
        *current_ma = tue_sts3215_raw_to_milliamps((uint16_t)raw);
    }
    return st;
}

tue_sts3215_status_t tue_sts3215_get_feedback(tue_sts3215_t *dev, uint8_t id, tue_sts3215_feedback_t *feedback)
{
    if (feedback == NULL) return TUE_STS3215_ERR_PARAM;

    /* Burst read 15 bytes 0x38..0x46: pos(2) spd(2) load(2) volt(1) temp(1)
     * async(1) status(1) moving(1) reserved(2) current(2). */
    uint8_t raw[15];
    tue_sts3215_status_t st = sts_read(dev, id, STS3215_REG_CURRENT_POS, raw, 15U);
    if (st != TUE_STS3215_OK && st != TUE_STS3215_ERR_STATUS) {
        return st;
    }

    feedback->id               = id;
    feedback->position         = tue_sts3215_unpack_s16_le(&raw[0]);
    feedback->speed            = tue_sts3215_unpack_s16_le(&raw[2]);
    feedback->load             = tue_sts3215_unpack_s16_le(&raw[4]);
    feedback->voltage_raw      = raw[6];
    feedback->voltage_v        = tue_sts3215_raw_to_volts(raw[6]);
    feedback->temperature_degc = (int8_t)raw[7];
    feedback->async_flag       = raw[8];
    feedback->status_mask      = raw[9];
    feedback->is_moving        = raw[10];
    feedback->current_raw      = tue_sts3215_unpack_s16_le(&raw[13]);
    feedback->current_ma       = tue_sts3215_raw_to_milliamps((uint16_t)feedback->current_raw);

    return st;
}

/* Multi-servo synchronized commands -----------------------------------------*/

tue_sts3215_status_t tue_sts3215_sync_write_position(tue_sts3215_t *dev, const tue_sts3215_sync_pos_t *targets, uint8_t count)
{
    if (targets == NULL || count == 0U || count > STS3215_SYNC_MAX_SERVOS) {
        return TUE_STS3215_ERR_PARAM;
    }

    tue_sts3215_sync_entry_t entries[STS3215_SYNC_MAX_SERVOS];
    for (uint8_t i = 0U; i < count; i++) {
        uint16_t pos = (uint16_t)targets[i].position;
        uint16_t spd = (uint16_t)targets[i].speed;
        entries[i].id = targets[i].id;
        entries[i].data[0] = targets[i].accel;
        entries[i].data[1] = (uint8_t)(pos & 0xFFU);
        entries[i].data[2] = (uint8_t)((pos >> 8U) & 0xFFU);
        entries[i].data[3] = 0U; /* running time low */
        entries[i].data[4] = 0U; /* running time high */
        entries[i].data[5] = (uint8_t)(spd & 0xFFU);
        entries[i].data[6] = (uint8_t)((spd >> 8U) & 0xFFU);
    }

    return sts_sync_write(dev, STS3215_REG_ACCELERATION, 7U, entries, count);
}

tue_sts3215_status_t tue_sts3215_sync_write_speed(tue_sts3215_t *dev, const tue_sts3215_sync_speed_t *targets, uint8_t count)
{
    if (targets == NULL || count == 0U || count > STS3215_SYNC_MAX_SERVOS) {
        return TUE_STS3215_ERR_PARAM;
    }

    tue_sts3215_sync_entry_t entries[STS3215_SYNC_MAX_SERVOS];
    for (uint8_t i = 0U; i < count; i++) {
        uint16_t spd = (uint16_t)targets[i].speed;
        entries[i].id = targets[i].id;
        entries[i].data[0] = (uint8_t)(spd & 0xFFU);
        entries[i].data[1] = (uint8_t)((spd >> 8U) & 0xFFU);
    }

    return sts_sync_write(dev, STS3215_REG_RUNNING_SPEED, 2U, entries, count);
}

tue_sts3215_status_t tue_sts3215_sync_write_torque_enable(tue_sts3215_t *dev, const uint8_t *ids, const uint8_t *enables, uint8_t count)
{
    if (ids == NULL || enables == NULL || count == 0U || count > STS3215_SYNC_MAX_SERVOS) {
        return TUE_STS3215_ERR_PARAM;
    }

    tue_sts3215_sync_entry_t entries[STS3215_SYNC_MAX_SERVOS];
    for (uint8_t i = 0U; i < count; i++) {
        entries[i].id = ids[i];
        entries[i].data[0] = enables[i] ? 1U : 0U;
    }

    return sts_sync_write(dev, STS3215_REG_TORQUE_SWITCH, 1U, entries, count);
}
