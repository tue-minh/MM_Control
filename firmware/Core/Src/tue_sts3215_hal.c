/**
  ******************************************************************************
  * @file    tue_sts3215_hal.c
  * @brief   STS3215 STM32 HAL layer implementation.
  ******************************************************************************
  * Ported from the reference C library (reference_for_sts325/stm32-sts3215-lib)
  * and adapted to single-wire half-duplex USART1.
  *
  * State machine:
  *
  *  [IDLE]
  *    |  tue_sts3215_hal_send_frame()
  *    |  +- copy frame -> tx_buf
  *    |  +- enable transmitter, HAL_UART_Transmit_DMA()
  *    v
  *  [TX_BUSY]
  *    |  HAL_UART_TxCpltCallback -> tue_sts3215_hal_tx_cplt_cb()
  *    |  +- is_broadcast -> state = IDLE (done)
  *    |  +- else: flush RX noise, enable receiver,
  *    |           HAL_UARTEx_ReceiveToIdle_DMA(rx_buf) -> state = RX_BUSY
  *    v
  *  [RX_BUSY]
  *    |
  *    +- HAL_UARTEx_RxEventCallback(size) -> tue_sts3215_hal_rx_event_cb()
  *    |    +- parse each expected reply -> on_reply()
  *    |    +- (all parsed) state = IDLE   else re-arm RX
  *    |
  *    +- tue_sts3215_hal_process(): timeout -> HAL_UART_DMAStop() + on_error()
  ******************************************************************************
  */

#include "tue_sts3215_hal.h"
#include <string.h>

/* =========================================================================
 * Static instance registry (single-bus architecture)
 * ========================================================================= */

static tue_sts3215_hal_t *s_instance = NULL;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
  * @brief Drop any stale bytes/flags left over from the TX half of the
  *        half-duplex transaction before arming the receiver.
  */
static void prv_prepare_rx(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    (void)huart->Instance->SR;
    (void)huart->Instance->DR;
}

/**
  * @brief Arm DMA reception terminated by the IDLE line event.
  */
static void prv_arm_rx(tue_sts3215_hal_t *hal)
{
    memset(hal->rx_buf, 0, TUE_STS3215_HAL_RX_BUF_SIZE);
    hal->rx_received_len = 0U;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(hal->huart, hal->rx_buf,
                                       (uint16_t)TUE_STS3215_HAL_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(hal->huart->hdmarx, DMA_IT_HT);
}

/**
  * @brief Parse all expected reply frames from the accumulated RX buffer.
  *        Called from IRQ context.
  */
static uint8_t prv_parse_rx_buffer(tue_sts3215_hal_t *hal, uint16_t buf_len)
{
    uint16_t offset = hal->rx_consumed_len;
    uint8_t  parsed = 0U;
    uint8_t  budget = (uint8_t)(hal->expected_replies - hal->replies_parsed);

    while ((parsed < budget) && (offset < buf_len)) {

        uint16_t remaining = (uint16_t)(buf_len - offset);
        tue_sts3215_reply_t reply;

        tue_sts3215_proto_status_t status = tue_sts3215_parse_reply(
                &hal->rx_accum[offset], remaining, &reply);

        if (status == TUE_STS3215_PROTO_OK || status == TUE_STS3215_PROTO_ERR_SERVO_FAULT) {
            if (hal->on_reply != NULL) {
                hal->on_reply(&reply, parsed, status, hal->user_ctx);
            }
            offset = (uint16_t)(offset + 6U + reply.data_len);
            parsed++;

        } else {
            if (status == TUE_STS3215_PROTO_ERR_FRAME_SHORT || remaining < 6U) {
                break;
            }
            if (hal->on_error != NULL) {
                hal->on_error(TUE_STS3215_HAL_ERR_PARSE, hal->user_ctx);
            }
            break;
        }
    }
    hal->rx_consumed_len = offset;
    return parsed;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void tue_sts3215_hal_init(tue_sts3215_hal_t *hal,
                          UART_HandleTypeDef *huart,
                          uint32_t reply_timeout_ms,
                          tue_sts3215_hal_reply_cb_t on_reply,
                          tue_sts3215_hal_error_cb_t on_error,
                          void *user_ctx)
{
    if (hal == NULL || huart == NULL) {
        return;
    }

    memset(hal, 0, sizeof(tue_sts3215_hal_t));

    hal->huart = huart;
    hal->reply_timeout_ms = (reply_timeout_ms > 0U) ? reply_timeout_ms
                                                    : TUE_STS3215_HAL_REPLY_TIMEOUT_MS;
    hal->tx_timeout_ms = TUE_STS3215_HAL_TX_TIMEOUT_MS;
    hal->on_reply = on_reply;
    hal->on_error = on_error;
    hal->user_ctx = user_ctx;
    hal->state = TUE_STS3215_HAL_STATE_IDLE;
    hal->last_error = TUE_STS3215_HAL_ERR_NONE;
}

void tue_sts3215_hal_register_instance(tue_sts3215_hal_t *hal)
{
    s_instance = hal;
}

tue_sts3215_proto_status_t tue_sts3215_hal_send_frame(tue_sts3215_hal_t *hal,
                                                     const uint8_t *frame,
                                                     uint16_t len,
                                                     bool is_broadcast,
                                                     uint8_t expected_replies)
{
    if (hal == NULL || frame == NULL) {
        return TUE_STS3215_PROTO_ERR_NULL_PTR;
    }

    if (hal->state != TUE_STS3215_HAL_STATE_IDLE) {
        if (hal->on_error != NULL) {
            hal->on_error(TUE_STS3215_HAL_ERR_BUSY, hal->user_ctx);
        }
        return TUE_STS3215_PROTO_ERR_INVALID_PARAM;
    }

    if (len > (uint16_t)STS3215_TX_BUF_SIZE) {
        return TUE_STS3215_PROTO_ERR_INVALID_PARAM;
    }

    memcpy(hal->tx_buf, frame, len);

    hal->is_broadcast      = is_broadcast;
    hal->expected_replies  = is_broadcast ? 0U : expected_replies;
    hal->replies_parsed    = 0U;
    hal->rx_accum_len      = 0U;
    hal->rx_consumed_len   = 0U;
    hal->state             = TUE_STS3215_HAL_STATE_TX_BUSY;
    hal->tx_start_ms       = HAL_GetTick();

    /* Half-duplex: drive the bus now, switch to receive on TX complete. */
    (void)HAL_HalfDuplex_EnableTransmitter(hal->huart);

    if (HAL_UART_Transmit_DMA(hal->huart, hal->tx_buf, len) != HAL_OK) {
        hal->state = TUE_STS3215_HAL_STATE_IDLE;
        hal->last_error = TUE_STS3215_HAL_ERR_DMA_TX;
        if (hal->on_error != NULL) {
            hal->on_error(TUE_STS3215_HAL_ERR_DMA_TX, hal->user_ctx);
        }
        return TUE_STS3215_PROTO_ERR_INVALID_PARAM;
    }

    return TUE_STS3215_PROTO_OK;
}

void tue_sts3215_hal_process(tue_sts3215_hal_t *hal)
{
    if (hal == NULL) {
        return;
    }

    if (hal->state == TUE_STS3215_HAL_STATE_TX_BUSY) {
        uint32_t elapsed = HAL_GetTick() - hal->tx_start_ms;
        if (elapsed >= hal->tx_timeout_ms) {
            (void)HAL_UART_DMAStop(hal->huart);
            __HAL_UART_FLUSH_DRREGISTER(hal->huart);

            hal->last_error = TUE_STS3215_HAL_ERR_TX_TIMEOUT;
            hal->state      = TUE_STS3215_HAL_STATE_IDLE;

            if (hal->on_error != NULL) {
                hal->on_error(TUE_STS3215_HAL_ERR_TX_TIMEOUT, hal->user_ctx);
            }
        }
        return;
    }

    if (hal->state == TUE_STS3215_HAL_STATE_RX_BUSY) {
        uint32_t elapsed = HAL_GetTick() - hal->tx_timestamp_ms;
        if (elapsed >= hal->reply_timeout_ms) {
            (void)HAL_UART_DMAStop(hal->huart);
            __HAL_UART_FLUSH_DRREGISTER(hal->huart);

            hal->last_error = TUE_STS3215_HAL_ERR_TIMEOUT;
            hal->state      = TUE_STS3215_HAL_STATE_IDLE;

            if (hal->on_error != NULL) {
                hal->on_error(TUE_STS3215_HAL_ERR_TIMEOUT, hal->user_ctx);
            }
        }
    }
}

void tue_sts3215_hal_abort(tue_sts3215_hal_t *hal)
{
    if (hal == NULL) {
        return;
    }

    (void)HAL_UART_DMAStop(hal->huart);
    __HAL_UART_FLUSH_DRREGISTER(hal->huart);

    hal->state      = TUE_STS3215_HAL_STATE_IDLE;
    hal->last_error = TUE_STS3215_HAL_ERR_NONE;
}

tue_sts3215_hal_state_t tue_sts3215_hal_get_state(const tue_sts3215_hal_t *hal)
{
    return (hal != NULL) ? hal->state : TUE_STS3215_HAL_STATE_ERROR;
}

bool tue_sts3215_hal_is_idle(const tue_sts3215_hal_t *hal)
{
    return (hal != NULL) && (hal->state == TUE_STS3215_HAL_STATE_IDLE);
}

/* =========================================================================
 * IRQ-context entry points
 * ========================================================================= */

void tue_sts3215_hal_tx_cplt_cb(tue_sts3215_hal_t *hal)
{
    if (hal == NULL || hal->state != TUE_STS3215_HAL_STATE_TX_BUSY) {
        return;
    }

    if (hal->is_broadcast || hal->expected_replies == 0U) {
        hal->state = TUE_STS3215_HAL_STATE_IDLE;
        return;
    }

    /* Single-wire half-duplex turnaround: release the bus, flush the RX side
     * of any artifacts produced while transmitting, then listen. */
    prv_prepare_rx(hal->huart);
    (void)HAL_HalfDuplex_EnableReceiver(hal->huart);
    prv_arm_rx(hal);

    hal->tx_timestamp_ms = HAL_GetTick();
    hal->state = TUE_STS3215_HAL_STATE_RX_BUSY;
}

void tue_sts3215_hal_rx_event_cb(tue_sts3215_hal_t *hal, uint16_t size)
{
    if (hal == NULL || hal->state != TUE_STS3215_HAL_STATE_RX_BUSY) {
        return;
    }

    if ((uint32_t)hal->rx_accum_len + size <= (uint32_t)TUE_STS3215_HAL_RX_BUF_SIZE) {
        memcpy(&hal->rx_accum[hal->rx_accum_len], hal->rx_buf, size);
        hal->rx_accum_len = (uint16_t)(hal->rx_accum_len + size);
    } else {
        hal->last_error = TUE_STS3215_HAL_ERR_PARSE;
        hal->replies_parsed = hal->expected_replies;
    }

    hal->rx_received_len = hal->rx_accum_len;

    uint16_t newly_parsed = prv_parse_rx_buffer(hal, hal->rx_accum_len);
    hal->replies_parsed = (uint8_t)(hal->replies_parsed + newly_parsed);

    if (hal->replies_parsed >= hal->expected_replies) {
        hal->state = TUE_STS3215_HAL_STATE_IDLE;
    } else {
        prv_arm_rx(hal);
    }
}

void tue_sts3215_hal_error_cb(tue_sts3215_hal_t *hal)
{
    if (hal == NULL) {
        return;
    }

    (void)HAL_UART_DMAStop(hal->huart);
    __HAL_UART_FLUSH_DRREGISTER(hal->huart);

    tue_sts3215_hal_error_t err = (hal->state == TUE_STS3215_HAL_STATE_TX_BUSY)
                                  ? TUE_STS3215_HAL_ERR_DMA_TX
                                  : TUE_STS3215_HAL_ERR_DMA_RX;

    hal->last_error = err;
    hal->state      = TUE_STS3215_HAL_STATE_IDLE;

    if (hal->on_error != NULL) {
        hal->on_error(err, hal->user_ctx);
    }
}

/* =========================================================================
 * Weak HAL callback overrides
 *
 * These override the __weak symbols in stm32f4xx_hal_uart.c. If another
 * translation unit needs to own these callbacks, define
 * TUE_STS3215_HAL_NO_WEAK_OVERRIDE in the build and route to the
 * tue_sts3215_hal_*_cb() entry points manually.
 * ========================================================================= */

#ifndef TUE_STS3215_HAL_NO_WEAK_OVERRIDE

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_instance != NULL) && (s_instance->huart == huart)) {
        tue_sts3215_hal_tx_cplt_cb(s_instance);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if ((s_instance != NULL) && (s_instance->huart == huart)) {
        tue_sts3215_hal_rx_event_cb(s_instance, size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_instance != NULL) && (s_instance->huart == huart)) {
        tue_sts3215_hal_error_cb(s_instance);
    }
}

#endif /* TUE_STS3215_HAL_NO_WEAK_OVERRIDE */
