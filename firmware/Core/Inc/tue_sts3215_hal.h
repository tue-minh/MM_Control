/**
  ******************************************************************************
  * @file    tue_sts3215_hal.h
  * @brief   STS3215 STM32 HAL layer (half-duplex USART + DMA + IDLE line).
  ******************************************************************************
  * Ported from the reference C library (reference_for_sts325/stm32-sts3215-lib),
  * adapted to this board's hardware:
  *   - USART1 single-wire half-duplex (HDSEL) on PA9 (SERVO_TX).
  *   - DMA2 Stream 7 (TX) and DMA2 Stream 2 (RX), both in Normal mode.
  *   - Receive via HAL_UARTEx_ReceiveToIdle_DMA (IDLE line frame termination).
  *
  * The reference library targets RS485 (DE pin), where the receiver can be
  * armed before TX. With single-wire half-duplex the transceiver direction is
  * switched inside the TX-complete callback instead.
  ******************************************************************************
  */

#ifndef TUE_STS3215_HAL_H
#define TUE_STS3215_HAL_H

#include "main.h"
#include "tue_sts3215_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Compile-time configuration
 * ========================================================================= */

/** Timeout waiting for a reply after TX complete. */
#ifndef TUE_STS3215_HAL_REPLY_TIMEOUT_MS
#define TUE_STS3215_HAL_REPLY_TIMEOUT_MS   (10U)
#endif

/** Timeout while a DMA TX transfer is in flight. */
#ifndef TUE_STS3215_HAL_TX_TIMEOUT_MS
#define TUE_STS3215_HAL_TX_TIMEOUT_MS      (5U)
#endif

/** RX DMA buffer: enough for STS3215_SYNC_MAX_SERVOS full reply frames. */
#define TUE_STS3215_HAL_RX_BUF_SIZE (STS3215_SYNC_MAX_SERVOS * STS3215_RX_BUF_SIZE)

/* =========================================================================
 * Types
 * ========================================================================= */

typedef enum {
    TUE_STS3215_HAL_STATE_IDLE = 0,
    TUE_STS3215_HAL_STATE_TX_BUSY,
    TUE_STS3215_HAL_STATE_RX_BUSY,
    TUE_STS3215_HAL_STATE_ERROR
} tue_sts3215_hal_state_t;

typedef enum {
    TUE_STS3215_HAL_ERR_NONE = 0,
    TUE_STS3215_HAL_ERR_TIMEOUT,
    TUE_STS3215_HAL_ERR_DMA_TX,
    TUE_STS3215_HAL_ERR_DMA_RX,
    TUE_STS3215_HAL_ERR_PARSE,
    TUE_STS3215_HAL_ERR_TX_TIMEOUT,
    TUE_STS3215_HAL_ERR_BUSY
} tue_sts3215_hal_error_t;

/**
  * @brief  Reply callback.
  * @param  reply      Parsed reply
  * @param  reply_idx  0-based index of this reply in the batch
  * @param  status     TUE_STS3215_PROTO_OK or TUE_STS3215_PROTO_ERR_SERVO_FAULT
  * @param  user_ctx   Opaque pointer from tue_sts3215_hal_init()
  */
typedef void (*tue_sts3215_hal_reply_cb_t)(const tue_sts3215_reply_t *reply,
                                           uint8_t reply_idx,
                                           tue_sts3215_proto_status_t status,
                                           void *user_ctx);

/**
  * @brief  Error callback.
  */
typedef void (*tue_sts3215_hal_error_cb_t)(tue_sts3215_hal_error_t err, void *user_ctx);

/** STS3215 HAL driver handle. One instance per physical bus. */
typedef struct {
    UART_HandleTypeDef *huart;

    /* Buffers - must remain valid for the full DMA transfer lifetime */
    uint8_t tx_buf[STS3215_TX_BUF_SIZE];
    uint8_t rx_buf[TUE_STS3215_HAL_RX_BUF_SIZE];
    uint8_t rx_accum[TUE_STS3215_HAL_RX_BUF_SIZE];

    /* State machine */
    volatile tue_sts3215_hal_state_t state;
    volatile tue_sts3215_hal_error_t last_error;
    volatile uint16_t rx_received_len;

    uint16_t rx_accum_len;
    uint16_t rx_consumed_len;

    /* Transfer metadata */
    bool     is_broadcast;
    uint8_t  expected_replies;
    uint8_t  replies_parsed;

    uint32_t tx_start_ms;
    uint32_t tx_timestamp_ms;
    uint32_t tx_timeout_ms;
    uint32_t reply_timeout_ms;

    tue_sts3215_hal_reply_cb_t on_reply;
    tue_sts3215_hal_error_cb_t on_error;
    void                      *user_ctx;
} tue_sts3215_hal_t;

/* =========================================================================
 * API
 * ========================================================================= */

/**
  * @brief  Initialise the HAL handle. Call once before any other function.
  */
void tue_sts3215_hal_init(tue_sts3215_hal_t *hal,
                          UART_HandleTypeDef *huart,
                          uint32_t reply_timeout_ms,
                          tue_sts3215_hal_reply_cb_t on_reply,
                          tue_sts3215_hal_error_cb_t on_error,
                          void *user_ctx);

/**
  * @brief  Register the instance the weak HAL callbacks route to.
  */
void tue_sts3215_hal_register_instance(tue_sts3215_hal_t *hal);

/**
  * @brief  Send a pre-built packet frame over DMA (non-blocking).
  *         The frame is copied into tx_buf, so the caller buffer can be reused.
  * @return TUE_STS3215_PROTO_OK on success.
  */
tue_sts3215_proto_status_t tue_sts3215_hal_send_frame(tue_sts3215_hal_t *hal,
                                                     const uint8_t *frame,
                                                     uint16_t len,
                                                     bool is_broadcast,
                                                     uint8_t expected_replies);

/** Periodic tick - call from main loop or RTOS task (not from IRQ). */
void tue_sts3215_hal_process(tue_sts3215_hal_t *hal);

/** Abort any in-progress transfer and reset to IDLE. */
void tue_sts3215_hal_abort(tue_sts3215_hal_t *hal);

/** Return current state machine state. */
tue_sts3215_hal_state_t tue_sts3215_hal_get_state(const tue_sts3215_hal_t *hal);

/** Return true only when the driver is idle and ready for a new frame. */
bool tue_sts3215_hal_is_idle(const tue_sts3215_hal_t *hal);

/* =========================================================================
 * IRQ-context entry points
 * ========================================================================= */

void tue_sts3215_hal_tx_cplt_cb(tue_sts3215_hal_t *hal);
void tue_sts3215_hal_rx_event_cb(tue_sts3215_hal_t *hal, uint16_t size);
void tue_sts3215_hal_error_cb(tue_sts3215_hal_t *hal);

#ifdef __cplusplus
}
#endif

#endif /* TUE_STS3215_HAL_H */
