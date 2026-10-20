/**
  ******************************************************************************
  * @file    tue_haptic_bridge.c
  * @brief   USB CDC -> UART haptic telemetry forwarding engine.
  ******************************************************************************
  * @attention
  *
  * Transmission: 2-slot ping-pong buffer + HAL_UART_Transmit_DMA(). Never
  * blocks and never hands a stack pointer to the DMA engine.
  *
  * Reception: one circular DMA buffer armed with
  * HAL_UARTEx_ReceiveToIdle_DMA(). The IDLE event reports the current DMA
  * write position, so the callback copies the delta into a byte ring that
  * application code can drain with tue_haptic_rx_pop().
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tue_haptic_bridge.h"

#include <string.h>

#include "tue_usb_binary.h"   /* tue_usb_crc8() */

/* Private variables ---------------------------------------------------------*/
volatile tue_haptic_payload_t g_tue_haptic_payload;
volatile uint32_t g_tue_haptic_timestamp_ms = 0U;
volatile uint32_t g_tue_haptic_frame_count  = 0U;
volatile uint32_t g_tue_haptic_tx_drops     = 0U;
volatile uint32_t g_tue_haptic_slip_alerts  = 0U;
volatile uint32_t g_tue_haptic_rx_bytes     = 0U;
volatile uint32_t g_tue_haptic_rx_errors    = 0U;

static bool s_initialized = false;

/* Transmit ping-pong buffers. Both live in .bss for the DMA lifetime. */
static uint8_t s_tx_slots[TUE_HAPTIC_TX_SLOTS][TUE_HAPTIC_FRAME_LEN];
static volatile uint8_t s_tx_active  = 0U;
static volatile bool    s_tx_busy    = false;
static volatile bool    s_tx_pending = false;

/* Receive path. */
static uint8_t s_rx_dma_buf[TUE_HAPTIC_RX_RING_SIZE];
static uint8_t s_rx_ring[TUE_HAPTIC_RX_RING_SIZE];
static volatile uint16_t s_rx_head     = 0U;
static volatile uint16_t s_rx_tail     = 0U;
static volatile uint16_t s_rx_last_pos = 0U;
static volatile bool     s_rx_full_seen = false;

/* Private helpers -----------------------------------------------------------*/

static void rx_ring_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) & (TUE_HAPTIC_RX_RING_SIZE - 1U));
    if (next == s_rx_tail) {
        return; /* ring full: drop newest byte */
    }
    s_rx_ring[s_rx_head] = byte;
    s_rx_head = next;
    g_tue_haptic_rx_bytes++;
}

static void rx_arm(void)
{
    s_rx_last_pos  = 0U;
    s_rx_full_seen = false;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&TUE_HAPTIC_UART, s_rx_dma_buf,
                                     (uint16_t)TUE_HAPTIC_RX_RING_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(TUE_HAPTIC_UART.hdmarx, DMA_IT_HT);
    }
}

static void rx_clear_flags(void)
{
    __HAL_UART_CLEAR_OREFLAG(&TUE_HAPTIC_UART);
    __HAL_UART_CLEAR_NEFLAG(&TUE_HAPTIC_UART);
    __HAL_UART_CLEAR_FEFLAG(&TUE_HAPTIC_UART);
    __HAL_UART_CLEAR_PEFLAG(&TUE_HAPTIC_UART);
}

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

bool tue_haptic_init(void)
{
    if (s_initialized) {
        return true;
    }

    s_tx_active  = 0U;
    s_tx_busy    = false;
    s_tx_pending = false;

    s_rx_head     = 0U;
    s_rx_tail     = 0U;
    s_rx_last_pos = 0U;
    s_rx_full_seen = false;
    memset(s_rx_ring, 0, sizeof(s_rx_ring));

    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    rx_clear_flags();

    if (HAL_UARTEx_ReceiveToIdle_DMA(&TUE_HAPTIC_UART, s_rx_dma_buf,
                                     (uint16_t)TUE_HAPTIC_RX_RING_SIZE) != HAL_OK) {
        return false;
    }
    __HAL_DMA_DISABLE_IT(TUE_HAPTIC_UART.hdmarx, DMA_IT_HT);

    s_initialized = true;
    return true;
}

bool tue_haptic_forward(const uint8_t *frame, uint16_t len)
{
    if ((frame == NULL) || (len != TUE_HAPTIC_FRAME_LEN) || !s_initialized) {
        return false;
    }

    /* This function runs in the USB protocol task, while the TX-complete
     * callback runs in the DMA ISR. Mask interrupts for the short
     * check/copy/start sequence so a completion cannot pick up a slot that is
     * still being filled. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool accepted = true;

    if (s_tx_busy) {
        /* In-flight transfer owns s_tx_active; the other slot is free. */
        uint8_t slot = (uint8_t)(s_tx_active ^ 1U);
        memcpy(s_tx_slots[slot], frame, TUE_HAPTIC_FRAME_LEN);
        s_tx_pending = true;
    } else {
        uint8_t slot = (uint8_t)(s_tx_active ^ 1U);
        memcpy(s_tx_slots[slot], frame, TUE_HAPTIC_FRAME_LEN);
        if (HAL_UART_Transmit_DMA(&TUE_HAPTIC_UART, s_tx_slots[slot],
                                  TUE_HAPTIC_FRAME_LEN) == HAL_OK) {
            s_tx_active = slot;
            s_tx_busy   = true;
        } else {
            accepted = false;
        }
    }

    if (!accepted) {
        g_tue_haptic_tx_drops++;
    }

    __set_PRIMASK(primask);
    return accepted;
}

bool tue_haptic_ingest(const uint8_t *raw, uint16_t len)
{
    tue_haptic_wire_frame_t f;

    if ((raw == NULL) || (len != TUE_HAPTIC_FRAME_LEN)) {
        return false;
    }

    memcpy(&f, raw, sizeof(f));

    if ((f.sof1 != TUE_HAPTIC_SOF1) || (f.sof2 != TUE_HAPTIC_SOF2) ||
        (f.eof1 != TUE_HAPTIC_EOF1) || (f.eof2 != TUE_HAPTIC_EOF2) ||
        (f.frame_type != TUE_HAPTIC_FRAME_TYPE_REQ) ||
        (f.can_id != TUE_HAPTIC_CAN_ID) ||
        (f.dlc != TUE_HAPTIC_PAYLOAD_LEN)) {
        return false;
    }

    if (tue_usb_crc8(&raw[2], (size_t)(TUE_HAPTIC_HEADER_LEN + TUE_HAPTIC_PAYLOAD_LEN))
        != f.crc8) {
        return false;
    }

    /* Local telemetry cache for reflex control / logging. */
    g_tue_haptic_payload.fz_left       = f.payload.fz_left;
    g_tue_haptic_payload.fz_right      = f.payload.fz_right;
    g_tue_haptic_payload.f_shear_avg   = f.payload.f_shear_avg;
    g_tue_haptic_payload.weight_tip    = f.payload.weight_tip;
    g_tue_haptic_payload.weight_mid    = f.payload.weight_mid;
    g_tue_haptic_payload.weight_base   = f.payload.weight_base;
    g_tue_haptic_payload.status_flags  = f.payload.status_flags;
    g_tue_haptic_payload.frame_counter = f.payload.frame_counter;

    g_tue_haptic_timestamp_ms = HAL_GetTick();
    g_tue_haptic_frame_count++;

    if ((f.payload.status_flags & TUE_HAPTIC_STATUS_SLIP_ALERT) != 0U) {
        g_tue_haptic_slip_alerts++;
    }

    /* Transparent pass-through of the exact 20 raw bytes. */
    return tue_haptic_forward(raw, len);
}

void tue_haptic_reset_stats(void)
{
    g_tue_haptic_timestamp_ms = 0U;
    g_tue_haptic_frame_count  = 0U;
    g_tue_haptic_tx_drops     = 0U;
    g_tue_haptic_slip_alerts  = 0U;
    g_tue_haptic_rx_bytes     = 0U;
    g_tue_haptic_rx_errors    = 0U;

    g_tue_haptic_payload.fz_left       = 0U;
    g_tue_haptic_payload.fz_right      = 0U;
    g_tue_haptic_payload.f_shear_avg   = 0U;
    g_tue_haptic_payload.weight_tip    = 0U;
    g_tue_haptic_payload.weight_mid    = 0U;
    g_tue_haptic_payload.weight_base   = 0U;
    g_tue_haptic_payload.status_flags  = 0U;
    g_tue_haptic_payload.frame_counter = 0U;

    s_rx_tail     = s_rx_head;
    s_rx_last_pos = 0U;
    s_rx_full_seen = false;
}

uint16_t tue_haptic_rx_available(void)
{
    return (uint16_t)((s_rx_head - s_rx_tail) & (TUE_HAPTIC_RX_RING_SIZE - 1U));
}

bool tue_haptic_rx_pop(uint8_t *byte)
{
    if ((byte == NULL) || (s_rx_head == s_rx_tail)) {
        return false;
    }
    *byte = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) & (TUE_HAPTIC_RX_RING_SIZE - 1U));
    return true;
}

/* ========================================================================== */
/* IRQ-context entry points                                                   */
/* ========================================================================== */

void tue_haptic_uart_tx_cplt_cb(UART_HandleTypeDef *huart)
{
    if (huart != &TUE_HAPTIC_UART) {
        return;
    }

    s_tx_busy = false;

    if (s_tx_pending) {
        uint8_t slot = (uint8_t)(s_tx_active ^ 1U);
        s_tx_pending = false;
        if (HAL_UART_Transmit_DMA(&TUE_HAPTIC_UART, s_tx_slots[slot],
                                  TUE_HAPTIC_FRAME_LEN) == HAL_OK) {
            s_tx_active = slot;
            s_tx_busy   = true;
        } else {
            g_tue_haptic_tx_drops++;
        }
    }
}

void tue_haptic_uart_rx_event_cb(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &TUE_HAPTIC_UART) {
        return;
    }

    /* Circular DMA reports the current write position, not the delta.
     * HAL may notify a full buffer twice (DMA TC followed by the IDLE path),
     * so a repeated "size == full buffer, position == 0" event is ignored. */
    uint16_t end = (size >= TUE_HAPTIC_RX_RING_SIZE) ? TUE_HAPTIC_RX_RING_SIZE : size;

    if (end == TUE_HAPTIC_RX_RING_SIZE) {
        if (s_rx_full_seen && (s_rx_last_pos == 0U)) {
            return;
        }
        s_rx_full_seen = true;
    } else if (end != 0U) {
        s_rx_full_seen = false;
    }

    uint16_t pos = s_rx_last_pos;

    while (pos != end) {
        rx_ring_push(s_rx_dma_buf[pos]);
        pos++;
        if (pos >= TUE_HAPTIC_RX_RING_SIZE) {
            pos = 0U;
        }
    }

    s_rx_last_pos = (end == TUE_HAPTIC_RX_RING_SIZE) ? 0U : end;
}

void tue_haptic_uart_error_cb(UART_HandleTypeDef *huart)
{
    if (huart != &TUE_HAPTIC_UART) {
        return;
    }

    g_tue_haptic_rx_errors++;
    rx_clear_flags();

    /* HAL aborts reception on overrun/framing errors: re-arm if idle. */
    if (huart->RxState != HAL_UART_STATE_BUSY_RX) {
        rx_arm();
    }
}
