/**
  ******************************************************************************
  * @file           : tue_usb_binary.c
  * @brief          : TUE binary CAN-over-USB protocol over USB CDC (Virtual COM Port).
  ******************************************************************************
  * @attention
  *
  * See tue_usb_binary.h for the wire format. In short:
  *
  *   [0xAA][0x55][TYPE][ID_LE(4)][DLC][SEQ][PAYLOAD(DLC)][CRC8][0x55][0xAA]
  *
  * USB reception happens in interrupt context, so CDC_Receive_FS() only pushes
  * bytes into a lock-free ring buffer and releases a semaphore. A dedicated
  * FreeRTOS task drains the ring through a defensive byte-wise state machine
  * (tolerant of fragmentation, concatenation and line noise), executes the
  * command (servo-bus transactions may block) and emits the framed reply.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tue_usb_binary.h"

#include <string.h>
#include <math.h>

#include "cmsis_os2.h"
#include "usart.h"
#include "tue_stepper.h"
#include "tue_haptic_bridge.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Private defines -----------------------------------------------------------*/
#define TUE_USB_RING_SIZE      1024U
#define TUE_USB_FRAME_MAX      TUE_USB_MAX_FRAME            /* 20 bytes        */
#define TUE_USB_MEM_MAX        6U                          /* raw mem read    */
#define TUE_USB_REPLY_SENT     (-1)                        /* already answered */

#define TUE_USB_DEFAULT_FEED_MM_S    60.0f
#define TUE_USB_DEFAULT_ACCEL_MM_S2  600.0f

/* Private types -------------------------------------------------------------*/
typedef enum {
    ST_WAIT_SOF1 = 0,
    ST_WAIT_SOF2,
    ST_HEADER,
    ST_PAYLOAD,
    ST_CRC,
    ST_EOF1,
    ST_EOF2
} tue_usb_rx_state_t;

/* Private variables ---------------------------------------------------------*/
static tue_sts3215_t    s_servo;
static bool             s_initialized = false;

static uint8_t          s_ring[TUE_USB_RING_SIZE];
static volatile uint16_t s_ring_head = 0U;   /* producer (USB ISR)         */
static volatile uint16_t s_ring_tail = 0U;   /* consumer (protocol task)   */

static osSemaphoreId_t  s_rx_sem      = NULL;
static osSemaphoreId_t  s_tx_cplt_sem = NULL;
static osMutexId_t      s_tx_mutex    = NULL;
static osThreadId_t     s_task        = NULL;

/* Parser state. */
static tue_usb_rx_state_t s_state = ST_WAIT_SOF1;
static tue_usb_frame_t    s_rx;
static uint8_t            s_hdr_idx = 0U;    /* header bytes collected     */
static uint8_t            s_pay_idx = 0U;    /* payload bytes collected    */
static uint8_t            s_raw[TUE_USB_MAX_FRAME]; /* exact wire bytes     */
static uint16_t           s_raw_idx = 0U;

/* Transmit double buffer. */
static uint8_t            s_tx[2][TUE_USB_FRAME_MAX];
static uint8_t            s_tx_idx = 0U;

/* Stepper command state. */
static float              s_default_feed  = TUE_USB_DEFAULT_FEED_MM_S;
static float              s_default_accel = TUE_USB_DEFAULT_ACCEL_MM_S2;
static int32_t            s_latch_x_um = 0;
static int32_t            s_latch_y_um = 0;
static bool               s_latch_valid = false;

/* Stepper non-blocking wait-idle state. */
static bool               s_wait_idle_pending = false;
static uint8_t            s_wait_idle_seq     = 0U;
static uint32_t           s_wait_idle_start   = 0U;
static uint32_t           s_wait_idle_timeout = 0U;

/* Private function prototypes -----------------------------------------------*/
static void     usb_task(void *argument);
static void     usb_rx_byte(uint8_t byte);
static void     usb_dispatch(const tue_usb_frame_t *f, const uint8_t *raw,
                             uint16_t raw_len);
static void     usb_transmit(uint8_t frame_type, uint32_t can_id, uint8_t seq,
                             const uint8_t *payload, uint8_t dlc);
static void     usb_check_pending_idle(void);
static bool     usb_ring_push(uint8_t byte);
static bool     usb_ring_pop(uint8_t *byte);

static int      sys_cmd(const tue_usb_frame_t *f, uint8_t *out, uint8_t *out_dlc);
static int      step_cmd(const tue_usb_frame_t *f, uint8_t *out, uint8_t *out_dlc);
static int      servo_cmd(const tue_usb_frame_t *f, uint8_t *out, uint8_t *out_dlc);

static tue_usb_error_t servo_map_err(tue_sts3215_status_t st);
static tue_usb_error_t step_map_err(tue_stepper_status_t st);

/* Endianness / field helpers ------------------------------------------------*/
static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static int16_t  rd_i16(const uint8_t *p) { return (int16_t)rd_u16(p); }
static uint32_t rd_u32(const uint8_t *p)
{
    return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t  rd_i32(const uint8_t *p) { return (int32_t)rd_u32(p); }

static void wr_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFU); p[1] = (uint8_t)((v >> 8) & 0xFFU); }
static void wr_i16(uint8_t *p, int16_t v)  { wr_u16(p, (uint16_t)v); }
static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}
static void wr_i32(uint8_t *p, int32_t v) { wr_u32(p, (uint32_t)v); }

static float    um_to_mm(int32_t um) { return (float)um / 1000.0f; }
static int32_t  mm_to_um(float mm)   { return (int32_t)lroundf(mm * 1000.0f); }

/* ========================================================================== */
/* CRC-8 SAE J1850                                                            */
/* ========================================================================== */

/* MSB-first lookup table for polynomial 0x1D. */
static const uint8_t s_crc8_table[256] = {
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53,
    0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,
    0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4,
    0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19,
    0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,
    0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D,
    0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7,
    0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,
    0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75,
    0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8,
    0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,
    0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F,
    0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66,
    0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,
    0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1,
    0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C,
    0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

uint8_t tue_usb_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFFU;
    if (data == NULL) {
        return 0U;
    }
    for (size_t i = 0U; i < len; i++) {
        crc = s_crc8_table[crc ^ data[i]];
    }
    return (uint8_t)(crc ^ 0xFFU);
}

bool tue_usb_crc8_selftest(void)
{
    static const uint8_t vec[] = "123456789";
    return (tue_usb_crc8(vec, sizeof(vec) - 1U) == 0x4BU);
}

/* ========================================================================== */
/* Ring buffer + task                                                         */
/* ========================================================================== */

static bool usb_ring_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_ring_head + 1U) & (TUE_USB_RING_SIZE - 1U));
    if (next == s_ring_tail) {
        return false; /* ring full: drop newest byte */
    }
    s_ring[s_ring_head] = byte;
    s_ring_head = next;
    return true;
}

static bool usb_ring_pop(uint8_t *byte)
{
    if (s_ring_tail == s_ring_head) {
        return false;
    }
    *byte = s_ring[s_ring_tail];
    s_ring_tail = (uint16_t)((s_ring_tail + 1U) & (TUE_USB_RING_SIZE - 1U));
    return true;
}

/* ========================================================================== */
/* Transmit engine                                                            */
/* ========================================================================== */

static void usb_transmit(uint8_t frame_type, uint32_t can_id, uint8_t seq,
                         const uint8_t *payload, uint8_t dlc)
{
    if (dlc > TUE_USB_MAX_DLC) {
        dlc = TUE_USB_MAX_DLC;
    }

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL) {
        return; /* USB not enumerated: nothing to answer to */
    }

    if (s_tx_mutex != NULL) {
        if (osMutexAcquire(s_tx_mutex, 50U) != osOK) {
            return; /* Mutex timeout */
        }
    }

    /* Event-driven synchronization: wait for previous transmission to complete.
     * Unblocks immediately (0µs) when USB hardware triggers CDC_TransmitCplt_FS. */
    if (s_tx_cplt_sem != NULL) {
        if (osSemaphoreAcquire(s_tx_cplt_sem, 20U) != osOK) {
            if (s_tx_mutex != NULL) {
                (void)osMutexRelease(s_tx_mutex);
            }
            return; /* USB endpoint stalled or disconnected */
        }
    }

    uint8_t *tx_buf = s_tx[s_tx_idx];
    s_tx_idx ^= 1U;

    tx_buf[0] = TUE_USB_SOF1;
    tx_buf[1] = TUE_USB_SOF2;

    tue_usb_frame_t *f = (tue_usb_frame_t *)&tx_buf[2];
    f->frame_type = frame_type;
    f->can_id    = can_id;
    f->dlc       = dlc;
    f->seq       = seq;
    if (payload != NULL && dlc > 0U) {
        memcpy(f->payload, payload, dlc);
    } else {
        memset(f->payload, 0, sizeof(f->payload));
    }

    /* The CRC byte sits immediately after the payload, i.e. at offset
     * TUE_USB_HEADER_SIZE + dlc from the frame header - NOT at the fixed
     * struct offset of the packed `crc8` member (which is only correct when
     * dlc == TUE_USB_MAX_DLC). Use the DLC-dependent wire offset so short
     * frames carry a valid CRC too. */
    uint8_t *frame = (uint8_t *)f;
    frame[TUE_USB_HEADER_SIZE + dlc] =
        tue_usb_crc8(frame, (size_t)(TUE_USB_HEADER_SIZE + dlc));

    uint16_t len = (uint16_t)(2U + TUE_USB_HEADER_SIZE + dlc + TUE_USB_CRC_SIZE + 2U);
    tx_buf[len - 2U] = TUE_USB_EOF1;
    tx_buf[len - 1U] = TUE_USB_EOF2;

    if (CDC_Transmit_FS(tx_buf, len) != USBD_OK) {
        /* Failed to initiate transfer: re-release semaphore so next sends don't hang */
        if (s_tx_cplt_sem != NULL) {
            (void)osSemaphoreRelease(s_tx_cplt_sem);
        }
    }

    if (s_tx_mutex != NULL) {
        (void)osMutexRelease(s_tx_mutex);
    }
}

void tue_usb_on_tx_cplt(void)
{
    if (s_tx_cplt_sem != NULL) {
        (void)osSemaphoreRelease(s_tx_cplt_sem);
    }
}

void tue_usb_send_ok(uint32_t can_id, uint8_t seq, const uint8_t *payload, uint8_t dlc)
{
    usb_transmit((uint8_t)TUE_USB_FRAME_RESP_OK, can_id, seq, payload, dlc);
}

void tue_usb_send_err(uint32_t can_id, uint8_t seq, tue_usb_error_t err_code)
{
    uint8_t code = (uint8_t)err_code;
    usb_transmit((uint8_t)TUE_USB_FRAME_RESP_ERR, can_id, seq, &code, 1U);
}

void tue_usb_send_async(uint32_t can_id, uint8_t seq, const uint8_t *payload, uint8_t dlc)
{
    usb_transmit((uint8_t)TUE_USB_FRAME_ASYNC, can_id, seq, payload, dlc);
}

/* ========================================================================== */
/* Receive state machine                                                      */
/* ========================================================================== */

static void usb_reset_parser(void)
{
    s_state   = ST_WAIT_SOF1;
    s_hdr_idx = 0U;
    s_pay_idx = 0U;
    s_raw_idx = 0U;
}

static void usb_rx_byte(uint8_t byte)
{
    switch (s_state) {
    case ST_WAIT_SOF1:
        if (byte == TUE_USB_SOF1) {
            s_state = ST_WAIT_SOF2;
        }
        break;

    case ST_WAIT_SOF2:
        if (byte == TUE_USB_SOF2) {
            memset(&s_rx, 0, sizeof(s_rx));
            s_hdr_idx = 0U;
            s_raw[0] = TUE_USB_SOF1;
            s_raw[1] = TUE_USB_SOF2;
            s_raw_idx = 2U;
            s_state = ST_HEADER;
        } else if (byte != TUE_USB_SOF1) {
            s_state = ST_WAIT_SOF1;
        }
        /* A repeated 0xAA keeps us waiting for 0x55. */
        break;

    case ST_HEADER:
        /* Header wire order: TYPE, CAN_ID[0..3], DLC, SEQ. */
        ((uint8_t *)&s_rx)[s_hdr_idx++] = byte;
        s_raw[s_raw_idx++] = byte;
        if (s_hdr_idx >= TUE_USB_HEADER_SIZE) {
            if (s_rx.dlc > TUE_USB_MAX_DLC) {
                usb_reset_parser(); /* ERR_BAD_FORMAT: DLC out of range */
                break;
            }
            s_pay_idx = 0U;
            s_state = (s_rx.dlc == 0U) ? ST_CRC : ST_PAYLOAD;
        }
        break;

    case ST_PAYLOAD:
        s_rx.payload[s_pay_idx++] = byte;
        s_raw[s_raw_idx++] = byte;
        if (s_pay_idx >= s_rx.dlc) {
            s_state = ST_CRC;
        }
        break;

    case ST_CRC: {
        uint8_t expected = tue_usb_crc8((const uint8_t *)&s_rx,
                                        (size_t)(TUE_USB_HEADER_SIZE + s_rx.dlc));
        if (byte != expected) {
            usb_reset_parser(); /* ERR_CRC: drop and resync */
            break;
        }
        s_rx.crc8 = byte;
        s_raw[s_raw_idx++] = byte;
        s_state = ST_EOF1;
        break;
    }

    case ST_EOF1:
        if (byte == TUE_USB_EOF1) {
            s_raw[s_raw_idx++] = byte;
            s_state = ST_EOF2;
        } else if (byte == TUE_USB_SOF1) {
            s_state = ST_WAIT_SOF2; /* candidate new frame */
        } else {
            usb_reset_parser();
        }
        break;

    case ST_EOF2:
        if (byte == TUE_USB_EOF2) {
            s_raw[s_raw_idx++] = byte;
            usb_dispatch(&s_rx, s_raw, s_raw_idx);
            usb_reset_parser();
        } else {
            usb_reset_parser(); /* framing mismatch: drop */
        }
        break;

    default:
        usb_reset_parser();
        break;
    }
}

static void on_stepper_idle_cb(void)
{
    if (s_rx_sem != NULL) {
        (void)osSemaphoreRelease(s_rx_sem);
    }
}

static void usb_check_pending_idle(void)
{
    if (!s_wait_idle_pending) {
        return;
    }
    if (!tue_stepper_is_busy()) {
        uint8_t out[4];
        wr_u32(out, HAL_GetTick() - s_wait_idle_start);
        tue_usb_send_ok(TUE_USB_CAN_STEP_WAIT_IDLE, s_wait_idle_seq, out, 4U);
        s_wait_idle_pending = false;
    } else if ((s_wait_idle_timeout != 0U) &&
               ((HAL_GetTick() - s_wait_idle_start) >= s_wait_idle_timeout)) {
        tue_usb_send_err(TUE_USB_CAN_STEP_WAIT_IDLE, s_wait_idle_seq, TUE_USB_ERR_TIMEOUT);
        s_wait_idle_pending = false;
    }
}

static void usb_task(void *argument)
{
    (void)argument;
    for (;;) {
        uint32_t wait_ticks = osWaitForever;
        if (s_wait_idle_pending && (s_wait_idle_timeout != 0U)) {
            uint32_t elapsed = HAL_GetTick() - s_wait_idle_start;
            if (elapsed >= s_wait_idle_timeout) {
                wait_ticks = 0U;
            } else {
                wait_ticks = s_wait_idle_timeout - elapsed;
            }
        }
        (void)osSemaphoreAcquire(s_rx_sem, wait_ticks);

        uint8_t byte;
        while (usb_ring_pop(&byte)) {
            usb_rx_byte(byte);
        }

        if (s_wait_idle_pending) {
            usb_check_pending_idle();
        }
    }
}

static void usb_dispatch(const tue_usb_frame_t *f, const uint8_t *raw,
                         uint16_t raw_len)
{
    /* Only host requests are acted upon. */
    if (f->frame_type != (uint8_t)TUE_USB_FRAME_REQ) {
        return;
    }

    uint8_t out[8];
    uint8_t out_dlc = 0U;
    memset(out, 0, sizeof(out));

    int err;
    if (f->can_id >= TUE_USB_CAN_SYS_PING && f->can_id <= TUE_USB_CAN_SYS_ECHO) {
        err = sys_cmd(f, out, &out_dlc);
    } else if ((f->can_id >= TUE_USB_CAN_STEP_GET_INFO && f->can_id <= TUE_USB_CAN_STEP_WAIT_IDLE) ||
               (f->can_id >= TUE_USB_CAN_STEP_RAW_GET_STEPS && f->can_id <= TUE_USB_CAN_STEP_RAW_MOVE)) {
        err = step_cmd(f, out, &out_dlc);
    } else if (f->can_id >= TUE_USB_CAN_SERVO_PING && f->can_id <= TUE_USB_CAN_SERVO_RESET) {
        err = servo_cmd(f, out, &out_dlc);
    } else if (f->can_id == TUE_USB_CAN_HAPTIC_TELEMETRY) {
        /* Unified tactile telemetry: cache locally and forward the exact
         * 20-byte frame to the UART slave. Fire-and-forget streaming: no
         * USB acknowledgement is generated so the host stays at 100 Hz. */
        if ((f->dlc == TUE_HAPTIC_PAYLOAD_LEN) && (raw_len == TUE_HAPTIC_FRAME_LEN)) {
            (void)tue_haptic_ingest(raw, raw_len);
        }
        return;
    } else if (TUE_USB_CAN_ID_IS_RESERVED(f->can_id)) {
        err = (int)TUE_USB_ERR_NOT_SUPPORTED;
    } else {
        err = (int)TUE_USB_ERR_UNKNOWN_CAN_ID;
    }

    if (err == TUE_USB_REPLY_SENT) {
        return; /* handler already emitted its frame(s) */
    }
    if (err == (int)TUE_USB_ERR_OK) {
        tue_usb_send_ok(f->can_id, f->seq, out, out_dlc);
    } else {
        tue_usb_send_err(f->can_id, f->seq, (tue_usb_error_t)err);
    }
}

/* ========================================================================== */
/* System dispatcher (0x100 - 0x107)                                          */
/* ========================================================================== */

static int sys_cmd(const tue_usb_frame_t *f, uint8_t *out, uint8_t *out_dlc)
{
    switch (f->can_id) {
    case TUE_USB_CAN_SYS_PING:
        wr_u32(out, (f->dlc >= 4U) ? rd_u32(f->payload) : 0U);
        *out_dlc = 4U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_SYS_INFO:
        if (f->dlc != 0U) return (int)TUE_USB_ERR_BAD_ARG;
        out[0] = TUE_USB_HW_ID;
        out[1] = TUE_USB_HW_REV;
        out[2] = TUE_USB_FW_VERSION_MAJOR;
        out[3] = TUE_USB_FW_VERSION_MINOR;
        wr_u32(&out[4], TUE_USB_BUILD_FLAGS);
        *out_dlc = 8U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_SYS_UID_PART1:
        if (f->dlc != 0U) return (int)TUE_USB_ERR_BAD_ARG;
        wr_u32(&out[0], HAL_GetUIDw0());
        wr_u32(&out[4], HAL_GetUIDw1());
        *out_dlc = 8U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_SYS_UID_PART2:
        if (f->dlc != 0U) return (int)TUE_USB_ERR_BAD_ARG;
        wr_u32(&out[0], HAL_GetUIDw2());
        *out_dlc = 4U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_SYS_UPTIME:
        if (f->dlc != 0U) return (int)TUE_USB_ERR_BAD_ARG;
        wr_u32(out, HAL_GetTick());
        *out_dlc = 4U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_SYS_LED_CTRL: {
        if (f->dlc < 2U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t led_idx = f->payload[0];
        uint8_t state   = f->payload[1];
        if (led_idx != 0U) return (int)TUE_USB_ERR_BAD_ARG;

        if (state == 0xFFU) {
            /* query only */
        } else if (state == 2U) {
            HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
        } else if (state == 0U || state == 1U) {
            HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin,
                              (state != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        } else {
            return (int)TUE_USB_ERR_BAD_ARG;
        }
        out[0] = led_idx;
        out[1] = (HAL_GPIO_ReadPin(LED_MCU_GPIO_Port, LED_MCU_Pin) == GPIO_PIN_SET) ? 1U : 0U;
        *out_dlc = 2U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SYS_RESET:
        if (f->dlc < 4U || rd_u32(f->payload) != 0xDEADBEEFU) {
            return (int)TUE_USB_ERR_BAD_ARG;
        }
        out[0] = 0U; /* rebooting */
        *out_dlc = 1U;
        tue_usb_send_ok(f->can_id, f->seq, out, *out_dlc);
        /* Event-driven: wait until the ACK packet actually leaves the endpoint over USB */
        if (s_tx_cplt_sem != NULL) {
            (void)osSemaphoreAcquire(s_tx_cplt_sem, 20U);
        }
        HAL_NVIC_SystemReset();
        return TUE_USB_REPLY_SENT; /* not reached */

    case TUE_USB_CAN_SYS_ECHO:
        if (f->dlc == 0U) return (int)TUE_USB_ERR_BAD_ARG;
        if (f->dlc > TUE_USB_MAX_DLC) return (int)TUE_USB_ERR_BAD_ARG;
        memcpy(out, f->payload, f->dlc);
        *out_dlc = f->dlc;
        return (int)TUE_USB_ERR_OK;

    default:
        return (int)TUE_USB_ERR_UNKNOWN_CAN_ID;
    }
}

/* ========================================================================== */
/* Stepper dispatcher (0x200 - 0x281)                                         */
/* ========================================================================== */

static tue_usb_error_t step_map_err(tue_stepper_status_t st)
{
    switch (st) {
        case TUE_STEPPER_OK:             return TUE_USB_ERR_OK;
        case TUE_STEPPER_ERR_PARAM:      return TUE_USB_ERR_BAD_ARG;
        case TUE_STEPPER_ERR_QUEUE_FULL: return TUE_USB_ERR_QUEUE_FULL;
        case TUE_STEPPER_BUSY:           return TUE_USB_ERR_BUSY;
        case TUE_STEPPER_ERR_TIMEOUT:    return TUE_USB_ERR_TIMEOUT;
        default:                         return TUE_USB_ERR_INTERNAL;
    }
}

static bool step_valid_motor(uint8_t motor_id)
{
    return (motor_id >= 1U) && (motor_id <= (uint8_t)TUE_STEPPER_MOTOR_COUNT);
}

static int step_cmd(const tue_usb_frame_t *f, uint8_t *out, uint8_t *out_dlc)
{
    const uint8_t *p = f->payload;
    uint8_t dlc = f->dlc;
    tue_stepper_status_t st;
    float x, y, z;

    switch (f->can_id) {

    case TUE_USB_CAN_STEP_GET_INFO: {
        if (dlc != 0U) return (int)TUE_USB_ERR_BAD_ARG;
        tue_stepper_config_t cfg;
        tue_stepper_get_config(&cfg);
        wr_u16(&out[0], (uint16_t)lroundf(cfg.steps_per_mm[TUE_STEPPER_M1]));
        wr_u16(&out[2], (uint16_t)lroundf(cfg.steps_per_mm[TUE_STEPPER_M3]));
        wr_u16(&out[4], (uint16_t)lroundf(TUE_STEPPER_MIN_RATE_STEPS_S));
        wr_u16(&out[6], (uint16_t)lroundf(TUE_STEPPER_MAX_RATE_STEPS_S));
        *out_dlc = 8U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_STEP_GET_STATUS:
        if (dlc != 0U) return (int)TUE_USB_ERR_BAD_ARG;
        out[0] = tue_stepper_is_busy() ? 1U : 0U;
        out[1] = tue_stepper_is_enabled() ? 1U : 0U;
        out[2] = tue_stepper_queue_depth();
        out[3] = 0U;
        *out_dlc = 4U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_ENABLE:
        if (dlc < 1U) return (int)TUE_USB_ERR_BAD_ARG;
        if (p[0] != 0xFFU) {
            tue_stepper_enable(p[0] != 0U);
        }
        out[0] = tue_stepper_is_enabled() ? 1U : 0U;
        *out_dlc = 1U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_GET_POS_XY:
        if (dlc != 0U) return (int)TUE_USB_ERR_BAD_ARG;
        tue_stepper_get_position_mm(&x, &y, &z);
        wr_i32(&out[0], mm_to_um(x));
        wr_i32(&out[4], mm_to_um(y));
        *out_dlc = 8U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_GET_POS_Z:
        if (dlc != 0U) return (int)TUE_USB_ERR_BAD_ARG;
        tue_stepper_get_position_mm(&x, &y, &z);
        wr_i32(&out[0], mm_to_um(z));
        *out_dlc = 4U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_GET_TGT_XY:
        if (dlc != 0U) return (int)TUE_USB_ERR_BAD_ARG;
        tue_stepper_get_target_mm(&x, &y, &z);
        wr_i32(&out[0], mm_to_um(x));
        wr_i32(&out[4], mm_to_um(y));
        *out_dlc = 8U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_GET_TGT_Z:
        if (dlc != 0U) return (int)TUE_USB_ERR_BAD_ARG;
        tue_stepper_get_target_mm(&x, &y, &z);
        wr_i32(&out[0], mm_to_um(z));
        *out_dlc = 4U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_SETPOS_XY:
        if (dlc < 8U) return (int)TUE_USB_ERR_BAD_ARG;
        tue_stepper_get_position_mm(&x, &y, &z);
        tue_stepper_set_position_mm(um_to_mm(rd_i32(&p[0])), um_to_mm(rd_i32(&p[4])), z);
        wr_i32(&out[0], rd_i32(&p[0]));
        wr_i32(&out[4], rd_i32(&p[4]));
        *out_dlc = 8U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_SETPOS_Z:
        if (dlc < 4U) return (int)TUE_USB_ERR_BAD_ARG;
        tue_stepper_get_position_mm(&x, &y, &z);
        tue_stepper_set_position_mm(x, y, um_to_mm(rd_i32(&p[0])));
        wr_i32(&out[0], rd_i32(&p[0]));
        *out_dlc = 4U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_MOVE_COORDS:
        if (dlc < 8U) return (int)TUE_USB_ERR_BAD_ARG;
        s_latch_x_um = rd_i32(&p[0]);
        s_latch_y_um = rd_i32(&p[4]);
        s_latch_valid = true;
        out[0] = 1U; /* ack */
        out[1] = tue_stepper_queue_depth();
        *out_dlc = 2U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_MOVE_EXEC:
        if (dlc < 8U) return (int)TUE_USB_ERR_BAD_ARG;
        if (!s_latch_valid) return (int)TUE_USB_ERR_BAD_FORMAT;
        st = tue_stepper_move_to_xyz_mm(um_to_mm(s_latch_x_um), um_to_mm(s_latch_y_um),
                                        um_to_mm(rd_i32(&p[0])),
                                        (float)rd_u16(&p[4]), (float)rd_u16(&p[6]));
        if (st != TUE_STEPPER_OK) return (int)step_map_err(st);
        wr_i32(&out[0], s_latch_x_um);
        wr_i32(&out[4], s_latch_y_um);
        *out_dlc = 8U;
        s_latch_valid = false;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_MOVE_XY:
        if (dlc < 8U) return (int)TUE_USB_ERR_BAD_ARG;
        st = tue_stepper_move_to_mm(um_to_mm(rd_i32(&p[0])), um_to_mm(rd_i32(&p[4])),
                                    s_default_feed, s_default_accel);
        if (st != TUE_STEPPER_OK) return (int)step_map_err(st);
        wr_i32(&out[0], rd_i32(&p[0]));
        wr_i32(&out[4], rd_i32(&p[4]));
        *out_dlc = 8U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_MOVE_Z:
        if (dlc < 8U) return (int)TUE_USB_ERR_BAD_ARG;
        st = tue_stepper_move_z_mm(um_to_mm(rd_i32(&p[0])), (float)rd_u16(&p[4]), (float)rd_u16(&p[6]));
        if (st != TUE_STEPPER_OK) return (int)step_map_err(st);
        wr_i32(&out[0], rd_i32(&p[0]));
        *out_dlc = 4U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_MOVE_REL_XY:
        if (dlc < 8U) return (int)TUE_USB_ERR_BAD_ARG;
        st = tue_stepper_move_by_mm(um_to_mm(rd_i32(&p[0])), um_to_mm(rd_i32(&p[4])),
                                    s_default_feed, s_default_accel);
        if (st != TUE_STEPPER_OK) return (int)step_map_err(st);
        wr_i32(&out[0], rd_i32(&p[0]));
        wr_i32(&out[4], rd_i32(&p[4]));
        *out_dlc = 8U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_SET_SPEED:
        if (dlc < 4U) return (int)TUE_USB_ERR_BAD_ARG;
        s_default_feed  = (float)rd_u16(&p[0]);
        s_default_accel = (float)rd_u16(&p[2]);
        wr_u16(&out[0], rd_u16(&p[0]));
        wr_u16(&out[2], rd_u16(&p[2]));
        *out_dlc = 4U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_JOG_XY:
        if (dlc < 6U) return (int)TUE_USB_ERR_BAD_ARG;
        st = tue_stepper_start_velocity((float)rd_i16(&p[0]),
                                        (float)rd_i16(&p[2]),
                                        (float)rd_u16(&p[4]));
        if (st != TUE_STEPPER_OK) return (int)step_map_err(st);
        wr_i16(&out[0], rd_i16(&p[0]));
        wr_i16(&out[2], rd_i16(&p[2]));
        *out_dlc = 4U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_JOG_Z:
        if (dlc < 4U) return (int)TUE_USB_ERR_BAD_ARG;
        st = tue_stepper_start_velocity_z((float)rd_i16(&p[0]), (float)rd_u16(&p[2]));
        if (st != TUE_STEPPER_OK) return (int)step_map_err(st);
        wr_i16(&out[0], rd_i16(&p[0]));
        *out_dlc = 2U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_STOP:
        tue_stepper_stop();
        out[0] = tue_stepper_is_busy() ? 1U : 0U;
        *out_dlc = 1U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_ABORT:
        tue_stepper_abort();
        out[0] = 0U;
        *out_dlc = 1U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_STEP_WAIT_IDLE: {
        uint32_t timeout_ms = (dlc >= 4U) ? rd_u32(&p[0]) : 0U;
        if (!tue_stepper_is_busy()) {
            wr_u32(out, 0U);
            *out_dlc = 4U;
            return (int)TUE_USB_ERR_OK;
        }
        if (s_wait_idle_pending) {
            return (int)TUE_USB_ERR_BUSY;
        }
        s_wait_idle_pending = true;
        s_wait_idle_seq     = f->seq;
        s_wait_idle_start   = HAL_GetTick();
        s_wait_idle_timeout = timeout_ms;
        return TUE_USB_REPLY_SENT;
    }

    case TUE_USB_CAN_STEP_RAW_GET_STEPS: {
        if (dlc < 1U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t motor_id = p[0];
        if (!step_valid_motor(motor_id)) return (int)TUE_USB_ERR_BAD_ARG;
        out[0] = motor_id;
        wr_i32(&out[1], tue_stepper_get_motor_steps((tue_stepper_motor_t)(motor_id - 1U)));
        *out_dlc = 5U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_STEP_RAW_MOVE: {
        if (dlc < 8U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t motor_id = p[0];
        if (!step_valid_motor(motor_id)) return (int)TUE_USB_ERR_BAD_ARG;
        int32_t steps = rd_i32(&p[2]);
        float feed = (rd_u16(&p[6]) > 0U) ? (float)rd_u16(&p[6]) : s_default_feed;
        st = tue_stepper_move_motor_steps((tue_stepper_motor_t)(motor_id - 1U), steps,
                                          feed, s_default_accel);
        if (st != TUE_STEPPER_OK) return (int)step_map_err(st);
        out[0] = motor_id;
        wr_i32(&out[1], steps);
        *out_dlc = 5U;
        return (int)TUE_USB_ERR_OK;
    }

    default:
        return (int)TUE_USB_ERR_UNKNOWN_CAN_ID;
    }
}

/* ========================================================================== */
/* STS3215 servo dispatcher (0x400 - 0x415)                                   */
/* ========================================================================== */

static tue_usb_error_t servo_map_err(tue_sts3215_status_t st)
{
    switch (st) {
        case TUE_STS3215_OK:          return TUE_USB_ERR_OK;
        case TUE_STS3215_ERR_PARAM:   return TUE_USB_ERR_BAD_ARG;
        case TUE_STS3215_ERR_TIMEOUT: return TUE_USB_ERR_BUS_TIMEOUT;
        case TUE_STS3215_ERR_BUSY:    return TUE_USB_ERR_BUSY;
        case TUE_STS3215_ERR_STATUS:  return TUE_USB_ERR_SERVO_FAULT;
        default:                      return TUE_USB_ERR_BUS_FAULT;
    }
}

static int servo_cmd(const tue_usb_frame_t *f, uint8_t *out, uint8_t *out_dlc)
{
    const uint8_t *p = f->payload;
    uint8_t dlc = f->dlc;
    tue_sts3215_status_t st;

    switch (f->can_id) {

    case TUE_USB_CAN_SERVO_PING:
        if (dlc < 1U) return (int)TUE_USB_ERR_BAD_ARG;
        st = tue_sts3215_ping(&s_servo, p[0]);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = p[0];
        out[1] = 0U;
        *out_dlc = 2U;
        return (int)TUE_USB_ERR_OK;

    case TUE_USB_CAN_SERVO_SET_POS: {
        if (dlc < 6U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t id = p[0];
        uint8_t accel = p[1];
        int16_t pos = rd_i16(&p[2]);
        int16_t speed = rd_i16(&p[4]);
        st = tue_sts3215_set_goal_position_ex(&s_servo, id, pos, speed, accel);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = id;
        wr_i16(&out[1], pos);
        *out_dlc = 3U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_GET_POS: {
        if (dlc < 1U) return (int)TUE_USB_ERR_BAD_ARG;
        int16_t pos = 0;
        st = tue_sts3215_get_current_position(&s_servo, p[0], &pos);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = p[0];
        wr_i16(&out[1], pos);
        *out_dlc = 3U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_SET_SPEED: {
        if (dlc < 4U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t id = p[0];
        uint8_t accel = p[1];
        int16_t speed = rd_i16(&p[2]);
        st = tue_sts3215_set_acceleration(&s_servo, id, accel);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        st = tue_sts3215_set_running_speed(&s_servo, id, speed);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = id;
        wr_i16(&out[1], speed);
        *out_dlc = 3U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_GET_SPEED: {
        if (dlc < 1U) return (int)TUE_USB_ERR_BAD_ARG;
        int16_t speed = 0;
        st = tue_sts3215_get_current_speed(&s_servo, p[0], &speed);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = p[0];
        wr_i16(&out[1], speed);
        *out_dlc = 3U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_TORQUE: {
        if (dlc < 2U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t id = p[0];
        uint8_t state = p[1];
        bool enabled = false;
        if (state == 0xFFU) {
            st = tue_sts3215_get_torque_enable(&s_servo, id, &enabled);
            if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        } else if (state == 128U) {
            st = tue_sts3215_calibrate_position_to_2048(&s_servo, id);
            if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
            st = tue_sts3215_get_torque_enable(&s_servo, id, &enabled);
            if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        } else {
            st = tue_sts3215_enable_torque(&s_servo, id, (state != 0U));
            if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
            enabled = (state != 0U);
        }
        out[0] = id;
        out[1] = enabled ? 1U : 0U;
        *out_dlc = 2U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_TLIM: {
        if (dlc < 3U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t id = p[0];
        uint16_t limit = rd_u16(&p[1]);
        st = tue_sts3215_set_torque_limit(&s_servo, id, limit);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = id;
        wr_u16(&out[1], limit);
        *out_dlc = 3U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_MODE: {
        if (dlc < 2U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t id = p[0];
        uint8_t mode = p[1];
        tue_sts3215_mode_t cur = STS3215_MODE_POSITION;
        if (mode == 0xFFU) {
            st = tue_sts3215_get_operation_mode(&s_servo, id, &cur);
            if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        } else {
            if (mode > 3U) return (int)TUE_USB_ERR_BAD_ARG;
            st = tue_sts3215_set_operation_mode(&s_servo, id, (tue_sts3215_mode_t)mode);
            if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
            cur = (tue_sts3215_mode_t)mode;
        }
        out[0] = id;
        out[1] = (uint8_t)cur;
        *out_dlc = 2U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_PWM: {
        if (dlc < 3U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t id = p[0];
        int16_t pwm = rd_i16(&p[1]);
        if (pwm < -1000 || pwm > 1000) return (int)TUE_USB_ERR_BAD_ARG;
        st = tue_sts3215_set_pwm_runtime(&s_servo, id, pwm);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = id;
        wr_i16(&out[1], pwm);
        *out_dlc = 3U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_GET_FB: {
        if (dlc < 1U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t id = p[0];
        tue_sts3215_feedback_t fb;
        memset(&fb, 0, sizeof(fb));
        st = tue_sts3215_get_feedback(&s_servo, id, &fb);
        if ((st != TUE_STS3215_OK) && (st != TUE_STS3215_ERR_STATUS)) {
            return (int)servo_map_err(st);
        }
        /* Part 1 - dynamics (0x410). */
        uint8_t dyn[8];
        dyn[0] = fb.id;
        wr_i16(&dyn[1], fb.position);
        wr_i16(&dyn[3], fb.speed);
        wr_i16(&dyn[5], fb.load);
        dyn[7] = fb.is_moving ? 1U : 0U;
        tue_usb_send_ok(TUE_USB_CAN_SERVO_FB_DYNAMICS, f->seq, dyn, 8U);

        /* Part 2 - health (0x411). */
        uint8_t hlt[8];
        hlt[0] = fb.id;
        hlt[1] = fb.voltage_raw;
        hlt[2] = (uint8_t)fb.temperature_degc;
        hlt[3] = fb.status_mask;
        wr_i16(&hlt[4], fb.current_raw);
        wr_u16(&hlt[6], 0U);
        tue_usb_send_ok(TUE_USB_CAN_SERVO_FB_HEALTH, f->seq, hlt, 8U);

        /* Both telemetry frames are out; the status byte in the health frame
         * already carries any servo fault bits. */
        return TUE_USB_REPLY_SENT;
    }

    case TUE_USB_CAN_SERVO_MEM_READ: {
        if (dlc < 3U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t id = p[0];
        uint8_t reg = p[1];
        uint8_t len = p[2];
        if (len < 1U || len > TUE_USB_MEM_MAX) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t buf[TUE_USB_MEM_MAX];
        st = tue_sts3215_read_raw(&s_servo, id, reg, buf, len);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = id;
        out[1] = reg;
        memcpy(&out[2], buf, len);
        *out_dlc = (uint8_t)(2U + len);
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_MEM_WRITE: {
        if (dlc < 3U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t id = p[0];
        uint8_t reg = p[1];
        uint8_t len = (uint8_t)(dlc - 2U);
        if (len > TUE_USB_MEM_MAX) return (int)TUE_USB_ERR_BAD_ARG;
        st = tue_sts3215_write_raw(&s_servo, id, reg, &p[2], len);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = id;
        out[1] = reg;
        out[2] = 0U;
        *out_dlc = 3U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_SET_ID: {
        if (dlc < 2U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t old_id = p[0];
        uint8_t new_id = p[1];
        st = tue_sts3215_set_id(&s_servo, old_id, new_id);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = new_id;
        out[1] = 0U;
        *out_dlc = 2U;
        return (int)TUE_USB_ERR_OK;
    }

    case TUE_USB_CAN_SERVO_RESET: {
        if (dlc < 1U) return (int)TUE_USB_ERR_BAD_ARG;
        uint8_t id = p[0];
        st = tue_sts3215_reset(&s_servo, id);
        if (st != TUE_STS3215_OK) return (int)servo_map_err(st);
        out[0] = id;
        out[1] = 0U;
        *out_dlc = 2U;
        return (int)TUE_USB_ERR_OK;
    }

    default:
        return (int)TUE_USB_ERR_UNKNOWN_CAN_ID;
    }
}

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

void tue_usb_init(void)
{
    if (s_initialized) {
        return;
    }

    /* The SERVO commands drive the Feetech bus on USART1. */
    (void)tue_sts3215_init(&s_servo, &huart1);

    /* The STEP commands drive the three stepper motors through the TIM1 step
     * clock. Idempotent: main() already calls it at boot. */
    tue_stepper_init();

    /* Slave haptic link: USART2 @ 250 kbaud, TX DMA + circular RX DMA. */
    (void)tue_haptic_init();

    const osSemaphoreAttr_t sem_attr = { .name = "usb_rx" };
    s_rx_sem = osSemaphoreNew(1U, 0U, &sem_attr);

    const osSemaphoreAttr_t sem_tx_attr = { .name = "usb_tx_cplt" };
    s_tx_cplt_sem = osSemaphoreNew(1U, 1U, &sem_tx_attr);

    const osMutexAttr_t mutex_attr = { .name = "usb_tx" };
    s_tx_mutex = osMutexNew(&mutex_attr);

    tue_stepper_set_idle_callback(on_stepper_idle_cb);

    const osThreadAttr_t task_attr = {
        .name = "tue_usb",
        .stack_size = 4096,
        .priority = (osPriority_t)osPriorityNormal,
    };
    s_task = osThreadNew(usb_task, NULL, &task_attr);

    s_initialized = true;
}

void tue_usb_on_rx(const uint8_t *data, uint32_t len)
{
    if (s_rx_sem == NULL || data == NULL) {
        return;
    }
    for (uint32_t i = 0; i < len; i++) {
        (void)usb_ring_push(data[i]);
    }
    (void)osSemaphoreRelease(s_rx_sem);
}

tue_sts3215_t *tue_usb_servo(void)
{
    return &s_servo;
}
