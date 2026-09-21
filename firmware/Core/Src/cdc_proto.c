/**
 * @file    cdc_proto.c
 * @brief   USB CDC binary protocol — full Phase-1 implementation.
 *
 * Contains:
 *   - CRC16-CCITT (poly 0x1021, init 0xFFFF) 256-entry lookup table.
 *   - Frame encoder / decoder.
 *   - RX state machine (byte-by-byte parser with resync).
 *   - CommsTask   (1024 words, osPriorityAboveNormal) — drains stream buffer,
 *                  parses, dispatches, heartbeat / silence monitoring.
 *   - TxSenderTask (512 words, osPriorityNormal) — dequeues encoded frames
 *                  from a message buffer and transmits via CDC_Transmit_FS with
 *                  retry-on-busy.
 *   - Device table (empty this phase) + inline PING handler.
 *   - Boot banner on init.
 *   - Safety: sequence tracking, RX silence counter, drop-oldest policy.
 */

#include "cdc_proto.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include "cmsis_os.h"

#include <string.h>

/* --------------- External USB CDC transmit function ----------------------- */
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/* USBD return codes we compare against. */
#ifndef USBD_OK
#define USBD_OK   0U
#endif
#ifndef USBD_BUSY
#define USBD_BUSY 1U
#endif

/* ========================================================================== */
/*  CRC16-CCITT (poly 0x1021, init 0xFFFF)                                    */
/* ========================================================================== */

static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x5085, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x4864, 0x5845, 0x6826, 0x7807, 0x08E0, 0x18C1, 0x28A2, 0x38A3,
    0xC94C, 0xD96D, 0xE90E, 0xF92F, 0x89C8, 0x99E9, 0xA98A, 0xB9AB,
    0x5A75, 0x4A54, 0x7A37, 0x6A16, 0x1AF1, 0x0AD0, 0x3AB3, 0x2A92,
    0xDB7D, 0xCB5C, 0xFB3F, 0xEB1E, 0x9BF9, 0x8BD8, 0xBBBB, 0xABBA,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x85A9, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD9EC, 0xC9CD, 0xF9AE, 0xE98F, 0x9968, 0x8949, 0xB92A, 0xA90B,
    0x58E4, 0x48C5, 0x78A6, 0x6887, 0x1860, 0x0841, 0x3822, 0x2803,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc = (uint16_t)((crc << 8) ^ crc16_table[(crc >> 8) ^ data[i]]);
    }
    return crc;
}

/* ========================================================================== */
/*  Encode / Decode                                                           */
/* ========================================================================== */

size_t cdc_encode(const ProtoFrame *frame, uint8_t *buf)
{
    uint8_t payload_len = frame->len;
    if (payload_len > CDC_PROTO_MAX_PAYLOAD)
        payload_len = CDC_PROTO_MAX_PAYLOAD;

    /* SOF */
    buf[0] = CDC_PROTO_SOF0;
    buf[1] = CDC_PROTO_SOF1;
    /* VER */
    buf[2] = CDC_PROTO_VERSION;
    /* LEN */
    buf[3] = payload_len;
    /* Header fields */
    buf[4] = frame->dev;
    buf[5] = frame->op;
    buf[6] = frame->seq;
    buf[7] = frame->flags;
    buf[8] = 0x00; /* RSVD */
    buf[9] = 0x00; /* RSVD */

    /* Payload */
    if (payload_len > 0) {
        memcpy(&buf[10], frame->payload, payload_len);
    }

    /* CRC16 over VER..PAYLOAD (bytes [2 .. 10+LEN-1]) */
    uint16_t crc = crc16_ccitt(&buf[2], (size_t)(8u + payload_len));
    buf[10 + payload_len]     = (uint8_t)(crc >> 8);   /* CRC high */
    buf[10 + payload_len + 1] = (uint8_t)(crc & 0xFF); /* CRC low  */

    return (size_t)(12u + payload_len);
}

int cdc_decode(const uint8_t *buf, size_t len, ProtoFrame *frame)
{
    if (len < CDC_PROTO_OVERHEAD)
        return -1;

    /* Check SOF */
    if (buf[0] != CDC_PROTO_SOF0 || buf[1] != CDC_PROTO_SOF1)
        return -1;

    /* Check version */
    if (buf[2] != CDC_PROTO_VERSION)
        return -1;

    uint8_t payload_len = buf[3];
    if (payload_len > CDC_PROTO_MAX_PAYLOAD)
        return -1;

    size_t expected_len = (size_t)(12u + payload_len);
    if (len < expected_len)
        return -1;

    /* Verify CRC — covers VER..PAYLOAD */
    uint16_t crc_calc = crc16_ccitt(&buf[2], (size_t)(8u + payload_len));
    uint16_t crc_wire = ((uint16_t)buf[10 + payload_len] << 8)
                      |  (uint16_t)buf[10 + payload_len + 1];
    if (crc_calc != crc_wire)
        return -1;

    frame->dev   = buf[4];
    frame->op    = buf[5];
    frame->seq   = buf[6];
    frame->flags = buf[7];
    frame->len   = payload_len;
    if (payload_len > 0) {
        memcpy(frame->payload, &buf[10], payload_len);
    }

    return 0;
}

/* ========================================================================== */
/*  Static handles and buffers                                                 */
/* ========================================================================== */

/** RX stream buffer — ISR pushes raw USB bytes here. */
StreamBufferHandle_t s_rxStream;

/** TX message buffer — tasks enqueue encoded frames here for TxSenderTask. */
static MessageBufferHandle_t s_txMsgBuf;

/** Task handles. */
static TaskHandle_t commsHandle;
static TaskHandle_t txSenderHandle;

/* Stream buffer size (bytes).  Trigger level = 1 byte. */
#define RX_STREAM_SIZE  512

/* TX message buffer size (bytes).  Must hold several max-size frames. */
#define TX_MSG_BUF_SIZE 4096

/* Maximum number of tracked DEV addresses for sequence checking. */
#define MAX_DEV_SLOTS   16

/* ========================================================================== */
/*  Statistics                                                                 */
/* ========================================================================== */

static CdcProtoStats s_stats;

const CdcProtoStats *cdc_proto_stats(void)
{
    return &s_stats;
}

/* ========================================================================== */
/*  Device table  (Phase 1: empty — only inline PING)                         */
/* ========================================================================== */

typedef struct {
    uint8_t        dev;
    CdcDevHandler  handler;
    CdcSafeStateFn safe_enter;
} DevTableEntry;

static const DevTableEntry kDevTable[] = {
    /* Phase 2 will populate this. */
    { 0, NULL, NULL }  /* sentinel */
};

#define DEV_TABLE_COUNT  (sizeof(kDevTable) / sizeof(kDevTable[0]) - 1u)

/* ========================================================================== */
/*  Safe-state stubs                                                           */
/* ========================================================================== */

static void safe_state_enter(void)
{
    for (size_t i = 0; i < DEV_TABLE_COUNT; i++) {
        if (kDevTable[i].safe_enter != NULL) {
            kDevTable[i].safe_enter();
        }
    }
}

/* safe_state_exit is a stub for now. */
static void safe_state_exit(void)
{
    (void)0;
}

/* ========================================================================== */
/*  Sequence tracking                                                          */
/* ========================================================================== */

static uint8_t  last_seq[MAX_DEV_SLOTS];
static uint8_t  seq_init[MAX_DEV_SLOTS]; /* 0 = not yet seen */

/**
 * Check and update sequence number for a device.
 * @return 0 on OK, -1 on non-monotonic sequence.
 */
static int seq_check(uint8_t dev, uint8_t seq)
{
    uint8_t slot = dev & (MAX_DEV_SLOTS - 1);
    if (!seq_init[slot]) {
        seq_init[slot] = 1;
        last_seq[slot] = seq;
        return 0;
    }
    /* Expect strictly increasing (modulo 256). */
    uint8_t expected = (uint8_t)(last_seq[slot] + 1u);
    if (seq != expected) {
        s_stats.seq_err++;
        last_seq[slot] = seq;  /* re-sync */
        return -1;
    }
    last_seq[slot] = seq;
    return 0;
}

/* ========================================================================== */
/*  TX helpers                                                                 */
/* ========================================================================== */

int cdc_send(const ProtoFrame *frame)
{
    uint8_t wire[CDC_PROTO_MAX_FRAME];
    size_t  n = cdc_encode(frame, wire);

    /* xMessageBufferSend: block 0 ticks (non-blocking). */
    size_t sent = xMessageBufferSend(s_txMsgBuf, wire, n, 0);
    if (sent == 0) {
        s_stats.tx_dropped++;
        return -1;
    }
    return 0;
}

int cdc_send_ack(const ProtoFrame *in, int ack_ok, uint8_t err_code)
{
    ProtoFrame out;
    memset(&out, 0, sizeof(out));

    out.dev  = in->dev;
    out.op   = in->op;
    out.seq  = in->seq;
    out.flags = (uint8_t)(ack_ok ? CDC_FLAG_ACK_OK : CDC_FLAG_ACK_ERR);

    if (!ack_ok) {
        out.payload[0] = err_code;
        out.len = 1;
    }

    return cdc_send(&out);
}

/* ========================================================================== */
/*  Boot banner                                                                */
/* ========================================================================== */

/**
 * Send a boot banner:  DEV=0xF0 OP=0x80 payload = "MMC\x01\x00\x00\x00\x00".
 */
static void send_boot_banner(void)
{
    ProtoFrame banner;
    memset(&banner, 0, sizeof(banner));

    banner.dev  = CDC_DEV_SYSTEM;
    banner.op   = CDC_OP_PING;
    banner.seq  = 0;
    banner.flags = 0;
    banner.len  = 8;
    banner.payload[0] = 0x4D; /* 'M' */
    banner.payload[1] = 0x4D; /* 'M' */
    banner.payload[2] = 0x43; /* 'C' */
    banner.payload[3] = 0x01; /* version major */
    banner.payload[4] = 0x00;
    banner.payload[5] = 0x00;
    banner.payload[6] = 0x00;
    banner.payload[7] = 0x00;

    cdc_send(&banner);
}

/* ========================================================================== */
/*  RX state machine                                                           */
/* ========================================================================== */

typedef enum {
    RX_WAIT_SOF0,
    RX_WAIT_SOF1,
    RX_VER,
    RX_LEN,
    RX_HEADER,      /* 6 bytes: DEV OP SEQ FLAGS RSVD RSVD */
    RX_PAYLOAD,
    RX_CRC,
} RxState;

typedef struct {
    RxState  state;
    uint8_t  buf[CDC_PROTO_MAX_FRAME];
    uint16_t pos;       /* write index into buf */
    uint8_t  payload_len;
    uint8_t  hdr_cnt;   /* header bytes received so far */
    uint8_t  payload_cnt;
    uint8_t  crc_cnt;
} RxParser;

static RxParser s_parser;

static void parser_reset(RxParser *p)
{
    p->state       = RX_WAIT_SOF0;
    p->pos         = 0;
    p->payload_len = 0;
    p->hdr_cnt     = 0;
    p->payload_cnt = 0;
    p->crc_cnt     = 0;
}

/**
 * Feed one byte to the state machine.
 * @return  1 if a complete frame is in p->buf (length = 12 + payload_len),
 *          0 otherwise.
 */
static int parser_feed(RxParser *p, uint8_t byte)
{
    switch (p->state) {

    case RX_WAIT_SOF0:
        if (byte == CDC_PROTO_SOF0) {
            p->pos = 0;
            p->buf[p->pos++] = byte;
            p->state = RX_WAIT_SOF1;
        }
        /* else: stay in WAIT_SOF0 */
        return 0;

    case RX_WAIT_SOF1:
        if (byte == CDC_PROTO_SOF1) {
            p->buf[p->pos++] = byte;
            p->state = RX_VER;
        } else if (byte == CDC_PROTO_SOF0) {
            /* Could be a new SOF0 — restart */
            p->pos = 0;
            p->buf[p->pos++] = byte;
            /* Stay in WAIT_SOF1 */
        } else {
            parser_reset(p);
        }
        return 0;

    case RX_VER:
        if (byte == CDC_PROTO_VERSION) {
            p->buf[p->pos++] = byte;
            p->state = RX_LEN;
        } else {
            /* Bad version — resync on 0xAA */
            if (byte == CDC_PROTO_SOF0) {
                p->pos = 0;
                p->buf[p->pos++] = byte;
                p->state = RX_WAIT_SOF1;
            } else {
                parser_reset(p);
            }
        }
        return 0;

    case RX_LEN:
        if (byte > CDC_PROTO_MAX_PAYLOAD) {
            /* Invalid length — resync */
            if (byte == CDC_PROTO_SOF0) {
                p->pos = 0;
                p->buf[p->pos++] = byte;
                p->state = RX_WAIT_SOF1;
            } else {
                parser_reset(p);
            }
            return 0;
        }
        p->payload_len = byte;
        p->buf[p->pos++] = byte;
        p->hdr_cnt = 0;
        p->state = RX_HEADER;
        return 0;

    case RX_HEADER:
        p->buf[p->pos++] = byte;
        p->hdr_cnt++;
        if (p->hdr_cnt >= CDC_PROTO_HDR_SIZE) {
            p->payload_cnt = 0;
            if (p->payload_len > 0) {
                p->state = RX_PAYLOAD;
            } else {
                p->crc_cnt = 0;
                p->state = RX_CRC;
            }
        }
        return 0;

    case RX_PAYLOAD:
        p->buf[p->pos++] = byte;
        p->payload_cnt++;
        if (p->payload_cnt >= p->payload_len) {
            p->crc_cnt = 0;
            p->state = RX_CRC;
        }
        return 0;

    case RX_CRC:
        p->buf[p->pos++] = byte;
        p->crc_cnt++;
        if (p->crc_cnt >= 2) {
            /* Frame complete — caller validates CRC */
            return 1;
        }
        return 0;

    default:
        parser_reset(p);
        return 0;
    }
}

/* ========================================================================== */
/*  Dispatcher                                                                 */
/* ========================================================================== */

static void dispatch_frame(ProtoFrame *frame)
{
    /* ---- Sequence check ---- */
    if (seq_check(frame->dev, frame->seq) != 0) {
        /* Sequence mismatch — still process, but stats recorded. */
        /* Optionally send ERR back: */
        if (frame->flags & CDC_FLAG_REPLY_REQ) {
            cdc_send_ack(frame, 0, CDC_ERR_SEQ_MISMATCH);
        }
    }

    /* ---- Inline PING handler (DEV=any, OP=0x80) ---- */
    if (frame->op == CDC_OP_PING) {
        ProtoFrame reply;
        memset(&reply, 0, sizeof(reply));
        reply.dev   = frame->dev;
        reply.op    = CDC_OP_PING;
        reply.seq   = frame->seq;
        reply.flags = CDC_FLAG_ACK_OK;
        /* Echo the payload back */
        reply.len   = frame->len;
        if (frame->len > 0) {
            memcpy(reply.payload, frame->payload, frame->len);
        }
        cdc_send(&reply);
        return;
    }

    /* ---- Look up device handler ---- */
    for (size_t i = 0; i < DEV_TABLE_COUNT; i++) {
        if (kDevTable[i].dev == frame->dev && kDevTable[i].handler != NULL) {
            ProtoFrame reply;
            memset(&reply, 0, sizeof(reply));
            int rc = kDevTable[i].handler(frame, &reply);
            if (rc == 0 && (frame->flags & CDC_FLAG_REPLY_REQ)) {
                cdc_send(&reply);
            }
            return;
        }
    }

    /* ---- Unknown device ---- */
    s_stats.unknown_dev++;
    if (frame->flags & CDC_FLAG_REPLY_REQ) {
        cdc_send_ack(frame, 0, CDC_ERR_UNKNOWN_DEV);
    }
}

/* ========================================================================== */
/*  CommsTask                                                                  */
/* ========================================================================== */

/** How long to wait for stream-buffer data before waking up for housekeeping. */
#define COMMS_POLL_MS       10

/** RX silence threshold in ms before triggering safe state. */
#define SILENCE_THRESHOLD   1000

static void CommsTask(void *argument)
{
    (void)argument;

    parser_reset(&s_parser);
    uint32_t last_frame_tick = xTaskGetTickCount();
    uint8_t  safe_state_active = 0;

    for (;;) {
        uint8_t  rxbyte;
        size_t received = xStreamBufferReceive(s_rxStream, &rxbyte, 1,
                                               pdMS_TO_TICKS(COMMS_POLL_MS));

        /* ---- Housekeeping: silence counter ---- */
        uint32_t now = xTaskGetTickCount();
        uint32_t elapsed = (now - last_frame_tick) * portTICK_PERIOD_MS;
        s_stats.rx_silence_ms = elapsed;

        if (elapsed >= SILENCE_THRESHOLD && !safe_state_active) {
            safe_state_enter();
            safe_state_active = 1;
        }

        if (received == 0) {
            continue; /* timeout — loop to re-check silence */
        }

        /* ---- Feed byte to parser ---- */
        int complete = parser_feed(&s_parser, rxbyte);
        if (!complete) {
            continue;
        }

        /* ---- Frame complete: decode ---- */
        ProtoFrame frame;
        size_t frame_len = (size_t)(12u + s_parser.payload_len);

        if (cdc_decode(s_parser.buf, frame_len, &frame) != 0) {
            s_stats.rx_crc_err++;
            parser_reset(&s_parser);
            continue;
        }

        /* Valid frame received */
        s_stats.rx_frames_ok++;
        last_frame_tick = now;

        /* Exit safe state if we were in it */
        if (safe_state_active) {
            safe_state_exit();
            safe_state_active = 0;
        }

        dispatch_frame(&frame);
        parser_reset(&s_parser);
    }
}

/* ========================================================================== */
/*  TxSenderTask                                                               */
/* ========================================================================== */

/** Max retries on USBD_BUSY before dropping. */
#define TX_MAX_RETRIES  5

static void TxSenderTask(void *argument)
{
    (void)argument;

    static uint8_t tx_wire[CDC_PROTO_MAX_FRAME];

    for (;;) {
        /* Block until a message arrives. */
        size_t n = xMessageBufferReceive(s_txMsgBuf, tx_wire,
                                         sizeof(tx_wire), portMAX_DELAY);
        if (n == 0) {
            continue;
        }

        /* Attempt to transmit with retry on USBD_BUSY. */
        uint8_t result = USBD_BUSY;
        for (int retry = 0; retry < TX_MAX_RETRIES; retry++) {
            result = CDC_Transmit_FS(tx_wire, (uint16_t)n);
            if (result == USBD_OK) {
                break;
            }
            s_stats.tx_busy++;
            osDelay(1);
        }

        if (result != USBD_OK) {
            s_stats.tx_dropped++;
        }
    }
}

/* ========================================================================== */
/*  cdc_proto_init                                                             */
/* ========================================================================== */

/* Task stack sizes (in words). */
#define COMMS_STACK_WORDS     1024
#define TX_SENDER_STACK_WORDS  512

void cdc_proto_init(void)
{
    /* ---- Create stream buffer (ISR → CommsTask) ---- */
    s_rxStream = xStreamBufferCreate(RX_STREAM_SIZE, /* trigger */ 1);
    configASSERT(s_rxStream != NULL);

    /* ---- Create message buffer (CommsTask / any → TxSenderTask) ---- */
    s_txMsgBuf = xMessageBufferCreate(TX_MSG_BUF_SIZE);
    configASSERT(s_txMsgBuf != NULL);

    /* ---- Zero stats ---- */
    memset(&s_stats, 0, sizeof(s_stats));
    memset(last_seq, 0, sizeof(last_seq));
    memset(seq_init, 0, sizeof(seq_init));

    /* ---- Create CommsTask ---- */
    static const osThreadAttr_t comms_attr = {
        .name       = "CommsTask",
        .stack_size = COMMS_STACK_WORDS * sizeof(uint32_t),
        .priority   = (osPriority_t) osPriorityAboveNormal,
    };
    commsHandle = (TaskHandle_t)osThreadNew(CommsTask, NULL, &comms_attr);
    configASSERT(commsHandle != NULL);

    /* ---- Create TxSenderTask ---- */
    static const osThreadAttr_t tx_attr = {
        .name       = "TxSender",
        .stack_size = TX_SENDER_STACK_WORDS * sizeof(uint32_t),
        .priority   = (osPriority_t) osPriorityNormal,
    };
    txSenderHandle = (TaskHandle_t)osThreadNew(TxSenderTask, NULL, &tx_attr);
    configASSERT(txSenderHandle != NULL);

    /* ---- Send boot banner ---- */
    send_boot_banner();
}
