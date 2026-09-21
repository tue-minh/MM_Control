/**
 * @file    cdc_proto.h
 * @brief   USB CDC binary protocol — public API.
 *
 * Frame layout (wire order, big-endian multi-byte fields):
 *
 *   Offset  Field       Size   Description
 *   ------  ----------  -----  -------------------------------------------
 *   0       SOF         2      0xAA 0x55
 *   2       VER         1      Protocol version (0x01)
 *   3       LEN         1      Payload length 0..200
 *   4       DEV         1      Device address
 *   5       OP          1      Opcode
 *   6       SEQ         1      Sequence number (per-DEV, monotonic)
 *   7       FLAGS       1      Bit-field (see below)
 *   8       RSVD        2      Reserved (0x00 0x00)
 *  10       PAYLOAD     LEN    Variable-length payload
 *  10+LEN   CRC16       2      CRC16-CCITT over bytes [2..10+LEN-1]
 *
 * FLAGS bit definitions:
 *   [0] REPLY_REQ  — host requests a reply
 *   [1] ACK_OK     — device acknowledges success
 *   [2] ACK_ERR    — device signals an error
 *   [3] IS_STREAM  — streaming / unsolicited push
 *   [7:4] reserved
 *
 * Total frame size = 12 + LEN  (max 212 bytes).
 */
#ifndef CDC_PROTO_H
#define CDC_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* -------------------------------------------------------------------------- */
/*  Constants                                                                  */
/* -------------------------------------------------------------------------- */

/** Maximum payload bytes in a single frame. */
#define CDC_PROTO_MAX_PAYLOAD   200

/** Maximum encoded frame size (SOF2 + VER1 + LEN1 + HDR6 + PAYLOAD200 + CRC2). */
#define CDC_PROTO_MAX_FRAME     (2 + 1 + 1 + 6 + CDC_PROTO_MAX_PAYLOAD + 2)

/** Protocol version. */
#define CDC_PROTO_VERSION       0x01

/** Start-of-frame magic bytes. */
#define CDC_PROTO_SOF0          0xAA
#define CDC_PROTO_SOF1          0x55

/** Header size (DEV + OP + SEQ + FLAGS + RSVD[2]). */
#define CDC_PROTO_HDR_SIZE      6

/** Fixed overhead bytes (SOF + VER + LEN + HDR + CRC). */
#define CDC_PROTO_OVERHEAD      (2 + 1 + 1 + CDC_PROTO_HDR_SIZE + 2)

/* -------------------------------------------------------------------------- */
/*  FLAGS bit masks                                                            */
/* -------------------------------------------------------------------------- */

#define CDC_FLAG_REPLY_REQ      (1u << 0)
#define CDC_FLAG_ACK_OK         (1u << 1)
#define CDC_FLAG_ACK_ERR        (1u << 2)
#define CDC_FLAG_IS_STREAM      (1u << 3)

/* -------------------------------------------------------------------------- */
/*  Well-known DEV addresses                                                   */
/* -------------------------------------------------------------------------- */

#define CDC_DEV_SYSTEM          0xF0   /**< System / management channel. */

/* -------------------------------------------------------------------------- */
/*  Well-known opcodes                                                         */
/* -------------------------------------------------------------------------- */

#define CDC_OP_SET              0x01
#define CDC_OP_GET              0x02
#define CDC_OP_STREAM           0x03
#define CDC_OP_PING             0x80

/* -------------------------------------------------------------------------- */
/*  Error codes (placed in first payload byte on ACK_ERR)                      */
/* -------------------------------------------------------------------------- */

#define CDC_ERR_NONE            0x00
#define CDC_ERR_BAD_CRC         0x01
#define CDC_ERR_BAD_LEN         0x02
#define CDC_ERR_BAD_VER         0x03
#define CDC_ERR_SEQ_MISMATCH    0x04
#define CDC_ERR_NOT_IMPL        0x05
#define CDC_ERR_UNKNOWN_DEV     0x06

/* -------------------------------------------------------------------------- */
/*  ProtoFrame — in-memory parsed / to-be-encoded frame                        */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t  dev;
    uint8_t  op;
    uint8_t  seq;
    uint8_t  flags;
    uint8_t  payload[CDC_PROTO_MAX_PAYLOAD];
    uint8_t  len;          /**< Payload length (0..200). */
} ProtoFrame;

/* -------------------------------------------------------------------------- */
/*  Statistics counters                                                         */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint32_t rx_frames_ok;
    uint32_t rx_crc_err;
    uint32_t rx_overrun;
    uint32_t tx_dropped;
    uint32_t tx_busy;
    uint32_t unknown_dev;
    uint32_t seq_err;
    uint32_t rx_silence_ms;
} CdcProtoStats;

/* -------------------------------------------------------------------------- */
/*  Device handler callback type                                               */
/* -------------------------------------------------------------------------- */

/**
 * Per-device handler.  Called from CommsTask context.
 * @param in   Incoming parsed frame.
 * @param out  Reply frame to fill.  The handler MUST set at least
 *             out->dev, out->op, out->flags, out->len, and payload.
 * @return 0 if a reply was generated, non-zero to suppress the reply.
 */
typedef int (*CdcDevHandler)(const ProtoFrame *in, ProtoFrame *out);

/** Safe-state function type — called on heartbeat timeout. */
typedef void (*CdcSafeStateFn)(void);

/* -------------------------------------------------------------------------- */
/*  Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * Initialise the CDC protocol library.
 * Creates the stream buffer, message buffer, CommsTask, and TxSenderTask.
 * Must be called once from a FreeRTOS context (after osKernelStart).
 */
void cdc_proto_init(void);

/**
 * Queue a frame for transmission.
 * @param frame  Fully populated ProtoFrame.
 * @return 0 on success, -1 if the TX message buffer is full.
 */
int cdc_send(const ProtoFrame *frame);

/**
 * Convenience: send an ACK (OK or ERR) reply to an incoming frame.
 * Copies dev, op, seq from @p in; sets flags and optional error payload.
 * @param in        The request frame being replied to.
 * @param ack_ok    Non-zero → ACK_OK, zero → ACK_ERR.
 * @param err_code  Error code byte (only used when ack_ok == 0).
 * @return 0 on success, -1 on TX buffer full.
 */
int cdc_send_ack(const ProtoFrame *in, int ack_ok, uint8_t err_code);

/**
 * Encode a ProtoFrame into a wire-format byte buffer.
 * @param frame   Source frame.
 * @param buf     Destination buffer (must be >= CDC_PROTO_MAX_FRAME).
 * @return Number of bytes written.
 */
size_t cdc_encode(const ProtoFrame *frame, uint8_t *buf);

/**
 * Decode a complete wire-format buffer into a ProtoFrame.
 * @param buf     Source buffer.
 * @param len     Number of bytes in buf.
 * @param frame   Destination frame.
 * @return 0 on success, -1 on CRC / format error.
 */
int cdc_decode(const uint8_t *buf, size_t len, ProtoFrame *frame);

/**
 * Return a pointer to the current statistics snapshot (read-only).
 */
const CdcProtoStats *cdc_proto_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* CDC_PROTO_H */
