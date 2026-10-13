/**
  ******************************************************************************
  * @file           : tue_acsi.c
  * @brief          : TUE ACSI command protocol over USB CDC (Virtual COM Port).
  ******************************************************************************
  * @attention
  *
  * Grammar:  <TARGET> <ID> <DIR> <FIELD> [ARG ...]
  *           <TARGET> <ID> <CODE> [VALUE ...]
  *           See docs/ACSI_PROTOCOL.md for the full specification.
  *
  * Header fields are numbers; payload values are numbers, except that a response
  * may carry literal text (e.g. the board name in SYS INFO). There is no
  * checksum field.
  *
  * USB reception happens in interrupt context, so it only pushes bytes into a
  * lock-free ring buffer. A dedicated task drains the ring, executes the
  * command (servo bus transactions may block on RTOS primitives) and emits the
  * reply.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tue_acsi.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include <math.h>

#include "cmsis_os2.h"
#include "usart.h"
#include "tue_stepper.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Private defines -----------------------------------------------------------*/
#define TUE_ACSI_RING_SIZE   512U
#define TUE_ACSI_PAYLOAD_MAX 200U
#define TUE_ACSI_FRAME_MAX   240U
#define TUE_ACSI_MEM_MAX     16U

/* Private variables ---------------------------------------------------------*/
static tue_sts3215_t     s_servo;
static bool              s_initialized = false;

static uint8_t           s_ring[TUE_ACSI_RING_SIZE];
static volatile uint16_t s_ring_head = 0U;   /* producer (USB ISR) writes here */
static volatile uint16_t s_ring_tail = 0U;   /* consumer (ACSI task) reads here */

static osSemaphoreId_t   s_rx_sem = NULL;
static osThreadId_t      s_task   = NULL;

static char              s_line[TUE_ACSI_MAX_LINE];
static uint16_t          s_line_len = 0U;

static uint8_t           s_tx[TUE_ACSI_FRAME_MAX];

/* Private function prototypes -----------------------------------------------*/
static bool acsi_parse_i32(const char *s, int32_t *out);
static int  acsi_append(char *buf, int cap, int off, const char *fmt, ...);
static int  acsi_append_bytes(char *buf, int cap, int off,
                              const uint8_t *bytes, int count);

static bool acsi_ring_push(uint8_t byte);
static bool acsi_ring_pop(uint8_t *byte);

static void acsi_transmit(const char *body);
static void acsi_respond(int32_t target, int32_t id, int code, const char *payload);
static void acsi_handle_line(char *line);
static void acsi_task(void *argument);

static int  acsi_map_err(tue_sts3215_status_t st);
static int  acsi_map_step_err(tue_stepper_status_t st);
static int  acsi_step_cmd(uint8_t id, bool is_set, int32_t field,
                          const int32_t *args, int nargs, char *payload);
static int  acsi_servo_cmd(uint8_t id, bool is_set, int32_t field,
                           const int32_t *args, int nargs, char *payload);
static int  acsi_sys_cmd(uint8_t id, bool is_set, int32_t field,
                         const int32_t *args, int nargs, char *payload);

/* Private helpers -----------------------------------------------------------*/

static bool acsi_parse_i32(const char *s, int32_t *out)
{
    if (s == NULL || *s == '\0') {
        return false;
    }
    char *end = NULL;
    long v = strtol(s, &end, 10);
    /* A token runs to the next space or the end of the line. */
    if (end == s || (*end != '\0' && *end != ' ')) {
        return false;
    }
    *out = (int32_t)v;
    return true;
}

static int acsi_append(char *buf, int cap, int off, const char *fmt, ...)
{
    if (off < 0 || off >= cap - 1) {
        return (off < 0) ? 0 : cap - 1;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(&buf[off], (size_t)(cap - off), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return off;
    }
    off += n;
    return (off >= cap) ? cap - 1 : off;
}

static int acsi_append_bytes(char *buf, int cap, int off, const uint8_t *bytes, int count)
{
    for (int i = 0; i < count; i++) {
        off = acsi_append(buf, cap, off, (off == 0) ? "%u" : " %u", (unsigned)bytes[i]);
    }
    return off;
}

static bool acsi_ring_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_ring_head + 1U) % TUE_ACSI_RING_SIZE);
    if (next == s_ring_tail) {
        return false; /* ring full: drop newest byte */
    }
    s_ring[s_ring_head] = byte;
    s_ring_head = next;
    return true;
}

static bool acsi_ring_pop(uint8_t *byte)
{
    if (s_ring_tail == s_ring_head) {
        return false;
    }
    *byte = s_ring[s_ring_tail];
    s_ring_tail = (uint16_t)((s_ring_tail + 1U) % TUE_ACSI_RING_SIZE);
    return true;
}

/* Transport -----------------------------------------------------------------*/

static void acsi_transmit(const char *body)
{
    int n = snprintf((char *)s_tx, sizeof(s_tx), "%s\r\n", body);
    if (n <= 0) {
        return;
    }
    uint16_t len = (n < (int)sizeof(s_tx)) ? (uint16_t)n : (uint16_t)(sizeof(s_tx) - 1U);

    for (int attempt = 0; attempt < 10; attempt++) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
        if (hcdc == NULL) {
            return; /* USB not enumerated: nothing to answer to */
        }
        if (hcdc->TxState == 0U) {
            if (CDC_Transmit_FS(s_tx, len) == 0U) {
                /* Keep s_tx stable until the transfer leaves the endpoint. */
                for (int wait = 0; wait < 50; wait++) {
                    if (hcdc->TxState == 0U) {
                        break;
                    }
                    osDelay(1);
                }
                return;
            }
        }
        osDelay(1);
    }
}

static void acsi_respond(int32_t target, int32_t id, int code, const char *payload)
{
    char body[TUE_ACSI_FRAME_MAX];
    if (payload != NULL && payload[0] != '\0') {
        (void)snprintf(body, sizeof(body), "%ld %ld %d %s",
                       (long)target, (long)id, code, payload);
    } else {
        (void)snprintf(body, sizeof(body), "%ld %ld %d",
                       (long)target, (long)id, code);
    }
    acsi_transmit(body);
}

/* Error mapping -------------------------------------------------------------*/

static int acsi_map_err(tue_sts3215_status_t st)
{
    switch (st) {
        case TUE_STS3215_OK:           return TUE_ACSI_OK;
        case TUE_STS3215_ERR_PARAM:    return TUE_ACSI_ERR_BAD_ARG;
        case TUE_STS3215_ERR_TIMEOUT:  return TUE_ACSI_ERR_BUS_TIMEOUT;
        case TUE_STS3215_ERR_BUSY:     return TUE_ACSI_ERR_BUSY;
        case TUE_STS3215_ERR_STATUS:   return TUE_ACSI_ERR_SERVO_FAULT;
        default:                       return TUE_ACSI_ERR_BUS_ERROR;
    }
}

/* SERVO target --------------------------------------------------------------*/

static int acsi_servo_cmd(uint8_t id, bool is_set, int32_t field,
                          const int32_t *args, int nargs, char *payload)
{
    tue_sts3215_status_t st = TUE_STS3215_OK;
    int32_t a0 = 0, a1 = 0, a2 = 0;

    if (nargs >= 1) a0 = args[0];
    if (nargs >= 2) a1 = args[1];
    if (nargs >= 3) a2 = args[2];

    switch (field) {
    case TUE_ACSI_SERVO_PING:
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        st = tue_sts3215_ping(&s_servo, id);
        if (st != TUE_STS3215_OK) return acsi_map_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", (unsigned)id);
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_POS:
        if (!is_set) {
            int16_t v = 0;
            st = tue_sts3215_get_current_position(&s_servo, id, &v);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)v);
        } else {
            if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
            if (nargs >= 3) {
                st = tue_sts3215_set_goal_position_ex(&s_servo, id,
                        (int16_t)a0, (int16_t)a1, (uint8_t)a2);
            } else if (nargs == 2) {
                st = tue_sts3215_set_goal_position_ex(&s_servo, id,
                        (int16_t)a0, (int16_t)a1, 0U);
            } else {
                st = tue_sts3215_set_goal_position(&s_servo, id, (int16_t)a0);
            }
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_VEL:
        if (!is_set) {
            int16_t v = 0;
            st = tue_sts3215_get_current_speed(&s_servo, id, &v);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)v);
        } else {
            if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
            if (nargs >= 2) {
                st = tue_sts3215_set_acceleration(&s_servo, id, (uint8_t)a1);
                if (st != TUE_STS3215_OK) return acsi_map_err(st);
            }
            st = tue_sts3215_set_running_speed(&s_servo, id, (int16_t)a0);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_ACC:
        if (!is_set) {
            uint8_t v = 0;
            st = tue_sts3215_get_acceleration(&s_servo, id, &v);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", (unsigned)v);
        } else {
            if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
            st = tue_sts3215_set_acceleration(&s_servo, id, (uint8_t)a0);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_TORQUE:
        if (!is_set) {
            bool en = false;
            st = tue_sts3215_get_torque_enable(&s_servo, id, &en);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", en ? 1U : 0U);
        } else {
            if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
            if (a0 == 128) {
                st = tue_sts3215_calibrate_position_to_2048(&s_servo, id);
            } else {
                st = tue_sts3215_enable_torque(&s_servo, id, (a0 != 0));
            }
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_TLIM:
        if (!is_set) {
            uint16_t v = 0;
            st = tue_sts3215_get_torque_limit(&s_servo, id, &v);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", (unsigned)v);
        } else {
            if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
            st = tue_sts3215_set_torque_limit(&s_servo, id, (uint16_t)a0);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_MODE:
        if (!is_set) {
            tue_sts3215_mode_t m = STS3215_MODE_POSITION;
            st = tue_sts3215_get_operation_mode(&s_servo, id, &m);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", (unsigned)m);
        } else {
            if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
            st = tue_sts3215_set_operation_mode(&s_servo, id, (tue_sts3215_mode_t)a0);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_PWM:
        if (!is_set) {
            int16_t v = 0;
            st = tue_sts3215_get_pwm_runtime(&s_servo, id, &v);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)v);
        } else {
            if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
            st = tue_sts3215_set_pwm_runtime(&s_servo, id, (int16_t)a0);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_MEM: {
        /*   GET MEM <addr> [len]          -> b0 b1 ...            */
        /*   SET MEM <addr> <b0> [b1 ...]  -> <addr>               */
        if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
        if (!is_set) {
            int32_t len = (nargs >= 2) ? a1 : 1;
            if (len < 1 || len > (int32_t)TUE_ACSI_MEM_MAX) return TUE_ACSI_ERR_BAD_ARG;
            uint8_t buf[TUE_ACSI_MEM_MAX];
            st = tue_sts3215_read_raw(&s_servo, id, (uint8_t)a0, buf, (uint8_t)len);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)acsi_append_bytes(payload, TUE_ACSI_PAYLOAD_MAX, 0, buf, (int)len);
        } else {
            if (nargs < 2) return TUE_ACSI_ERR_BAD_ARG;
            int32_t cnt = nargs - 1;
            if (cnt > (int32_t)TUE_ACSI_MEM_MAX) return TUE_ACSI_ERR_BAD_ARG;
            uint8_t buf[TUE_ACSI_MEM_MAX];
            for (int32_t i = 0; i < cnt; i++) {
                int32_t b = args[i + 1];
                if (b < 0 || b > 255) return TUE_ACSI_ERR_BAD_ARG;
                buf[i] = (uint8_t)b;
            }
            st = tue_sts3215_write_raw(&s_servo, id, (uint8_t)a0, buf, (uint8_t)cnt);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;
    }

    case TUE_ACSI_SERVO_FB: {
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        tue_sts3215_feedback_t fb;
        memset(&fb, 0, sizeof(fb));
        st = tue_sts3215_get_feedback(&s_servo, id, &fb);
        if (st != TUE_STS3215_OK && st != TUE_STS3215_ERR_STATUS) {
            return acsi_map_err(st);
        }
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d %d %d %u %d %u %u %d",
                       (int)fb.position, (int)fb.speed, (int)fb.load,
                       (unsigned)fb.voltage_raw, (int)fb.temperature_degc,
                       (unsigned)fb.status_mask, (unsigned)fb.is_moving,
                       (int)fb.current_raw);
        return (st == TUE_STS3215_ERR_STATUS) ? TUE_ACSI_ERR_SERVO_FAULT : TUE_ACSI_OK;
    }

    case TUE_ACSI_SERVO_ID:
        if (!is_set) {
            uint8_t v = 0;
            st = tue_sts3215_get_id(&s_servo, id, &v);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", (unsigned)v);
        } else {
            if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
            st = tue_sts3215_set_id(&s_servo, id, (uint8_t)a0);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_BAUD:
        if (!is_set) {
            tue_sts3215_baud_t b = STS3215_BAUD_1000000;
            st = tue_sts3215_get_baud_rate(&s_servo, id, &b);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", (unsigned)b);
        } else {
            if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
            st = tue_sts3215_set_baud_rate(&s_servo, id, (tue_sts3215_baud_t)a0);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_LIMIT:
        if (!is_set) {
            uint16_t mn = 0, mx = 0;
            st = tue_sts3215_get_angle_limits(&s_servo, id, &mn, &mx);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u %u",
                           (unsigned)mn, (unsigned)mx);
        } else {
            if (nargs < 2) return TUE_ACSI_ERR_BAD_ARG;
            st = tue_sts3215_set_angle_limits(&s_servo, id, (uint16_t)a0, (uint16_t)a1);
            if (st != TUE_STS3215_OK) return acsi_map_err(st);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SERVO_TEMP: {
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        int8_t v = 0;
        st = tue_sts3215_get_current_temperature(&s_servo, id, &v);
        if (st != TUE_STS3215_OK) return acsi_map_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)v);
        return TUE_ACSI_OK;
    }
    case TUE_ACSI_SERVO_VOLT: {
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        uint8_t v = 0;
        st = tue_sts3215_get_current_voltage(&s_servo, id, &v);
        if (st != TUE_STS3215_OK) return acsi_map_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", (unsigned)v);
        return TUE_ACSI_OK;
    }
    case TUE_ACSI_SERVO_LOAD: {
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        int16_t v = 0;
        st = tue_sts3215_get_current_load(&s_servo, id, &v);
        if (st != TUE_STS3215_OK) return acsi_map_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)v);
        return TUE_ACSI_OK;
    }
    case TUE_ACSI_SERVO_CUR: {
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        int16_t v = 0;
        st = tue_sts3215_get_motor_current_raw(&s_servo, id, &v);
        if (st != TUE_STS3215_OK) return acsi_map_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)v);
        return TUE_ACSI_OK;
    }
    case TUE_ACSI_SERVO_STATUS: {
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        uint8_t v = 0;
        st = tue_sts3215_get_servo_status(&s_servo, id, &v);
        if (st != TUE_STS3215_OK) return acsi_map_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", (unsigned)v);
        return TUE_ACSI_OK;
    }
    case TUE_ACSI_SERVO_MOVING: {
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        bool mv = false;
        st = tue_sts3215_is_moving(&s_servo, id, &mv);
        if (st != TUE_STS3215_OK) return acsi_map_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", mv ? 1U : 0U);
        return TUE_ACSI_OK;
    }

    case TUE_ACSI_SERVO_RESET:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        st = tue_sts3215_reset(&s_servo, id);
        if (st != TUE_STS3215_OK) return acsi_map_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", (unsigned)id);
        return TUE_ACSI_OK;

    default:
        return TUE_ACSI_ERR_UNKNOWN_FIELD;
    }
}

/* SYS target ----------------------------------------------------------------*/

static int acsi_sys_cmd(uint8_t id, bool is_set, int32_t field,
                        const int32_t *args, int nargs, char *payload)
{
    int32_t a0 = 0, a1 = 0;
    (void)id;

    if (nargs >= 1) a0 = args[0];
    if (nargs >= 2) a1 = args[1];

    switch (field) {
    case TUE_ACSI_SYS_PING:
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "1");
        return TUE_ACSI_OK;

    case TUE_ACSI_SYS_ECHO:
        if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a0);
        return TUE_ACSI_OK;

    case TUE_ACSI_SYS_INFO: {
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        char fw[16];
        (void)snprintf(fw, sizeof(fw), "%u.%u",
                       (unsigned)TUE_ACSI_FW_VERSION_MAJOR,
                       (unsigned)TUE_ACSI_FW_VERSION_MINOR);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%s %s", TUE_ACSI_BOARD_NAME, fw);
        return TUE_ACSI_OK;
    }

    case TUE_ACSI_SYS_UID:
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%lu %lu %lu",
                       (unsigned long)HAL_GetUIDw0(),
                       (unsigned long)HAL_GetUIDw1(),
                       (unsigned long)HAL_GetUIDw2());
        return TUE_ACSI_OK;

    case TUE_ACSI_SYS_UPTIME:
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%lu", (unsigned long)HAL_GetTick());
        return TUE_ACSI_OK;

    case TUE_ACSI_SYS_LED:
        if (!is_set) {
            GPIO_PinState s = HAL_GPIO_ReadPin(LED_MCU_GPIO_Port, LED_MCU_Pin);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u",
                           (s == GPIO_PIN_SET) ? 1U : 0U);
        } else {
            if (nargs < 2) return TUE_ACSI_ERR_BAD_ARG;
            if (a0 != 0) return TUE_ACSI_ERR_BAD_ARG;
            HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin,
                              (a1 != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%d", (int)a1);
        }
        return TUE_ACSI_OK;

    case TUE_ACSI_SYS_RESET:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        acsi_transmit("4 0 0");     /* answer before resetting */
        osDelay(20);
        HAL_NVIC_SystemReset();
        return TUE_ACSI_OK;

    default:
        return TUE_ACSI_ERR_UNKNOWN_FIELD;
    }
}

/* STEP target ----------------------------------------------------------------*/
/*
 * Bridges the tue_stepper library to the numeric protocol. The wire carries
 * integer micrometres (1 um = 0.001 mm); the library works in millimetres.
 * ID 0 addresses the tool (CoreXY X/Y + Z), ID 1..3 a single motor.
 */

static float acsi_um_to_mm(int32_t um)
{
    return (float)um / 1000.0f;
}

static int32_t acsi_mm_to_um(float mm)
{
    return (int32_t)lroundf(mm * 1000.0f);
}

static int acsi_map_step_err(tue_stepper_status_t st)
{
    switch (st) {
        case TUE_STEPPER_OK:             return TUE_ACSI_OK;
        case TUE_STEPPER_ERR_PARAM:      return TUE_ACSI_ERR_BAD_ARG;
        case TUE_STEPPER_ERR_QUEUE_FULL: return TUE_ACSI_ERR_QUEUE_FULL;
        case TUE_STEPPER_BUSY:           return TUE_ACSI_ERR_BUSY;
        case TUE_STEPPER_ERR_TIMEOUT:    return TUE_ACSI_ERR_TIMEOUT;
        default:                         return TUE_ACSI_ERR_INTERNAL;
    }
}

/* Emit "<x> <y> <z>" in um, either the measured or the commanded position. */
static void acsi_step_pos_payload(char *payload, bool wanted_target)
{
    float x = 0.0f, y = 0.0f, z = 0.0f;
    if (wanted_target) {
        tue_stepper_get_target_mm(&x, &y, &z);
    } else {
        tue_stepper_get_position_mm(&x, &y, &z);
    }
    (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld %ld %ld",
                   (long)acsi_mm_to_um(x), (long)acsi_mm_to_um(y), (long)acsi_mm_to_um(z));
}

static int acsi_step_cmd(uint8_t id, bool is_set, int32_t field,
                         const int32_t *args, int nargs, char *payload)
{
    tue_stepper_status_t st = TUE_STEPPER_OK;
    int32_t a0 = 0, a1 = 0, a2 = 0, a3 = 0, a4 = 0;
    bool motor_ok = (id >= 1U) && (id <= (uint8_t)TUE_STEPPER_MOTOR_COUNT);

    if (nargs >= 1) a0 = args[0];
    if (nargs >= 2) a1 = args[1];
    if (nargs >= 3) a2 = args[2];
    if (nargs >= 4) a3 = args[3];
    if (nargs >= 5) a4 = args[4];

    switch (field) {

    case TUE_ACSI_STEP_INFO: {
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        tue_stepper_config_t cfg;
        tue_stepper_get_config(&cfg);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld %ld %ld %ld",
                       (long)lroundf(cfg.steps_per_mm[TUE_STEPPER_M1]),
                       (long)lroundf(cfg.steps_per_mm[TUE_STEPPER_M3]),
                       (long)lroundf(TUE_STEPPER_MIN_RATE_STEPS_S),
                       (long)lroundf(TUE_STEPPER_MAX_RATE_STEPS_S));
        return TUE_ACSI_OK;
    }

    case TUE_ACSI_STEP_POS:
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        acsi_step_pos_payload(payload, false);
        return TUE_ACSI_OK;

    case TUE_ACSI_STEP_TARGET:
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        acsi_step_pos_payload(payload, true);
        return TUE_ACSI_OK;

    case TUE_ACSI_STEP_STATUS:
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u %u",
                       tue_stepper_is_busy() ? 1U : 0U,
                       tue_stepper_is_enabled() ? 1U : 0U);
        return TUE_ACSI_OK;

    case TUE_ACSI_STEP_ENABLE:
        if (!is_set) {
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u",
                           tue_stepper_is_enabled() ? 1U : 0U);
        } else {
            if (nargs < 1) return TUE_ACSI_ERR_BAD_ARG;
            tue_stepper_enable(a0 != 0);
            (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u", (a0 != 0) ? 1U : 0U);
        }
        return TUE_ACSI_OK;

    /* --- absolute / relative position moves ---------------------------- */
    case TUE_ACSI_STEP_MOVE:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        if (nargs < 5) return TUE_ACSI_ERR_BAD_ARG;
        st = tue_stepper_move_to_xyz_mm(acsi_um_to_mm(a0), acsi_um_to_mm(a1),
                                        acsi_um_to_mm(a2), acsi_um_to_mm(a3),
                                        acsi_um_to_mm(a4));
        if (st != TUE_STEPPER_OK) return acsi_map_step_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld %ld %ld",
                       (long)a0, (long)a1, (long)a2);
        return TUE_ACSI_OK;

    case TUE_ACSI_STEP_MOVE2D:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        if (nargs < 4) return TUE_ACSI_ERR_BAD_ARG;
        st = tue_stepper_move_to_mm(acsi_um_to_mm(a0), acsi_um_to_mm(a1),
                                    acsi_um_to_mm(a2), acsi_um_to_mm(a3));
        if (st != TUE_STEPPER_OK) return acsi_map_step_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld %ld", (long)a0, (long)a1);
        return TUE_ACSI_OK;

    case TUE_ACSI_STEP_MOVEZ:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        if (nargs < 3) return TUE_ACSI_ERR_BAD_ARG;
        st = tue_stepper_move_z_mm(acsi_um_to_mm(a0), acsi_um_to_mm(a1), acsi_um_to_mm(a2));
        if (st != TUE_STEPPER_OK) return acsi_map_step_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld", (long)a0);
        return TUE_ACSI_OK;

    case TUE_ACSI_STEP_MOVEREL:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        if (nargs < 4) return TUE_ACSI_ERR_BAD_ARG;
        st = tue_stepper_move_by_mm(acsi_um_to_mm(a0), acsi_um_to_mm(a1),
                                    acsi_um_to_mm(a2), acsi_um_to_mm(a3));
        if (st != TUE_STEPPER_OK) return acsi_map_step_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld %ld", (long)a0, (long)a1);
        return TUE_ACSI_OK;

    /* --- velocity jogs -------------------------------------------------- */
    case TUE_ACSI_STEP_JOG:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        if (nargs < 3) return TUE_ACSI_ERR_BAD_ARG;
        st = tue_stepper_start_velocity(acsi_um_to_mm(a0), acsi_um_to_mm(a1),
                                        acsi_um_to_mm(a2));
        if (st != TUE_STEPPER_OK) return acsi_map_step_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld %ld", (long)a0, (long)a1);
        return TUE_ACSI_OK;

    case TUE_ACSI_STEP_JOGZ:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        if (nargs < 2) return TUE_ACSI_ERR_BAD_ARG;
        st = tue_stepper_start_velocity_z(acsi_um_to_mm(a0), acsi_um_to_mm(a1));
        if (st != TUE_STEPPER_OK) return acsi_map_step_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld", (long)a0);
        return TUE_ACSI_OK;

    /* --- stopping ------------------------------------------------------- */
    case TUE_ACSI_STEP_STOP:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        tue_stepper_stop();
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%u",
                       tue_stepper_is_busy() ? 1U : 0U);
        return TUE_ACSI_OK;

    case TUE_ACSI_STEP_ABORT:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        tue_stepper_abort();
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "0");
        return TUE_ACSI_OK;

    /* --- position bookkeeping ------------------------------------------- */
    case TUE_ACSI_STEP_SETPOS:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        if (nargs < 3) return TUE_ACSI_ERR_BAD_ARG;
        tue_stepper_set_position_mm(acsi_um_to_mm(a0), acsi_um_to_mm(a1),
                                    acsi_um_to_mm(a2));
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld %ld %ld",
                       (long)a0, (long)a1, (long)a2);
        return TUE_ACSI_OK;

    /* --- per-motor raw access (ID 1..3) --------------------------------- */
    case TUE_ACSI_STEP_STEPS:
        if (is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        if (!motor_ok) return TUE_ACSI_ERR_BAD_ARG;
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld",
                       (long)tue_stepper_get_motor_steps((tue_stepper_motor_t)(id - 1U)));
        return TUE_ACSI_OK;

    case TUE_ACSI_STEP_RAW:
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        if (!motor_ok) return TUE_ACSI_ERR_BAD_ARG;
        if (nargs < 3) return TUE_ACSI_ERR_BAD_ARG;
        st = tue_stepper_move_motor_steps((tue_stepper_motor_t)(id - 1U), a0,
                                          acsi_um_to_mm(a1), acsi_um_to_mm(a2));
        if (st != TUE_STEPPER_OK) return acsi_map_step_err(st);
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%ld", (long)a0);
        return TUE_ACSI_OK;

    /* --- wait for the motion queue to drain ----------------------------- */
    case TUE_ACSI_STEP_WAIT: {
        if (!is_set) return TUE_ACSI_ERR_NOT_SUPPORTED;
        uint32_t timeout_ms = ((nargs >= 1) && (a0 > 0)) ? (uint32_t)a0 : 0U;
        uint32_t start = HAL_GetTick();
        while (tue_stepper_is_busy()) {
            if ((timeout_ms != 0U) && ((HAL_GetTick() - start) >= timeout_ms)) {
                return TUE_ACSI_ERR_TIMEOUT;
            }
            osDelay(1);
        }
        (void)snprintf(payload, TUE_ACSI_PAYLOAD_MAX, "%lu",
                       (unsigned long)(HAL_GetTick() - start));
        return TUE_ACSI_OK;
    }

    default:
        return TUE_ACSI_ERR_UNKNOWN_FIELD;
    }
}

/* Frame handling ------------------------------------------------------------*/

static void acsi_handle_line(char *line)
{
    size_t len = strlen(line);
    if (len > 0U && line[len - 1U] == '\r') {
        line[len - 1U] = '\0';
        len--;
    }
    if (len == 0U) {
        return; /* ignore empty lines */
    }

    /* Tokenise the frame into numbers (header + numeric arguments). */
    int32_t vals[TUE_ACSI_MAX_TOKENS];
    int nt = 0;
    char *p = line;
    while (*p != '\0') {
        while (*p == ' ') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        int32_t v = 0;
        if (!acsi_parse_i32(p, &v)) {
            acsi_respond((nt >= 1) ? vals[0] : TUE_ACSI_TARGET_SYS,
                         (nt >= 2) ? vals[1] : 0, TUE_ACSI_ERR_BAD_FORMAT, NULL);
            return;
        }
        if (nt < (int)TUE_ACSI_MAX_TOKENS) {
            vals[nt++] = v;
        }
        while (*p != '\0' && *p != ' ') {
            p++;
        }
        if (*p != '\0') {
            *p++ = '\0';
        }
    }

    if (nt < 4) {
        acsi_respond((nt >= 1) ? vals[0] : TUE_ACSI_TARGET_SYS,
                     (nt >= 2) ? vals[1] : 0, TUE_ACSI_ERR_BAD_FORMAT, NULL);
        return;
    }

    int32_t target = vals[0];
    int32_t id32 = vals[1];
    int32_t dir = vals[2];
    int32_t field = vals[3];

    if (id32 < 0 || id32 > 255) {
        acsi_respond(target, id32, TUE_ACSI_ERR_BAD_ARG, NULL);
        return;
    }
    if (dir != (int32_t)TUE_ACSI_DIR_GET && dir != (int32_t)TUE_ACSI_DIR_SET) {
        acsi_respond(target, id32, TUE_ACSI_ERR_BAD_FORMAT, NULL);
        return;
    }
    bool is_set = (dir == (int32_t)TUE_ACSI_DIR_SET);
    uint8_t dev_id = (uint8_t)id32;

    char payload[TUE_ACSI_PAYLOAD_MAX];
    payload[0] = '\0';

    const int32_t *args = &vals[4];
    int nargs = nt - 4;

    int err;
    switch (target) {
    case TUE_ACSI_TARGET_SERVO:
        err = acsi_servo_cmd(dev_id, is_set, field, args, nargs, payload);
        break;
    case TUE_ACSI_TARGET_SYS:
        err = acsi_sys_cmd(dev_id, is_set, field, args, nargs, payload);
        break;
    case TUE_ACSI_TARGET_STEP:
        err = acsi_step_cmd(dev_id, is_set, field, args, nargs, payload);
        break;
    case TUE_ACSI_TARGET_DC:
        err = TUE_ACSI_ERR_NOT_SUPPORTED; /* reserved target */
        break;
    default:
        err = TUE_ACSI_ERR_UNKNOWN_TARGET;
        break;
    }

    acsi_respond(target, id32, err, (err == TUE_ACSI_OK) ? payload : NULL);
}

static void acsi_task(void *argument)
{
    (void)argument;
    for (;;) {
        (void)osSemaphoreAcquire(s_rx_sem, osWaitForever);

        uint8_t byte;
        while (acsi_ring_pop(&byte)) {
            if (byte == (uint8_t)'\n') {
                s_line[s_line_len] = '\0';
                acsi_handle_line(s_line);
                s_line_len = 0U;
            } else if (s_line_len < (TUE_ACSI_MAX_LINE - 1U)) {
                s_line[s_line_len++] = (char)byte;
            } else {
                s_line_len = 0U; /* line too long: drop it and resync */
            }
        }
    }
}

/* Public API ----------------------------------------------------------------*/

void tue_acsi_init(void)
{
    if (s_initialized) {
        return;
    }

    /* The SERVO target drives the Feetech bus on USART1. */
    (void)tue_sts3215_init(&s_servo, &huart1);

    /* The STEP target drives the three stepper motors through the TIM1 step
     * clock. Idempotent: main() already calls it at boot. */
    tue_stepper_init();

    const osSemaphoreAttr_t sem_attr = { .name = "acsi_rx" };
    s_rx_sem = osSemaphoreNew(1U, 0U, &sem_attr);

    const osThreadAttr_t task_attr = {
        .name = "tue_acsi",
        .stack_size = 4096,
        .priority = (osPriority_t)osPriorityNormal,
    };
    s_task = osThreadNew(acsi_task, NULL, &task_attr);

    s_initialized = true;
}

void tue_acsi_on_cdc_rx(const uint8_t *data, uint32_t len)
{
    if (s_rx_sem == NULL || data == NULL) {
        return;
    }
    for (uint32_t i = 0; i < len; i++) {
        (void)acsi_ring_push(data[i]);
    }
    (void)osSemaphoreRelease(s_rx_sem);
}

tue_sts3215_t *tue_acsi_servo(void)
{
    return &s_servo;
}
