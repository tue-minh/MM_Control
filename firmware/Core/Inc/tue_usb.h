/**
  ******************************************************************************
  * @file           : tue_usb.h
  * @brief          : TUE USB binary protocol header for STM32F407
  ******************************************************************************
  * @attention
  *
  * Robust, modular binary request/response protocol over USB CDC.
  * Explicit little-endian serialization, CRC-16/CCITT-FALSE integrity,
  * sequence numbers, and FreeRTOS task-based decoupled frame processing.
  *
  ******************************************************************************
  */

#ifndef TUE_USB_H_
#define TUE_USB_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"

/* Protocol Frame Constants --------------------------------------------------*/
#define TUE_USB_SOF1                0xAAU
#define TUE_USB_SOF2                0x55U
#define TUE_USB_HEADER_SIZE         7U   /* SOF1(1) + SOF2(1) + SEQ(1) + CMD(1) + STATUS(1) + LEN(2) */
#define TUE_USB_CRC_SIZE            2U   /* CRC16(2) */
#define TUE_USB_MAX_PAYLOAD         256U
#define TUE_USB_MAX_FRAME_SIZE      (TUE_USB_HEADER_SIZE + TUE_USB_MAX_PAYLOAD + TUE_USB_CRC_SIZE) /* 265 bytes */

/* Status Codes --------------------------------------------------------------*/
typedef enum {
    TUE_STATUS_OK                   = 0x00U,
    TUE_STATUS_ERR_UNKNOWN_CMD      = 0x01U,
    TUE_STATUS_ERR_INVALID_LEN      = 0x02U,
    TUE_STATUS_ERR_INVALID_PARAM    = 0x03U,
    TUE_STATUS_ERR_CRC              = 0x04U,
    TUE_STATUS_ERR_BUSY             = 0x05U,
    TUE_STATUS_ERR_EXEC             = 0x06U
} tue_status_t;

/* Command IDs ---------------------------------------------------------------*/
/* 0x01–0x0F: System Commands */
#define TUE_CMD_SYS_PING            0x01U
#define TUE_CMD_SYS_GET_INFO        0x02U
#define TUE_CMD_SYS_RESET           0x03U
#define TUE_CMD_SYS_LED_CTRL        0x04U
#define TUE_CMD_SYS_GET_STATUS      0x05U

/* 0x10–0x1F: DC Motor Commands */
#define TUE_CMD_DC_SET_DUTY         0x10U
#define TUE_CMD_DC_STOP             0x11U
#define TUE_CMD_DC_GET_STATUS       0x12U
#define TUE_CMD_DC_SET_PID          0x13U

/* 0x20–0x2F: Stepper Motor Commands */
#define TUE_CMD_STEP_ENABLE         0x20U
#define TUE_CMD_STEP_MOVE           0x21U
#define TUE_CMD_STEP_STOP           0x22U
#define TUE_CMD_STEP_GET_POS        0x23U
#define TUE_CMD_STEP_SET_HOMING     0x24U

/* 0x30–0x3F: Servo Commands (STS3215) */
#define TUE_CMD_SERVO_PING          0x30U
#define TUE_CMD_SERVO_READ_REG      0x31U
#define TUE_CMD_SERVO_WRITE_REG     0x32U
#define TUE_CMD_SERVO_SET_POS       0x33U
#define TUE_CMD_SERVO_GET_FEEDBACK  0x34U
#define TUE_CMD_SERVO_TORQUE_ENABLE 0x35U

/* 0x40–0x4F: Sensor Commands */
#define TUE_CMD_SENSOR_READ_IMU     0x40U
#define TUE_CMD_SENSOR_READ_ENCODER 0x41U
#define TUE_CMD_SENSOR_READ_ADC     0x42U
#define TUE_CMD_SENSOR_READ_GPIO    0x43U

/* 0x50–0x5F: Bus Passthrough Commands */
#define TUE_CMD_BUS_CAN_TX          0x50U
#define TUE_CMD_BUS_CAN_RX          0x51U
#define TUE_CMD_BUS_I2C_TRANSFER    0x52U
#define TUE_CMD_BUS_SPI_TRANSFER    0x53U

/* 0x60–0x6F: Configuration and Control Modes */
#define TUE_CMD_CFG_SET_MODE        0x60U
#define TUE_CMD_CFG_GET_MODE        0x61U
#define TUE_CMD_CFG_SAVE_FLASH      0x62U
#define TUE_CMD_CFG_LOAD_DEFAULT    0x63U

/* LED Control Modes ---------------------------------------------------------*/
#define TUE_LED_MODE_OFF            0x00U
#define TUE_LED_MODE_ON             0x01U
#define TUE_LED_MODE_TOGGLE         0x02U
#define TUE_LED_MODE_QUERY          0xFFU

/* Frame Structure Definition ------------------------------------------------*/
typedef struct {
    uint8_t  seq;
    uint8_t  cmd;
    uint8_t  status;
    uint16_t len;
    uint8_t  payload[TUE_USB_MAX_PAYLOAD];
    uint16_t crc;
} tue_frame_t;

/* Command Handler Signature -------------------------------------------------*/
/**
  * @brief  Command handler function pointer type.
  * @param  req_payload  Pointer to input request payload bytes.
  * @param  req_len      Length of input request payload.
  * @param  resp_payload Buffer to populate output response payload.
  * @param  resp_len     Pointer to store output response payload length.
  * @param  max_resp_len Capacity of resp_payload buffer.
  * @retval tue_status_t Execution status.
  */
typedef tue_status_t (*tue_cmd_handler_t)(
    const uint8_t *req_payload,
    uint16_t req_len,
    uint8_t *resp_payload,
    uint16_t *resp_len,
    uint16_t max_resp_len
);

/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  Initializes the TUE USB subsystem: creates FreeRTOS stream buffer,
  *         mutex, and worker task.
  */
void tue_usb_init(void);

/**
  * @brief  Non-blocking ISR callback to be invoked directly from CDC_Receive_FS.
  *         Transfers received byte stream into FreeRTOS stream buffer without parsing.
  * @param  data  Pointer to received data buffer from USB endpoint.
  * @param  len   Number of received bytes.
  */
void tue_usb_on_cdc_rx(const uint8_t *data, uint32_t len);

/**
  * @brief  Feed a single byte into the frame parser state machine.
  * @param  byte  Raw byte to process.
  */
void tue_usb_feed_byte(uint8_t byte);

/**
  * @brief  Registers a command handler into the dynamic command table.
  * @param  cmd_id       Command ID (0x01..0xFF).
  * @param  min_req_len  Minimum accepted request payload length.
  * @param  max_req_len  Maximum accepted request payload length.
  * @param  handler      Function pointer to command handler.
  * @retval tue_status_t TUE_STATUS_OK on success, error if table full.
  */
tue_status_t tue_usb_register_command(
    uint8_t cmd_id,
    uint16_t min_req_len,
    uint16_t max_req_len,
    tue_cmd_handler_t handler
);

/**
  * @brief  Thread-safe transmission of a protocol frame over USB CDC.
  * @param  seq     Sequence number to echo or send.
  * @param  cmd     Command ID.
  * @param  status  Status code.
  * @param  payload Pointer to payload bytes (or NULL if len == 0).
  * @param  len     Payload length.
  * @retval tue_status_t TUE_STATUS_OK or error code.
  */
tue_status_t tue_usb_send_response(
    uint8_t seq,
    uint8_t cmd,
    tue_status_t status,
    const uint8_t *payload,
    uint16_t len
);

/**
  * @brief  Calculates CRC-16/CCITT-FALSE over a buffer using fast table lookup.
  * @param  data     Pointer to data.
  * @param  len      Length in bytes.
  * @param  init_crc Initial CRC value (typically 0xFFFF).
  * @retval uint16_t Computed CRC-16.
  */
uint16_t tue_usb_crc16(const uint8_t *data, size_t len, uint16_t init_crc);

#ifdef __cplusplus
}
#endif

#endif /* TUE_USB_H_ */
