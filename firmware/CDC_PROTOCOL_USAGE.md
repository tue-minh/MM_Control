# CDC Protocol Library

This is a USB CDC (Virtual COM Port) binary protocol library for STM32F407VET6 microcontrollers using FreeRTOS and the STM32Cube HAL.

## Overview

The CDC protocol provides a structured binary communication interface over USB to enable host-to-device and device-to-host data exchange. It implements:

- Binary frame encoding/decoding with CRC16-CCITT
- Asynchronous command/response pattern
- Sequence tracking and error handling
- Heartbeat monitoring and safe state management
- Comprehensive statistics and debugging support

## Key Features

- **Binary Protocol**: SOF (0xAA 0x55), VER (0x01), LEN, DEV, OP, SEQ, FLAGS, RSVD, PAYLOAD, CRC16
- **Error Handling**: CRC validation, sequence tracking, unknown device detection
- **Reliability**: Resynchronization on invalid bytes, RX silence monitoring
- **Safety**: Heartbeat PING responses, configurable safe state handling
- **Statistics**: Real-time counters for all major operations
- **Non-blocking**: ISR-safe with task-based processing

## Frame Format

```
Offset  Field       Size   Description
------  ----------  -----  -------------------------------------------
0       SOF         2      0xAA 0x55 (Start-of-frame marker)
2       VER         1      Protocol version (0x01)
3       LEN         1      Payload length (0..200)
4       DEV         1      Device address (0xF0 for system, 0x10-0xFF for modules)
5       OP          1      Operation code
6       SEQ         1      Sequence number (per-DEV, monotonic)
7       FLAGS       1      Bit-field:
                             [0] REPLY_REQ - host requests reply
                             [1] ACK_OK    - success acknowledgment
                             [2] ACK_ERR   - error acknowledgment
                             [3] IS_STREAM  - unsolicited push
8       RSVD        2      Reserved (0x0000)
10      PAYLOAD     LEN    Variable-length payload
10+LEN  CRC16       2      CRC16-CCITT over bytes [2..10+LEN-1]
```

## API Reference

### Initialization

```c
void cdc_proto_init(void);
```

Initializes the protocol library:
- Creates stream buffer for RX (ISR → CommsTask)
- Creates message buffer for TX (CommsTask → TxSenderTask)
- Creates CommsTask (1024 words, AboveNormal priority)
- Creates TxSenderTask (512 words, Normal priority)
- Sends boot banner (DEV=0xF0 OP=0x80, payload "MM C\x01\x00\x00\x00\x00")

### Core Operations

```c
int cdc_send(const ProtoFrame *frame);
int cdc_send_ack(const ProtoFrame *in, int ack_ok, uint8_t err_code);
size_t cdc_encode(const ProtoFrame *frame, uint8_t *buf);
int cdc_decode(const uint8_t *buf, size_t len, ProtoFrame *frame);
```

### Device Handlers

Module handlers should follow this signature:

```c
typedef int (*CdcDevHandler)(const ProtoFrame *in, ProtoFrame *out);
```

The handler must:
- Parse the incoming frame and determine the operation
- Fill the output ProtoFrame with response (DEV, OP, SEQ, flags, len, payload)
- Return 0 if a reply was generated, non-zero to suppress reply

### Statistics

```c
const CdcProtoStats *cdc_proto_stats(void);
```

Returns a pointer to statistics structure with counters:

```c
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
```

### Error Codes

```c
#define CDC_ERR_NONE            0x00
#define CDC_ERR_BAD_CRC         0x01
#define CDC_ERR_BAD_LEN         0x02
#define CDC_ERR_BAD_VER         0x03
#define CDC_ERR_SEQ_MISMATCH    0x04
#define CDC_ERR_NOT_IMPL        0x05
#define CDC_ERR_UNKNOWN_DEV     0x06
```

## Usage Examples

### 1. Basic PING Command

Host sends (hex): `AA 55 01 00 F0 80 00 00 00 00 00`
- SOF: AA 55
- VER: 01
- LEN: 00 (no payload)
- DEV: F0 (system)
- OP: 80 (PING)
- SEQ: 00
- FLAGS: 00
- RSVD: 00 00
- CRC: 00 00 (placeholder)

Device responds with same payload plus ACK_OK and incremented SEQ:
`AA 55 01 00 F0 80 00 00 04 00 00 ...`

### 2. Error Response

```c
void handle_client_command(const ProtoFrame *in, ProtoFrame *out) {
    // Process request...
    if (request_is_invalid) {
        // Send error response
        out->dev = in->dev;
        out->op = in->op;
        out->seq = in->seq;
        out->flags = CDC_FLAG_ACK_ERR;
        out->len = 1;
        out->payload[0] = CDC_ERR_BAD_DATA; // Custom error code
    } else {
        // Send success response
        out->dev = in->dev;
        out->op = in->op;
        out->seq = in->seq;
        out->flags = CDC_FLAG_ACK_OK;
        out->len = 4;
        memcpy(out->payload, &response_data, 4);
    }
    return 0; // Reply sent
}
```

### 3. Stream Processing

```c
void handle_stream_data(const ProtoFrame *in, ProtoFrame *out) {
    if (in->flags & CDC_FLAG_IS_STREAM) {
        // Process streaming data immediately
        process_stream_data(in->payload, in->len);
        // Don't send reply for streams
        return 1; // Suppress reply
    }
    return 0;
}
```

### 4. Device Registration

Device handlers are registered in `cdc_proto.c`:

```c
static const DevTableEntry kDevTable[] = {
    // System handler
    { CDC_DEV_SYSTEM, system_handler, system_safe_state },
    // Module handlers
    { 0x10, dc_motor_handler, dc_motor_safe_state },
    { 0x20, step_motor_handler, step_motor_safe_state },
    // ... more modules
    { 0, NULL, NULL }  // sentinel
};
```

And corresponding safe state handlers:

```c
static const SafeStateFn kSafeTable[] = {
    system_safe_state,
    dc_motor_safe_state,
    step_motor_safe_state,
    // ... more modules
};
```

## Thread Safety and ISR Compatibility

- **RX Stream**: USB receive ISR → `xStreamBufferSendFromISR()` → CommsTask
- **TX Sender**: CommsTask → `xMessageBufferReceive()` → TxSenderTask with `CDC_Transmit_FS()`
- **Non-blocking**: All operations are non-blocking with proper retry logic
- **Atomic operations**: Sequence tracking and statistics updates are atomic

## Safety Protocol

The library implements comprehensive safety measures:

1. **Heartbeat Monitoring**: PING responses every 100ms reset `rx_silence_ms`
2. **Silence Timeout**: After 1s of zero valid frames, triggers `safe_state_enter()`
3. **Sequence Tracking**: Prevents out-of-order frame processing
4. **Error Recovery**: Automatic resynchronization on invalid bytes
5. **Watchdog Pet**: IWDG pet every 100ms inside CommsTask
6. **TX Guard**: Monitor stuck transmissions and force CDC IN re-arm after 100ms

## Statistics Monitoring

Track protocol performance via `cdc_proto_stats()`:

- `rx_frames_ok`: Valid frames received
- `rx_crc_err`: Frames with CRC errors
- `rx_overrun`: Stream buffer overflow (buffer full)
- `tx_dropped`: Failed transmission attempts
- `tx_busy`: Retries due to USBD_BUSY
- `unknown_dev`: Commands from unregistered devices
- `seq_err`: Sequence mismatches
- `rx_silence_ms`: Current silence duration (updated every 10ms)

## Integration Instructions

### Required Code Changes

1. **USB CDC Interface** (`USB_DEVICE/App/usbd_cdc_if.c`, USER CODE BEGIN 6):

```c
extern StreamBufferHandle_t s_rxStream;
BaseType_t hp = pdFALSE;
if (s_rxStream != NULL) {
    xStreamBufferSendFromISR(s_rxStream, Buf, *Len, &hp);
}
USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
USBD_CDC_ReceivePacket(&hUsbDeviceFS);
portYIELD_FROM_ISR(hp);
```

2. **FreeRTOS Initialization** (`Core/Src/freertos.c`, USER CODE BEGIN Application):

```c
extern void cdc_proto_init(void);
cdc_proto_init();
```

3. **Build System** (`cmake/stm32cubemx/CMakeLists.txt`):

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/cdc_proto.c
)
```

### CubeMX Reconfiguration (Phase 4)

Update `MM_control.ioc` and regenerate for:

- **A1**: Enable IWDG (500ms counter, 64 prescaler)
- **D1**: Set `configTOTAL_HEAP_SIZE = 24576` (from 15360)
- **D2**: Enable `configUSE_MALLOC_FAILED_HOOK = 1`
- **D3**: Enable `configCHECK_FOR_STACK_OVERFLOW = 2`

## Build and Verification

### Build Commands

```bash
# Debug build
cmake --preset Debug
cmake --build --preset Debug

# Release build
cmake --preset Release
cmake --build --preset Release
```

### Smoke Test

1. **Build**: `cmake --preset Debug && cmake --build --preset Debug`
2. **Verify**: No warnings from `cdc_proto.c` (except harmless `-Wunused-parameter`)
3. **Run**: Flash and run on target
4. **Test**: Use serial tool to send:

   - **PING**: `AA 55 01 00 F0 80 00 00 00 00 00` → expect echoed PING with ACK_OK
   - **Bad CRC**: Frame with invalid CRC → expect `rx_crc_err` increment
   - **Bad LEN**: Frame with LEN=201 → expect resync to next 0xAA, no crash

## Device-Specific Implementation

Phase 2 provides empty handlers for seven device types:

- `dc_motor.h/.c` - DC motor driver (4 channels, TIM3 + TIM4)
- `step_motor.h/.c` - Stepper motor driver (TIM1 + GPIOs)
- `sts3215.h/.c` - STS3215 servo controller (USART1 @ 1MHz)
- `bus_spi.h/.c` - SPI bus driver (SPI1 + SPI3)
- `bus_uart.h/.c` - UART bus driver (USART2)
- `bus_i2c.h/.c` - I2C bus driver (I2C1)
- `bus_can.h/.c` - CAN bus driver (CAN1 @ 875kbps)

Each implements:
- Device handler (main processing)
- Safe state handler (hardware shutdown)

## Future Enhancements (Phase 3)

1. **Complete Device Drivers**: Implement actual peripheral drivers
2. **Host Protocol Spec**: Define wire format and opcodes
3. **Telemetry**: `0x03 STREAM` subscription for push notifications
4. **DMA Integration**: Optimize SPI/I2C/UART/USART transfers
5. **Configuration System**: Host-configurable device parameters
6. **Advanced Error Handling**: Nonce validation, authentication

## License

Copyright (c) 2026 STMicroelectronics.
All rights reserved.

See LICENSE file for license terms.