# USB ↔ UART Haptic Bridge — Implementation Report

> **Source specification**: [`USB_STM32_UPDATE.md`](USB_STM32_UPDATE.md)
> **Target**: STM32F407VET6 (`MM_Control`), FreeRTOS + STM32 HAL
> **Scope**: USB CDC (host) → USART2 (ESP32 haptic slave) transparent forwarding
> **Status**: Firmware and host library implemented, builds clean, host tests pass.
> **Hardware verification**: not performed (no device was flashed/actuated).

---

## 1. What was implemented

The firmware now ingests the unified **20-byte tactile frame (CAN ID `0x300`,
DLC = 8)** from USB CDC, caches its 8-byte payload locally, and forwards the
**exact raw 20 bytes** to the ESP32 slave on **USART2 @ 250 kbaud 8-N-1** using
TX DMA. The USB side is fire-and-forget (no ACK), as required for 100 Hz
streaming. RX from the slave is captured through a circular DMA buffer with
IDLE-line termination.

```
Host PC ──USB CDC──► STM32F407 ──USART2 DMA──► ESP32 (5x MCP4725 + valves)
                       │
                       └─ local payload cache (g_tue_haptic_payload + timestamp)
```

---

## 2. Files added

### `Core/Inc/tue_haptic_bridge.h`
Task 1 deliverable — constants, packed structures, and API:

- `TUE_HAPTIC_CAN_ID = 0x00000300`
- Frame constants: `SOF1/2`, `EOF1/2`, `FRAME_TYPE_REQ`, 20-byte length
- Status bitmask: `LEFT_CONTACT`, `RIGHT_CONTACT`, `SLIP_ALERT`, `TARE_ACTIVE`
- `#pragma pack(push, 1)` structures:
  - `tue_haptic_payload_t` — 8 bytes (`fz_left`, `fz_right`, `f_shear_avg`,
    `weight_tip`, `weight_mid`, `weight_base`, `status_flags`, `frame_counter`)
  - `tue_haptic_wire_frame_t` — full 20-byte wire frame
- Compile-time size checks (`_Static_assert`) for 8-byte payload / 20-byte frame
- Exposed local state:
  - `g_tue_haptic_payload` (volatile), `g_tue_haptic_timestamp_ms` (volatile)
  - counters: frames, TX drops, slip alerts, RX bytes, RX errors
- API: `tue_haptic_init`, `tue_haptic_ingest`, `tue_haptic_forward`,
  `tue_haptic_reset_stats`, `tue_haptic_rx_available`, `tue_haptic_rx_pop`
- USART2 handle mapping: `TUE_HAPTIC_UART`, `TUE_HAPTIC_UART_TX_DMA`,
  `TUE_HAPTIC_UART_RX_DMA`

### `Core/Src/tue_haptic_bridge.c`
Tasks 2 & 4 deliverable — the non-blocking forwarding engine:

- **Ping-pong TX buffer** (`2 × 20` bytes in `.bss`) with `s_tx_busy` /
  `s_tx_pending`; newest pending frame replaces the queued one. The DMA source
  is always persistent static RAM, never a stack/USB FIFO pointer.
- **Zero blocking**: no `HAL_UART_Transmit()`; only `HAL_UART_Transmit_DMA()`.
- **ISR-safe handoff**: the short slot check/copy/start sequence masks
  interrupts so a TX-complete ISR cannot start a slot that is still being
  filled.
- `HAL_UART_TxCpltCallback` routing clears busy and immediately launches the
  pending frame.
- **Circular RX DMA**: `HAL_UARTEx_ReceiveToIdle_DMA()` with a 128-byte buffer;
  the half-transfer interrupt is disabled after arming. The IDLE/TC callback
  converts the HAL-reported DMA position into a delta and pushes it into a
  128-byte byte ring.
- **Frame validation** in `tue_haptic_ingest()`: SOF1/SOF2, EOF1/EOF2,
  `FRAME_TYPE == REQ`, `CAN_ID == 0x300`, `DLC == 8`, and a second SAE J1850
  CRC-8 check over bytes `2..16` before forwarding.
- Error callback clears ORE/NE/FE/PE and re-arms RX if HAL aborted reception.

---

## 3. Files modified

### `Core/Src/tue_usb_binary.c`
- Included `tue_haptic_bridge.h`.
- Added a raw-frame capture buffer (`s_raw[20]`) filled by the parser
  (SOF → header → payload → CRC → EOF). This guarantees **byte-for-byte
  transparent** forwarding (no reconstruction).
- `usb_dispatch()` now receives the raw frame and handles `TUE_USB_CAN_HAPTIC_TELEMETRY`:
  - requires `dlc == 8` and 20 raw bytes,
  - calls `tue_haptic_ingest()`,
  - returns immediately **without a USB response frame**.
- `tue_usb_init()` now calls `tue_haptic_init()` after the STS3215 and stepper
  initialization.
- Reserved-range frames other than `0x300` still return `ERR_NOT_SUPPORTED`.

### `Core/Inc/tue_usb_binary.h`
- Added `TUE_USB_CAN_HAPTIC_TELEMETRY (0x300U)` documented as the user-custom
  stream allocated inside the reserved `0x3..` range.
- Updated the CAN ID map comment in the file header.
- No `_Static_assert` was added for `0x300` because it intentionally lives in
  the user-reserved range.

### `Core/Src/usart.c`
- **USART2 TX DMA changed from `DMA_CIRCULAR` to `DMA_NORMAL`**
  (one 20-byte frame per transfer).
- **USART2 RX DMA changed from `DMA_NORMAL` to `DMA_CIRCULAR`**
  (persistent circular receive buffer).
- USART2 GPIO (PD5/PD6) changed from `GPIO_NOPULL` to `GPIO_PULLUP`.
- Added comments identifying USART2 as the haptic slave link.
- USART2 remains 250000 baud, 8-N-1, no flow control.

### `Core/Inc/usart.h`
- Documented `huart2` as the dedicated haptic telemetry link.

### `Core/Src/tue_sts3215_hal.c`
- The shared weak HAL UART callbacks now route by instance:
  - USART1 → existing STS3215 HAL,
  - everything else → `tue_haptic_uart_*_cb()`.
- This adds haptic support to `HAL_UART_TxCpltCallback`,
  `HAL_UARTEx_RxEventCallback`, and `HAL_UART_ErrorCallback` without creating a
  second definition of the weak symbols.

### `Core/Src/tim.c` and `MM_control.ioc`
- TIM1 (stepper step clock, `TIM1_UP_TIM10_IRQn` and `TIM1_BRK_TIM9_IRQn`)
  priority changed from **5 → 1**, so the step generator preempts UART/DMA
  (priority 5) and keeps zero jitter, matching the specification.
- `.ioc` DMA modes updated for USART2 (RX circular, TX normal) so CubeMX
  regeneration stays consistent.

### `CMakeLists.txt`
- Added `Core/Src/tue_haptic_bridge.c` to the build.

### `docs/USB_BINARY_PROTOCOL.md`
- Marked `0x300` as the haptic telemetry stream inside the reserved range.
- Added section **5.1 Haptic telemetry stream** with the field/encoding table.
- Added verification entries for the haptic passthrough and USART2 DMA modes.

---

## 4. Host-side (Python) parity

Kept the host library consistent with the firmware change:

### `python/tue_usb/defs.py`
- Added `CanId.HAPTIC_TELEMETRY = 0x300`.

### `python/tue_usb/payloads.py`
- Added `HapticPayload` dataclass with physical units (N / 0..1 / booleans) and
  `to_bytes()` / `from_bytes()` conversion.
- Added `Encoders.haptic_telemetry()` and `Decoders.haptic_telemetry()`.

### `python/tue_usb/client.py`
- Added `TueUsbClient.send_haptic_telemetry(payload)` — fire-and-forget 0x300
  transmit that deliberately bypasses the generic reserved-ID guard and awaits
  no response.

### `python/tue_usb/__init__.py`
- Exported `HapticPayload`.

### `python/tests/test_protocol.py`
- Reserved-ID test now allows `CanId.HAPTIC_TELEMETRY` (it lives in the user
  range by design).
- Added `test_07_haptic_telemetry_roundtrip` covering payload encoding,
  decoding, and a 20-byte fire-and-forget frame on a mock client.

---

## 5. Task-by-task compliance

| Spec item | Status | Implementation |
|:--|:--:|:--|
| Task 1 — constants, packed structs, state exposure | Done | `tue_haptic_bridge.h`, `_Static_assert` size checks |
| Task 2 — UART + dual DMA driver | Done | USART2 250 kbaud, TX normal DMA, RX circular DMA, pull-ups, NVIC 5 |
| Task 3 — ISR mapping | Already present, verified | `USART2_IRQHandler`, `DMA1_Stream5_IRQHandler`, `DMA1_Stream6_IRQHandler` |
| Task 4 — ingestion + forwarding engine | Done | Raw capture, 0x300 filter, local cache, ping-pong DMA, callbacks |
| Zero-blocking rule | Done | Only `HAL_UART_Transmit_DMA()` |
| Persistent DMA source | Done | Static `s_tx_slots[2][20]` |
| Interrupt priority hierarchy | Done | Stepper TIM1 = 1; UART/DMA = 5 |
| Fire-and-forget (no USB ACK) | Done | Dispatcher returns before response generation |
| Data transparency (20 raw bytes) | Done | Parser keeps the exact wire bytes |
| Baud rate exactness | Done | 42 MHz / (16 × 250 kbaud) = 10.5 → 0% error |

### Naming / design adaptations
- The specification names `huart_slave` / `hdma_usart_slave_tx/rx`. This project
  already generates `huart2` / `hdma_usart2_tx/rx` from CubeMX, so the driver
  uses those handles through the `TUE_HAPTIC_UART*` aliases. Functionally
  identical, and CubeMX regeneration will not break the build.
- The spec contains a small contradiction for DMA modes (section 3.2 vs.
  Task 2). The implementation follows **Task 2 / the dual-DMA architecture**:
  TX = normal, RX = circular.
- The spec's acceptance table mentions PA2 for the TX pin; the actual board
  (`MM_control.ioc`) routes USART2 to **PD5/PD6**, which is what was used.

---

## 6. Verification performed

### Firmware build
```
cmake --preset Debug
cmake --build --preset Debug
```
Result: **success**.

```
Memory region   Used Size   Region Size   %age Used
RAM:             32320 B       128 KB     24.66%
CCMRAM:              0 GB       64 KB      0.00%
FLASH:           86996 B       512 KB     16.59%
```

Generated artifacts:
- `build/Debug/MM_control.elf`
- `build/Debug/MM_control.bin`

No new compiler warnings from the added/modified modules. Two pre-existing
`main.c` implicit-declaration warnings for `tue_stepper_init()` /
`tue_stepper_on_timer_elapsed()` remain untouched (unrelated to this task).

### Host tests
```
python -m unittest discover -s tests -v
```
Result: **7/7 tests passed**, including the new haptic round-trip test.

### Not performed
- Oscilloscope / logic-analyser measurements (baud accuracy, latency).
- On-target USB CDC ↔ USART2 loopback or ESP32 slave test.
- Flashing and motor/valve actuation.
- 100 Hz long-run jitter and CPU-load profiling.

---

## 7. Runtime API for application code

```c
/* Latest cached telemetry (updated from the USB protocol task). */
extern volatile tue_haptic_payload_t g_tue_haptic_payload;
extern volatile uint32_t g_tue_haptic_timestamp_ms;

/* Diagnostics */
extern volatile uint32_t g_tue_haptic_frame_count;
extern volatile uint32_t g_tue_haptic_tx_drops;
extern volatile uint32_t g_tue_haptic_slip_alerts;
extern volatile uint32_t g_tue_haptic_rx_bytes;
extern volatile uint32_t g_tue_haptic_rx_errors;

/* Drain bytes returned by the ESP32 slave. */
uint8_t b;
while (tue_haptic_rx_pop(&b)) { /* ... */ }
```

Slip-alert detection is latched in `g_tue_haptic_slip_alerts` via
`payload.status_flags & TUE_HAPTIC_STATUS_SLIP_ALERT`, ready for a future reflex
control hook.

---

## 8. Hardware verification checklist (to run next)

1. Flash `build/Debug/MM_control.bin` (VS Code task **Flash Firmware - USB DFU**).
2. Probe USART2 TX (PD5) with a logic analyser:
   - bit width ≈ 4.00 µs,
   - exactly 20 bytes per host frame,
   - content identical to the USB input (including CRC).
3. Stream 100 Hz frames from the host:
   ```python
   from tue_usb import TueUsbClient, HapticPayload
   with TueUsbClient("COM7") as dev:
       dev.send_haptic_telemetry(HapticPayload(fz_left_n=1.2, fz_right_n=0.8))
   ```
4. Confirm no USB response frame is generated for `0x300`.
5. Run a CoreXY move during streaming and check step-pulse jitter.
6. Check `g_tue_haptic_tx_drops == 0` in the debugger during sustained streaming.
