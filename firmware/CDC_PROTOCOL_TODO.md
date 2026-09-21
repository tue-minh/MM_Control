# CDC Protocol TODO

Tracks every action item from the final plan for the USB CDC binary protocol
library. Group by phase. Each item is a checkbox; mark it done when verified.

---

## Phase 1 — Core library (no host interaction yet)

### Files to create
- [ ] `Core/Inc/cdc_proto.h` — public API, frame layout, `ProtoFrame` struct,
      `CDC_PROTO_MAX_PAYLOAD` (= 200), `CDC_PROTO_MAX_FRAME` (= 211),
      `cdc_proto_init`, `cdc_send`, `cdc_send_ack`, `cdc_encode`, `cdc_decode`,
      `cdc_proto_stats`.
- [ ] `Core/Src/cdc_proto.c` — everything in one file:
      - `static` RX/TX handles (`s_rxStream`, `s_rxQueue`, `s_txMsgBuf`,
        `commsHandle`, `txSenderHandle`).
      - CRC16-CCITT table (256-entry, poly 0x1021, init 0xFFFF).
      - `cdc_encode` / `cdc_decode`.
      - RX state machine: `WAIT_SOF0 -> SOF1 -> VER -> LEN -> HEADER[6B] ->
        PAYLOAD[LEN] -> CRC[2B] -> DISPATCH`; resync to `0xAA` on any
        invalid byte; cap `LEN` to 200; drop on CRC fail.
      - `CommsTask` (1024 words, `osPriorityAboveNormal`) drains the stream
        buffer, parses, dispatches, blocks on `osMessageQueueGet`.
      - `TxSenderTask` (512 words, `osPriorityNormal`) — non-blocking
        `xMessageBufferReceive`, retries `CDC_Transmit_FS` up to 5× on
        `USBD_BUSY` with `osDelay(1)`, then drops.
      - `kDevTable[]` (empty this phase) + inline `OP=0x80 PING` handler.
      - `safe_state_enter` / `safe_state_exit` stubs (no devices yet).

### Files to edit (CubeMX `USER CODE` blocks only)
- [ ] `USB_DEVICE/App/usbd_cdc_if.c` `USER CODE BEGIN 6` — push received
      bytes via `xStreamBufferSendFromISR(s_rxStream, Buf, *Len, &hp)` and
      `portYIELD_FROM_ISR(hp)` before the existing re-arm. Add
      `extern StreamBufferHandle_t s_rxStream;` at file scope inside the
      same `USER CODE` block.
- [ ] `Core/Src/freertos.c` `USER CODE BEGIN Application` — call
      `extern void cdc_proto_init(void); cdc_proto_init();` once.

### Build system
- [ ] `cmake/stm32cubemx/CMakeLists.txt` — append
      `${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/cdc_proto.c` to
      `MX_Application_Src`. `Core/Inc` is already in `MX_Include_Dirs`.

### Safety protocol (in `cdc_proto.c`)
- [ ] Heartbeat `PING` handler (inline in dispatcher) — echo payload, set
      `ACK_OK`, reset `rx_silence_ms`.
- [ ] Boot banner — `cdc_proto_init` sends one `DEV=0xF0 OP=0x80`
      payload `[0x4D,0x4D,0x43,0x01,0x00,0x00,0x00,0x00]`.
- [ ] Sequence tracking per `DEV` — `last_seq[16]`, `ERR 0x04 seq_mismatch`
      on non-monotonic.
- [ ] RX silence counter — incremented every 1 ms tick inside `CommsTask`;
      triggers `safe_state_enter` after 1000 ms of zero valid frames.
- [ ] IWDG pet — every 100 ms inside `CommsTask`. Skip if IWDG not enabled.
- [ ] TX stuck guard — `xTaskGetTickCount` snapshot on enqueue; if head
      hasn't progressed in 100 ms, force re-arm of CDC IN.
- [ ] RX queue drop-oldest policy when `s_rxQueue` is full.
- [ ] `cdc_proto_stats` — counters: `rx_frames_ok`, `rx_crc_err`,
      `rx_overrun`, `tx_dropped`, `tx_busy`, `unknown_dev`, `seq_err`,
      `rx_silence_ms`.

### Smoke test (host-side, any serial tool)
- [ ] Send `AA 55 01 00 F0 80 00 00 00 00 00` (10-byte PING, LEN=0).
      Expect `AA 55 01 00 F0 80 00 00 04 00 00 ...` echoed (PING with
      `ACK_OK` and matching SEQ).
- [ ] Send a frame with bad CRC, expect silence + `rx_crc_err` to increment.
- [ ] Send a frame with `LEN=201`, expect resync to next `0xAA`, no crash.
- [ ] Reset the host mid-stream; verify the device re-synchronizes within
      one valid frame and re-emits the boot banner on the next host
      connection.

### Build & verify
- [ ] `cmake --preset Debug` and `cmake --build --preset Debug` from
      `firmware/` succeed.
- [ ] `cmake --preset Release` and `cmake --build --preset Release` succeed.
- [ ] No warnings from `cdc_proto.c` (other than the harmless
      `-Wunused-parameter` from CubeMX-generated files).

---

## Phase 2 — Module stubs (dev table filler)

No device logic yet, just empty handlers so the dispatcher compiles cleanly
and the safety handshake has something to call on silence.

- [ ] `dc_motor.h` / `dc_motor.c` — `dc_motor_handle(const ProtoFrame *in,
      ProtoFrame *out)` stub. Implements `safe_state_enter` only (no-op or
      log).
- [ ] `step_motor.h` / `step_motor.c` — same shape, stub only.
- [ ] `sts3215.h` / `sts3215.c` — same shape, stub only.
- [ ] `bus_spi.h` / `bus_spi.c` — same shape, stub only.
- [ ] `bus_uart.h` / `bus_uart.c` — same shape, stub only.
- [ ] `bus_i2c.h` / `bus_i2c.c` — same shape, stub only.
- [ ] `bus_can.h` / `bus_can.c` — same shape, stub only.
- [ ] Populate `kDevTable[]` in `cdc_proto.c` with the seven handlers.
- [ ] Register each module's `safe_state_enter` in a
      `static const SafeStateFn kSafeTable[]` parallel to `kDevTable[]`.
- [ ] Rebuild + re-run smoke test, this time with
      `DEV=0x10 OP=0x01 FLAGS=0x01` (SET, REPLY_REQ) — expect
      `ERR 0x05 not_implemented` (new code), and `safe_state_enter` should
      fire after 1 s of silence.

---

## Phase 3 — Real device drivers

### DC motor (4 channels, TIM3 + TIM4 + DIR GPIOs)
- [ ] `dc_motor_set_dir(ch, dir)` — `HAL_GPIO_WritePin(STEP_DIRx_*)`.
- [ ] `dc_motor_set_speed(ch, duty_q15)` —
      `__HAL_TIM_SET_COMPARE(&htim3|htim4, channel, value)`.
- [ ] `dc_motor_brake(ch)`, `dc_motor_coast(ch)`.
- [ ] `dc_motor_safe_state` — set all DIRs, CCR=0, leave bridges disabled.
- [ ] Protocol payloads — TBD once host-side schema is agreed.

### Stepper (4 channels, TIM1 STEP pulse + DIR + EN)
- [ ] Add `TIM1_UP_TIM10_IRQHandler` to `stm32f4xx_it.c` `USER CODE BEGIN 1`
      → `HAL_TIM_IRQHandler(&htim1)` + per-channel counter decrement.
- [ ] Set NVIC priority for TIM1 break/update to 5 in
      `MM_control.ioc` (regenerate).
- [ ] `step_motor_move(ch, steps, dir, period_us)` — `HAL_TIM_PWM_Start_IT`
      with one-shot config, then assert `STEP_ENABLE` low.
- [ ] `step_motor_stop_all` for safe state.

### STS3215 (USART1 half-duplex, 1 Mbaud)
- [ ] Change USART1 baud to 1 000 000 in `MM_control.ioc` and regenerate.
- [ ] `sts_write_reg(id, reg, val)` — half-duplex packet with checksum.
- [ ] `sts_read_reg(id, reg, &val)` — TX then RX with 1 ms turnaround.
- [ ] `sts_safe_state` — no-op (servo holds position; power-down is
      application-level).

### SPI bus (SPI1 + SPI3, software NSS)
- [ ] `spi_xfer(bus, cs_index, tx[], rx[], len)` — manual CS via pin
      labels in `Core/Inc/main.h`.
- [ ] `spi_xfer_chain(segments[], n)` for multi-slave transactions.
- [ ] `spi_safe_state` — set all CS lines high.

### UART bus (USART2)
- [ ] `uart2_write(buf, len)`, `uart2_read(buf, len, timeout_ms)` — IT
      backed, ring buffer in module.

### I2C bus (I2C1, 100/400 kHz)
- [ ] `i2c1_read8`, `i2c1_write8`, `i2c1_mem_read16/write16`.
- [ ] `i2c1_scan` for bus health.

### CAN bus (CAN1, 875 kbit/s)
- [ ] Add `CAN1_RX0_IRQHandler` to `stm32f4xx_it.c` `USER CODE BEGIN 1`.
- [ ] `HAL_CAN_Start(&hcan1)` + `ActivateNotification(... RX_FIFO0 ...
      ERROR ... BUSOFF)`.
- [ ] `can_send(id, dlc, data)`, `can_register_rx_callback(cb)`.
- [ ] Default filter: accept-all, or configurable mask via protocol.
- [ ] `can_safe_state` — `HAL_CAN_Stop(&hcan1)`.

---

## Phase 4 — CubeMX reconfiguration list

Each item here is a no-code `.ioc` toggle, applied via STM32CubeMX
(recommended) or by hand-editing the ioc text and regenerating.

### A. Watchdog & reset
- [ ] A1. Enable **IWDG** (Timers → IWDG, counter 500 ms, prescaler 64).
- [ ] A2. (Optional) Enable **WWDG** (50 ms).
- [ ] A3. Note: `Error_Handler` spin → IWDG will reset. No `.ioc` toggle
      needed; rely on IWDG.

### B. USB / VCP hardening
- [ ] B1. Skip — no VBUS sense on this board's pinout.
- [ ] B6. **USART1 baud = 1 000 000** for STS3215.

### C. DMA
- [ ] C1. (When ready) Add DMA streams for SPI1 RX/TX, SPI3 RX/TX, I2C1
      RX/TX, USART1 RX/TX, USART2 RX/TX.
- [ ] C2. Set all new DMA-stream IRQs to priority 5 in NVIC view.

### D. FreeRTOS kernel
- [ ] D1. **`configTOTAL_HEAP_SIZE = 24576`** (from 15360) — required if
      Phase 1 uses dynamic allocation and the 4 KB message buffer is kept.
- [ ] D2. **`configUSE_MALLOC_FAILED_HOOK = 1`**.
- [ ] D3. **`configCHECK_FOR_STACK_OVERFLOW = 2`**.
- [ ] D4. Leave `configSUPPORT_DYNAMIC_ALLOCATION = 1` (default).

### E. GPIO / hardware defaults
- [ ] E1. Verify motor `STEP_ENABLE` polarity on schematic. CubeMX view
      of PC5: GPIO output level. Default reset is brake; flip to set if
      bridge is active-high enable.
- [ ] E2. Verify SPI CS idle-high (default). Flip if any slave is
      active-high.
- [ ] E3. **I2C1 PB6/PB7 → pull-up** in `.ioc` (currently NOPULL; no
      external pull-ups assumed).

### F. Clock & power
- (no changes)

### G. NVIC priorities
- [ ] G1. **OTG_FS_IRQn = 5** (already set).
- [ ] G2. When TIM1 IRQ is added, set to **5**.
- [ ] G3. When USART1 IRQ is added, set to **5**.
- [ ] G4. When CAN1 RX0 IRQ is added, set to **5**.

### H. Not via .ioc (code only — flag for review)
- [ ] H1. `cdc_proto_init` call in `freertos.c` `USER CODE Application`.
- [ ] H2. IWDG pet in `CommsTask`.
- [ ] H3. Protocol implementation in `cdc_proto.c`.

---

## Open decisions still pending

- [ ] **Q1 (resolved)**: dynamic init + RT-safe hot path; `FromISR` in
      ISRs, non-blocking sends on producers.
- [ ] **Q2 (resolved)**: bump heap to 24576.
- [ ] **Q3 (resolved)**: IWDG pet from `CommsTask` only.
- [ ] **Q4 (resolved)**: module handler time-cap via contract — handler
      authors must call `cdc_check_periodic()` at most every 5 ms.
- [ ] **Q5 (resolved)**: 100 ms heartbeat / 1000 ms silence.
- [ ] **Q6 (resolved)**: see Phase 4 checkboxes above.
- [ ] **Q7 (resolved)**: USART1 → 1 000 000 in `.ioc`.
- [ ] **Q8 (resolved)**: keep `APP_TX_DATA_SIZE=2048`, message buffer
      4096.

## Open decisions still open (host-side, not blocking firmware)

- [ ] Wire format agreed; **host protocol spec document** to be written
      alongside. Owner: PC-side dev.
- [ ] Module opcodes (0x10.0x01..0xFF) and payload schemas for each
      DEV class.
- [ ] Telemetry opcode `0x03 STREAM` — subscription bitmap (which DEVs
      to push, what rate).

---

## How to use this file

1. Tick an item only after the action is **done and verified** (e.g. the
   build succeeded, or the device replies to the smoke-test command).
2. Re-run the smoke test from Phase 1 after every CubeMX regen.
3. After any commit that touches `cdc_proto.c`, also re-run
   `cmake --build --preset Release` to catch warnings.
