# TODO: Python Library & PyQt6 GUI for STM32 USB Binary Protocol

> **Target Platform**: STM32F407VET6 (`MM_Control`) over **USB CDC (Virtual COM Port)**  
> **Frame Architecture**: CAN-over-USB Binary Envelope (Little-Endian, 12B Framing Overhead + 0..8B DLC)  
> **Integrity Check**: SAE J1850 CRC-8 (Poly `0x1D`, Init `0xFF`, XorOut `0xFF`)  
> **Firmware Sources**: [`Core/Src/tue_usb_binary.c`](../Core/Src/tue_usb_binary.c), [`Core/Inc/tue_usb_binary.h`](../Core/Inc/tue_usb_binary.h), [`docs/USB_BINARY_PROTOCOL.md`](../docs/USB_BINARY_PROTOCOL.md)  
> **Host Package**: `tue_usb` in [`python/tue_usb`](./tue_usb)  
> **Host GUI Application**: [`python/gui/app.py`](./gui/app.py) or `python run_gui.py`  

---

## 1. Project Overview & Architecture

This checklist details the architecture, file organization, encoding/decoding specifications, command list, and PyQt6 GUI application for interfacing host machines with the STM32F407VET6 motor control firmware via the high-speed binary CAN-over-USB protocol.

### 1.1 Envelope Format
```
+------------+------------+------------+---------------+------------+------------+------------------+------------+------------+------------+
|  SOF1      |  SOF2      | FRAME_TYPE | CAN_ID (4B)   | DLC        | SEQ        | PAYLOAD (0..8B)  | CRC8       | EOF1       | EOF2       |
|  0xAA      |  0x55      | uint8_t    | uint32_t (LE) | uint8_t    | uint8_t    | 0..8 Bytes       | SAE J1850  | 0x55       | 0xAA       |
+------------+------------+------------+---------------+------------+------------+------------------+------------+------------+------------+
|<-- 2B Preamble -------->|<----------------------- 7B Header -------------->|<--- 0..8B Data ->|<- 1B CRC ->|<-- 2B Postamble ------>|
```

- **Overhead**: Fixed 12 bytes per packet (min 12 bytes, max 20 bytes).
- **CRC-8**: SAE J1850 polynomial `0x1D`, init `0xFF`, xorout `0xFF`, computed over `[FRAME_TYPE .. PAYLOAD]` (7 + DLC bytes).
- **Critical Constraint**: CAN IDs in `0x3...` (`0x300`–`0x3FF`, `0x3000`–`0x3FFF`, `0x30000000`–`0x3FFFFFFF`) are STRICTLY RESERVED for user custom communications.

---

## 2. File Organization & Package Structure

```
python/
├── TODO_PYTHON_LIBRARY.md        # This implementation plan and checklist
├── pyproject.toml                # Package metadata, dependencies, build config
├── requirements.txt              # Host dependencies (pyserial, PyQt6, etc.)
├── README.md                     # Library documentation and quickstart
├── run_gui.py                    # Direct entrypoint for launching the PyQt6 GUI
│
├── tue_usb/                      # Core Python communication library
│   ├── __init__.py               # Top-level exports (TueUsbClient, Frame, CanId, etc.)
│   ├── crc.py                   # SAE J1850 CRC-8 lookup table & self-test
│   ├── defs.py                  # Protocol Enums (CanId, FrameType, ErrorCode, ServoMode)
│   ├── frame.py                 # Low-level Frame dataclass, serialization, parsing
│   ├── protocol.py              # Streaming FrameParser state-machine & packet builder
│   ├── payloads.py              # Typed payload encoders & decoders (SYS, STEP, SERVO)
│   └── client.py                # High-level TueUsbClient with serial I/O & mock support
│
├── gui/                         # PyQt6 Control & Monitoring GUI
│   ├── __init__.py              # GUI module init
│   └── app.py                   # Main PyQt6 Application Window with:
│                                #   - Port connection & auto-scan
│                                #   - Live Packet Inspector & Command Preview
│                                #   - System Management tab
│                                #   - Stepper Motion & Position tab
│                                #   - Raw Stepper Channels tab
│                                #   - STS3215 Bus Servos & Telemetry tab
│
├── examples/                    # Script examples
│   ├── sys_info_demo.py         # Query firmware version, uptime, LED toggle
│   ├── stepper_motion_demo.py   # Multi-axis move, jog, position read
│   └── servo_feedback_demo.py   # Bus servo feedback dynamics & health monitor
│
└── tests/                       # Unit tests
    ├── __init__.py
    └── test_protocol.py         # 100% test coverage for CRC, framing, codecs, reserved IDs
```

---

## 3. Detailed Task Checklist

### Phase 1: Core Protocol & Mathematical Integrity
- [x] **Task 1.1 - CRC-8 Engine (`tue_usb/crc.py`)**:
  - Implement 256-byte MSB-first lookup table matching STM32 firmware polynomial `0x1D`.
  - Implement `crc8_sae_j1850(data: bytes) -> int`.
  - Implement `crc8_selftest() -> bool` verifying standard vector `CRC8("123456789") == 0x4B`.
- [x] **Task 1.2 - Protocol Definitions (`tue_usb/defs.py`)**:
  - `FrameType(IntEnum)`: `REQ = 0x01`, `RESP_OK = 0x02`, `RESP_ERR = 0x03`, `ASYNC_EVENT = 0x04`.
  - `ErrorCode(IntEnum)`: `OK = 0`, `ERR_CRC = 1`, `ERR_BAD_FORMAT = 2`, `ERR_UNKNOWN_CAN_ID = 3`, `ERR_BAD_ARG = 4`, `ERR_BUS_TIMEOUT = 5`, `ERR_BUS_FAULT = 6`, `ERR_BUSY = 7`, `ERR_SERVO_FAULT = 8`, `ERR_QUEUE_FULL = 9`, `ERR_TIMEOUT = 10`, `ERR_NOT_SUPPORTED = 11`, `ERR_INTERNAL = 12`.
  - `CanId(IntEnum)`: Complete CAN ID map matching `tue_usb_binary.h`:
    - System: `0x100` (`SYS_PING`) to `0x107` (`SYS_ECHO`).
    - Stepper: `0x200` (`STEP_GET_INFO`) to `0x21A` (`STEP_WAIT_IDLE`).
    - Stepper Raw: `0x280` (`STEP_RAW_GET_STEPS`) and `0x281` (`STEP_RAW_MOVE`).
    - Servos: `0x400` (`SERVO_PING`) to `0x415` (`SERVO_RESET`), plus `0x410` / `0x411` telemetry frames.
  - Guard function `is_reserved_can_id(can_id: int) -> bool` enforcing exclusion of `0x3...`.
- [x] **Task 1.3 - Frame Dataclass & Serialization (`tue_usb/frame.py`)**:
  - Define `Frame` dataclass: `frame_type: FrameType`, `can_id: int`, `dlc: int`, `seq: int`, `payload: bytes`, `crc8: int`.
  - Method `to_bytes() -> bytes` producing exact wire bytes `[0xAA, 0x55, TYPE, ID_LE(4), DLC, SEQ, PAYLOAD, CRC8, 0x55, 0xAA]`.
  - Method `from_bytes(raw: bytes) -> Frame` with strict boundary, length, and CRC validation.

### Phase 2: Frame Parsing & Protocol Stream State Machine
- [x] **Task 2.1 - Frame Parser (`tue_usb/protocol.py`)**:
  - Implement streaming byte-by-byte state machine matching STM32 firmware `usb_rx_byte()`:
    `ST_WAIT_SOF1 -> ST_WAIT_SOF2 -> ST_HEADER -> ST_PAYLOAD -> ST_CRC -> ST_EOF1 -> ST_EOF2`.
  - Buffer resilience: handles packet fragmentation, concatenation, dropped bytes, and noise resynchronization.
  - Method `feed(chunk: bytes) -> List[Frame]`.
- [x] **Task 2.2 - Packet Builders (`tue_usb/protocol.py`)**:
  - Factory helpers: `build_req_frame(can_id, payload, seq) -> bytes`.

### Phase 3: Typed Payload Encoders and Decoders (`tue_usb/payloads.py`)
- [x] **Task 3.1 - System Payloads**:
  - `SYS_PING`: Pack/unpack `u32` token.
  - `SYS_INFO`: Unpack `hw_id (u8)`, `hw_rev (u8)`, `fw_maj (u8)`, `fw_min (u8)`, `build_flags (u32)`.
  - `SYS_UID`: Unpack 96-bit STM32 unique ID across Part 1 (`uid0, uid1`) and Part 2 (`uid2`).
  - `SYS_UPTIME`: Unpack `uptime_ms (u32)`.
  - `SYS_LED_CTRL`: Pack `[led_idx, state]`, unpack `[led_idx, current_state]`.
  - `SYS_RESET`: Pack `0xDEADBEEF` magic key, unpack `ack (u8)`.
  - `SYS_ECHO`: Loopback arbitrary 1..8 bytes.
- [x] **Task 3.2 - Stepper Payloads**:
  - `STEP_GET_INFO`: Unpack `xy_spmm (u16)`, `z_spmm (u16)`, `min_rate (u16)`, `max_rate (u16)`.
  - `STEP_GET_STATUS`: Unpack `is_busy (bool)`, `is_enabled (bool)`, `queue_depth (int)`.
  - `STEP_ENABLE`: Pack `0/1/0xFF`, unpack `state (bool)`.
  - `STEP_GET_POS_XY` & `STEP_GET_POS_Z`: Unpack micrometre `int32` to millimetre `float`.
  - `STEP_SETPOS_XY` & `STEP_SETPOS_Z`: Pack millimetres into micrometres `int32`.
  - `STEP_MOVE_COORDS` + `STEP_MOVE_EXEC`: Latch X/Y micrometres, execute with Z, feed, accel.
  - `STEP_MOVE_XY` & `STEP_MOVE_Z` & `STEP_MOVE_REL_XY`: Pack coordinates/feed/accel.
  - `STEP_SET_SPEED`: Pack default feed and accel.
  - `STEP_JOG_XY` & `STEP_JOG_Z`: Pack signed 16-bit velocities.
  - `STEP_STOP` & `STEP_ABORT`: Emergency halt and controlled deceleration.
  - `STEP_RAW_GET_STEPS` & `STEP_RAW_MOVE`: Per-motor raw step pulse counts.
- [x] **Task 3.3 - STS3215 Servo Payloads**:
  - `SERVO_PING`: Ping servo ID `1..254`.
  - `SERVO_SET_POS` & `SERVO_GET_POS`: Steps `0..4095` (`0.088°/step`), speed, accel.
  - `SERVO_SET_SPEED` & `SERVO_GET_SPEED`: Speed steps/sec.
  - `SERVO_TORQUE`: Enable, disable, query, or calibrate (`128`).
  - `SERVO_TLIM`: Torque limit in 0.1% units.
  - `SERVO_MODE`: Position (0), Speed (1), PWM (2), Step (3).
  - `SERVO_PWM`: Duty cycle `-1000..1000`.
  - `SERVO_GET_FB`: Parse dual-frame telemetry:
    - Frame 1 `0x410` Dynamics: `pos (i16)`, `speed (i16)`, `load (i16, 0.1%)`, `moving (bool)`.
    - Frame 2 `0x411` Health: `voltage (0.1V)`, `temperature (°C)`, `status (fault bitmask)`, `current (mA)`.
  - `SERVO_MEM_READ` & `SERVO_MEM_WRITE`: EEPROM / SRAM register access.
  - `SERVO_SET_ID` & `SERVO_RESET`: Change ID or restore factory defaults.

### Phase 4: High-Level Client & Transport Layer (`tue_usb/client.py`)
- [x] **Task 4.1 - Serial Transport**:
  - Connection lifecycle using `pyserial`.
  - Monotonic 8-bit sequence generator (`0..255`).
  - Synchronous `transact(can_id, payload, timeout) -> Frame` with response correlation.
  - Support for dual-frame responses (`SERVO_GET_FB` -> `0x410` + `0x411`).
  - Mock mode for offline testing, verification, and GUI preview without physical board.
- [x] **Task 4.2 - High-Level Driver Methods**:
  - Wrap all payload functions into intuitive Python methods:
    `client.ping()`, `client.get_info()`, `client.step_get_pos()`, `client.step_move_xyz()`, `client.servo_get_feedback()`, etc.

### Phase 5: PyQt6 GUI Application (`gui/app.py`)
- [x] **Task 5.1 - Modern Dark/Light Themed Layout**:
  - Clean modular interface with tabs and responsive status bar.
- [x] **Task 5.2 - Serial Connection & Port Scanner**:
  - Dropdown populated with auto-detected COM ports (`pyserial.tools.list_ports`).
  - Refresh button, baud rate selector, Connect/Disconnect button.
  - Connection status indicator with real-time feedback.
  - Offline Mock Mode toggle to test GUI without hardware.
- [x] **Task 5.3 - Command Preview & Live Packet Inspector**:
  - **Command Preview**: Pick any firmware command, adjust arguments, and see the exact serialized raw hex frame + field breakdown before sending.
  - **Live Packet Log**: Real-time table displaying TX & RX frames with:
    - Timestamp
    - Direction (TX / RX / ASYNC)
    - CAN ID (Hex + Descriptive Name)
    - DLC & Sequence number
    - Raw Payload hex & decoded interpretation
    - CRC-8 status (Pass/Fail)
  - Log control buttons: Clear, Filter, Pause auto-scroll, Copy hex.
- [x] **Task 5.4 - System Management Panel**:
  - Ping with latency display.
  - Query System Info (HW ID, Rev, FW Version).
  - Query 96-bit Unique Chip ID (Part 1 + Part 2).
  - Query Uptime (formatted as hh:mm:ss.ms).
  - Status LED control (On, Off, Toggle, Query).
  - System Reset button with confirmation dialog.
  - Echo test payload generator.
- [x] **Task 5.5 - Stepper Motion & Position Panel**:
  - Status monitor (Busy flag, Enabled flag, Motion queue depth).
  - Driver Enable / Disable switch.
  - Current Position (X, Y, Z in mm and um) with Zero/Set Position buttons.
  - Absolute Moves: Move XY (mm), Move Z (mm), Coordinated XYZ move (`STEP_MOVE_COORDS` + `STEP_MOVE_EXEC`).
  - Relative Moves (dX, dY mm).
  - Speed & Acceleration settings (default feed mm/s, accel mm/s²).
  - Directional Jog Pad (X+, X-, Y+, Y-, Z+, Z-) with configurable jog speed.
  - Emergency Abort (instant halt) & Controlled Stop buttons.
  - Stepper info readout (steps/mm for XY and Z, min/max step rates).
- [x] **Task 5.6 - Raw Stepper Channels Panel**:
  - Per-motor status (M1, M2, M3).
  - Query raw step counts per motor.
  - Command direct raw step moves at set feed rate.
- [x] **Task 5.7 - STS3215 Bus Servos Panel**:
  - Servo ID selector (`1..254`).
  - Ping button with response status.
  - Torque Enable/Disable switch & Calibrate to 2048 button.
  - Position control: Slider & Spinbox (`0..4095` / `0°..360°`), Speed, Accel.
  - Read Position & Read Speed.
  - Operating Mode selector (Position, Speed, PWM, Step).
  - PWM Duty Cycle slider (`-1000..+1000`).
  - Dual-Frame Telemetry Monitor:
    - Dynamics: Position, Speed, Load (%), Moving indicator.
    - Health: Voltage (V), Temperature (°C), Current (mA).
    - Status Alarm Bitmask Decoder: Voltage, Sensor, Temp, Current, Angle, Overload alarm indicators.
  - Raw Register Read & Write tool.
  - Set New Servo ID & Factory Reset buttons.

### Phase 6: Unit Testing & Verification
- [x] **Task 6.1 - Automated Unit Tests (`tests/test_protocol.py`)**:
  - SAE J1850 CRC-8 calculation on standard test vectors.
  - Exact frame serialization to bytes.
  - Streaming parser fuzzing (concatenated frames, noisy preambles, fragmented chunks).
  - Reserved CAN ID rejection check (`0x3...`).
  - Payload encoder/decoder roundtrip tests.
  - Mock client transaction test.
