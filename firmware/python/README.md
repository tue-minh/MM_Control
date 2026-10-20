# TUE USB Binary Protocol — Python Driver & PyQt6 GUI

Host client library and graphical control suite for the **STM32F407VET6** (`MM_Control`) motor controller over **USB CDC (Virtual COM Port)**.

- **Envelope Overhead**: Fixed 12 bytes per packet (`0xAA 0x55 ... 0x55 0xAA`)
- **Payload**: CAN 2.0B compatible `0..8` bytes
- **Integrity**: SAE J1850 CRC-8 (Poly `0x1D`, Init `0xFF`, XorOut `0xFF`)
- **CAN ID Constraint**: `0x3...` is STRICTLY RESERVED for user custom communication.

---

## 1. Quick Start

### 1.1 Installation
```bash
pip install -r requirements.txt
```
Or install in editable mode:
```bash
pip install -e .
```

### 1.2 Running the PyQt6 GUI
Launch the graphical interface directly:
```bash
python run_gui.py
```
Or via the package command:
```bash
python -m gui.app
```

The GUI is intentionally plain and non-colorful. It has one tab per subsystem
(**System**, **Stepper (CoreXY + Z)**, **Raw Stepper**, **Servo (STS3215)**);
each tab lists **every** command of that subsystem as its own row (CAN ID +
name + button). Each tab has its own **Feedback** box pinned below the command
list showing the human-readable *response* of each command (not the raw/parsed
frame) — e.g. `0x100 SYS_PING -> echo_token=0x12345678`.

Ticking **Mock mode** in the connection bar runs the whole UI against the
built-in mock engine, so you can exercise every command without STM32 hardware.

---

## 2. Python API Usage

```python
from tue_usb import TueUsbClient, LedState

# Connect to hardware Virtual COM Port (or mock=True)
with TueUsbClient(port="COM7", baudrate=115200) as dev:
    # 1. System Management
    info = dev.get_info()
    print(f"Firmware: {info.version_string}, HW: {info.hw_id}")
    dev.set_led(0, LedState.TOGGLE)

    # 2. Stepper Kinematics
    dev.step_enable(True)
    dev.step_setpos_xy(0.0, 0.0)
    dev.step_move_xy(50.0, 30.0)
    dev.step_move_z(10.0, feed_mm_s=20.0, accel_mm_s2=200.0)
    dev.step_wait_idle(5000)

    # 3. STS3215 Bus Servos
    dev.servo_torque_enable(1, True)
    dev.servo_set_pos(1, pos=2048, speed=800, accel=30)
    
    # Dual-frame feedback (Dynamics 0x410 + Health 0x411)
    fb = dev.servo_get_feedback(1)
    print(f"Angle: {fb.dynamics.angle_deg:.1f}°, Temp: {fb.health.temperature_c}°C")
```

---

## 3. Package File Structure

- `tue_usb/crc.py`: Fast lookup table SAE J1850 CRC-8 engine & self-test
- `tue_usb/defs.py`: Framing constants, CAN IDs, ErrorCodes, ServoModes, and reserved ID guard
- `tue_usb/frame.py`: Core `Frame` dataclass, packing, unpacking, and validation
- `tue_usb/protocol.py`: Streaming `FrameParser` state-machine & packet builder
- `tue_usb/payloads.py`: Typed payload encoders/decoders for SYS, STEP, and SERVO
- `tue_usb/client.py`: Thread-safe `TueUsbClient` with synchronous transactions & mock engine
- `gui/app.py`: Simple PyQt6 GUI — one tab per subsystem listing every command, each tab with its own feedback box
- `run_gui.py`: One-click runner for the GUI application
- `examples/`: Ready-to-run demo scripts for System, Steppers, and Servos
- `tests/`: Automated unit test suite with 100% protocol and codec coverage

---

## 4. Running Unit Tests

```bash
python -m unittest discover -s tests -v
```
All unit tests validate SAE J1850 CRC-8 against `"123456789"` -> `0x4B`, verify CAN ID `0x3...` reservation checks, packet fuzzing resynchronization, payload codecs, and client transactions.
