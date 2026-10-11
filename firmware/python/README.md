# TUE USB Python Client Library

Reusable Python client library and tools for the **STM32F407VET6** motion-control board (`MM_Control`) over USB CDC.

## Features

- **Decoupled Architecture**: Framing and serialization (`tue_usb.protocol`) are independent of serial transport, allowing reuse in ROS 2 nodes, simulation harnesses, or UI dashboards.
- **Robust Framing**: Preamble sync markers (`0xAA 0x55`), CRC-16/CCITT-FALSE verification, sequence counter matching, and timeout handling.
- **Stream Resilient**: Reassembles fragmented USB packets and unpacks concatenated frames seamlessly.
- **Thread-safe**: Protected client interface with auto-incrementing sequence numbers.
- **Complete Test Suite**: Unit tests verifying CRC calculations, framing, and fragmentation recovery.

---

## Installation

### Prerequisites
- Python 3.8 or higher.
- `pip` package manager.

### Install Dependencies
From the `firmware/python/` directory:

```bash
pip install -r requirements.txt
```

Or install the package in editable development mode:

```bash
pip install -e .
```

---

## Quick Start: Python API

```python
from tue_usb import TueClient, LedMode

# Connect using a context manager (auto-closes port)
with TueClient(port="COM3") as client:
    # 1. Ping the controller
    echo = client.ping(b"HELLO_STM32")
    print(f"Ping response: {echo}")

    # 2. Query board info and STM32 Unique ID
    info = client.get_info()
    print(f"Board: {info['board_id']}, FW: {info['version_str']}, UID: {info['mcu_uid']}")

    # 3. Control the onboard LED (PE5)
    client.set_led(led_index=0, state=True)   # Turn ON
    client.set_led(led_index=0, state=False)  # Turn OFF
    client.toggle_led(led_index=0)            # Toggle

    # 4. Query current LED state without changing it
    is_on = client.get_led(led_index=0)
    print(f"LED is currently: {'ON' if is_on else 'OFF'}")
```

---

## Onboard LED CLI Tool (`led_control.py`)

The example script `firmware/python/examples/led_control.py` provides command-line control of the onboard LED:

### Scan Serial Ports
```bash
python examples/led_control.py --scan
```

### Turn LED ON
```bash
python examples/led_control.py --port COM3 --on
```

### Turn LED OFF
```bash
python examples/led_control.py --port COM3 --off
```

### Toggle LED
```bash
python examples/led_control.py --port COM3 --toggle
```

### Query LED State
```bash
python examples/led_control.py --port COM3 --query
```

### Blink LED 5 Times
```bash
python examples/led_control.py --port COM3 --blink 5 --interval 0.2
```

### Query Device Info & MCU UID
```bash
python examples/led_control.py --port COM3 --info
```

### Interactive Console Mode
```bash
python examples/led_control.py --port COM3 --interactive
```

---

## Running Unit Tests

Run the test suite from `firmware/python/`:

```bash
python -m unittest discover -s tests -v
```

Or using `pytest`:

```bash
pytest tests -v
```
