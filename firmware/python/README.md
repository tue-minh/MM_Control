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

## Onboard LED CLI Tool & Client Commands (`led_control.py` / `TueClient`)

The table below lists all commands and methods provided in `TueClient` ([python/tue_usb/client.py](file:///D:/Project_Kicad/MM_Control/firmware/python/tue_usb/client.py)) along with their corresponding CLI options in `examples/led_control.py`:

| Client Method (`TueClient`) | Command ID | CLI Option | Description | CLI Example |
|:---|:---|:---|:---|:---|
| `list_ports()` | — | `--scan` | Scan and list available serial COM ports | `python examples/led_control.py --scan` |
| `set_led(led_index, True)` | `CMD_SYS_LED_CTRL` (`0x04`) | `--on` | Turn onboard LED ON | `python examples/led_control.py --port COM3 --on` |
| `set_led(led_index, False)` | `CMD_SYS_LED_CTRL` (`0x04`) | `--off` | Turn onboard LED OFF | `python examples/led_control.py --port COM3 --off` |
| `toggle_led(led_index)` | `CMD_SYS_LED_CTRL` (`0x04`) | `--toggle` | Toggle onboard LED state | `python examples/led_control.py --port COM3 --toggle` |
| `get_led(led_index)` | `CMD_SYS_LED_CTRL` (`0x04`) | `--query` | Query current LED state (0 = OFF, 1 = ON) | `python examples/led_control.py --port COM3 --query` |
| `set_led(...)` | `CMD_SYS_LED_CTRL` (`0x04`) | `--blink <N>` | Blink LED N times (`--interval <sec>`) | `python examples/led_control.py --port COM3 --blink 5 --interval 0.2` |
| `ping(data)` | `CMD_SYS_PING` (`0x01`) | `--ping` | Send ping echo request to MCU | `python examples/led_control.py --port COM3 --ping` |
| `get_info()` | `CMD_SYS_GET_INFO` (`0x02`) | `--info` | Query board signature, version & MCU UID | `python examples/led_control.py --port COM3 --info` |
| `send_command(...)` | Any `CommandId` | — | Low-level synchronous command transmission | *(Python API)* |
| — | Multiple | `--interactive`, `-i` | Launch interactive console session | `python examples/led_control.py --port COM3 --interactive` |

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
