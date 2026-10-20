#!/usr/bin/env python3
"""
Example: Query STM32 system info, UID, uptime, and toggle on-board LED.
"""

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tue_usb import TueUsbClient, LedState

def main():
    # Use mock=False and port="COM3" (or /dev/ttyACM0) for physical STM32
    port = sys.argv[1] if len(sys.argv) > 1 else "MOCK_PORT"
    use_mock = (port == "MOCK_PORT")

    print(f"Connecting to {port} (mock={use_mock})...")
    with TueUsbClient(port=port, mock=use_mock) as client:
        # Ping
        token = 0xDEADBEEF
        echo_tok = client.ping(token)
        print(f"[PING] Sent 0x{token:08X}, received echo 0x{echo_tok:08X}")

        # Info
        info = client.get_info()
        print(f"[INFO] Hardware: ID={info.hw_id}, Rev={info.hw_rev}")
        print(f"[INFO] Firmware: {info.version_string}, Build Flags: 0x{info.build_flags:08X}")

        # UID
        uid = client.get_uid()
        print(f"[UID]  96-bit Unique ID: {uid.hex_string}")

        # Uptime
        uptime = client.get_uptime_ms()
        print(f"[TIME] MCU Uptime: {uptime} ms ({uptime / 1000.0:.2f} s)")

        # LED Toggle
        state = client.set_led(0, LedState.TOGGLE)
        print(f"[LED]  Toggled LED state -> {state}")

if __name__ == "__main__":
    main()
