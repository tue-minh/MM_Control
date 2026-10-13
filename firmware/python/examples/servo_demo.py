"""Minimal ACSI demo: talk to a Feetech STS3215 over the board's USB CDC port.

Usage:
    python examples/servo_demo.py COM7 1

The script only reads/observes by default. Pass ``--move`` to actually command
motion (the servo will move - make sure the mechanism is clear).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tue_acsi import AcsiClient


def main() -> int:
    parser = argparse.ArgumentParser(description="TUE ACSI servo demo")
    parser.add_argument("port", help="USB CDC port, e.g. COM7 or /dev/ttyACM0")
    parser.add_argument("servo_id", nargs="?", type=int, default=1)
    parser.add_argument("--move", action="store_true", help="command a small motion")
    args = parser.parse_args()

    with AcsiClient(args.port) as dev:
        info = dev.sys_info()                     # text decoded from numbers
        uid = dev.sys_uid()
        print(f"board={info['board']} fw={info['fw']} uid={uid}")
        print(f"uptime={dev.sys_uptime()} ms")

        dev.sys_set_led(True, 0)
        print(f"servo {args.servo_id} ping -> {dev.servo_ping(args.servo_id)}")

        fb = dev.servo_feedback(args.servo_id)
        print(
            "feedback: "
            f"pos={fb['pos']} spd={fb['spd']} load={fb['load']} "
            f"volt={fb['volt']} temp={fb['temp']} status={fb['status']}"
        )

        if args.move:
            target = fb["pos"] + 200
            dev.servo_set_torque(True, args.servo_id)
            dev.servo_set_pos(target, args.servo_id, speed=800, accel=30)
            print(f"commanded position {target}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
