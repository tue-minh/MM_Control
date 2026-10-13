"""Minimal ACSI demo: drive the three stepper motors over the board's USB port.

Usage:
    python examples/stepper_demo.py COM7                 # read-only
    python examples/stepper_demo.py COM7 --move          # actually move

Motors 1 and 2 form a CoreXY belt pair and motor 3 is Z, so moves are given as
tool coordinates ``(x, y, z)`` in millimetres; the client converts them to the
micrometre units used on the wire. The script only observes by default - pass
``--move`` to command motion, and make sure the mechanism is clear first.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tue_acsi import AcsiClient, StepId


def main() -> int:
    parser = argparse.ArgumentParser(description="TUE ACSI stepper demo (CoreXY + Z)")
    parser.add_argument("port", help="USB CDC port, e.g. COM7 or /dev/ttyACM0")
    parser.add_argument("--move", action="store_true", help="command actual motion")
    parser.add_argument("--feed", type=float, default=30.0, help="feed rate (mm/s)")
    parser.add_argument("--accel", type=float, default=300.0, help="acceleration (mm/s^2)")
    parser.add_argument("--box", type=float, default=10.0, help="square size (mm)")
    parser.add_argument("--z", type=float, default=5.0, help="working Z height (mm)")
    args = parser.parse_args()

    with AcsiClient(args.port) as dev:
        info = dev.sys_info()
        print(f"board={info['board']} fw={info['fw']}")

        geo = dev.step_info()
        print(
            "geometry: "
            f"xy={geo['xy_steps_per_mm']} steps/mm  z={geo['z_steps_per_mm']} steps/mm  "
            f"rate {geo['min_rate']}..{geo['max_rate']} steps/s"
        )

        status = dev.step_status()
        print(f"status: busy={status['busy']} enabled={status['enabled']}")
        print(f"position (mm): {dev.step_position()}")
        print(
            "motor steps: "
            f"A={dev.step_motor_steps(StepId.M1)} "
            f"B={dev.step_motor_steps(StepId.M2)} "
            f"Z={dev.step_motor_steps(StepId.M3)}"
        )

        if not args.move:
            print("read-only run - re-run with --move to command motion")
            return 0

        # -- energise the drivers -------------------------------------------
        dev.step_enable(True)

        # -- 1) absolute move to an exact CoreXY point ----------------------
        print(f"\nmove -> (20, 20, {args.z}) mm at {args.feed} mm/s")
        dev.step_move(20.0, 20.0, args.z, feed=args.feed, accel=args.accel)
        print(f"  finished in {dev.step_wait(10000)} ms -> {dev.step_position()}")

        # -- 2) a square built from queued moves (no pause at the corners) ---
        box = args.box
        dev.step_move_xy(20.0 + box, 20.0, args.feed, args.accel)
        dev.step_move_xy(20.0 + box, 20.0 + box, args.feed, args.accel)
        dev.step_move_xy(20.0, 20.0 + box, args.feed, args.accel)
        dev.step_move_xy(20.0, 20.0, args.feed, args.accel)
        print(f"\n{box} mm square -> finished in {dev.step_wait(30000)} ms")

        # -- 3) Z up and back down ------------------------------------------
        dev.step_move_z(15.0, feed=10.0, accel=200.0)
        dev.step_wait(10000)
        dev.step_move_z(args.z, feed=10.0, accel=200.0)
        dev.step_wait(10000)
        print(f"Z cycle done -> {dev.step_position()}")

        # -- 4) velocity jog: ramp up, change speed, controlled stop ---------
        print("\njog +X at 20 mm/s")
        dev.step_jog_x(20.0, accel=500.0)
        time.sleep(0.5)
        print("  slowing to 5 mm/s (the direction is kept while a jog runs)")
        dev.step_jog_x(5.0, accel=500.0)
        time.sleep(0.5)
        dev.step_stop()                      # ramps down, drops anything queued
        print(f"  stopped in {dev.step_wait(5000)} ms -> {dev.step_position()}")

        # -- 5) raw single-motor move, bypassing the kinematics --------------
        # 3200 steps at 320 steps/mm = 10 mm of belt, i.e. A = x + y moves by
        # 10 mm, so the tool shifts by +5 mm on both X and Y.
        dev.step_raw(StepId.M1, 3200, feed=20.0, accel=300.0)
        dev.step_wait(10000)
        print(
            f"\nraw +3200 steps on motor 1 -> "
            f"motor 1 at {dev.step_motor_steps(StepId.M1)} steps, "
            f"tool at {dev.step_position()}"
        )

        # -- 6) redefine zero (what you do after a manual home) --------------
        dev.step_set_position(0.0, 0.0, 0.0)
        print(f"position zeroed -> {dev.step_position()}")

        # -- 7) emergency abort and release ----------------------------------
        dev.step_abort()
        dev.step_enable(False)
        print("aborted, drivers released")

    return 0


if __name__ == "__main__":
    sys.exit(main())
