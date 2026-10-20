#!/usr/bin/env python3
"""
Example: Stepper motor position query, enabling, absolute moves, and status check.
"""

import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tue_usb import TueUsbClient

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "MOCK_PORT"
    use_mock = (port == "MOCK_PORT")

    print(f"Connecting to {port} (mock={use_mock})...")
    with TueUsbClient(port=port, mock=use_mock) as client:
        # Config & Status
        cfg = client.step_get_config()
        print(f"[CFG] Steps/mm: XY={cfg.xy_steps_per_mm}, Z={cfg.z_steps_per_mm}")
        print(f"[CFG] Rate limits: min={cfg.min_rate_steps_s}, max={cfg.max_rate_steps_s} steps/s")

        st = client.step_get_status()
        print(f"[STAT] Busy={st.is_busy}, Enabled={st.is_enabled}, Queue Depth={st.queue_depth}")

        # Enable
        client.step_enable(True)
        print("[EN] Stepper drivers enabled.")

        # Read initial position
        pos = client.step_get_pos_xyz()
        print(f"[POS] Start position: X={pos.x_mm:.3f} mm, Y={pos.y_mm:.3f} mm, Z={pos.z_mm:.3f} mm")

        # Zero position
        client.step_setpos_xy(0.0, 0.0)
        client.step_setpos_z(0.0)
        print("[ZERO] Reset tool position to (0, 0, 0)")

        # Move XY
        print("[MOVE] Moving to X=50.0 mm, Y=30.0 mm...")
        client.step_move_xy(50.0, 30.0)

        # Move Z
        print("[MOVE] Moving to Z=10.0 mm...")
        client.step_move_z(10.0, feed_mm_s=20.0, accel_mm_s2=200.0)

        # Coordinated 3D move
        print("[MOVE] Coordinated 3D move to X=10.0 mm, Y=10.0 mm, Z=2.0 mm...")
        client.step_move_xyz(10.0, 10.0, 2.0, feed_mm_s=60.0, accel_mm_s2=600.0)

        # Wait until motion idle
        print("[WAIT] Waiting for motion completion...")
        client.step_wait_idle(5000)

        pos_end = client.step_get_pos_xyz()
        print(f"[POS] Final position: X={pos_end.x_mm:.3f} mm, Y={pos_end.y_mm:.3f} mm, Z={pos_end.z_mm:.3f} mm")

if __name__ == "__main__":
    main()
