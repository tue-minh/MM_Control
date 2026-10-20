
"""
Example: STS3215 Bus servo position drive and dual-frame telemetry feedback reading.
"""
#!/usr/bin/env python3
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tue_usb import TueUsbClient

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "MOCK_PORT"
    use_mock = (port == "MOCK_PORT")
    servo_id = 1

    print(f"Connecting to {port} (mock={use_mock})...")
    with TueUsbClient(port=port, mock=use_mock) as client:
        # Ping
        found = client.servo_ping(servo_id)
        print(f"[PING] Servo {servo_id}: {'ONLINE' if found else 'NO RESPONSE'}")

        # Torque enable
        client.servo_torque_enable(servo_id, True)
        print(f"[TORQUE] Enabled torque on servo {servo_id}")

        # Set goal position
        target_pos = 2048  # Center position (180 deg)
        client.servo_set_pos(servo_id, pos=target_pos, speed=800, accel=30)
        print(f"[MOVE] Commanded servo {servo_id} to position {target_pos} (0..4095)")

        # Query dual-frame feedback
        fb = client.servo_get_feedback(servo_id)
        dyn = fb.dynamics
        hlt = fb.health

        print(f"\n--- Servo {servo_id} Dynamics (0x410) ---")
        print(f"  Position: {dyn.position} ({dyn.angle_deg:.2f}°)")
        print(f"  Speed:    {dyn.speed} steps/s")
        print(f"  Load:     {dyn.load_percent:.1f}%")
        print(f"  Moving:   {dyn.is_moving}")

        print(f"\n--- Servo {servo_id} Health (0x411) ---")
        print(f"  Voltage:  {hlt.voltage_v:.1f} V")
        print(f"  Temp:     {hlt.temperature_c} °C")
        print(f"  Current:  {hlt.current_ma} mA")
        print(f"  Alarms:   {hlt.alarms if hlt.alarms else 'NORMAL (No faults)'}")

if __name__ == "__main__":
    main()
