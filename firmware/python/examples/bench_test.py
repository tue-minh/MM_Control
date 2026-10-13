"""Interactive bench console: pick the port and actions from an on-screen menu.

Usage (Windows / PowerShell):
    python examples\\bench_test.py

Run it, then choose a COM port from the list and pick what to do. Nothing is
commanded until you select it from the menu, so you keep control while it runs.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tue_acsi import AcsiClient, AcsiError, StepId


# -- prompts ----------------------------------------------------------------

def ask_str(prompt: str, default: str = "") -> str:
    raw = input(f"{prompt} [{default}]: ").strip()
    return raw or default


def ask_int(prompt: str, default: int) -> int:
    while True:
        raw = input(f"{prompt} [{default}]: ").strip()
        if raw == "":
            return default
        try:
            return int(raw)
        except ValueError:
            print("  enter a whole number")


def ask_float(prompt: str, default: float) -> float:
    while True:
        raw = input(f"{prompt} [{default}]: ").strip()
        if raw == "":
            return default
        try:
            return float(raw)
        except ValueError:
            print("  enter a number")


def ask_bool(prompt: str, default: bool = True) -> bool:
    hint = "Y/n" if default else "y/N"
    while True:
        raw = input(f"{prompt} [{hint}]: ").strip().lower()
        if raw == "":
            return default
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False
        print("  answer y or n")


def pick_port() -> str:
    """Let the user choose from the detected ports, or type one in."""
    try:
        from serial.tools import list_ports

        ports = sorted(list_ports.comports(), key=lambda p: p.device)
    except Exception:
        ports = []

    if ports:
        print("available ports:")
        for i, port in enumerate(ports, 1):
            print(f"  {i}) {port.device}  {port.description}")
        print("  0) type a port manually")
        while True:
            choice = input("select port [1]: ").strip()
            if choice == "":
                return ports[0].device
            if choice == "0":
                break
            if choice.isdigit() and 1 <= int(choice) <= len(ports):
                return ports[int(choice) - 1].device
            print("  invalid selection")

    return ask_str("port (e.g. COM7)")


def connect(port: str) -> "AcsiClient | None":
    try:
        dev = AcsiClient(port)
    except Exception as exc:
        print(f"could not open {port}: {exc}")
        return None
    print(f"connected to {port}\n")
    return dev


# -- actions ----------------------------------------------------------------

def do_link(dev: AcsiClient) -> None:
    info = dev.sys_info()
    print(f"  board={info['board']} fw={info['fw']} ping={dev.sys_ping()} uptime={dev.sys_uptime()} ms")


def do_scan(dev: AcsiClient) -> None:
    start = ask_int("scan start id", 1)
    end = ask_int("scan end id", 16)
    found = []
    for sid in range(start, end + 1):
        try:
            dev.servo_ping(sid)
        except AcsiError:
            continue
        found.append(sid)
        print(f"  servo {sid} responded")
    print(f"  found: {found if found else 'none'}")


def do_servo_feedback(dev: AcsiClient) -> None:
    sid = ask_int("servo id", 1)
    fb = dev.servo_feedback(sid)
    print(
        f"  pos={fb['pos']} spd={fb['spd']} load={fb['load']} "
        f"volt={fb['volt']} temp={fb['temp']} status={fb['status']} moving={fb['moving']}"
    )


def do_servo_set_pos(dev: AcsiClient) -> None:
    sid = ask_int("servo id", 1)
    pos = ask_int("target position (steps)", 2048)
    speed = ask_int("speed", 800)
    accel = ask_int("accel", 30)
    if ask_bool("energise torque first?", True):
        dev.servo_set_torque(True, sid)
    dev.servo_set_pos(pos, servo_id=sid, speed=speed, accel=accel)
    time.sleep(0.6)
    fb = dev.servo_feedback(sid)
    print(f"  commanded {pos} -> pos={fb['pos']} spd={fb['spd']} status={fb['status']}")
    if ask_bool("release torque now?", False):
        dev.servo_set_torque(False, sid)
        print("  torque released")


def do_step_status(dev: AcsiClient) -> None:
    info = dev.step_info()
    status = dev.step_status()
    pos = dev.step_position()
    print(
        f"  xy={info['xy_steps_per_mm']} steps/mm z={info['z_steps_per_mm']} steps/mm "
        f"rate {info['min_rate']}..{info['max_rate']} steps/s"
    )
    print(f"  busy={status['busy']} enabled={status['enabled']}")
    print(f"  position: x={pos['x']} y={pos['y']} z={pos['z']} mm")


def do_step_enable(dev: AcsiClient) -> None:
    dev.step_enable(True)
    print(f"  drivers energised (enabled={dev.step_status()['enabled']})")


def do_step_disable(dev: AcsiClient) -> None:
    dev.step_enable(False)
    print(f"  drivers released (enabled={dev.step_status()['enabled']})")


def do_step_move(dev: AcsiClient) -> None:
    x = ask_float("x (mm)", 20.0)
    y = ask_float("y (mm)", 20.0)
    z = ask_float("z (mm)", 5.0)
    feed = ask_float("feed (mm/s)", 30.0)
    accel = ask_float("accel (mm/s^2)", 300.0)
    dev.step_enable(True)
    dev.step_move(x, y, z, feed=feed, accel=accel)
    elapsed = dev.step_wait(10000)
    pos = dev.step_position()
    print(f"  done in {elapsed} ms -> x={pos['x']} y={pos['y']} z={pos['z']} mm")


def do_step_stop(dev: AcsiClient) -> None:
    dev.step_stop()
    dev.step_wait(5000)
    print("  stopped")


MENU = """
1) link check             2) scan servos
3) servo feedback         4) servo set position
5) stepper enable         6) stepper disable
7) stepper move           8) stepper status
9) stepper stop           0) quit
"""


def menu(dev: AcsiClient) -> None:
    actions = {
        "1": do_link,
        "2": do_scan,
        "3": do_servo_feedback,
        "4": do_servo_set_pos,
        "5": do_step_enable,
        "6": do_step_disable,
        "7": do_step_move,
        "8": do_step_status,
        "9": do_step_stop,
    }
    while True:
        print(MENU)
        choice = input("select> ").strip().lower()
        if choice in ("0", "q", "quit"):
            return
        action = actions.get(choice)
        if action is None:
            print("  unknown option")
            continue
        try:
            action(dev)
        except AcsiError as exc:
            print(f"  FAIL: {exc}")
        except KeyboardInterrupt:
            print("\n  interrupted")


def main() -> int:
    try:
        port = pick_port()
    except (EOFError, KeyboardInterrupt):
        print()
        return 0
    if not port:
        print("no port given")
        return 1

    dev = connect(port)
    if dev is None:
        return 1

    with dev:
        try:
            menu(dev)
        except (EOFError, KeyboardInterrupt):
            print()
    print("bye")
    return 0


if __name__ == "__main__":
    sys.exit(main())
