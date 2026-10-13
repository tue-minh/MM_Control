"""Unit tests for the ACSI protocol and client.

No hardware and no ``pyserial`` needed: the client runs against an in-memory
fake serial port.
"""

import os
import re
import sys
import types
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tue_acsi.defs import (  # noqa: E402
    Code,
    Dir,
    ServoFault,
    ServoField,
    StepField,
    StepId,
    SysField,
    Target,
    mm_to_um,
    um_to_mm,
)
from tue_acsi.exceptions import (  # noqa: E402
    AcsiDeviceError,
    AcsiFormatError,
    AcsiTimeout,
)
from tue_acsi.protocol import (  # noqa: E402
    build_body,
    decode,
    encode,
    encode_line,
)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ACSI_HEADER = os.path.join(REPO_ROOT, "Core", "Inc", "tue_acsi.h")


def firmware_enum(type_name: str):
    """Parse ``typedef enum { ... } <type_name>;`` out of the firmware header.

    Used to prove the host-side enums cannot drift away from the C ones.
    """
    with open(ACSI_HEADER, encoding="utf-8", errors="replace") as handle:
        text = handle.read()
    end = text.index(f"}} {type_name};")
    begin = text.rindex("typedef enum {", 0, end)
    members = {}
    for line in text[begin:end].splitlines():
        line = re.sub(r"/\*.*?\*/", "", line).strip().rstrip(",")
        if "=" not in line:
            continue
        key, _, value = line.partition("=")
        key, value = key.strip(), value.strip()
        if key and value.isdigit():
            members[key] = int(value)
    return members


# SYS INFO response: board "MM_Control", fw "1.0", sent as literal text.
INFO_BODY = "4 0 0 MM_Control 1.0"


class TestEncoding(unittest.TestCase):
    def test_build_body(self):
        self.assertEqual(
            build_body(Target.SERVO, 1, Dir.SET, ServoField.POS, 2048),
            "1 1 1 2 2048",
        )

    def test_encode_line_has_no_checksum(self):
        self.assertEqual(
            encode_line(Target.SERVO, 1, Dir.GET, ServoField.PING),
            "1 1 0 1",
        )

    def test_encode_bytes_terminator(self):
        self.assertEqual(
            encode(Target.SYS, 0, Dir.SET, SysField.LED, 0, 1),
            b"4 0 1 6 0 1\n",
        )

    def test_requests_are_numbers_only(self):
        line = encode_line(Target.SERVO, 2, Dir.SET, ServoField.POS, 2048)
        self.assertTrue(all(part.lstrip("-").isdigit() for part in line.split()))


class TestDecoding(unittest.TestCase):
    def test_decode_ok(self):
        resp = decode("1 1 0 2048")
        self.assertTrue(resp.ok)
        self.assertEqual((resp.target, resp.device_id), (1, 1))
        self.assertEqual(resp.payload, [2048])
        self.assertEqual(resp.value, 2048)

    def test_decode_feedback(self):
        resp = decode("1 1 0 2048 0 0 120 30 0 0 0")
        self.assertEqual(resp.payload, [2048, 0, 0, 120, 30, 0, 0, 0])

    def test_decode_error_frame(self):
        resp = decode("1 1 6")
        self.assertFalse(resp.ok)
        self.assertEqual(resp.error_code, 6)
        self.assertEqual(resp.error_name, "BUS_TIMEOUT")

    def test_decode_info_text(self):
        resp = decode(INFO_BODY)
        self.assertEqual(resp.payload, ["MM_Control", "1.0"])

    def test_decode_strips_line_ending(self):
        self.assertEqual(decode("1 1 0 1\r\n").payload, [1])

    def test_non_numeric_header_raises(self):
        with self.assertRaises(AcsiFormatError):
            decode("SERVO 1 0 2")

    def test_truncated_raises(self):
        with self.assertRaises(AcsiFormatError):
            decode("1 1")


class FakeSerial:
    """Minimal in-memory stand-in for ``serial.Serial``."""

    def __init__(self, *_args, **_kwargs):
        self.written = bytearray()
        self._resp = bytearray()

    def queue(self, line: str) -> None:
        self._resp += (line + "\n").encode("ascii")

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def read(self, size: int = 1) -> bytes:
        if not self._resp:
            return b""
        out = bytes(self._resp[:size])
        del self._resp[:size]
        return out

    def reset_input_buffer(self) -> None:  # keep queued data for the test
        pass

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


class TestClientAgainstFakeSerial(unittest.TestCase):
    def setUp(self):
        self.fake = FakeSerial()
        module = types.ModuleType("serial")
        module.Serial = lambda *_a, **_k: self.fake
        sys.modules["serial"] = module

        from tue_acsi.client import AcsiClient

        self.client = AcsiClient("FAKE")

    def test_request_bytes_and_parse(self):
        self.fake.queue("1 1 0 1")
        self.assertEqual(self.client.servo_ping(1), 1)
        self.assertEqual(bytes(self.fake.written), b"1 1 0 1\n")

    def test_set_pos_request_bytes(self):
        self.fake.queue("1 1 0 2048")
        self.assertEqual(self.client.servo_set_pos(2048, 1), 2048)
        self.assertEqual(bytes(self.fake.written), b"1 1 1 2 2048\n")

    def test_device_error_raised(self):
        self.fake.queue("1 1 6")
        with self.assertRaises(AcsiDeviceError) as ctx:
            self.client.servo_get_pos(1)
        self.assertEqual(ctx.exception.code, 6)
        self.assertEqual(ctx.exception.name, "BUS_TIMEOUT")

    def test_sys_info_reads_text(self):
        self.fake.queue(INFO_BODY)
        self.assertEqual(self.client.sys_info(), {"board": "MM_Control", "fw": "1.0"})


class TestStepProtocol(unittest.TestCase):
    """Codes, units and frames of the STEP target (tue_stepper over ACSI)."""

    def test_field_codes_match_firmware(self):
        # must stay in sync with tue_acsi_step_field_t in Core/Inc/tue_acsi.h
        self.assertEqual([int(f) for f in StepField], list(range(1, 18)))
        self.assertEqual([int(i) for i in StepId], [0, 1, 2, 3])
        self.assertEqual(int(StepField.INFO), 1)
        self.assertEqual(int(StepField.MOVE), 6)
        self.assertEqual(int(StepField.MOVE2D), 7)
        self.assertEqual(int(StepField.MOVEZ), 8)
        self.assertEqual(int(StepField.JOG), 10)
        self.assertEqual(int(StepField.STOP), 12)
        self.assertEqual(int(StepField.ABORT), 13)
        self.assertEqual(int(StepField.STEPS), 15)
        self.assertEqual(int(StepField.RAW), 16)
        self.assertEqual(int(StepField.WAIT), 17)
        self.assertEqual(int(StepId.TOOL), 0)
        self.assertEqual(int(StepId.M1), 1)
        self.assertEqual(int(StepId.M2), 2)
        self.assertEqual(int(StepId.M3), 3)

    def test_error_codes_match_firmware(self):
        # tue_acsi_code_t: 12 QUEUE_FULL, 13 TIMEOUT
        self.assertEqual(int(Code.QUEUE_FULL), 12)
        self.assertEqual(int(Code.TIMEOUT), 13)
        self.assertEqual(Code(12).name, "QUEUE_FULL")
        self.assertEqual(Code(13).name, "TIMEOUT")

    def test_mm_um_conversion(self):
        self.assertEqual(mm_to_um(50.0), 50000)
        self.assertEqual(mm_to_um(-2.5), -2500)
        self.assertEqual(mm_to_um(0.0), 0)
        self.assertEqual(mm_to_um(0.003125), 3)   # one XY step (320 steps/mm)
        self.assertEqual(mm_to_um(0.000625), 1)   # one Z step (1600 steps/mm)
        self.assertEqual(um_to_mm(50000), 50.0)
        self.assertEqual(um_to_mm(-320), -0.32)
        self.assertEqual(um_to_mm(mm_to_um(12.345)), 12.345)

    def test_move_frame_is_numbers_only(self):
        line = encode_line(
            Target.STEP,
            StepId.TOOL,
            Dir.SET,
            StepField.MOVE,
            mm_to_um(50.0),
            mm_to_um(30.0),
            mm_to_um(10.0),
            mm_to_um(60.0),
            mm_to_um(600.0),
        )
        self.assertEqual(line, "2 0 1 6 50000 30000 10000 60000 600000")
        self.assertTrue(all(part.lstrip("-").isdigit() for part in line.split()))

    def test_raw_frame_addresses_one_motor(self):
        line = encode_line(
            Target.STEP,
            StepId.M1,
            Dir.SET,
            StepField.RAW,
            3200,
            mm_to_um(20.0),
            mm_to_um(200.0),
        )
        self.assertEqual(line, "2 1 1 16 3200 20000 200000")

    @unittest.skipUnless(os.path.exists(ACSI_HEADER), "firmware header not available")
    def test_field_codes_match_c_header(self):
        members = firmware_enum("tue_acsi_step_field_t")
        self.assertEqual(len(members), len(StepField))
        for name, value in members.items():
            short = name[len("TUE_ACSI_STEP_"):]
            self.assertEqual(int(StepField[short]), value, name)

    @unittest.skipUnless(os.path.exists(ACSI_HEADER), "firmware header not available")
    def test_error_codes_match_c_header(self):
        members = firmware_enum("tue_acsi_code_t")
        self.assertEqual(len(members), len(Code))
        for name, value in members.items():
            short = name[len("TUE_ACSI_ERR_"):] if name.startswith("TUE_ACSI_ERR_") \
                else name[len("TUE_ACSI_"):]
            self.assertEqual(int(Code[short]), value, name)

    @unittest.skipUnless(os.path.exists(ACSI_HEADER), "firmware header not available")
    def test_target_codes_match_c_header(self):
        members = firmware_enum("tue_acsi_target_t")
        for name, value in members.items():
            short = name[len("TUE_ACSI_TARGET_"):]
            self.assertEqual(int(Target[short]), value, name)

    def test_all_ids_are_numbers_only(self):
        # request frames must never carry text, whatever the frame says
        for motor in (StepId.M1, StepId.M2, StepId.M3):
            line = encode_line(Target.STEP, motor, Dir.GET, StepField.STEPS)
            self.assertTrue(all(part.lstrip("-").isdigit() for part in line.split()))


class TestStepClientAgainstFakeSerial(unittest.TestCase):
    """Exact request bytes and response decoding for the STEP target."""

    def setUp(self):
        self.fake = FakeSerial()
        module = types.ModuleType("serial")
        module.Serial = lambda *_a, **_k: self.fake
        sys.modules["serial"] = module

        from tue_acsi.client import AcsiClient

        self.client = AcsiClient("FAKE")

    def assert_sent(self, expected: str) -> None:
        """Assert the whole request line."""
        self.assertEqual(bytes(self.fake.written), (expected + "\n").encode("ascii"))

    def sent_body(self) -> str:
        """The request line as written (trailing newline removed)."""
        return bytes(self.fake.written).decode("ascii").strip()

    # -- moves ---------------------------------------------------------------
    def test_move_request_bytes_and_echo(self):
        self.fake.queue("2 0 0 50000 30000 10000")
        pos = self.client.step_move(50.0, 30.0, 10.0, feed=60.0, accel=600.0)
        self.assert_sent("2 0 1 6 50000 30000 10000 60000 600000")
        self.assertEqual(pos, {"x": 50.0, "y": 30.0, "z": 10.0})

    def test_move_xy_request_bytes(self):
        self.fake.queue("2 0 0 20000 20000")
        self.assertEqual(
            self.client.step_move_xy(20.0, 20.0, feed=30.0, accel=300.0),
            {"x": 20.0, "y": 20.0},
        )
        self.assert_sent("2 0 1 7 20000 20000 30000 300000")

    def test_move_z_request_bytes(self):
        self.fake.queue("2 0 0 15000")
        self.assertEqual(self.client.step_move_z(15.0, feed=10.0, accel=200.0), 15.0)
        self.assert_sent("2 0 1 8 15000 10000 200000")

    def test_move_rel_request_bytes(self):
        self.fake.queue("2 0 0 -5000 5000")
        self.assertEqual(
            self.client.step_move_rel(-5.0, 5.0, feed=30.0, accel=300.0),
            {"x": -5.0, "y": 5.0},
        )
        self.assertEqual(self.sent_body(), "2 0 1 9 -5000 5000 30000 300000")

    # -- jogs ----------------------------------------------------------------
    def test_jog_x_request_bytes(self):
        self.fake.queue("2 0 0 20000 0")
        self.assertEqual(self.client.step_jog_x(20.0, accel=500.0), {"x": 20.0, "y": 0.0})
        self.assertEqual(self.sent_body(), "2 0 1 10 20000 0 500000")

    def test_jog_z_keeps_the_sign(self):
        self.fake.queue("2 0 0 -5000")
        self.assertEqual(self.client.step_jog_z(-5.0, accel=300.0), -5.0)
        self.assert_sent("2 0 1 11 -5000 300000")

    # -- stops ---------------------------------------------------------------
    def test_stop_and_abort_request_bytes(self):
        self.fake.queue("2 0 0 1")
        self.assertEqual(self.client.step_stop(), 1)
        self.assert_sent("2 0 1 12")

        self.fake.written.clear()
        self.fake.queue("2 0 0 0")
        self.assertEqual(self.client.step_abort(), 0)
        self.assert_sent("2 0 1 13")

    # -- queries -------------------------------------------------------------
    def test_position_decodes_um_to_mm(self):
        self.fake.queue("2 0 0 50000 30000 10000")
        self.assertEqual(self.client.step_position(), {"x": 50.0, "y": 30.0, "z": 10.0})
        self.assertEqual(self.sent_body(), "2 0 0 2")

    def test_target_uses_its_own_field(self):
        self.fake.queue("2 0 0 50000 30000 10000")
        self.assertEqual(self.client.step_target()["y"], 30.0)
        self.assertEqual(self.sent_body(), "2 0 0 3")

    def test_info_parsing(self):
        self.fake.queue("2 0 0 320 1600 400 50000")
        self.assertEqual(
            self.client.step_info(),
            {
                "xy_steps_per_mm": 320,
                "z_steps_per_mm": 1600,
                "min_rate": 400,
                "max_rate": 50000,
            },
        )
        self.assertEqual(self.sent_body(), "2 0 0 1")

    def test_status_and_busy(self):
        self.fake.queue("2 0 0 1 1")
        self.assertEqual(self.client.step_status(), {"busy": 1, "enabled": 1})
        self.assertEqual(self.sent_body(), "2 0 0 4")

        self.fake.written.clear()
        self.fake.queue("2 0 0 0 1")
        self.assertFalse(self.client.step_is_busy())

    def test_enable_request_bytes(self):
        self.fake.queue("2 0 0 1")
        self.assertEqual(self.client.step_enable(True), 1)
        self.assert_sent("2 0 1 5 1")

        self.fake.written.clear()
        self.fake.queue("2 0 0 0")
        self.assertEqual(self.client.step_enable(False), 0)
        self.assertEqual(self.sent_body(), "2 0 1 5 0")

    # -- per-motor -----------------------------------------------------------
    def test_motor_steps_addresses_the_motor(self):
        self.fake.queue("2 3 0 -8000")
        self.assertEqual(self.client.step_motor_steps(StepId.M3), -8000)
        self.assert_sent("2 3 0 15")

    def test_raw_move_request_bytes(self):
        self.fake.queue("2 1 0 3200")
        self.assertEqual(self.client.step_raw(StepId.M1, 3200, 20.0, 200.0), 3200)
        self.assert_sent("2 1 1 16 3200 20000 200000")

    def test_set_position_request_bytes(self):
        self.fake.queue("2 0 0 0 0 0")
        self.assertEqual(
            self.client.step_set_position(0.0, 0.0, 0.0),
            {"x": 0.0, "y": 0.0, "z": 0.0},
        )
        self.assert_sent("2 0 1 14 0 0 0")

    # -- wait ----------------------------------------------------------------
    def test_wait_returns_elapsed_ms(self):
        self.fake.queue("2 0 0 412")
        self.assertEqual(self.client.step_wait(5000), 412)
        self.assert_sent("2 0 1 17 5000")

    def test_wait_forever_still_sends_the_argument(self):
        self.fake.queue("2 0 0 0")
        self.assertEqual(self.client.step_wait(), 0)
        self.assertEqual(self.sent_body(), "2 0 1 17 0")

    def test_wait_read_deadline_is_used(self):
        # nothing queued: the client must give up after the requested deadline
        with self.assertRaises(AcsiTimeout):
            self.client.step_wait(5000, read_timeout=0.05)

    # -- errors --------------------------------------------------------------
    def test_queue_full_reported(self):
        self.fake.queue("2 0 12")
        with self.assertRaises(AcsiDeviceError) as ctx:
            self.client.step_move(1.0, 2.0, 3.0, feed=10.0, accel=100.0)
        self.assertEqual(ctx.exception.code, 12)
        self.assertEqual(ctx.exception.name, "QUEUE_FULL")

    def test_bad_argument_reported(self):
        self.fake.queue("2 0 5")
        with self.assertRaises(AcsiDeviceError) as ctx:
            self.client.step_move(1.0, 2.0, 3.0, feed=0.0, accel=0.0)
        self.assertEqual(ctx.exception.name, "BAD_ARG")

    def test_wait_timeout_reported(self):
        self.fake.queue("2 0 13")
        with self.assertRaises(AcsiDeviceError) as ctx:
            self.client.step_wait(10)
        self.assertEqual(ctx.exception.name, "TIMEOUT")

    def test_busy_reported(self):
        self.fake.queue("2 0 9")
        with self.assertRaises(AcsiDeviceError) as ctx:
            self.client.step_move(1.0, 2.0, 3.0, feed=10.0, accel=100.0)
        self.assertEqual(ctx.exception.name, "BUSY")


class TestServoClientAgainstFakeSerial(unittest.TestCase):
    """Exact request bytes for the full SERVO field set (tue_sts3215 over ACSI)."""

    def setUp(self):
        self.fake = FakeSerial()
        module = types.ModuleType("serial")
        module.Serial = lambda *_a, **_k: self.fake
        sys.modules["serial"] = module

        from tue_acsi.client import AcsiClient

        self.client = AcsiClient("FAKE")

    def sent_body(self) -> str:
        """The request line as written (trailing newline removed)."""
        return bytes(self.fake.written).decode("ascii").strip()

    def test_set_acc(self):
        self.fake.queue("1 1 0 50")
        self.assertEqual(self.client.servo_set_acc(50, 1), 50)
        self.assertEqual(self.sent_body(), "1 1 1 4 50")

    def test_get_acc(self):
        self.fake.queue("1 1 0 50")
        self.assertEqual(self.client.servo_get_acc(1), 50)
        self.assertEqual(self.sent_body(), "1 1 0 4")

    def test_get_torque(self):
        self.fake.queue("1 1 0 1")
        self.assertEqual(self.client.servo_get_torque(1), 1)
        self.assertEqual(self.sent_body(), "1 1 0 5")

    def test_torque_limit_roundtrip(self):
        self.fake.queue("1 1 0 1000")
        self.assertEqual(self.client.servo_set_torque_limit(1000, 1), 1000)
        self.assertEqual(self.sent_body(), "1 1 1 6 1000")

        self.fake.written.clear()
        self.fake.queue("1 1 0 1000")
        self.assertEqual(self.client.servo_get_torque_limit(1), 1000)
        self.assertEqual(self.sent_body(), "1 1 0 6")

    def test_mode_get(self):
        self.fake.queue("1 1 0 0")
        self.assertEqual(self.client.servo_get_mode(1), 0)
        self.assertEqual(self.sent_body(), "1 1 0 7")

    def test_pwm_signed(self):
        self.fake.queue("1 1 0 -500")
        self.assertEqual(self.client.servo_set_pwm(-500, 1), -500)
        self.assertEqual(self.sent_body(), "1 1 1 8 -500")

    def test_id_roundtrip(self):
        self.fake.queue("1 1 0 3")
        self.assertEqual(self.client.servo_set_id(3, 1), 3)
        self.assertEqual(self.sent_body(), "1 1 1 11 3")

    def test_baud_roundtrip(self):
        self.fake.queue("1 1 0 0")
        self.assertEqual(self.client.servo_set_baud(0, 1), 0)
        self.assertEqual(self.sent_body(), "1 1 1 12 0")

    def test_limits_returns_tuple(self):
        self.fake.queue("1 1 0 0 4095")
        self.assertEqual(self.client.servo_get_limits(1), (0, 4095))
        self.assertEqual(self.sent_body(), "1 1 0 13")

    def test_set_limits_bytes(self):
        self.fake.queue("1 1 0 0")
        self.assertEqual(self.client.servo_set_limits(0, 4095, 1), 0)
        self.assertEqual(self.sent_body(), "1 1 1 13 0 4095")

    def test_scalar_reads(self):
        for field, method in (
            (14, "servo_get_temp"),
            (15, "servo_get_volt"),
            (16, "servo_get_load"),
            (17, "servo_get_current"),
        ):
            self.fake.written.clear()
            self.fake.queue("1 1 0 42")
            self.assertEqual(getattr(self.client, method)(1), 42)
            self.assertEqual(self.sent_body(), f"1 1 0 {field}")

    def test_moving(self):
        self.fake.queue("1 1 0 1")
        self.assertTrue(self.client.servo_is_moving(1))
        self.assertEqual(self.sent_body(), "1 1 0 19")

    def test_reset(self):
        self.fake.queue("1 1 0 1")
        self.assertEqual(self.client.servo_reset(1), 1)
        self.assertEqual(self.sent_body(), "1 1 1 20")

    def test_fault_mask(self):
        self.assertEqual(int(ServoFault.OVERLOAD), 0x20)
        faults = ServoFault(0x21)
        self.assertIn(ServoFault.VOLTAGE, faults)
        self.assertIn(ServoFault.OVERLOAD, faults)
        self.assertNotIn(ServoFault.SENSOR, faults)


if __name__ == "__main__":
    unittest.main()
