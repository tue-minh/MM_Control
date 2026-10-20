"""
Comprehensive unit tests for the TUE USB Binary Protocol library.
"""

import os
import sys
import unittest
import struct

# Ensure parent directory (python/) is in sys.path when test script is executed directly
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from tue_usb import (
    CanId,
    ErrorCode,
    Frame,
    FrameParser,
    FrameType,
    LedState,
    PositionXYZ,
    ServoMode,
    ServoTorque,
    TueUsbClient,
    crc8_sae_j1850,
    crc8_selftest,
    is_reserved_can_id,
    mm_to_um,
    um_to_mm,
    Decoders,
    Encoders,
)


class TestProtocol(unittest.TestCase):

    def test_01_crc8_selftest(self):
        """Validate SAE J1850 CRC-8 implementation."""
        self.assertTrue(crc8_selftest())
        self.assertEqual(crc8_sae_j1850(b"123456789"), 0x4B)
        self.assertEqual(crc8_sae_j1850(b""), 0x00)

    def test_02_reserved_can_ids(self):
        """Firmware rule: no internal command may use 0x3..."""
        self.assertTrue(is_reserved_can_id(0x300))
        self.assertTrue(is_reserved_can_id(0x350))
        self.assertTrue(is_reserved_can_id(0x3FF))
        self.assertTrue(is_reserved_can_id(0x3000))
        self.assertTrue(is_reserved_can_id(0x3ABC))
        self.assertTrue(is_reserved_can_id(0x30000000))

        # Check that none of the defined CanId enum values are reserved
        for cid in CanId:
            self.assertFalse(
                is_reserved_can_id(cid.value),
                f"CanId {cid.name} (0x{cid.value:X}) collides with reserved range!"
            )

    def test_03_frame_roundtrip(self):
        """Serialize and deserialize a Frame and check byte exactness."""
        f = Frame(
            frame_type=FrameType.REQ,
            can_id=CanId.SYS_PING,
            payload=b"\x12\x34\x56\x78",
            seq=42
        )
        raw = f.to_bytes()

        self.assertEqual(raw[0], 0xAA)
        self.assertEqual(raw[1], 0x55)
        self.assertEqual(raw[-2], 0x55)
        self.assertEqual(raw[-1], 0xAA)
        self.assertEqual(len(raw), 12 + 4)  # 16 bytes

        parsed = Frame.from_bytes(raw)
        self.assertEqual(parsed.frame_type, FrameType.REQ)
        self.assertEqual(parsed.can_id, CanId.SYS_PING)
        self.assertEqual(parsed.payload, b"\x12\x34\x56\x78")
        self.assertEqual(parsed.seq, 42)
        self.assertTrue(parsed.is_crc_valid())

    def test_04_stream_parser_fuzz_and_resync(self):
        """Test stream parser recovery over noisy, fragmented, or concatenated bytes."""
        parser = FrameParser()

        f1 = Frame(FrameType.REQ, CanId.SYS_INFO, b"", seq=1)
        f2 = Frame(FrameType.REQ, CanId.STEP_GET_POS_XY, b"", seq=2)

        garbage = b"\x00\xFF\xAA\xAA\x12\x34\x55\xAA\x01\x02\x03"
        stream = garbage + f1.to_bytes() + b"\xDE\xAD" + f2.to_bytes() + b"\xBE\xEF"

        # Feed 1 byte at a time to test fragmentation
        frames = []
        for b in stream:
            res = parser.feed_byte(b)
            if res is not None:
                frames.append(res)

        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0].can_id, CanId.SYS_INFO)
        self.assertEqual(frames[0].seq, 1)
        self.assertEqual(frames[1].can_id, CanId.STEP_GET_POS_XY)
        self.assertEqual(frames[1].seq, 2)

    def test_05_payload_encoders_decoders(self):
        """Verify SYS, STEP, and SERVO payload packing/unpacking."""
        # 1. System
        p_ping = Encoders.sys_ping(0xCAFEBABE)
        self.assertEqual(Decoders.sys_ping(p_ping), 0xCAFEBABE)

        raw_info = struct.pack("<BBBB I", 1, 2, 3, 4, 0x12345678)
        info = Decoders.sys_info(raw_info)
        self.assertEqual(info.hw_id, 1)
        self.assertEqual(info.hw_rev, 2)
        self.assertEqual(info.fw_major, 3)
        self.assertEqual(info.fw_minor, 4)
        self.assertEqual(info.build_flags, 0x12345678)

        # 2. Stepper
        p_xy = Encoders.step_move_xy(12.345, -67.890)
        x, y = Decoders.step_pos_xy(p_xy)
        self.assertAlmostEqual(x, 12.345, places=3)
        self.assertAlmostEqual(y, -67.890, places=3)

        p_z = Encoders.step_move_z(45.678, feed_mm_s=25, accel_mm_s2=250)
        z = Decoders.step_pos_z(p_z)
        self.assertAlmostEqual(z, 45.678, places=3)

        # 3. Servo
        p_spos = Encoders.servo_set_pos(1, pos=2048, speed=500, accel=30)
        self.assertEqual(len(p_spos), 6)
        s_id_req, accel_req, pos_req, spd_req = struct.unpack("<BBhh", p_spos)
        self.assertEqual(s_id_req, 1)
        self.assertEqual(accel_req, 30)
        self.assertEqual(pos_req, 2048)
        self.assertEqual(spd_req, 500)

        # Test response decoder
        resp_spos = struct.pack("<Bh", 1, 2048)
        s_id, pos = Decoders.servo_pos(resp_spos)
        self.assertEqual(s_id, 1)
        self.assertEqual(pos, 2048)

        # Dual telemetry frames
        raw_dyn = struct.pack("<BhhhB", 1, 2048, 120, -350, 1)
        dyn = Decoders.servo_fb_dynamics(raw_dyn)
        self.assertEqual(dyn.servo_id, 1)
        self.assertEqual(dyn.position, 2048)
        self.assertEqual(dyn.speed, 120)
        self.assertEqual(dyn.load_raw, -350)
        self.assertAlmostEqual(dyn.load_percent, -35.0, places=1)
        self.assertTrue(dyn.is_moving)

        raw_hlt = struct.pack("<BBbBhH", 1, 118, 38, 0x01, 250, 0)
        hlt = Decoders.servo_fb_health(raw_hlt)
        self.assertEqual(hlt.servo_id, 1)
        self.assertAlmostEqual(hlt.voltage_v, 11.8, places=1)
        self.assertEqual(hlt.temperature_c, 38)
        self.assertEqual(hlt.current_ma, 250)
        self.assertIn("VOLTAGE", hlt.alarms)

    def test_06_mock_client_transactions(self):
        """Verify TueUsbClient high-level API in mock mode."""
        with TueUsbClient(mock=True) as client:
            self.assertTrue(client.is_connected())

            # System
            echo_tok = client.ping(0xAABBCCDD)
            self.assertEqual(echo_tok, 0xAABBCCDD)

            info = client.get_info()
            self.assertEqual(info.hw_id, 1)

            uid = client.get_uid()
            self.assertEqual(uid.hex_string, "11223344-55667788-99AABBCC")

            led_state = client.set_led(0, LedState.ON)
            self.assertEqual(led_state, 1)

            # Stepper
            cfg = client.step_get_config()
            self.assertEqual(cfg.xy_steps_per_mm, 80)

            client.step_setpos_xy(10.0, 20.0)
            x, y = client.step_get_pos_xy()
            self.assertAlmostEqual(x, 10.0, places=3)
            self.assertAlmostEqual(y, 20.0, places=3)

            client.step_move_rel_xy(5.0, -2.0)
            x, y = client.step_get_pos_xy()
            self.assertAlmostEqual(x, 15.0, places=3)
            self.assertAlmostEqual(y, 18.0, places=3)

            # Servo
            client.servo_set_pos(1, 1500)
            self.assertEqual(client.servo_get_pos(1), 1500)

            fb = client.servo_get_feedback(1)
            self.assertEqual(fb.dynamics.servo_id, 1)
            self.assertEqual(fb.dynamics.position, 1500)
            self.assertAlmostEqual(fb.health.voltage_v, 12.0, places=1)


if __name__ == "__main__":
    unittest.main()
