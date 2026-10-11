import os
import sys
import unittest

# Ensure package is discoverable regardless of working directory
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tue_usb.crc import crc16_ccitt, verify_crc16
from tue_usb.defs import (
    CommandId,
    LedMode,
    StatusCode,
    SOF1,
    SOF2,
    HEADER_SIZE,
    CRC_SIZE,
    MIN_FRAME_SIZE,
)
from tue_usb.exceptions import TueCrcError, TueError
from tue_usb.protocol import (
    TueFrame,
    TueFrameParser,
    decode_frame,
    decode_info_response,
    decode_led_response,
    encode_frame,
    encode_led_cmd,
)


class TestCrc16(unittest.TestCase):
    """Test CRC-16/CCITT-FALSE implementation."""

    def test_standard_check_vector(self):
        """Standard check value for b'123456789' must equal 0x29B1."""
        test_data = b"123456789"
        crc = crc16_ccitt(test_data)
        self.assertEqual(crc, 0x29B1)
        self.assertTrue(verify_crc16(test_data, 0x29B1))

    def test_empty_buffer(self):
        """Empty buffer should return initial CRC value 0xFFFF."""
        self.assertEqual(crc16_ccitt(b""), 0xFFFF)

    def test_single_byte(self):
        """Single byte calculation check."""
        crc = crc16_ccitt(b"\x00")
        self.assertIsInstance(crc, int)
        self.assertTrue(0 <= crc <= 0xFFFF)

    def test_verify_failure(self):
        """verify_crc16 should return False for mismatched CRC."""
        self.assertFalse(verify_crc16(b"123456789", 0x1234))


class TestFrameEncodingDecoding(unittest.TestCase):
    """Test frame encoding and decoding."""

    def test_encode_empty_payload(self):
        """Encode frame with 0 payload bytes."""
        frame_bytes = encode_frame(seq=1, cmd=CommandId.SYS_PING, status=StatusCode.OK, payload=b"")
        self.assertEqual(len(frame_bytes), MIN_FRAME_SIZE)  # 9 bytes
        self.assertEqual(frame_bytes[0], SOF1)
        self.assertEqual(frame_bytes[1], SOF2)
        self.assertEqual(frame_bytes[2], 1)                   # SEQ
        self.assertEqual(frame_bytes[3], CommandId.SYS_PING) # CMD
        self.assertEqual(frame_bytes[4], StatusCode.OK)      # STATUS
        self.assertEqual(frame_bytes[5], 0)                   # LEN_L
        self.assertEqual(frame_bytes[6], 0)                   # LEN_H

    def test_encode_decode_roundtrip(self):
        """Encode and decode frame round-trip."""
        payload = b"Hello STM32F407!"
        encoded = encode_frame(
            seq=42,
            cmd=CommandId.SYS_PING,
            status=StatusCode.OK,
            payload=payload,
        )

        decoded = decode_frame(encoded)
        self.assertEqual(decoded.seq, 42)
        self.assertEqual(decoded.cmd, CommandId.SYS_PING)
        self.assertEqual(decoded.status, StatusCode.OK)
        self.assertEqual(decoded.payload, payload)
        self.assertTrue(decoded.is_ok)

    def test_decode_truncated_frame(self):
        """Decoding truncated bytes must raise TueError."""
        frame_bytes = encode_frame(seq=1, cmd=0x01, payload=b"TestData")
        with self.assertRaises(TueError):
            decode_frame(frame_bytes[:5])

    def test_decode_corrupted_crc(self):
        """Corrupted CRC byte must raise TueCrcError."""
        frame_bytes = bytearray(encode_frame(seq=1, cmd=0x01, payload=b"TestData"))
        frame_bytes[-1] ^= 0xFF  # Corrupt CRC MSB
        with self.assertRaises(TueCrcError):
            decode_frame(bytes(frame_bytes))

    def test_decode_bad_sof(self):
        """Corrupted SOF markers must raise TueError."""
        frame_bytes = bytearray(encode_frame(seq=1, cmd=0x01, payload=b"TestData"))
        frame_bytes[0] = 0x00
        with self.assertRaises(TueError):
            decode_frame(bytes(frame_bytes))

    def test_payload_exceeding_max_raises(self):
        """Payload exceeding 256 bytes must raise ValueError."""
        oversized = bytes(257)
        with self.assertRaises(ValueError):
            encode_frame(seq=1, cmd=0x01, payload=oversized)


class TestStreamingParser(unittest.TestCase):
    """Test TueFrameParser streaming fragmentation and concatenation handling."""

    def setUp(self):
        self.parser = TueFrameParser()

    def test_parse_single_complete_frame(self):
        """Feed a single complete frame in one chunk."""
        data = encode_frame(seq=10, cmd=CommandId.SYS_PING, payload=b"EchoData")
        frames = self.parser.feed(data)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 10)
        self.assertEqual(frames[0].payload, b"EchoData")

    def test_parse_fragmented_byte_by_byte(self):
        """Simulate worst-case USB fragmentation by feeding 1 byte at a time."""
        data = encode_frame(seq=55, cmd=CommandId.SYS_LED_CTRL, payload=b"\x00\x01\x00\x00")
        all_frames = []
        for byte_val in data:
            frames = self.parser.feed(bytes([byte_val]))
            all_frames.extend(frames)

        self.assertEqual(len(all_frames), 1)
        frame = all_frames[0]
        self.assertEqual(frame.seq, 55)
        self.assertEqual(frame.cmd, CommandId.SYS_LED_CTRL)
        self.assertEqual(frame.payload, b"\x00\x01\x00\x00")

    def test_parse_concatenated_frames(self):
        """Feed multiple frames in a single chunk."""
        frame1 = encode_frame(seq=1, cmd=0x01, payload=b"Frame1")
        frame2 = encode_frame(seq=2, cmd=0x02, payload=b"Frame2")
        frame3 = encode_frame(seq=3, cmd=0x04, payload=b"\x00\x02")

        combined = frame1 + frame2 + frame3
        frames = self.parser.feed(combined)

        self.assertEqual(len(frames), 3)
        self.assertEqual(frames[0].seq, 1)
        self.assertEqual(frames[0].payload, b"Frame1")
        self.assertEqual(frames[1].seq, 2)
        self.assertEqual(frames[1].payload, b"Frame2")
        self.assertEqual(frames[2].seq, 3)
        self.assertEqual(frames[2].payload, b"\x00\x02")

    def test_recovery_from_garbage_preamble(self):
        """Parser must discard noise bytes and synchronize onto valid frame."""
        garbage = b"\x00\xFF\xAA\x12\x34\xFE\xAA\xAA\xAA"
        valid = encode_frame(seq=99, cmd=CommandId.SYS_PING, payload=b"Recovered")

        frames = self.parser.feed(garbage + valid)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 99)
        self.assertEqual(frames[0].payload, b"Recovered")

    def test_crc_error_dropped(self):
        """Frame with corrupted CRC must be dropped by parser without crashing."""
        corrupted = bytearray(encode_frame(seq=7, cmd=0x01, payload=b"BadCRC"))
        corrupted[-1] ^= 0x55  # Damage CRC

        valid = encode_frame(seq=8, cmd=0x01, payload=b"GoodFrame")

        frames = self.parser.feed(bytes(corrupted) + valid)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 8)
        self.assertEqual(frames[0].payload, b"GoodFrame")


class TestCommandPayloadHelpers(unittest.TestCase):
    """Test LED and Info payload encoding/decoding."""

    def test_led_command_encoding(self):
        """Test encoding LED commands for various modes."""
        # OFF
        payload_off = encode_led_cmd(led_index=0, mode=LedMode.OFF)
        self.assertEqual(payload_off, b"\x00\x00\x00\x00")

        # ON
        payload_on = encode_led_cmd(led_index=0, mode=LedMode.ON)
        self.assertEqual(payload_on, b"\x00\x01\x00\x00")

        # TOGGLE
        payload_toggle = encode_led_cmd(led_index=0, mode=LedMode.TOGGLE)
        self.assertEqual(payload_toggle, b"\x00\x02\x00\x00")

        # QUERY
        payload_query = encode_led_cmd(led_index=0, mode=LedMode.QUERY)
        self.assertEqual(payload_query, b"\x00\xFF\x00\x00")

    def test_led_response_decoding(self):
        """Test decoding LED response payload."""
        led_idx, state = decode_led_response(b"\x00\x01")
        self.assertEqual(led_idx, 0)
        self.assertEqual(state, 1)

        led_idx, state = decode_led_response(b"\x00\x00")
        self.assertEqual(led_idx, 0)
        self.assertEqual(state, 0)

    def test_info_response_decoding(self):
        """Test decoding get_info response payload."""
        # 4 bytes "MM01", versions 1.0.0, 12 bytes UID
        dummy_uid = b"\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C"
        payload = b"MM01\x01\x00\x00" + dummy_uid
        info = decode_info_response(payload)

        self.assertEqual(info["board_id"], "MM01")
        self.assertEqual(info["fw_major"], 1)
        self.assertEqual(info["fw_minor"], 0)
        self.assertEqual(info["fw_patch"], 0)
        self.assertEqual(info["version_str"], "1.0.0")
        self.assertEqual(info["mcu_uid"], "0102030405060708090A0B0C")


if __name__ == "__main__":
    unittest.main()
