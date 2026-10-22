import struct
import unittest

from protocol import (
    CMD_MOVE_RELATIVE,
    CMD_MOVE_OUTPUT_RELATIVE,
    CMD_SET_MOTION_PROFILE,
    CMD_WRITE_PA,
    CMD_READ_SPEED,
    CMD_LIMIT_DEBUG,
    ResponseParser,
    append_crc,
    build_move_relative,
    build_move_output_relative,
    build_motion_profile,
    build_write_pa,
    build_limit_debug,
    build_continuous_rotation, build_rotation_keepalive,
    CMD_ROTATE_CONTINUOUS, CMD_ROTATE_KEEPALIVE,
    verify_crc,
)


def response(device: int, command: int, status: int, data: bytes = b"") -> bytes:
    payload = bytes((device, command, status, 0, 0)) + data
    return append_crc(b"\xAA\x55" + bytes((len(payload) + 2,)) + payload)


class ProtocolTests(unittest.TestCase):
    def test_continuous_rotation_frames(self):
        frame = build_continuous_rotation(1, -5)
        self.assertTrue(verify_crc(frame))
        self.assertEqual(frame[4], CMD_ROTATE_CONTINUOUS)
        self.assertEqual(struct.unpack(">i", frame[7:11])[0], -5000)
        heartbeat = build_rotation_keepalive(1)
        self.assertTrue(verify_crc(heartbeat))
        self.assertEqual(heartbeat[3:7], bytes((1, CMD_ROTATE_KEEPALIVE, 0x43, 0x52)))
        with self.assertRaises(ValueError):
            build_continuous_rotation(1, 0)

    def test_limit_debug_frames(self):
        for action in (0, 1, 2):
            frame = build_limit_debug(2, action)
            self.assertTrue(verify_crc(frame))
            self.assertEqual(frame[3:8], bytes((2, CMD_LIMIT_DEBUG, 0x4C, 0x44, action)))
        with self.assertRaises(ValueError):
            build_limit_debug(1, 3)

    def test_move_relative_frame(self):
        frame = build_move_relative(2, 50, -2500)
        self.assertTrue(verify_crc(frame))
        self.assertEqual(frame[:2], b"\xAA\x55")
        self.assertEqual(frame[3:7], bytes((2, CMD_MOVE_RELATIVE, 0, 0)))
        self.assertEqual(struct.unpack(">hi", frame[7:13]), (50, -2500))

    def test_invalid_motion_is_rejected(self):
        with self.assertRaises(ValueError):
            build_move_relative(1, 0, 100)
        with self.assertRaises(ValueError):
            build_move_relative(3, 50, 100)
        with self.assertRaises(ValueError):
            build_move_relative(1, 50, 0)

    def test_temporary_pa_write_frame(self):
        frame = build_write_pa(1, 42, 100)
        self.assertTrue(verify_crc(frame))
        self.assertEqual(frame[3:7], bytes((1, CMD_WRITE_PA, 0, 42)))
        self.assertEqual(frame[7:10], bytes((0, 0, 100)))

    def test_saved_pa_write_frame(self):
        frame = build_write_pa(2, 40, 1500, save=True)
        self.assertEqual(frame[3:7], bytes((2, CMD_WRITE_PA, 0, 40)))
        self.assertEqual(frame[7:10], bytes((1, 0x05, 0xDC)))

    def test_output_axis_move_frame(self):
        frame = build_move_output_relative(2, 0.2, -180.0)
        self.assertTrue(verify_crc(frame))
        self.assertEqual(frame[3:7], bytes((2, CMD_MOVE_OUTPUT_RELATIVE, 0x50, 0x54)))
        self.assertEqual(struct.unpack(">Hi", frame[7:13]), (20, -180000))

        positive = build_move_output_relative(2, 0.2, 180.0)
        self.assertEqual(struct.unpack(">Hi", positive[7:13]), (20, 180000))

    def test_motion_profile_frame(self):
        frame = build_motion_profile(1, 1500, 1500, 100)
        self.assertEqual(frame[3:7], bytes((1, CMD_SET_MOTION_PROFILE, 0, 0)))
        self.assertEqual(struct.unpack(">HHH", frame[7:13]), (1500, 1500, 100))

    def test_parser_handles_noise_sticky_and_partial_frames(self):
        parser = ResponseParser()
        frame = response(1, CMD_MOVE_RELATIVE, 0, struct.pack(">i", 2500))
        packets = parser.feed(b"noise" + frame + frame[:5])
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].device_id, 1)
        packets = parser.feed(frame[5:])
        self.assertEqual(len(packets), 1)
        self.assertEqual(struct.unpack(">i", packets[0].data)[0], 2500)

    def test_parser_recovers_from_false_length(self):
        parser = ResponseParser()
        frame = response(1, CMD_MOVE_RELATIVE, 0, struct.pack(">i", 90))
        packets = parser.feed(b"\xAA\x55\x30bad" + frame)
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].command, CMD_MOVE_RELATIVE)

    def test_speed_response_contains_run_state(self):
        parser = ResponseParser()
        packets = parser.feed(response(2, CMD_READ_SPEED, 0, struct.pack(">hH", 0, 3)))
        self.assertEqual(len(packets), 1)
        self.assertEqual(struct.unpack(">hH", packets[0].data), (0, 3))

    def test_speed_response_preserves_limit_flags(self):
        parser = ResponseParser()
        data = struct.pack(">hHB", -10, 3, 3)
        packets = parser.feed(response(1, CMD_READ_SPEED, 0, data))
        self.assertEqual(packets[0].data, data)

    def test_limit_rejection_status(self):
        packet = ResponseParser().feed(response(1, CMD_MOVE_OUTPUT_RELATIVE, 8))[0]
        self.assertEqual(packet.status, 8)
        self.assertIn("限位", packet.status_text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
