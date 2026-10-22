"""无硬件 API 回归测试，从项目根目录运行 unittest discover。"""
import struct
import unittest
from unittest.mock import Mock, patch

from app import MotorController, MotorError, CommunicationError, CommandRejected, DriveAlarm
from app.protocol import (
    CMD_STOP, CMD_READ_SPEED, CMD_READ_POSITION, CMD_READ_ALARM,
    CMD_MOVE_OUTPUT_RELATIVE, CMD_READ_PA, CMD_SET_MOTION_PROFILE,
    CMD_ROTATE_CONTINUOUS, CMD_ROTATE_KEEPALIVE,
    append_crc,
)


class FakeSerial:
    def __init__(self):
        self.buffer = bytearray()
        self.frames = []
        self.closed = False
        self.rpm = 0
        self.alarm = 0
        self.pa = {11: 10000, 40: 1500, 41: 1500, 42: 100}
        self.reject_command = None
        self.reject_status = 8
        self.mute = False
        self.mute_command = None
        self.partial_write = False
        self.short_data = False
        self.speed_prefix = b"noise [LIMIT] CLEAR\r\n"
        self.custom_responses = {}

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self, frame):
        self.frames.append(frame)
        command = frame[4]
        status = self.reject_status if command == self.reject_command else 0
        data = b"\x00\x00"
        if command == CMD_READ_SPEED:
            data = struct.pack(">hHB", self.rpm, 3, 1)
        elif command == CMD_READ_ALARM:
            data = struct.pack(">H", self.alarm)
        elif command == CMD_READ_PA:
            parameter = struct.unpack(">H", frame[5:7])[0]
            data = struct.pack(">H", self.pa.get(parameter, 0))
        elif command == CMD_MOVE_OUTPUT_RELATIVE:
            data = frame[9:13]
        elif command == CMD_READ_POSITION:
            data = struct.pack(">i", -12345)
        elif command == CMD_SET_MOTION_PROFILE:
            self.pa.update(zip((40, 41, 42), struct.unpack(">HHH", frame[7:13])))
        elif command == CMD_ROTATE_CONTINUOUS:
            raw = struct.unpack(">i", frame[7:11])[0]
            data = struct.pack(">h", 42 if raw > 0 else -42)
        elif command == CMD_ROTATE_KEEPALIVE:
            data = struct.pack(">h", 42)
        if self.short_data:
            data = b""
        if command in self.custom_responses:
            data = self.custom_responses[command]
        payload = bytes((frame[3], command, status)) + frame[5:7] + data
        response = append_crc(b"\xAA\x55" + bytes((len(payload) + 2,)) + payload)
        if not self.mute and command != self.mute_command:
            self.buffer.extend(self.speed_prefix + response)
        return len(frame) - 1 if self.partial_write else len(frame)

    def read(self, size):
        size = min(size, 3)  # Exercise partial-frame handling.
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def close(self):
        self.closed = True


class MotorApiTests(unittest.TestCase):
    def setUp(self):
        self.sleep_patch = patch("app.motor_controller.time.sleep", return_value=None)
        self.sleep_patch.start()
        self.addCleanup(self.sleep_patch.stop)
        self.serial = FakeSerial()
        self.factory = Mock(return_value=self.serial)
        self.controller = MotorController("TEST", timeout=0.03, serial_factory=self.factory)

    def tearDown(self):
        self.controller.close(stop=False)

    def connect(self):
        self.controller.connect()

    def test_construction_and_connect_do_not_move(self):
        self.factory.assert_not_called()
        self.connect()
        self.assertEqual(self.serial.frames, [])
        self.assertEqual(self.factory.call_args.kwargs["baudrate"], 115200)

    def test_continuous_rotation_heartbeat_and_stop(self):
        self.connect()
        receipt = self.controller.start_continuous(1, -5)
        self.assertEqual(receipt.motor_rpm, -42)
        self.assertAlmostEqual(receipt.actual_deg_per_sec, -5.04)
        self.controller._renew_continuous()
        self.assertEqual(self.serial.frames[-1][4], CMD_ROTATE_KEEPALIVE)
        self.controller.stop(1)
        count = len(self.serial.frames)
        self.controller._renew_continuous()
        self.assertEqual(len(self.serial.frames), count)
        self.assertNotIn(1, self.controller._auto_axes)

    def test_continuous_rejection_does_not_register_heartbeat(self):
        self.connect()
        self.serial.reject_command = CMD_ROTATE_CONTINUOUS
        with self.assertRaises(CommandRejected):
            self.controller.start_continuous(1, 5)
        self.assertFalse(self.controller._auto_axes)

    def test_background_heartbeat_failure_is_reported_and_never_restarts(self):
        self.connect()
        self.controller.start_continuous(1, 5)
        self.serial.reject_command = CMD_ROTATE_KEEPALIVE
        event = Mock()
        event.wait.side_effect = [False, True]
        event.is_set.return_value = False
        self.controller._heartbeat_loop(event)
        with self.assertRaises(CommandRejected):
            self.controller.check_auto_rotation()
        self.assertFalse(self.controller._auto_axes)
        self.assertEqual([f[4] for f in self.serial.frames].count(CMD_ROTATE_CONTINUOUS), 1)

    def test_status_accepts_noise_fragments_and_extra_telemetry(self):
        self.connect()
        self.serial.rpm = -10
        state = self.controller.get_status(2)
        self.assertEqual((state.motor_id, state.output_rpm, state.run_state), (2, -0.2, 3))
        self.assertEqual(state.alarm_code, 0)

    def test_reverse_stops_first_and_sends_signed_output_units(self):
        self.connect()
        receipt = self.controller.rotate_reverse(1, 5, speed_rpm=0.2)
        commands = [f[4] for f in self.serial.frames]
        self.assertEqual(commands, [CMD_STOP, CMD_READ_SPEED, CMD_READ_SPEED,
                                    CMD_READ_ALARM, CMD_MOVE_OUTPUT_RELATIVE])
        self.assertEqual(struct.unpack(">Hi", self.serial.frames[-1][7:13]), (20, -5000))
        self.assertEqual(receipt.output_angle_deg, -5)
        self.assertEqual(receipt.output_speed_rpm, 0.2)

    def test_stop_keeps_two_real_zero_reads_with_short_poll_gap(self):
        self.connect()
        with patch('app.motor_controller.time.sleep') as sleep:
            self.controller.stop(1)
        self.assertEqual([f[4] for f in self.serial.frames], [CMD_STOP,CMD_READ_SPEED,CMD_READ_SPEED])
        polling_gaps=[call.args[0] for call in sleep.call_args_list if call.args[0]!=0.005]
        self.assertEqual(polling_gaps,[0.02])  # Exclude byte-stream receive polling.

    def test_alarm_prevents_new_motion(self):
        self.connect()
        self.serial.alarm = 13
        with self.assertRaises(DriveAlarm) as caught:
            self.controller.rotate_forward(1, 5)
        self.assertEqual(caught.exception.alarm_code, 13)
        self.assertNotIn(CMD_MOVE_OUTPUT_RELATIVE, [f[4] for f in self.serial.frames])

    def test_firmware_rejection_is_not_hidden_or_retried(self):
        self.connect()
        self.serial.reject_command = CMD_MOVE_OUTPUT_RELATIVE
        with self.assertRaises(CommandRejected) as caught:
            self.controller.rotate_forward(1, 5)
        self.assertEqual(caught.exception.status, 8)
        self.assertEqual([f[4] for f in self.serial.frames].count(CMD_MOVE_OUTPUT_RELATIVE), 1)

    def test_timeout_requires_reconnect_and_never_retries(self):
        self.connect()
        self.serial.mute = True
        with self.assertRaises(CommunicationError):
            self.controller.read_alarm(1)
        count = len(self.serial.frames)
        with self.assertRaises(CommunicationError):
            self.controller.read_alarm(1)
        self.assertEqual(len(self.serial.frames), count)

    def test_motion_timeout_never_resends_relative_move(self):
        self.connect()
        self.serial.mute_command = CMD_MOVE_OUTPUT_RELATIVE
        with self.assertRaises(CommunicationError):
            self.controller.rotate_forward(1, 5)
        with self.assertRaises(CommunicationError):
            self.controller.rotate_forward(1, 5)
        self.assertEqual([f[4] for f in self.serial.frames].count(CMD_MOVE_OUTPUT_RELATIVE), 1)

    def test_unconfirmed_stop_prevents_new_move(self):
        self.connect()
        self.serial.rpm = 10
        with self.assertRaises(MotorError):
            self.controller.move_relative(1, 5, stop_timeout=0.001)
        self.assertNotIn(CMD_MOVE_OUTPUT_RELATIVE, [f[4] for f in self.serial.frames])

    def test_rs485_failure_response_marks_execution_uncertain(self):
        self.connect()
        self.serial.reject_command = CMD_MOVE_OUTPUT_RELATIVE
        self.serial.reject_status = 3
        with self.assertRaises(CommandRejected):
            self.controller.rotate_forward(1, 5)
        with self.assertRaises(CommunicationError):
            self.controller.rotate_forward(1, 5)
        self.assertEqual([f[4] for f in self.serial.frames].count(CMD_MOVE_OUTPUT_RELATIVE), 1)

    def test_bad_data_and_partial_write_are_communication_errors(self):
        self.connect()
        self.serial.short_data = True
        with self.assertRaises(CommunicationError):
            self.controller.get_status(1)
        self.controller.close(stop=False)
        self.connect()
        self.serial.short_data = False
        self.serial.partial_write = True
        with self.assertRaises(CommunicationError):
            self.controller.read_alarm(1)

    def test_position_is_signed_raw_feedback(self):
        self.connect()
        self.assertEqual(self.controller.read_position_raw(1), -12345)

    def test_profile_is_read_back(self):
        self.connect()
        result = self.controller.set_motion_profile(2, accel_ms=2000, decel_ms=1800, s_curve_ms=0)
        self.assertEqual((result.accel_ms, result.decel_ms, result.s_curve_ms), (2000, 1800, 0))

    def test_invalid_motion_has_no_serial_side_effect(self):
        self.connect()
        for device, angle, speed in ((3, 5, 0.2), (1, 0, 0.2), (1, 5, 101)):
            with self.assertRaises(ValueError):
                self.controller.move_relative(device, angle, speed_rpm=speed)
        self.assertEqual(self.serial.frames, [])

    def test_context_closes_and_stops_both_axes(self):
        with self.controller:
            pass
        stopped = [f[3] for f in self.serial.frames if f[4] == CMD_STOP]
        self.assertEqual(stopped, [1, 2])
        self.assertTrue(self.serial.closed)

    def test_no_silent_stop_failure_on_close(self):
        self.connect()
        self.serial.mute = True
        with self.assertRaises(CommunicationError):
            self.controller.close()
        self.assertTrue(self.serial.closed)


if __name__ == "__main__":
    unittest.main()
