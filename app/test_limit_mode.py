import struct
import unittest
from unittest.mock import Mock, patch

from main import SimpleServoApp
from protocol import CMD_LIMIT_DEBUG, CMD_READ_SPEED, Response


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class LimitModeTests(unittest.TestCase):
    def setUp(self):
        self.app = SimpleServoApp.__new__(SimpleServoApp)
        a = self.app
        a.LIMIT_FEATURES_ENABLED = True  # 保留的限位实现回归测试；正式界面默认停用。
        a.root = Mock()
        a._log = Mock()
        a._send = Mock(return_value=True)
        a.motor_id = Value(1)
        a.limit_debug = Value(False)
        a.limit_mode_text = Value("")
        a.result_text = Value("")
        a.limit_modes = {1: None, 2: None}
        a.limit_renew = {1: False, 2: False}
        a.limit_renew_at = {1: 0.0, 2: 0.0}
        a.limit_action = None
        a.pending = None
        a.queued_motion = None
        a.parameter_check_queue = None
        a.profile_read_queue = None
        a.stop_waiting = False
        a.diagnostic_active = False
        a.monitor_until = 0.0
        a.motion_seen_running = False
        a.last_drive_status = ""
        a.port = Mock()
        a.auto_motor = None
        a.auto_renew = False
        a.queued_auto = None
        a.motor_buttons = []
        a.debug_panel = None
        a.home_running = False
        a.active_motor = None

    def reply(self, command, data, status=0, device=1):
        self.app.pending = (device, command, 0.0)
        self.app._handle_response(Response(device, command, status, 0, data, b""))

    def test_enable_requires_confirmation_and_ack(self):
        a = self.app
        a.limit_debug.set(True)
        with patch("main.messagebox.askyesno", return_value=True):
            a._toggle_limit_debug()
        self.assertFalse(a.limit_debug.get())
        self.assertFalse(a.limit_renew[1])
        self.assertEqual(a.limit_action, (1, 1))
        self.reply(CMD_LIMIT_DEBUG, b"\x00\x01")
        self.assertTrue(a.limit_debug.get())
        self.assertTrue(a.limit_renew[1])
        self.assertFalse(a.limit_renew[2])

    def test_cancel_does_not_send(self):
        self.app.limit_debug.set(True)
        with patch("main.messagebox.askyesno", return_value=False):
            self.app._toggle_limit_debug()
        self.app._send.assert_not_called()
        self.assertFalse(self.app.limit_debug.get())

    def test_default_feature_is_disabled(self):
        self.assertFalse(SimpleServoApp.LIMIT_FEATURES_ENABLED)
        self.app.LIMIT_FEATURES_ENABLED = False
        self.app.limit_debug.set(True)
        self.app.limit_renew[1] = True
        self.app._toggle_limit_debug()
        self.app._service_limit_debug()
        self.app._send.assert_not_called()

    def test_disabled_ui_ignores_extra_limit_telemetry(self):
        self.app.LIMIT_FEATURES_ENABLED = False
        self.app.queued_motion = (1, 1, object())
        self.reply(CMD_READ_SPEED, struct.pack(">hHB", 0, 3, 1))
        self.assertIsNotNone(self.app.queued_motion)
        self.assertNotIn("限位", self.app.result_text.get())

    def test_old_firmware_never_shows_bypass_enabled(self):
        self.app.limit_action = (1, 1)
        self.reply(CMD_LIMIT_DEBUG, b"\x00\x00", status=7)
        self.assertFalse(self.app.limit_debug.get())
        self.assertFalse(self.app.limit_renew[1])
        self.assertIn("固件", self.app.result_text.get())

    def test_bypassed_input_does_not_cancel_queued_motion(self):
        self.app.queued_motion = (1, 1, object())
        self.reply(CMD_READ_SPEED, struct.pack(">hHB", 0, 3, 7))
        self.assertIsNotNone(self.app.queued_motion)
        self.assertIn("旁路", self.app.result_text.get())
        self.assertTrue(self.app.limit_debug.get())
        self.assertFalse(self.app.limit_renew[1])  # Never adopt an external lease.

    def test_protected_input_cancels_queued_motion(self):
        self.app.queued_motion = (1, 1, object())
        self.reply(CMD_READ_SPEED, struct.pack(">hHB", 0, 3, 1))
        self.assertIsNone(self.app.queued_motion)
        self.assertFalse(self.app.limit_debug.get())

    def test_expired_renewal_does_not_reenable(self):
        self.app.limit_renew[1] = True
        self.app.limit_action = (1, 2)
        self.reply(CMD_LIMIT_DEBUG, b"\x00\x00", status=6)
        self.app._service_limit_debug()
        self.app._send.assert_not_called()
        self.assertFalse(self.app.limit_renew[1])

    def test_renewal_targets_original_axis(self):
        self.app.motor_id.set(2)
        self.app.limit_renew[1] = True
        self.app._service_limit_debug()
        self.assertEqual(self.app.limit_action, (1, 2))
        self.assertEqual(self.app._send.call_args.args[0][3], 1)


if __name__ == "__main__":
    unittest.main()
