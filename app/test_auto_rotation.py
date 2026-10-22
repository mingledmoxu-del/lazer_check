import struct
import unittest
from unittest.mock import patch

import test_limit_mode as fixture
from test_limit_mode import Value
from protocol import CMD_STOP, CMD_READ_SPEED, CMD_ROTATE_CONTINUOUS, CMD_ROTATE_KEEPALIVE


class AutoRotationTests(unittest.TestCase):
    reply = fixture.LimitModeTests.reply

    def setUp(self):
        fixture.LimitModeTests.setUp(self)
        a = self.app
        a.LIMIT_FEATURES_ENABLED = False
        a.auto_speed = Value(5.0)
        a.auto_direction = Value(1)
        a.auto_renew_at = 0.0
        a.next_monitor_seconds = 5.0

    def test_start_first_requests_stop_not_motion(self):
        with patch("main.messagebox.askyesno", return_value=True):
            self.app._start_auto()
        self.assertEqual(self.app._send.call_args.args[1], CMD_STOP)
        self.assertEqual(self.app.queued_auto, (1, 5.0))
        self.assertFalse(self.app.auto_renew)

    def test_stop_cancels_delayed_start_and_targets_running_axis(self):
        a = self.app
        a.auto_motor = 1
        a.motor_id.set(2)
        a.queued_auto = (1, 5.0)
        a.auto_renew = True
        a._stop()
        self.assertEqual(a._send.call_args.args[0][3], 1)
        self.assertIsNone(a.queued_auto)
        self.assertFalse(a.auto_renew)
        before = a._send.call_count
        a._send_queued_auto()
        a._service_auto()
        self.assertEqual(a._send.call_count, before)

    def test_ack_starts_monitoring_but_zero_speed_is_not_completion(self):
        self.app.auto_motor = 1
        self.reply(CMD_ROTATE_CONTINUOUS, struct.pack(">h", 42))
        self.assertTrue(self.app.auto_renew)
        self.assertIn("5.04", self.app.result_text.get())
        self.reply(CMD_READ_SPEED, struct.pack(">hH", 0, 3))
        self.assertEqual(self.app.monitor_until, float("inf"))

    def test_stop_confirmation_releases_axis(self):
        self.app.auto_motor = 1
        self.app.stop_waiting = True
        self.reply(CMD_READ_SPEED, struct.pack(">hH", 0, 3))
        self.assertTrue(self.app.stop_waiting)
        self.reply(CMD_READ_SPEED, struct.pack(">hH", 0, 3))
        self.assertIsNone(self.app.auto_motor)
        self.assertFalse(self.app.stop_waiting)

    def test_confirmed_stop_clears_all_homing_and_position_busy_flags(self):
        a=self.app
        a.auto_motor=1;a.active_motor=1;a.home_running=True;a.auto_renew=True
        a.monitor_until=float("inf");a.stop_waiting=True;a.diagnostic_active=True
        self.reply(CMD_READ_SPEED,struct.pack(">hH",0,3))
        self.assertTrue(a.stop_waiting)
        self.reply(CMD_READ_SPEED,struct.pack(">hH",0,3))
        self.assertIsNone(a.auto_motor);self.assertIsNone(a.active_motor)
        self.assertFalse(a.home_running);self.assertFalse(a.auto_renew)
        self.assertFalse(a.stop_waiting);self.assertFalse(a.diagnostic_active)
        self.assertEqual(a.monitor_until,0)

    def test_readback_jobs_do_not_block_next_horizontal_action(self):
        from debug_panel import DebugPanel
        from protocol import CMD_READ_GUARD,build_request
        from collections import deque
        a=self.app;a.pending=(1,CMD_READ_GUARD,0);a.pending_background=True
        a.deferred_send=None;a.trajectory_running=False
        p=DebugPanel.__new__(DebugPanel);p.app=a;p.window=None;p.after_stop=None
        p.jobs=deque([(build_request(1,CMD_READ_GUARD),"readback")])
        self.assertTrue(p.ready(1));self.assertFalse(p.jobs)

    def test_nonzero_stop_feedback_keeps_busy_state(self):
        a=self.app;a.auto_motor=1;a.stop_waiting=True
        self.reply(CMD_READ_SPEED,struct.pack(">hH",5,3))
        self.assertTrue(a.stop_waiting);self.assertEqual(a.auto_motor,1)

    def test_fresh_stopped_feedback_releases_stale_busy_owner(self):
        from debug_panel import DebugPanel
        from collections import deque
        a=self.app;a.active_motor=1;a.monitor_until=float('inf');a.diagnostic_active=True
        a.last_motion_sent_at=8;a.deferred_send=None;a.trajectory_running=False
        p=DebugPanel.__new__(DebugPanel);p.app=a;p.window=None;p.jobs=deque();p.after_stop=None
        p.raised_request=None;p.last_feedback=(9.8,0,7,0,0)
        with patch('debug_panel.time.monotonic',return_value=10):self.assertTrue(p.ready(1))
        self.assertIsNone(a.active_motor);self.assertFalse(a.diagnostic_active)

    def test_old_feedback_cannot_clear_new_motion_owner(self):
        from debug_panel import DebugPanel
        from collections import deque
        a=self.app;a.active_motor=1;a.monitor_until=float('inf');a.last_motion_sent_at=10
        a.deferred_send=None;a.trajectory_running=False
        p=DebugPanel.__new__(DebugPanel);p.app=a;p.window=None;p.jobs=deque();p.after_stop=None
        p.raised_request=None;p.last_feedback=(9.8,0,7,0,0)
        with patch('debug_panel.time.monotonic',return_value=10.1),patch('debug_panel.messagebox.showwarning'):
            self.assertFalse(p.ready(1))
        self.assertEqual(a.active_motor,1)

    def test_expired_heartbeat_does_not_restart(self):
        self.app.auto_motor = 1
        self.app.auto_renew = True
        self.reply(CMD_ROTATE_KEEPALIVE, b"\0\0", status=6)
        self.app._service_auto()
        self.assertFalse(self.app.auto_renew)
        self.app._send.assert_not_called()

    def test_stop_timeout_cancels_queued_auto(self):
        self.app.stop_waiting = True
        self.app.queued_auto = (1, 5.0)
        self.app.monitor_until = 0.0
        self.app._monitor_motion()
        self.assertIsNone(self.app.queued_auto)

    def test_old_firmware_failure_is_visible(self):
        self.app.auto_motor = 1
        self.reply(CMD_ROTATE_CONTINUOUS, b"\0\0", status=7)
        self.assertIn("固件", self.app.result_text.get())
        self.assertFalse(self.app.auto_renew)


if __name__ == "__main__":
    unittest.main()
