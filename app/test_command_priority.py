import unittest
import struct
from types import SimpleNamespace
from unittest.mock import Mock
from unittest.mock import patch

import test_limit_mode as fixture
from main import SimpleServoApp
from protocol import CMD_READ_FEEDBACK, CMD_READ_SPEED, CMD_READ_PA, CMD_STOP, Response, build_request


class CommandPriorityTests(unittest.TestCase):
    def test_stop_invalidates_old_callback_and_reverse_waits_for_two_zeros(self):
        a=self.app;callbacks=[]
        a.root.after.side_effect=lambda _delay,callback:callbacks.append(callback)
        a._motion_values=Mock(return_value=SimpleNamespace(output_angle_deg=5,output_speed_rpm=.5,
                                                         motor_angle_deg=250,motor_speed_rpm=25))
        a.queued_motion=(1,1,a._motion_values())
        a._schedule_motion_callback(80,a._send_queued_motion);old=callbacks[-1]
        a._stop();a._move(-1)
        before=a.port.write.call_count;old();self.assertEqual(a.port.write.call_count,before)
        def reply(op,data):
            a.pending=(1,op,0);a.pending_parameter=0
            a._handle_response(Response(1,op,0,0,data,b""))
        reply(CMD_READ_SPEED,struct.pack('>hH',0,3));self.assertTrue(a.stop_waiting)
        reply(CMD_READ_SPEED,struct.pack('>hH',0,3));self.assertFalse(a.stop_waiting)
        callbacks[-1]() # current alarm query, not an old motion
        reply(0x09,b'\0\0');callbacks[-1]()
        frame=a.port.write.call_args.args[0]
        self.assertEqual(frame[4],0x19)
        self.assertEqual(struct.unpack('>i',frame[9:13])[0],-5000)
    def setUp(self):
        fixture.LimitModeTests.setUp(self)
        a = self.app
        del a._send  # Exercise the real dispatcher, not the fixture mock.
        a.pending_background = False
        a.deferred_send = None

    def background(self):
        a = self.app
        a._send(build_request(1, CMD_READ_FEEDBACK), CMD_READ_FEEDBACK, "refresh", background=True)
        a.port.write.reset_mock()

    def test_foreground_waits_for_query_not_for_motor(self):
        a = self.app
        self.background()
        frame = build_request(1, CMD_READ_SPEED)
        with patch("main.messagebox.showwarning") as warning:
            self.assertTrue(a._send(frame, CMD_READ_SPEED, "operator"))
            warning.assert_not_called()
        a.port.write.assert_not_called()
        a._handle_response(Response(1, CMD_READ_FEEDBACK, 0, 0, b"", b""))
        a._service_foreground()
        a.port.write.assert_called_once_with(frame)
        self.assertFalse(a.pending_background)

    def test_stop_discards_deferred_command_and_late_reply(self):
        a = self.app
        self.background()
        a._send(build_request(1, CMD_READ_SPEED), CMD_READ_SPEED, "operator")
        a._stop()
        self.assertIsNone(a.deferred_send)
        self.assertEqual(a.pending[1], CMD_STOP)
        a._handle_response(Response(1, CMD_READ_FEEDBACK, 0, 0, b"", b""))
        self.assertEqual(a.pending[1], CMD_STOP)
        self.assertIsNone(a.deferred_send)

    def test_failed_query_cancels_deferred_operation(self):
        a = self.app
        self.background()
        a._send(build_request(1, CMD_READ_PA), CMD_READ_PA, "operator")
        a._handle_response(Response(1, CMD_READ_FEEDBACK, 3, 0, b"", b""))
        self.assertIsNone(a.deferred_send)
        a._service_foreground()
        a.port.write.assert_not_called()

    def test_foreground_cannot_be_overwritten_by_background(self):
        a = self.app
        a._send(build_request(1, CMD_READ_SPEED), CMD_READ_SPEED, "operator")
        a.port.write.reset_mock()
        with patch("main.messagebox.showwarning") as warning:
            self.assertFalse(a._send(build_request(1, CMD_READ_FEEDBACK), CMD_READ_FEEDBACK, "refresh", background=True))
            warning.assert_not_called()
        a.port.write.assert_not_called()

    def test_query_timeout_drops_waiting_motion(self):
        a = self.app
        self.background()
        a.pending = (1, CMD_READ_FEEDBACK, 0.0)
        a._send(build_request(1, CMD_READ_SPEED), CMD_READ_SPEED, "operator")
        a.queued_motion = (1, 1, object())
        a.port.in_waiting = 0
        a._service_auto = lambda: None
        a._service_limit_debug = lambda: None
        a._show_limit_mode = lambda: None
        with patch("main.time.monotonic", return_value=10.0), patch.object(a,"_disconnect") as disconnect:
            a._poll_serial()
            disconnect.assert_not_called()
        self.assertIsNone(a.deferred_send)
        self.assertIsNone(a.queued_motion)
        self.assertFalse(a.diagnostic_active)
        a.port.write.assert_not_called()

    def test_wrong_session_reply_cannot_complete_new_request(self):
        a=self.app
        a._send(build_request(1,0x34,parameter=17),0x34,"session status")
        a._handle_response(Response(1,0x34,0,18,b"",b""))
        self.assertIsNotNone(a.pending)
        a._handle_response(Response(1,0x34,0,17,b"",b""))
        self.assertIsNone(a.pending)

    def test_axis_snapshot_allowed_while_trajectory_active(self):
        a=self.app;a.trajectory_running=True
        frame=build_request(1,0x37)
        with patch("main.messagebox.showwarning") as warning:
            self.assertTrue(a._send(frame,0x37,"positions",background=True))
            warning.assert_not_called()
        a.port.write.assert_called_once_with(frame)

    def test_startup_diagnostic_is_read_only_and_keeps_axis2_selected(self):
        a=self.app;a.parameter_values={}
        callbacks=[]
        a.root.after.side_effect=lambda _delay,callback:callbacks.append(callback)
        a._check_parameters(2)
        values={4:3,14:3,22:0,24:100,25:500,26:1000,27:2000,53:0,72:1152,73:0}
        a.motor_id.set(1)
        for number,value in values.items():
            self.assertEqual(a.pending_parameter,number)
            a._handle_response(Response(2,CMD_READ_PA,0,number,value.to_bytes(2,"big"),b""))
            # Let only the callback scheduled by the real response handler
            # advance the query. A missing PA22 handler must fail this test.
            self.assertEqual(len(callbacks),1)
            callbacks.pop(0)()
        self.assertIsNone(a.parameter_check_queue)
        frames=[call.args[0] for call in a.port.write.call_args_list]
        self.assertEqual(len(frames),10)
        self.assertTrue(all(frame[3]==2 and frame[4]==CMD_READ_PA for frame in frames))
        self.assertIn("上电速度",a.result_text.get())
        self.assertEqual(a.parameter_values,values)


if __name__ == "__main__":
    unittest.main()
