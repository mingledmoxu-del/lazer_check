import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from business_panel import BusinessPanel, business_ready
from test_limit_mode import Value
from protocol import (Response, CMD_STOP, CMD_READ_AXES, CMD_SET_ZERO, CMD_MOVE_OUTPUT_RELATIVE)
from trajectory import decode_axes, business_trajectory, business_loop, TrajectoryPoint, quick_joint_test


def snapshot(angle1=25, angle2=0, rpm1=0, flags1=3, flags2=1):
    return b"".join(struct.pack(">iihHIHBB",round(angle*1000),0,rpm,flags,0,0,0,0)
                    for angle,rpm,flags in ((angle1,rpm1,flags1),(angle2,0,flags2)))


class BusinessFlowTests(unittest.TestCase):
    def setUp(self):
        a=self.app=SimpleNamespace(port=Mock(),pending=None,deferred_send=None,trajectory_running=False,
            home_running=False,auto_motor=None,active_motor=None,stop_waiting=False,diagnostic_active=False,
            parameter_check_queue=None,profile_read_queue=None,trajectory_panel=None,debug_panel=None,root=None,
            _send=Mock(return_value=True),_lock_auto_axis=Mock(),_open_trajectory=Mock(),reset_speed_rpm=Value("0.6"))
        p=self.panel=BusinessPanel.__new__(BusinessPanel)
        p.app=a;p.phase=None;p.positions=None;p.received_at=0;p.next_read=0;p.next_frame=None
        p.axis=1;p.zeros=0;p.horizontal_samples=0;p.generation=0
        p.message=Value("");p.position_text=Value("");p.readiness=Value("")

    def reply(self,command,data=b"\0\0",axis=1,status=0):
        return self.panel.handle(Response(axis,command,status,0,data,b""))

    def positions(self,**kw):self.reply(CMD_READ_AXES,snapshot(**kw))

    def test_reference_confirmation_stops_then_records_but_never_moves(self):
        p=self.panel
        with patch("business_panel.messagebox.askyesno",return_value=True):p.confirm_reference(2)
        p.service();self.reply(CMD_STOP,axis=2)
        self.positions(flags2=0);self.assertIsNone(p.next_frame)
        self.positions(flags2=0);self.assertEqual(p.next_frame[4],CMD_SET_ZERO)
        p.service();self.reply(CMD_SET_ZERO,axis=2);self.positions()
        self.assertIsNone(p.phase);self.assertIn("+0",p.message.get())
        self.assertNotIn(CMD_MOVE_OUTPUT_RELATIVE,[call.args[1] for call in self.app._send.call_args_list])

    def test_horizontal_uses_readback_coordinate_then_waits_for_actual_stop(self):
        p=self.panel;p.begin(1,"horizontal");p.service();self.reply(CMD_STOP)
        self.positions();self.positions()
        self.assertEqual(struct.unpack(">Hi",p.next_frame[7:-2]),(10,-25000))
        p.service();self.reply(CMD_MOVE_OUTPUT_RELATIVE)
        self.positions(angle1=0,rpm1=1);self.assertEqual(p.phase,"moving")
        self.positions(angle1=0);self.assertEqual(p.phase,"moving")
        self.positions(angle1=0);self.assertIsNone(p.phase)
        self.assertTrue(business_ready(p.positions));self.assertGreaterEqual(p.horizontal_samples,2)

    def test_missing_reference_never_emits_move(self):
        p=self.panel;p.begin(1,"horizontal");p.service();self.reply(CMD_STOP)
        self.positions(flags1=2);self.positions(flags1=2)
        self.assertIsNone(p.phase);self.assertIsNone(p.next_frame)
        self.assertNotIn(CMD_MOVE_OUTPUT_RELATIVE,[call.args[1] for call in self.app._send.call_args_list])

    def test_cancel_during_modal_prevents_delayed_reference(self):
        p=self.panel
        with patch("business_panel.messagebox.askyesno",side_effect=lambda *_a,**_k:(p.cancel() or True)):
            p.confirm_reference(2)
        self.assertIsNone(p.phase);self.assertIsNone(p.next_frame);self.app._send.assert_not_called()

    def test_old_axis1_manual_reference_is_disabled(self):
        with patch("business_panel.messagebox.askyesno") as confirm:
            self.panel.confirm_reference(1)
        confirm.assert_not_called();self.assertIsNone(self.panel.phase)
        self.app._send.assert_not_called()

    def test_lost_move_ack_requests_stop_without_replaying_motion(self):
        p=self.panel;p.begin(1,"horizontal");p.phase="commanding_move";p.next_frame=b"queued"
        p.fail("lost reply")
        self.assertIsNone(p.phase);self.assertIsNone(p.next_frame)
        self.assertEqual(self.app._send.call_args.args[1],CMD_STOP)
        self.positions(angle1=0);self.assertIsNone(p.next_frame)

    def test_position_expiration_clears_readiness(self):
        self.positions(angle1=0);self.positions(angle1=0)
        self.assertTrue(business_ready(self.panel.positions))
        self.panel.received_at=1
        with patch("business_panel.time.monotonic",return_value=5):self.panel.service()
        self.assertEqual(self.panel.horizontal_samples,0);self.assertIn("未更新",self.panel.readiness.get())

    def test_axis2_no_bounds_still_needs_reference(self):
        self.assertTrue(business_ready(decode_axes(snapshot(angle1=0,flags2=1))))
        self.assertFalse(business_ready(decode_axes(snapshot(angle1=0,flags2=0))))
        points,amplitude=quick_joint_test((0,150),((-35,35),None))
        self.assertEqual(amplitude,2);self.assertEqual(points[0].axis2_deg,150)

    def test_business_preset_preserves_angles_and_slows_time(self):
        original=(TrajectoryPoint(0,0,0),TrajectoryPoint(1,15,20),TrajectoryPoint(2,0,0))
        points=business_trajectory(original)
        self.assertGreater(points[-1].time_s,original[-1].time_s)
        self.assertEqual([(p.axis1_deg,p.axis2_deg) for p in points],[(p.axis1_deg,p.axis2_deg) for p in original])
        for a,b in zip(points,points[1:]):
            self.assertLessEqual(abs(b.axis2_deg-a.axis2_deg)/(b.time_s-a.time_s),1.5)

    def test_daily_loop_starts_and_returns_to_actual_pose(self):
        points=business_loop((0,150))
        self.assertEqual((points[0].axis1_deg,points[0].axis2_deg),(0,150))
        self.assertEqual((points[-1].axis1_deg,points[-1].axis2_deg),(0,150))
        self.assertTrue(all(-5<=p.axis1_deg<=5 for p in points))

    def test_acceleration_override_changes_retiming(self):
        points=(TrajectoryPoint(0,0,0),TrajectoryPoint(1,2,0),TrajectoryPoint(2,0,0))
        slow=business_trajectory(points,max_speed=10,max_accel=3)
        fast=business_trajectory(points,max_speed=10,max_accel=5)
        self.assertGreater(slow[-1].time_s,fast[-1].time_s)


if __name__=="__main__":unittest.main()
