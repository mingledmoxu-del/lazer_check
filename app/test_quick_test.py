from collections import deque
import struct
import unittest
from unittest.mock import Mock, patch

import test_limit_mode as fixture
from test_limit_mode import Value
from trajectory import quick_joint_test
from trajectory_panel import TrajectoryPanel
from protocol import (Response,CMD_READ_GUARD,CMD_READ_FEEDBACK,CMD_READ_ALARM,
    CMD_TRAJECTORY_BEGIN,CMD_TRAJECTORY_SEAL,CMD_TRAJECTORY_START,CMD_TRAJECTORY_STATUS)


class QuickPathTests(unittest.TestCase):
    def test_both_axes_start_at_current_pose_and_return(self):
        points,amplitude=quick_joint_test((25,0),((-35,35),(-90,90)))
        self.assertEqual(amplitude,2)
        self.assertEqual((points[0].axis1_deg,points[0].axis2_deg),(25,0))
        self.assertEqual((points[-1].axis1_deg,points[-1].axis2_deg),(25,0))
        self.assertEqual(points[-1].time_s,20)
        for p in points:self.assertAlmostEqual(p.axis1_deg-25,p.axis2_deg)

    def test_amplitude_shrinks_without_expanding_limits(self):
        points,amplitude=quick_joint_test((25,0),((-35,35),(-3,3)))
        self.assertEqual(amplitude,1)
        self.assertTrue(all(-1<=p.axis2_deg<=1 for p in points))

    def test_too_close_to_boundary_refuses_motion(self):
        with self.assertRaises(ValueError):quick_joint_test((34,0),((-35,35),(-90,90)))
        with self.assertRaises(ValueError):quick_joint_test((25,float("nan")),((-35,35),(-90,90)))


class QuickWorkflowTests(unittest.TestCase):
    def setUp(self):
        fixture.LimitModeTests.setUp(self)
        self.app.trajectory_running=False;self.app.trajectory_panel=None
        p=self.panel=TrajectoryPanel.__new__(TrajectoryPanel)
        p.app=self.app;p.window=None;p.jobs=deque();p.status=Value("");p.preview=Mock()
        p.quick_phase="checking";p.quick_generation=1;p.sealed=False;p.renew=False;p.blocked=False
        p.session=0;p.points=();p.revision=0;p.upload_revision=-1
        p.values={key:Value("") for key in ("center1","center2","amplitude1","amplitude2","period","cycles",
            "cycle_ms","speed","accel","error","tolerance")}
        p.quick_configs={1:(255,255,255,1,0,0,0,1,-35000,35000,0),
                         2:(255,255,255,1,0,0,0,1,-90000,90000,0)}
        p.quick_feedback={axis:(0,0,0,3,10000,21,25000 if axis==1 else 0,3,0,0,0,0,0,0) for axis in (1,2)}
        p.quick_alarms={1:0,2:0};p.error=Mock()
        self.app.trajectory_panel=p

    def status_reply(self,command,state=2,status=0):
        data=struct.pack(">HBBHHIIIIiiii",self.panel.session,state,0,33,33,0,20000,0,0,0,0,0,0)
        return Response(1,command,status,self.panel.session,data,b"")

    def test_confirm_upload_then_auto_start_only_after_seal(self):
        p=self.panel
        with patch("trajectory_panel.messagebox.askyesno",return_value=True):p._finish_quick_check()
        self.assertEqual(p.quick_phase,"uploading")
        self.assertEqual(self.app._send.call_args.args[1],CMD_TRAJECTORY_BEGIN)
        self.assertFalse(self.app.trajectory_running)
        self.assertEqual(p.points[0].axis1_deg,25)
        self.assertEqual(p.points[0].axis2_deg,0)
        p.jobs.clear();p.handle(self.status_reply(CMD_TRAJECTORY_SEAL))
        self.assertTrue(self.app.trajectory_running)
        self.assertEqual(self.app._send.call_args.args[1],CMD_TRAJECTORY_START)

    def test_missing_axis2_soft_policy_allows_referenced_axis(self):
        p=self.panel;cfg=list(p.quick_configs[2]);cfg[7]=0;p.quick_configs[2]=tuple(cfg)
        with patch("trajectory_panel.messagebox.askyesno",return_value=True):p._finish_quick_check()
        self.assertEqual(p.quick_phase,"uploading");p.error.assert_not_called()

    def test_invalid_reference_does_not_get_replaced(self):
        p=self.panel;feedback=list(p.quick_feedback[1]);feedback[7]=2;p.quick_feedback[1]=tuple(feedback)
        p._finish_quick_check()
        self.app._send.assert_not_called();self.assertIsNone(p.quick_phase)

    def test_declining_confirmation_never_uploads(self):
        with patch("trajectory_panel.messagebox.askyesno",return_value=False):self.panel._finish_quick_check()
        self.app._send.assert_not_called();self.assertIsNone(self.panel.quick_phase)

    def test_stop_during_confirmation_cannot_resume(self):
        p=self.panel
        with patch("trajectory_panel.messagebox.askyesno",side_effect=lambda *_a,**_kw:(p.stop() or True)):
            p._finish_quick_check()
        commands=[call.args[1] for call in self.app._send.call_args_list]
        self.assertEqual(commands,[0x35]);self.assertFalse(p.jobs);self.assertIsNone(p.quick_phase)

    def test_edit_during_upload_cancels_auto_start(self):
        p=self.panel
        with patch("trajectory_panel.messagebox.askyesno",return_value=True):p._finish_quick_check()
        p.jobs.clear();p.invalidate();self.app._send.reset_mock()
        p.handle(self.status_reply(CMD_TRAJECTORY_SEAL))
        self.app._send.assert_not_called();self.assertIsNone(p.quick_phase)

    def test_late_seal_after_stop_cannot_auto_start(self):
        p=self.panel;p.quick_phase="uploading";p.stop();self.app._send.reset_mock()
        p.handle(self.status_reply(CMD_TRAJECTORY_SEAL))
        self.app._send.assert_not_called()

    def test_communication_failure_clears_quick_pipeline(self):
        p=self.panel;p.jobs.append((b"", "unused"));p.on_error()
        self.assertFalse(p.jobs);self.assertIsNone(p.quick_phase);self.assertTrue(p.blocked)

    def test_existing_firmware_task_is_not_replaced(self):
        p=self.panel;p.jobs.append((b"", "unused"))
        p.handle(self.status_reply(CMD_TRAJECTORY_STATUS,state=3))
        self.assertFalse(p.jobs);self.assertIsNone(p.quick_phase);self.assertFalse(p.renew)

    def test_nonzero_rpm_or_alarm_prevents_quick_motion(self):
        p=self.panel;p.quick_alarms[2]=13
        p._finish_quick_check();self.app._send.assert_not_called()


if __name__=="__main__":unittest.main()
