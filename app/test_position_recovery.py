import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from app import MotorController, MotorError, TrajectoryPoint
from app.protocol import build_trajectory_begin, CMD_TRAJECTORY_START, CMD_TRAJECTORY_SEAL
from app.trajectory import AxisPosition, decode_axes, return_to_start_points
import test_quick_test as quick_fixture
from test_limit_mode import Value
from protocol import CMD_TRAJECTORY_CLEAR, CMD_TRAJECTORY_STATUS


def position(angle, **kw):
    values=dict(angle_deg=angle,actual_counts=0,motor_rpm=0,flags=3,
                sample_age_ms=0,alarm_code=0,guard_fault=0,source=0)
    values.update(kw)
    return AxisPosition(**values)


class PositionRecoveryTests(unittest.TestCase):
    def test_position_age_validity_and_signed_values(self):
        raw=struct.pack(">iihHIHBB",-12345,-123,-8,3,40,0,0,1)
        raw+=struct.pack(">iihHIHBB",0,500,0,2,0xFFFFFFFF,0,0,2)
        a,b=decode_axes(raw)
        self.assertEqual(a.angle_deg,-12.345);self.assertEqual(a.motor_rpm,-8)
        self.assertTrue(a.usable);self.assertIsNone(b.angle_deg);self.assertFalse(b.usable)
        self.assertFalse(position(0,sample_age_ms=1001).usable)
        with self.assertRaises(ValueError):decode_axes(raw[:-1])

    def test_adaptive_wire_flag_and_legacy_default(self):
        adaptive=build_trajectory_begin(1,3,cycle_ms=100,adaptive=True)
        self.assertEqual(struct.unpack_from(">H",adaptive,9)[0],0x8000|100)
        self.assertEqual(struct.unpack_from(">H",build_trajectory_begin(1,3),9)[0],250)

    def test_recovery_smooth_path_within_limits(self):
        points=return_to_start_points((10,-5),(25,0),((-35,35),(-90,90)))
        self.assertEqual(points[0],TrajectoryPoint(0,10,-5))
        self.assertEqual((points[-1].axis1_deg,points[-1].axis2_deg),(25,0))
        velocities=[(b.axis1_deg-a.axis1_deg)/(b.time_s-a.time_s) for a,b in zip(points,points[1:])]
        self.assertLessEqual(max(velocities),1.5)
        self.assertLess(velocities[0],velocities[7]);self.assertLess(velocities[-1],velocities[7])
        with self.assertRaises(ValueError):return_to_start_points((35,0),(25,0),((-35,35),(-90,90)))

    def controller(self,positions):
        api=MotorController("unused")
        api.get_axis_positions=Mock(return_value=positions)
        api.stop_trajectory=Mock()
        api.get_guard_config=Mock(return_value=SimpleNamespace(minimum_deg=-35,maximum_deg=35,soft_enabled=True))
        api.prepare_trajectory=Mock(return_value="ready")
        return api

    def test_sdk_recovery_only_preloads(self):
        api=self.controller((position(20),position(5)))
        original=(TrajectoryPoint(0,25,0),TrajectoryPoint(20,25,0))
        self.assertEqual(api.prepare_return_to_start(original),"ready")
        api.stop_trajectory.assert_called_once()
        points=api.prepare_trajectory.call_args.args[0]
        self.assertEqual((points[0].axis1_deg,points[0].axis2_deg),(20,5))
        self.assertTrue(api.prepare_trajectory.call_args.kwargs["adaptive"])
        self.assertIsNone(api._trajectory_session)

    def test_sdk_recovery_refuses_invalid_stale_moving_or_cached_positions(self):
        for bad in (position(None),position(20,sample_age_ms=1001),position(20,motor_rpm=1),
                    position(20,alarm_code=3),position(20,guard_fault=1),position(20,source=1),position(20,flags=1)):
            api=self.controller((bad,position(0)))
            with self.assertRaises(MotorError):api.prepare_return_to_start((TrajectoryPoint(0,25,0),TrajectoryPoint(20,25,0)))
            api.prepare_trajectory.assert_not_called()

    def test_daily_sdk_accepts_axis2_reference_without_bounds_and_only_preloads(self):
        api=self.controller((position(0),position(150,flags=1)))
        api.get_trajectory_status=Mock(return_value=SimpleNamespace(active=False))
        points=(TrajectoryPoint(0,0,150),TrajectoryPoint(1,10,160),TrajectoryPoint(2,0,150))
        self.assertEqual(api.prepare_business_trajectory(points),"ready")
        kwargs=api.prepare_trajectory.call_args.kwargs
        self.assertTrue(kwargs["adaptive"]);self.assertEqual(kwargs["cycle_ms"],100)
        self.assertGreater(api.prepare_trajectory.call_args.args[0][-1].time_s,2)

    def test_daily_sdk_requires_horizontal_and_matching_first_point(self):
        api=self.controller((position(25),position(0,flags=1)))
        api.get_trajectory_status=Mock(return_value=SimpleNamespace(active=False))
        with self.assertRaises(MotorError):api.prepare_business_trajectory((TrajectoryPoint(0,25,0),TrajectoryPoint(20,25,0)))
        api.prepare_trajectory.assert_not_called()

    def test_horizontal_sdk_uses_reference_and_confirms_two_zero_samples(self):
        api=self.controller((position(25),position(0,flags=1)))
        api.get_axis_positions.side_effect=[(position(25),position(0)),(position(0),position(0)),(position(0),position(0))]
        api.move_to_angle=Mock();api.set_current_reference=Mock()
        with patch("app.motor_controller.time.sleep"):
            result=api.move_axis1_to_horizontal()
        self.assertEqual(result.angle_deg,0)
        api.move_to_angle.assert_called_once_with(1,0,speed_rpm=0.1)
        api.set_current_reference.assert_not_called()

    def test_daily_acceleration_override_is_forwarded_without_driver_parameter_writes(self):
        api=self.controller((position(0),position(0,flags=1)))
        api.get_trajectory_status=Mock(return_value=SimpleNamespace(active=False))
        api.set_motion_profile=Mock()
        points=(TrajectoryPoint(0,0,0),TrajectoryPoint(1,2,2),TrajectoryPoint(2,0,0))
        api.prepare_business_trajectory(points,max_speed=2,max_accel=5)
        self.assertEqual(api.prepare_trajectory.call_args.kwargs["max_accel"],5)
        self.assertEqual(api.prepare_trajectory.call_args.kwargs["max_speed"],2)
        api.set_motion_profile.assert_not_called()
        api.get_axis_positions.reset_mock()
        with self.assertRaises(ValueError):api.prepare_business_trajectory(points,max_accel=121)
        api.get_axis_positions.assert_not_called()


class RecoveryWorkflowTests(unittest.TestCase):
    def setUp(self):
        quick_fixture.QuickWorkflowTests.setUp(self)
        p=self.panel;p.quick_phase="return_check";p.return_revision=p.revision
        p.return_original=(TrajectoryPoint(0,25,0),TrajectoryPoint(20,25,0))
        p.return_positions=(position(20),position(5))

    def test_return_seal_never_auto_starts_and_original_is_preserved(self):
        p=self.panel
        with patch("trajectory_panel.messagebox.askyesno",return_value=True):p._finish_return_check()
        self.assertEqual(p.quick_phase,"return_uploading");self.assertEqual(p.saved_points,p.return_original)
        self.assertFalse(self.app.trajectory_running)
        p.jobs.clear()
        p.handle(quick_fixture.QuickWorkflowTests.status_reply(self,CMD_TRAJECTORY_SEAL))
        self.assertTrue(p.sealed);self.assertIsNone(p.quick_phase)
        self.assertNotEqual(self.app._send.call_args.args[1],CMD_TRAJECTORY_START)

    def test_cancel_dialog_never_uploads(self):
        with patch("trajectory_panel.messagebox.askyesno",return_value=False):self.panel._finish_return_check()
        self.app._send.assert_not_called();self.assertIsNone(self.panel.quick_phase)

    def test_stop_during_confirmation_cancels_return(self):
        with patch("trajectory_panel.messagebox.askyesno",side_effect=lambda *_a,**_k:(self.panel.stop() or True)):
            self.panel._finish_return_check()
        self.assertIsNone(self.panel.quick_phase);self.assertFalse(self.panel.jobs)
        self.assertFalse(self.panel.sealed)

    def test_clear_uses_observed_fault_session_and_never_starts(self):
        p=self.panel;p.quick_phase=None;p.ready=Mock(return_value=True);p.session=41
        with patch("trajectory_panel.messagebox.askyesno",return_value=True):p.clear_fault()
        self.assertEqual(self.app._send.call_args.args[1],CMD_TRAJECTORY_STATUS)
        p.handle(quick_fixture.QuickWorkflowTests.status_reply(self,CMD_TRAJECTORY_STATUS,state=7))
        p.service()
        frame=self.app._send.call_args.args[0]
        self.assertEqual(frame[4],CMD_TRAJECTORY_CLEAR)
        self.assertEqual(struct.unpack_from(">H",frame,5)[0],41)
        p.handle(quick_fixture.QuickWorkflowTests.status_reply(self,CMD_TRAJECTORY_CLEAR,state=0))
        self.assertFalse(p.sealed);self.assertIsNone(p.quick_phase);self.assertEqual(p.session,0)
        self.assertNotIn(CMD_TRAJECTORY_START,[c.args[1] for c in self.app._send.call_args_list])

    def test_collapsing_advanced_editor_keeps_acceleration_override(self):
        p=self.panel;p.quick_phase=None;p.ready=Mock(return_value=True)
        p.points=(TrajectoryPoint(0,0,0),TrajectoryPoint(1,2,2),TrajectoryPoint(2,0,0))
        p.show_advanced=Value(False);p.adaptive=Value(True)
        for key,value in {"cycle_ms":100,"speed":2,"accel":5,"error":0.75,"tolerance":0.3}.items():
            p.values[key].set(str(value))
        p.upload()
        frame=self.app._send.call_args.args[0]
        self.assertEqual(struct.unpack(">HHiiii",frame[7:-2])[3],5000)


if __name__ == "__main__":unittest.main()
