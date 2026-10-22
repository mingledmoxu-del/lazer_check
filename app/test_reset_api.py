import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from app import MotorController, MotorError
from app.test_motor_controller import FakeSerial
from app.protocol import CMD_READ_DUAL_LIMIT,CMD_HOME_DUAL_LIMIT


def feedback(angle=None, fault=0):
    return SimpleNamespace(output_angle_deg=angle,fault=fault,motor_rpm=0,flags=7 if angle is not None else 2)


class ResetApiTests(unittest.TestCase):
    def test_explicit_clear_reference_stops_and_configures_without_motion(self):
        c=MotorController('TEST');c.configure_axis1_limits=Mock();c.get_feedback=Mock(return_value=feedback())
        c.start_axis1_limit_homing=Mock();c.move_to_angle=Mock()
        result=c.clear_axis1_reference()
        self.assertIsNone(result.output_angle_deg)
        c.configure_axis1_limits.assert_called_once_with(active_high=False,positive_is_up=False,horizontal_deg=-29,raised_deg=35)
        c.start_axis1_limit_homing.assert_not_called();c.move_to_angle.assert_not_called()
    def test_first_reset_speed_is_output_rpm_converted_to_wire_deg_s(self):
        for speed in (None,0.2,1.5):
            serial=FakeSerial()
            serial.custom_responses[CMD_READ_DUAL_LIMIT]=struct.pack('>BBBBbBiiBB',1,2,0,0,-1,3,-29000,35000,0,0)
            c=MotorController('TEST',timeout=.1,serial_factory=lambda *_a,**_k:serial).connect()
            c.stop=Mock();c.get_feedback=Mock(return_value=feedback());c.wait_homing=Mock(return_value=feedback(35))
            try:
                with patch('app.motor_controller.threading.Thread'):
                    result=c.reset_axis1(**({} if speed is None else {'speed_rpm':speed}))
                f=next(f for f in serial.frames if f[4]==CMD_HOME_DUAL_LIMIT)
                self.assertEqual(struct.unpack('>biii',f[7:-2]),(-1,round((.6 if speed is None else speed)*1000),45000,120000))
                self.assertEqual(result.output_angle_deg,35)
            finally:c.close(stop=False)

    def test_existing_reference_never_reconfigures_or_homes(self):
        c=MotorController('TEST');c.stop=Mock();c.get_feedback=Mock(return_value=feedback(12))
        c.configure_axis1_limits=Mock();c.start_axis1_limit_homing=Mock()
        c._move_to_angle_stopped=Mock();c.wait_for_angle=Mock(return_value=feedback(35))
        self.assertEqual(c.reset_axis1(speed_rpm=1.2).output_angle_deg,35)
        c._move_to_angle_stopped.assert_called_once_with(1,35,1.2/6,c.get_feedback.return_value)
        c.stop.assert_called_once_with(1)
        c.configure_axis1_limits.assert_not_called();c.start_axis1_limit_homing.assert_not_called()

    def test_axis2_reset_moves_and_confirm_zero_only_records_reference(self):
        c=MotorController('TEST');c.move_to_angle=Mock();c.wait_for_angle=Mock(return_value=feedback(0))
        c.set_current_angle=Mock(return_value=feedback(0))
        c.reset_axis2();c.move_to_angle.assert_called_once_with(2,0,speed_rpm=.6/6)
        c.confirm_axis2_zero();c.set_current_angle.assert_called_once_with(2,0)

    def test_invalid_speed_rejected_before_io(self):
        c=MotorController('TEST');c.stop=Mock();c.move_to_angle=Mock()
        for speed in (True,0.19,1.51,float('nan'),'0.6',None):
            for action in (c.reset_axis1,c.reset_axis2,lambda **kw:c.move_to_zero(1,**kw)):
                with self.assertRaises(ValueError):action(speed_rpm=speed)
        c.stop.assert_not_called();c.move_to_angle.assert_not_called()

    def test_wait_requires_two_stopped_samples_and_stops_on_lost_reference(self):
        c=MotorController('TEST');c.stop=Mock()
        c.get_feedback=Mock(side_effect=[feedback(2),feedback(0),feedback(0)])
        with patch('app.motor_controller.time.sleep'):self.assertEqual(c.wait_for_angle(1,0).output_angle_deg,0)
        self.assertEqual(c.get_feedback.call_count,3)
        c.get_feedback=Mock(return_value=feedback())
        with self.assertRaises(MotorError):c.wait_for_angle(1,0)
        c.stop.assert_called_once_with(1)

    def test_valid_reference_with_fault_is_not_silently_replaced(self):
        c=MotorController('TEST');c.stop=Mock();c.get_feedback=Mock(return_value=feedback(12,2))
        c.configure_axis1_limits=Mock()
        with self.assertRaises(MotorError):c.reset_axis1()
        c.configure_axis1_limits.assert_not_called()

    def test_axis2_without_reference_never_moves_or_confirms_zero(self):
        c=MotorController('TEST');c.stop=Mock();c.get_feedback=Mock(return_value=feedback())
        c.move_relative=Mock();c.set_current_angle=Mock()
        with self.assertRaises(MotorError):c.reset_axis2()
        c.move_relative.assert_not_called();c.set_current_angle.assert_not_called()

    def test_external_stop_cancels_wait_without_redefining_reference(self):
        c=MotorController('TEST')
        def sample(_axis):
            c._stop_versions[1]+=1
            return feedback(12)
        c.get_feedback=Mock(side_effect=sample);c.set_current_angle=Mock()
        with patch('app.motor_controller.time.sleep'),self.assertRaisesRegex(MotorError,'停止操作取消'):
            c.wait_for_angle(1,35)
        c.set_current_angle.assert_not_called()
