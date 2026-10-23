import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from app import MotorController, MotorError, CommunicationError
from app.protocol import Response, CMD_READ_AXES
from app.trajectory import AxisPosition


def poses(flags=0x41, source=0, age=0, fault=0):
    return tuple(AxisPosition(float(i), 0, 0, flags, age, 0, fault, source) for i in (0, 1))


class SettleFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.c=MotorController('TEST')
        self.c.stop=Mock()

    def test_firmware_completion_needs_one_combined_direct_snapshot(self):
        self.c.get_axis_positions=Mock(return_value=poses())
        result=self.c.wait_for_axes((0,1))
        self.assertEqual(len(result),2)
        self.c.get_axis_positions.assert_called_once_with(fresh=True)
        self.c.stop.assert_not_called()

    def test_zero_speed_during_active_task_is_not_completion(self):
        self.c.get_axis_positions=Mock(side_effect=[poses(0x21),poses(1),poses(1)])
        with patch('app.motor_controller.time.sleep'):
            self.c.wait_for_axes((0,1))
        self.assertEqual(self.c.get_axis_positions.call_count,3)

    def test_cached_and_old_completed_snapshot_cannot_finish(self):
        self.c.get_axis_positions=Mock(side_effect=[poses(source=1),poses(age=500),poses()])
        with patch('app.motor_controller.time.sleep'):
            self.c.wait_for_axes((0,1))
        self.assertEqual(self.c.get_axis_positions.call_count,3)

    def test_guard_fault_requests_both_stops_without_movement(self):
        self.c.get_axis_positions=Mock(return_value=poses(fault=8))
        with self.assertRaises(MotorError):self.c.wait_for_axes((0,1))
        self.assertEqual(self.c.stop.call_count,2)

    def test_one_failed_stop_does_not_skip_the_other_axis(self):
        self.c.stop.side_effect=[CommunicationError('offline'),None]
        self.c.get_axis_positions=Mock(return_value=poses(fault=8))
        with self.assertRaises(MotorError):self.c.wait_for_axes((0,1))
        self.assertEqual(self.c.stop.call_count,2)

    def test_query_failure_preserves_error_and_attempts_both_stops(self):
        self.c.get_axis_positions=Mock(side_effect=CommunicationError('read failed'))
        with self.assertRaisesRegex(CommunicationError,'read failed'):self.c.wait_for_axes((0,1))
        self.assertEqual(self.c.stop.call_count,2)

    def test_single_axis_reuses_two_zero_proof_but_not_active_zero(self):
        live=SimpleNamespace(output_angle_deg=0,fault=0,motor_rpm=0,flags=0x21)
        done=SimpleNamespace(output_angle_deg=0,fault=0,motor_rpm=0,flags=0x41)
        self.c.get_feedback=Mock(side_effect=[live,done])
        with patch('app.motor_controller.time.sleep'):
            self.assertIs(self.c.wait_for_angle(1,0),done)
        self.assertEqual(self.c.get_feedback.call_count,2)

    def test_fresh_query_has_explicit_protocol_parameter(self):
        data=b''.join(struct.pack('>iihHIHBB', i*1000,0,0,0x41,0,0,0,0) for i in (0,1))
        self.c._request=Mock(return_value=Response(1,CMD_READ_AXES,0,0x4652,data,b''))
        self.c.get_axis_positions(fresh=True)
        self.assertEqual(self.c._request.call_args.args[0][5:7],b'FR')


if __name__=='__main__':unittest.main()
