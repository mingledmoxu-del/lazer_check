import struct
import unittest
from unittest.mock import Mock, patch

from app import MotorController, CommandRejected
from app.protocol import build_move_axes_point, Response
from app.test_motor_controller import FakeSerial
from app.test_settle_feedback import poses
from app.timing import MotionTiming


class JointPointApiTests(unittest.TestCase):
    def test_frame_and_ranges(self):
        frame=build_move_axes_point((10,-5),.1)
        self.assertEqual(frame[3:7],b'\x01\x1aJP')
        self.assertEqual(struct.unpack('>Hii',frame[7:17]),(10,10000,-5000))
        for angles,speed in (((float('nan'),0),.1),((0,),.1),((0,0),0),((0,0),101)):
            with self.assertRaises(ValueError):build_move_axes_point(angles,speed)

    def test_sdk_sends_one_packet_without_extra_preflight_or_stop(self):
        wire=FakeSerial();wire.custom_responses={0x1A:struct.pack('>II',80,90)}
        c=MotorController('TEST',serial_factory=lambda *_a,**_kw:wire).connect()
        try:
            receipt=c.move_axes_to((10,0),speed_rpm=.1)
            self.assertEqual(receipt['board_prepare_ms'],(80,90))
            self.assertEqual([f[4] for f in wire.frames],[0x1A])
        finally:c.close(stop=False)

    def test_old_firmware_rejection_never_falls_back_to_axis_commands(self):
        c=MotorController('TEST');c._request=Mock(side_effect=CommandRejected(Response(1,0x1A,7,0x4A50,b'',b'')))
        c._request_pair_stop=Mock()
        with self.assertRaises(CommandRejected):c.move_axes_to((10,0))
        self.assertEqual(c._request.call_count,1)
        c._request_pair_stop.assert_called_once()

    def test_pair_timing_ack_and_finish_track_both_axes(self):
        rows=[];t=MotionTiming(emit=rows.append)
        t.sent(build_move_axes_point((10,0),.1))
        t.received(Response(1,0x1A,0,0x4A50,struct.pack('>II',80,90),b''))
        self.assertEqual([r['axis'] for r in rows if r['event']=='motion_ack'],[1,2])
        t.observe(1,10,100,0,0x41);t.observe(2,0,0,0,0x41)
        self.assertEqual([r['axis'] for r in rows if r['event']=='completed'],[1,2])

    def test_wait_reuses_moving_samples_and_forces_direct_final_evidence(self):
        c=MotorController('TEST');c.get_axis_positions=Mock(side_effect=[poses(0x21,source=1),poses(source=1),poses()])
        with patch('app.motor_controller.time.sleep'):c.wait_for_axes((0,1))
        self.assertEqual([call.kwargs['fresh'] for call in c.get_axis_positions.call_args_list],[False,False,True])

    def test_board_timing_has_explicit_units_and_no_motion(self):
        c=MotorController('TEST');c._request=Mock(return_value=Response(1,0x1B,0,0x544D,struct.pack('>8I',80,10,50,100,90,12,52,105),b''))
        result=c.get_point_timing()
        self.assertEqual(result[0]['init_ms'],80)
        self.assertEqual(result[1]['max_cycle_period_ms'],105)
        self.assertEqual(c._request.call_args.args[0][3:7],b'\x01\x1bTM')


if __name__=='__main__':unittest.main()
