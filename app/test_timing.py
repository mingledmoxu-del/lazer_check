import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import Mock

from app.timing import MotionTiming
from app.protocol import Response, build_move_output_relative, build_request, CMD_STOP
from app import MotorController
from app.test_motor_controller import FakeSerial


class MotionTimingTests(unittest.TestCase):
    def test_monotonic_phase_durations_and_single_completion(self):
        now=[0.0];rows=[];t=MotionTiming(emit=rows.append,clock=lambda:now[0])
        t.begin(1,'test',10)
        now[0]=.1;frame=build_move_output_relative(1,.2,10);t.sent(frame)
        now[0]=.15;t.received(Response(1,0x19,0,0x5054,b'',b''))
        now[0]=.3;t.observe(1,1,100,10,0x21)
        now[0]=.6;t.observe(1,9.6,12000,2,0x21)
        now[0]=.8;t.observe(1,10,13889,0,0x41)
        t.observe(1,10,13889,0,0x41)
        end=rows[-1]
        self.assertEqual(end['event'],'completed')
        self.assertEqual((end['elapsed_ms'],end['from_sent_ms'],end['tx_ack_ms']),(800,700,50))
        self.assertEqual((end['from_motion_observed_ms'],end['from_angle_window_ms']),(500,200))
        self.assertEqual(sum(r['event']=='completed' for r in rows),1)

    def test_zero_speed_alone_and_stale_snapshot_do_not_complete(self):
        rows=[];t=MotionTiming(emit=rows.append)
        t.sent(build_move_output_relative(1,.2,5))
        t.observe(1,5,100,0,0x21)
        t.observe(1,5,100,0,0x41,sample_age_ms=500)
        self.assertFalse(any(r['event']=='completed' for r in rows))
        t.sent(build_request(1,CMD_STOP));t.stop_confirmed(1)
        self.assertEqual(rows[-1]['event'],'cancelled')

    def test_rejected_command_is_not_success(self):
        rows=[];t=MotionTiming(emit=rows.append)
        t.sent(build_move_output_relative(1,.2,5))
        t.received(Response(1,0x19,6,0x5054,b'',b''))
        self.assertEqual(rows[-1]['event'],'rejected')
        self.assertNotIn(1,t.axes)

    def test_jsonl_output_and_failed_callback_do_not_change_control(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'timing.jsonl'
            callback=Mock(side_effect=RuntimeError('observer failed'))
            t=MotionTiming(path,callback)
            with self.assertLogs('app.timing',level='WARNING'):t.begin(1,'测试')
            self.assertEqual(json.loads(path.read_text(encoding='utf-8'))['operation'],'测试')
            self.assertIn(1,t.axes)

    def test_sdk_instrumentation_adds_no_wire_queries(self):
        wire=FakeSerial();rows=[]
        c=MotorController('TEST',serial_factory=lambda *_a,**_kw:wire,timing_callback=rows.append).connect()
        try:
            c.move_relative(1,5)
            self.assertEqual([f[4] for f in wire.frames],[6,7,7,9,0x19])
            wire.custom_responses={0x20:struct.pack('>iiiHHHiHBBBBih',100,100,0,3,10000,17,5000,0x41,0,0,0,0,0,0)}
            c.get_feedback(1)
            self.assertEqual(rows[-1]['event'],'completed')
            self.assertEqual(len({r['trace_id'] for r in rows}),1)
        finally:c.close(stop=False)


if __name__=='__main__':unittest.main()
