import struct
import csv
import tempfile
from pathlib import Path
import unittest
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

from app.joint_timing_panel import JointTimingPanel, parse_plan
from app.protocol import Response
from app.trajectory import AxisPosition


class JointTimingTests(unittest.TestCase):
    def panel(self):
        p=JointTimingPanel.__new__(JointTimingPanel)
        p.app=SimpleNamespace(_log=Mock(),_timing=Mock(),_release_stopped_axis=Mock())
        p.status=Mock();p.table=Mock();p.rows=[];p.jobs=deque();p.points=deque([(10,10),(0,0),(10,10)])
        p.loaded_routes={}
        p.speed_rpm=.1;p.step=0;p.phase='preflight';p.segment=None;p.awaiting=(1,0x37);p.run_started=0
        return p

    def packet(self,a=0,b=0,flags1=1,flags2=1,source=0):
        data=b''.join(struct.pack('>iihHIHBB',round(angle*1000),0,0,flags,0,0,0,source)
                      for angle,flags in ((a,flags1),(b,flags2)))
        return Response(1,0x37,0,0x4652,data,b'')

    def read(self,p,*args,**kwargs):
        p.awaiting=(1,0x37);p.handle(self.packet(*args,**kwargs))

    def test_plan_two_axis_targets_and_actual_rpm(self):
        points,speed=parse_plan('10,5\n0,0\n10,-5',2,.6)
        self.assertEqual(len(points),6);self.assertEqual(points[2],(10,-5));self.assertAlmostEqual(speed,.6)
        for text in ('nan,0','10','36,0','10,0,1'):
            with self.assertRaises(ValueError):parse_plan(text,1,.6)
        for rpm in (.01,1.5,3,5,100):
            self.assertEqual(parse_plan('0,0',1,rpm)[1],rpm)
        for rpm in (0,.009,100.01,float('nan'),float('inf')):
            with self.assertRaises(ValueError):parse_plan('0,0',1,rpm)

    def test_no_motion_before_both_references_valid(self):
        p=self.panel();self.read(p,flags2=0)
        self.assertEqual(p.phase,'stopping')
        self.assertEqual([frame[4] for frame in p.jobs],[6,6])

    def test_next_segment_waits_for_both_firmware_completions(self):
        p=self.panel();self.read(p)
        self.assertEqual([frame[4] for frame in p.jobs],[0x1A])
        p.jobs.clear()
        for axis in (1,2):p.segment['axes'][axis]['sent']=0
        self.read(p,10,9,0x41,0x21)
        self.assertEqual(p.step,1);self.assertEqual(p.rows,[])
        self.read(p,10,10,0x41,0x41)
        self.assertEqual(p.step,2);self.assertEqual(len(p.rows),2)
        self.assertTrue(all(r['result']=='完成' for r in p.rows))
        self.assertEqual(p.rows[0]['joint_total'],p.rows[1]['joint_total'])

    def test_zero_speed_without_completion_never_advances(self):
        p=self.panel();self.read(p);p.jobs.clear()
        for axis in (1,2):p.segment['axes'][axis]['sent']=0
        self.read(p,10,10,0x21,0x21)
        self.assertEqual(p.step,1)

    def test_ack_separates_board_preparation_from_remaining_time(self):
        p=self.panel();self.read(p);p.jobs.clear()
        for axis in (1,2):p.segment['axes'][axis]['sent']=100
        p.awaiting=(1,0x1A)
        from unittest.mock import patch
        with patch('app.joint_timing_panel.time.monotonic',return_value=100.6):
            p.handle(Response(1,0x1A,0,0x4A50,struct.pack('>II',180,220),b''))
        self.assertAlmostEqual(p.segment['board_prep'],.4)
        self.read(p,10,10,0x41,0x41)
        self.assertEqual(p.rows[0]['board_prep'],.4)
        self.assertEqual(p.rows[0]['ack_other'],.2)

    def test_export_includes_new_timing_fields_and_both_axis_rows(self):
        from unittest.mock import patch
        p=self.panel();self.read(p);p.jobs.clear();p.window=Mock()
        for axis in (1,2):p.segment['axes'][axis].update(sent=100,ack=100.6,done=102)
        p.segment['board_prep']=.4;p.record_segment('完成')
        with tempfile.TemporaryDirectory() as folder:
            target=Path(folder)/'timing.csv'
            with patch('app.joint_timing_panel.filedialog.asksaveasfilename',return_value=str(target)):
                p.export()
            with target.open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
        self.assertEqual(len(rows),2)
        self.assertEqual([r['axis'] for r in rows],['1','2'])
        self.assertEqual(rows[0]['board_prep'],'0.4');self.assertEqual(rows[1]['ack_other'],'0.2')

    def test_empty_export_does_not_create_header_only_file(self):
        from unittest.mock import patch
        p=self.panel();p.window=Mock()
        with patch('app.joint_timing_panel.filedialog.asksaveasfilename') as choose, \
             patch('app.joint_timing_panel.messagebox.showinfo') as info:
            p.export()
        choose.assert_not_called();info.assert_called_once()

    def test_stop_cancels_remaining_steps_and_handles_late_ack(self):
        p=self.panel();self.read(p);p.awaiting=(1,0x1A)
        p.stop()
        self.assertTrue(p.handle(Response(1,0x1A,0,0x4A50,b'',b'')))
        self.assertEqual(len(p.points),0);self.assertEqual(p.phase,'stopping')
        self.assertEqual([f[3] for f in p.jobs],[1,2])
        p.jobs.clear();self.read(p);self.assertEqual(p.phase,'stopping')
        self.read(p);self.assertIsNone(p.phase)
        self.assertEqual([c.args for c in p.app._release_stopped_axis.call_args_list],[(1,),(2,)])
        self.assertFalse(any(r['result']=='完成' for r in p.rows))

    def test_cached_feedback_aborts_instead_of_completing(self):
        p=self.panel();self.read(p,source=1)
        self.assertEqual(p.phase,'stopping')

    def test_moving_control_cache_allowed_but_cached_completion_cannot_advance(self):
        p=self.panel();self.read(p);p.jobs.clear()
        for axis in (1,2):p.segment['axes'][axis]['sent']=0
        self.read(p,5,5,0x21,0x21,source=1)
        self.assertEqual(p.phase,'running')
        self.read(p,10,10,0x41,0x41,source=1)
        self.assertEqual(p.step,1)
        self.read(p,10,10,0x41,0x41)
        self.assertEqual(p.step,2)

    def test_board_timing_read_displays_both_axes_when_idle(self):
        p=self.panel();p.phase=None
        p.handle(Response(1,0x1B,0,0x544D,struct.pack('>8I',80,10,50,100,90,12,52,105),b''))
        text=p.status.set.call_args.args[0]
        self.assertIn('轴1',text);self.assertIn('轴2',text);self.assertIn('105ms',text)


if __name__=='__main__':unittest.main()
