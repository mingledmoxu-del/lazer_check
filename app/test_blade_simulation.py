import csv
import json
import math
import struct
import tempfile
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from app.blade_simulation import BladePlan, encoder_stats
from app.blade_test_panel import BladeTestPanel
from app.protocol import Response, CMD_READ_PA, CMD_WRITE_PA, CMD_SET_MOTION_PROFILE
from app.trajectory import AxisPosition


class BladePlanTests(unittest.TestCase):
    def test_geometry_matches_supplied_three_and_four_point_table(self):
        self.assertEqual(BladePlan().points,((6.884,.234),(16.311,-9.748),(25.347,.234),(16.311,10.216)))
        self.assertEqual(BladePlan(count=3,radius_index=5).points,((.496,.234),(23.197,-13.771),(23.197,14.239)))
        for count in (3,4):
            for radius in range(1,6):
                plan=BladePlan(count=count,radius_index=radius,cycles=2)
                self.assertEqual(len(plan.points),count)
                self.assertEqual(plan.leg(count-1)['point'],0)
                self.assertEqual(plan.leg(2*count-1)['blade_s'],10)

    def test_absolute_clock_and_delays_are_not_relative_to_completion(self):
        plan=BladePlan(radius_index=1)
        self.assertAlmostEqual(plan.delay_s,.410)
        self.assertAlmostEqual(plan.leg(1)['send_s'],1.660)
        self.assertAlmostEqual(plan.leg(1)['blade_s'],2.5)
        self.assertAlmostEqual(BladePlan(mode='B').delay_s,.200)
        self.assertAlmostEqual(BladePlan(period_s=10).passage_s,.140)

    def test_invalid_inputs_rejected_before_upload(self):
        for kwargs in (dict(speed_rpm=0),dict(period_s=math.nan),dict(period_s=.1),dict(count=6),
                       dict(radius_index=0),dict(cycles=101),dict(distance_m=0),dict(pitch_origin=90)):
            with self.assertRaises(ValueError): BladePlan(**kwargs)

    def test_encoder_statistics_are_about_mean_not_target(self):
        samples=[dict(observed_s=t,angle_deg=a,target_deg=10) for t,a in ((0,9.9),(.1,10.1),(.2,10))]
        stats=encoder_stats(samples)
        self.assertEqual(stats['pp_deg'],.2); self.assertEqual(stats['max_error_deg'],.1)
        self.assertAlmostEqual(stats['rms_deg'],.08165,places=5)
        self.assertIsNone(encoder_stats([])['pp_deg'])


class BladePanelTests(unittest.TestCase):
    def panel(self):
        p=BladeTestPanel.__new__(BladeTestPanel)
        p.app=SimpleNamespace(port=Mock(),pending=None,deferred_send=None,_send=Mock(return_value=True),
            _timing=Mock(),_log=Mock(),_release_stopped_axis=Mock())
        p.prepared=BladePlan(); p.completion_tolerance=.2; p.speed_rpm=1
        p.phase='waiting';p.mode='test';p.rows=[];p.samples=[];p.sample_rows={};p.metadata=dict(events=[])
        p.jobs=deque();p.points=deque();p.segment=None;p.awaiting=None;p.run_selection=None
        p.token=123;p.step=0;p.next_leg=0;p.active_leg=None;p.next_read=0;p.deadline=200;p.t0=100
        p.run_started=100;p.missed=False;p.status=Mock();p.table=Mock();p.table.selection.return_value=()
        p.table.get_children.return_value=()
        p.summary=Mock();p.loaded_routes={};p.loaded_route=None;p.window=Mock();p.profiles={}
        return p

    def tick(self,p,now):
        with patch('time.monotonic',return_value=now): p.service()

    def feedback(self,p,now,target=None,flags=0x41,source=0,rpm=0):
        target=target or p.prepared.points[0]
        data=b''.join(struct.pack('>iihHIHBB',round(a*1000),0,rpm,flags,0,0,0,source) for a in target)
        p.awaiting=(1,0x37)
        with patch('time.monotonic',return_value=now): p.handle(Response(1,0x37,0,0,data,b''))

    def test_no_run_before_absolute_send_time(self):
        p=self.panel();self.tick(p,100.05)
        self.assertEqual(p.app._send.call_args.args[1],0x37)
        self.assertIsNone(p.segment)
        self.tick(p,100.10)
        self.assertEqual(p.app._send.call_args.args[1],0x43)
        self.assertEqual(p.active_leg['point'],1)

    def test_late_scheduler_does_not_catch_up(self):
        p=self.panel();self.tick(p,101)
        self.assertEqual(p.phase,'stopping');self.assertEqual(p.next_leg,0)
        self.assertEqual([f[4] for f in p.jobs],[6,6])

    def test_completion_waits_for_next_absolute_deadline(self):
        p=self.panel();self.tick(p,100.10)
        self.feedback(p,100.5,p.active_leg['target'])
        self.assertEqual(p.phase,'waiting');self.assertEqual(len(p.rows),2)
        self.assertAlmostEqual(p.rows[0]['margin'],.75)
        p.app._send.reset_mock();self.tick(p,100.8)
        self.assertEqual(p.app._send.call_args.args[1],0x37)
        self.assertEqual(p.next_leg,1)

    def test_unfinished_motion_at_blade_deadline_stops_both(self):
        p=self.panel();self.tick(p,100.10);self.tick(p,101.25)
        self.assertEqual(p.phase,'stopping');self.assertEqual([f[3] for f in p.jobs],[1,2])
        self.assertEqual(p.rows[0]['verdict'],'未及时取得完成反馈')

    def test_cached_zero_without_board_completion_cannot_pass(self):
        p=self.panel();self.tick(p,100.10)
        self.feedback(p,100.5,p.active_leg['target'],source=1)
        self.assertEqual(p.phase,'running');self.assertFalse(p.rows)
        self.feedback(p,100.6,p.active_leg['target'],flags=1)
        self.assertEqual(p.phase,'running')

    def test_jitter_uses_direct_post_completion_samples(self):
        p=self.panel();self.tick(p,100.10)
        self.feedback(p,100.5,p.active_leg['target'])
        self.feedback(p,100.6,p.active_leg['target'],source=1)
        self.assertEqual(p.sample_rows[(1,1)],[])
        self.feedback(p,100.7,p.active_leg['target'])
        self.assertEqual(len(p.sample_rows[(1,1)]),1)
        self.assertEqual(len(p.samples),6)

    def test_full_loop_returns_to_point_zero_then_observes_without_extra_run(self):
        p=self.panel()
        for n in range(4):
            self.tick(p,100+p.prepared.leg(n)['send_s']+.001)
            self.feedback(p,100+p.prepared.leg(n)['send_s']+.3,p.active_leg['target'])
        self.assertEqual(p.active_leg['point'],0);self.assertEqual(len(p.rows),8)
        self.tick(p,105.4);self.assertEqual(p.phase,'waiting')
        self.tick(p,105.5);self.assertIsNone(p.phase)
        self.assertEqual(p.metadata['summary']['on_time'],4)

    def test_upload_ack_cannot_start_clock_or_following_points(self):
        p=self.panel();p.mode='preparing';p.begin_segment([AxisPosition(0,0,0,1,0,0,0,0)]*2)
        self.assertEqual([f[4] for f in p.jobs],[CMD_READ_PA]*6+[0x40]+[0x41]*4+[0x42])
        self.assertFalse(any(f[4] in (CMD_WRITE_PA,CMD_SET_MOTION_PROFILE) for f in p.jobs))
        p.jobs.clear();p.points=deque([p.prepared.points[0]]);p.awaiting=(1,0x42)
        data=struct.pack('>IHHBBB',123,100,200,4,4,1)
        p.handle(Response(1,0x42,0,6,data,b''))
        self.assertEqual(p.mode,'positioning');self.assertEqual([f[4] for f in p.jobs],[0x43])
        p.jobs.clear()
        for m in p.segment['axes'].values():m['sent']=100
        self.feedback(p,100.5)
        self.assertEqual(p.phase,'armed');self.assertFalse(p.rows)
        self.assertIn('initial_move',p.metadata)

    def test_export_contains_config_raw_samples_profiles_and_visual_observations(self):
        p=self.panel();self.tick(p,100.10);self.feedback(p,100.5,p.active_leg['target'])
        p.rows[0].update(visual='明显',note='停止瞬间')
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'test.csv'
            with patch('app.blade_test_panel.filedialog.asksaveasfilename',return_value=str(path)):
                p.export()
            report=json.loads(path.with_suffix('.json').read_text(encoding='utf-8'))
            with path.open(encoding='utf-8-sig') as f: rows=list(csv.DictReader(f))
        self.assertEqual(rows[0]['visual'],'明显');self.assertEqual(len(report['samples']),2)
        self.assertEqual(report['metadata']['summary']['visual_ratings']['明显'],1)

    def test_manual_stop_and_late_run_ack_never_restart(self):
        p=self.panel();self.tick(p,100.10);p.stop()
        p.handle(Response(1,0x43,0,6,struct.pack('>II',40,40),b''))
        self.assertEqual(p.phase,'stopping');self.assertEqual([f[4] for f in p.jobs],[6,6])

    def test_profile_reads_save_both_axes_without_parameter_writes(self):
        p=self.panel();p.mode='preparing';p.begin_segment([AxisPosition(0,0,0,1,0,0,0,0)]*2)
        for axis in (1,2):
            for pa in (40,41,42):
                p.awaiting=(axis,CMD_READ_PA)
                p.handle(Response(axis,CMD_READ_PA,0,pa,struct.pack('>H',150),b''))
        self.assertEqual(len(p.profiles),6)
        self.assertEqual(p.profiles['axis1_PA41'],150)

    def test_reference_loss_while_waiting_stops_and_clears_future_points(self):
        p=self.panel();self.feedback(p,100.01,flags=0)
        self.assertEqual(p.phase,'stopping');self.assertEqual(p.next_leg,0)

    def test_post_completion_drift_outside_tolerance_invalidates_pass(self):
        p=self.panel();self.tick(p,100.10);self.feedback(p,100.5,p.active_leg['target'])
        a,b=p.active_leg['target'];self.feedback(p,100.7,(a+.3,b))
        self.assertEqual(p.phase,'stopping')
        self.assertEqual(p.rows[0]['verdict'],'停稳后离开目标容差')

    def test_earlier_completed_axis_must_still_be_at_target_when_other_finishes(self):
        p=self.panel();self.tick(p,100.10);a,b=p.active_leg['target']
        self.feedback(p,100.4,(a,b-1))
        self.feedback(p,100.5,(a+.3,b))
        self.assertEqual(p.phase,'running');self.assertFalse(p.rows)
        self.feedback(p,100.6,(a,b))
        self.assertEqual(p.phase,'waiting');self.assertEqual(len(p.rows),2)

    def test_stop_requires_two_direct_zero_snapshots(self):
        p=self.panel();p.stop();p.jobs.clear()
        self.feedback(p,100.1);self.assertEqual(p.phase,'stopping')
        self.feedback(p,100.2);self.assertIsNone(p.phase)

    def test_clock_starts_only_from_fresh_armed_pose(self):
        p=self.panel();p.phase='armed';p.mode='armed';p.config=Mock(return_value=p.prepared)
        p.preview=Mock(return_value=p.prepared);p.confirmed=Mock();p.confirmed.get.return_value=True
        self.feedback(p,100,p.prepared.points[0])
        with patch('time.monotonic',return_value=101): p.start()
        self.assertEqual(p.phase,'armed')
        self.feedback(p,101,p.prepared.points[0])
        with patch('time.monotonic',return_value=101.1): p.start()
        self.assertEqual(p.phase,'waiting');self.assertEqual(p.t0,101.1)


if __name__=='__main__': unittest.main()
