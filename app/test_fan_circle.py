import csv
import json
import math
import struct
import tempfile
import unittest
from collections import deque
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from app.fan_circle import FanCircle
from app.fan_circle_panel import FanCirclePanel
from app.protocol import Response, build_request
from app.trajectory_panel import TrajectoryPanel
from app.joint_timing_panel import JointTimingPanel


class FanGeometryTests(unittest.TestCase):
    def test_closed_circle_same_start_end_with_ramps(self):
        plan=FanCircle();points=plan.points()
        self.assertEqual(len(points),65);self.assertEqual(points[-1].time_s,22)
        self.assertEqual((points[0].axis1_deg,points[0].axis2_deg),(6.884,.234))
        self.assertEqual((points[0].axis1_deg,points[0].axis2_deg),(points[-1].axis1_deg,points[-1].axis2_deg))
        self.assertAlmostEqual(plan.phase(22)-plan.phase(0),2*math.pi)

    def test_rotated_plane_and_optical_inverse_match(self):
        plan=FanCircle(plane_yaw_deg=20,plane_tilt_deg=-15,center_x_m=.3)
        center=plan.center;u,v,n=plan.basis
        for t in (0,3,5,10,20,22):
            xyz=plan.position(t);delta=[a-b for a,b in zip(xyz,center)]
            self.assertAlmostEqual(sum(a*b for a,b in zip(delta,n)),0)
            self.assertAlmostEqual(math.sqrt(sum(a*a for a in delta)),plan.radius_m)
            projected=plan.project_angles(*plan.angles(t))
            self.assertAlmostEqual(math.hypot(*projected),plan.radius_m)

    def test_direction_and_phase_define_real_spatial_circle_not_joint_sine(self):
        p=FanCircle(ramp_s=0,fan_rpm=3)
        self.assertLess(p.position(5)[0],0)
        self.assertGreater(FanCircle(ramp_s=0,clockwise=False).position(5)[0],0)
        self.assertAlmostEqual(p.position(0)[1],.73-.44)
        self.assertAlmostEqual(p.position(10)[1],.73+.44)

    def test_middle_speed_matches_fan_rpm_and_ramps_start_end_at_rest(self):
        p=FanCircle();dt=1e-4
        steady=(p.phase(10+dt)-p.phase(10))/dt
        start=(p.phase(dt)-p.phase(0))/dt
        end=(p.phase(p.duration_s)-p.phase(p.duration_s-dt))/dt
        self.assertAlmostEqual(steady,2*math.pi*3/60,places=5)
        self.assertLess(abs(start),1e-6);self.assertLess(abs(end),1e-6)

    def test_multiple_circles_single_finite_upload_without_seam_stop(self):
        p=FanCircle(cycles=3,segments_per_cycle=32)
        self.assertEqual(len(p.points()),97)
        self.assertAlmostEqual(p.phase(p.duration_s)-p.phase(0),6*math.pi)

    def test_real_distance_reduces_motor_excursion(self):
        near=FanCircle();far=FanCircle(distance_m=80)
        self.assertGreater(abs(near.angles(10)[1]-near.yaw_origin_deg),abs(far.angles(10)[1]-far.yaw_origin_deg))

    def test_invalid_geometry_rejected(self):
        for kwargs in (dict(diameter_m=0),dict(distance_m=0),dict(fan_rpm=0),dict(radius_percent=101),dict(fan_rpm=math.nan),
                       dict(cycles=2),dict(blade_count=1.5),dict(ramp_s=25),dict(plane_yaw_deg=90),dict(max_error_deg=.2)):
            with self.assertRaises(ValueError):FanCircle(**kwargs)

    def test_upload_density_and_pitch_margin_checked(self):
        for kwargs in (dict(fan_rpm=120,ramp_s=0),dict(fan_height_m=10),dict(max_speed_dps=.01),dict(max_accel_dps2=.01)):
            with self.assertRaises(ValueError):FanCircle(**kwargs).points()

    def test_lowering_rpm_passes_without_automatic_retiming(self):
        p=FanCircle(fan_rpm=12,ramp_s=1,max_speed_dps=60,max_accel_dps2=120)
        self.assertEqual(p.points()[-1].time_s,6)


class CirclePanelTests(unittest.TestCase):
    def panel(self):
        p=FanCirclePanel.__new__(FanCirclePanel)
        p.plan=FanCircle();p.run_plan=p.plan;p.points=p.plan.points();p.dirty=False
        p.operation=None;p.session=None;p.clock=None;p.samples=[];p.events=[];p.report={};p.uploaded_revision=None;p.intervals=None
        p.app=SimpleNamespace(business_panel=SimpleNamespace(available=Mock(return_value=True)),trajectory_running=False,pending=None)
        e=SimpleNamespace(points=(),revision=5,session=11,sealed=False,status=Mock(),jobs=deque(),status_interval=.5,position_interval=1)
        e.values={key:Mock() for key in ('cycle_ms','speed','accel','error','tolerance')}
        e.show_advanced=Mock();e.adaptive=Mock();e.upload=Mock(side_effect=lambda:e.jobs.append('upload'))
        e.start=Mock();e.stop=Mock();e.invalidate=Mock()
        p.engine=e;p.checked=Mock();p.checked.get.return_value=True;p.scene=Mock();p.scene.get.return_value='模型台架'
        p.status=Mock();p.metrics=Mock();p.draw=Mock();p.window=Mock();p.visual=Mock();p.visual.get.return_value='轻微'
        p.note=Mock();p.note.get.return_value='结束时';p.preview=Mock()
        return p

    def packet(self,command=0x34,state=3,session=11,ok=0,elapsed=1000,gap=110):
        data=struct.pack('>HBBHHIIIIiiii',session,state,0,65,65,elapsed,22000,gap,90,6884,234,20,-10)
        return Response(1,command,ok,session,data,b'')

    def test_upload_never_starts_and_forces_fixed_time_without_retiming(self):
        p=self.panel();p.upload()
        p.engine.adaptive.set.assert_called_once_with(False)
        p.engine.show_advanced.set.assert_called_once_with(True)
        p.engine.upload.assert_called_once();p.engine.start.assert_not_called()
        self.assertEqual(p.operation,'uploading');self.assertFalse(p.report['real_rotor_phase_locked'])

    def test_seal_only_marks_ready_never_automatically_starts(self):
        p=self.panel();p.upload();p.engine.sealed=True
        p.observe(self.packet(command=0x32,state=2))
        self.assertEqual(p.operation,'ready');p.engine.start.assert_not_called()

    def test_edits_require_regenerate_and_upload(self):
        p=self.panel();p.changed()
        with patch('app.fan_circle_panel.messagebox.showwarning'):p.upload()
        p.engine.upload.assert_not_called()
        p.dirty=False;p.uploaded_revision=None;p.session=11;p.engine.sealed=True
        p.start();p.engine.start.assert_not_called()

    def test_actual_start_clock_begins_on_serial_write_not_upload(self):
        p=self.panel();p.operation='starting';p.uploaded_revision=5
        p.sent(build_request(1,0x33,parameter=11),100)
        self.assertEqual(p.clock,100);self.assertEqual(p.operation,'running');self.assertIsNone(p.uploaded_revision)

    def test_upload_late_seal_after_stop_cannot_enable_start(self):
        p=self.panel();p.session=11;p.operation='stopping'
        p.observe(self.packet(command=0x32,state=2))
        self.assertEqual(p.operation,'stopping');self.assertIsNone(p.uploaded_revision)

    def test_feedback_records_errors_and_geometry_without_extra_drive_reads(self):
        p=self.panel();p.session=11;p.clock=100;p.operation='running'
        with patch('time.monotonic',return_value=101):p.observe(self.packet())
        self.assertEqual(len(p.samples),1);self.assertIn('plane_u_m',p.samples[0])
        self.assertEqual(p.report['summary']['max_error_deg'],[.02,.01])

    def test_invalid_feedback_not_counted_as_zero_error(self):
        p=self.panel();p.session=11;p.clock=100;p.operation='starting'
        p.observe(self.packet(state=2,elapsed=0,gap=0))
        self.assertNotIn('summary',p.report)
        self.assertFalse(p.samples[0]['feedback_available'])

    def test_done_ends_test_without_replay_and_restores_polling(self):
        p=self.panel();p.session=11;p.clock=100;p.operation='running';p.intervals=(.5,1)
        with patch('time.monotonic',return_value=123):p.observe(self.packet(state=6,elapsed=22000))
        self.assertIsNone(p.operation);self.assertEqual(p.engine.status_interval,.5)
        self.assertEqual(p.report['wall_elapsed_s'],23)
        p.engine.start.assert_not_called()

    def test_disconnect_invalidates_without_restarting(self):
        p=self.panel();p.operation='running';p.uploaded_revision=5
        p.interrupted('断开')
        self.assertIsNone(p.operation);self.assertIsNone(p.uploaded_revision)
        p.engine.start.assert_not_called()

    def test_export_contains_config_feedback_and_separate_trajectory_csv(self):
        p=self.panel();p.session=11;p.clock=100;p.operation='running'
        p.report['config']=asdict(p.plan);p.observe(self.packet())
        p.operation=None
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'circle.json'
            with patch('app.fan_circle_panel.filedialog.asksaveasfilename',return_value=str(path)):p.export()
            report=json.loads(path.read_text(encoding='utf-8'))
            with path.with_suffix('.csv').open(encoding='utf-8-sig') as f:rows=list(csv.DictReader(f))
            with path.with_name('circle_trajectory.csv').open(encoding='utf-8-sig') as f:points=list(csv.DictReader(f))
        self.assertEqual(report['visual'],'轻微');self.assertEqual(len(rows),1);self.assertEqual(len(points),65)

    def test_real_upload_frames_use_existing_protocol_and_fixed_clock(self):
        class Value:
            def __init__(self,value):self.value=value
            def get(self):return self.value
            def set(self,value):self.value=value
        p=self.panel();e=p.engine
        e.values={key:Value('0') for key in e.values}
        e.show_advanced=Value(False);e.adaptive=Value(True);e.service=Mock();e.ready=Mock(return_value=True);e.error=Mock()
        e.upload=lambda:TrajectoryPanel.upload(e)
        p.upload()
        frames=[frame for frame,_ in e.jobs]
        self.assertEqual([f[4] for f in frames],[0x30]+[0x31]*65+[0x32])
        count,cycle=struct.unpack('>HH',frames[0][7:11])
        self.assertEqual((count,cycle),(65,100));e.error.assert_not_called()

    def test_stop_during_start_confirmation_prevents_delayed_start(self):
        e=TrajectoryPanel.__new__(TrajectoryPanel)
        e.ready=Mock(return_value=True);e.sealed=True;e.quick_generation=1;e.revision=4;e.session=7;e.window=Mock();e._send_start=Mock()
        def stopped(*args,**kwargs):
            e.sealed=False;e.quick_generation+=1;return True
        with patch('app.trajectory_panel.messagebox.askyesno',side_effect=stopped):e.start()
        e._send_start.assert_not_called()

    def test_start_positioning_appends_existing_timing_records(self):
        p=JointTimingPanel.__new__(JointTimingPanel)
        p.app=Mock();p.app.motion_generation=0;p.status=Mock();p.table=Mock()
        old=[dict(step=1,axis=1),dict(step=1,axis=2)];p.rows=list(old)
        p._begin_run([(1,2)],.1,append_records=True)
        self.assertEqual(p.rows,old);p.table.delete.assert_not_called();self.assertEqual(p.step,1)


if __name__=='__main__':unittest.main()
