import struct
import threading
import unittest
from unittest.mock import Mock

from app import MotorController,MotorError,PointScan
from app.test_motor_controller import FakeSerial
from app.examples.point_scan_business import prepare_selected_routes,run_until_decision_holds


class SixBankSerial(FakeSerial):
    def __init__(self):
        super().__init__();self.banks={}

    def write(self,frame):
        cmd=frame[4];slot=struct.unpack_from('>H',frame,5)[0]
        if cmd==0x40:
            token,speed,tol,count=struct.unpack_from('>IHHB',frame,7)
            self.banks[slot]=dict(token=token,speed=speed,tol=tol,count=count,loaded=0,ready=0,points=[])
        if cmd in (0x41,0x42):
            b=self.banks[slot]
            if cmd==0x41:b['loaded']+=1;b['points'].append(struct.unpack_from('>ii',frame,12))
            else:b['ready']=1
        if cmd in (0x40,0x41,0x42,0x44):
            b=self.banks[slot]
            self.custom_responses[cmd]=struct.pack('>IHHBBB',b['token'],b['speed'],b['tol'],b['count'],b['loaded'],b['ready'])
        if cmd==0x43:
            token,index=struct.unpack_from('>IB',frame,7);b=self.banks[slot]
            assert token==b['token'] and index<b['count']
            self.custom_responses[cmd]=struct.pack('>II',35,36)
        return super().write(frame)


class PointScanTests(unittest.TestCase):
    def setUp(self):
        self.wire=SixBankSerial()
        self.c=MotorController('TEST',serial_factory=lambda *_a,**_kw:self.wire).connect()
        self.c.wait_for_axes=Mock(return_value=('fresh-axis1','fresh-axis2'))

    def tearDown(self):self.c.close(stop=False)

    def prepare(self,count=2,slot=1,token=123):
        return self.c.prepare_point_scan([(i%10,0) for i in range(count)],route_id=slot,
                                         speed_rpm=3,tolerance_deg=.2,token=token,name='selected-by-business')

    def test_six_independent_variable_length_preloads_never_run(self):
        scans=[self.prepare(count,slot,100+slot) for slot,count in enumerate((6,8,10,12,14,16),1)]
        self.assertEqual(len(self.wire.banks),6)
        self.assertFalse(any(f[4]==0x43 for f in self.wire.frames))
        before=len(self.wire.frames)
        for slot,scan in enumerate(scans,1):
            self.assertIsInstance(scan,PointScan);self.assertEqual(scan.status()['phase'],'READY')
            self.assertEqual(scan.status()['token'],100+slot)
        self.assertEqual(len(self.wire.frames),before)  # status is local, no polling.
        for scan in scans:
            for n in range(scan.status()['point_count']):
                scan.move_next_and_wait();scan.confirm_acquisition(n,f'sample-{n}',passed=True)
        slots=[struct.unpack_from('>H',f,5)[0] for f in self.wire.frames if f[4]==0x43]
        self.assertEqual(slots,[slot for slot,count in enumerate((6,8,10,12,14,16),1) for _ in range(count)])

    def test_another_scan_cannot_move_during_current_laser_acquisition(self):
        first=self.prepare(count=1);second=self.prepare(slot=2,token=124)
        first.move_next_and_wait();before=len(self.wire.frames)
        with self.assertRaises(MotorError):second.start_next_point()
        self.assertEqual(len(self.wire.frames),before)
        first.confirm_acquisition(0,'finished',passed=True)
        second.move_next_and_wait();self.assertEqual(second.status()['phase'],'AWAITING_ACQUISITION')

    def test_wait_then_capture_ack_then_explicit_next_only(self):
        s=self.prepare();before=len(self.wire.frames)
        first=s.move_next_and_wait()
        self.assertTrue(first['completed']);self.assertEqual(s.status()['phase'],'AWAITING_ACQUISITION')
        self.assertEqual(len(self.wire.frames),before+1)
        self.c.wait_for_axes.assert_called_once_with((0,0),tolerance_deg=.2,timeout_s=120)
        with self.assertRaises(MotorError):s.start_next_point()
        self.assertEqual(len(self.wire.frames),before+1)
        s.confirm_acquisition(0,'laser-0',passed=False)
        self.assertEqual(s.status()['phase'],'AWAITING_ACQUISITION')
        s.confirm_acquisition(0,'laser-0-retry',passed=True)
        s.confirm_acquisition(0,'laser-0-retry',passed=True)  # exact duplicate is a no-op.
        self.assertEqual(len(self.wire.frames),before+1)
        self.assertTrue(s.status()['can_start_next'])
        s.move_next_and_wait();self.assertEqual(s.status()['phase'],'AWAITING_ACQUISITION')
        s.confirm_acquisition(1,'laser-1',passed=True)
        self.assertEqual(s.status()['phase'],'COMPLETED')
        with self.assertRaises(MotorError):s.start_next_point()
        self.assertEqual(len(self.wire.frames),before+2)

    def test_ack_is_not_completion_and_capture_confirm_cannot_arrive_early(self):
        s=self.prepare();ack=s.start_next_point()
        self.assertFalse(ack['completed']);self.assertEqual(s.status()['phase'],'MOVING')
        with self.assertRaises(MotorError):s.confirm_acquisition(0,'early',passed=True)
        with self.assertRaises(MotorError):s.start_next_point()
        s.wait_current_point();calls=self.c.wait_for_axes.call_count
        s.wait_current_point();self.assertEqual(self.c.wait_for_axes.call_count,calls)
        with self.assertRaises(MotorError):s.confirm_acquisition(1,'wrong-index',passed=True)

    def test_late_capture_ack_does_not_advance_a_different_current_point(self):
        s=self.prepare();s.move_next_and_wait();s.confirm_acquisition(0,'a',passed=True)
        s.move_next_and_wait()
        with self.assertRaises(MotorError):s.confirm_acquisition(0,'a',passed=True)
        self.assertEqual(s.status()['current_index'],1)

    def test_overwritten_bank_and_reconnect_invalidate_old_session(self):
        s=self.prepare();self.prepare(token=124)
        before=len(self.wire.frames)
        with self.assertRaises(MotorError):s.start_next_point()
        self.assertEqual(len(self.wire.frames),before);self.assertEqual(s.status()['phase'],'FAULT')
        s=self.prepare(token=125);self.c.close(stop=False);self.c.connect()
        with self.assertRaises(MotorError):s.start_next_point()

    def test_feedback_failure_latches_session_without_advancing(self):
        s=self.prepare();self.c.wait_for_axes.side_effect=MotorError('feedback invalid')
        with self.assertRaises(MotorError):s.move_next_and_wait()
        self.assertEqual(s.status()['phase'],'FAULT')
        self.assertEqual(sum(f[4]==0x43 for f in self.wire.frames),1)
        self.assertEqual([f[3] for f in self.wire.frames if f[4]==6],[1,2])
        with self.assertRaises(MotorError):s.start_next_point()

    def test_example_validates_all_plans_before_upload_and_holds_failed_capture(self):
        with self.assertRaises(ValueError):prepare_selected_routes(self.c,[
            dict(name='valid',points=[(0,0)]),dict(name='invalid',points=[]),
        ])
        self.assertEqual(self.wire.frames,[])
        scans=prepare_selected_routes(self.c,[dict(name='business-choice',points=[(0,0),(5,5)])])
        judge=Mock(return_value=('acquisition-001',False))
        state=run_until_decision_holds(scans[1],judge)
        self.assertEqual(state['phase'],'AWAITING_ACQUISITION')
        self.assertEqual(sum(f[4]==0x43 for f in self.wire.frames),1)
        judge.assert_called_once()

    def test_stop_can_interrupt_wait_and_never_enters_acquisition(self):
        s=self.prepare();entered=threading.Event();release=threading.Event();errors=[]
        def wait(*_a,**_kw):entered.set();release.wait(2);return ('fresh1','fresh2')
        self.c.wait_for_axes.side_effect=wait;s.start_next_point()
        def run():
            try:s.wait_current_point()
            except MotorError as exc:errors.append(str(exc))
        worker=threading.Thread(target=run);worker.start()
        try:
            self.assertTrue(entered.wait(2));s.stop(wait=False)
        finally:release.set();worker.join(2)
        self.assertFalse(worker.is_alive());self.assertTrue(errors)
        self.assertEqual(s.status()['phase'],'STOPPED')
        self.assertEqual([f[3] for f in self.wire.frames if f[4]==6],[1,2])

    def test_stop_failure_attempts_both_axes_and_reports_unknown(self):
        s=self.prepare();self.c.stop=Mock(side_effect=[MotorError('axis1 offline'),None])
        self.c._request_pair_stop=Mock()
        with self.assertRaises(MotorError):s.stop()
        self.assertEqual(self.c.stop.call_count,2);self.c._request_pair_stop.assert_called_once()
        self.assertEqual(s.status()['phase'],'STOPPED')

    def test_invalid_input_has_no_upload_or_motion(self):
        for slot in (0,7,True):
            with self.assertRaises(ValueError):self.prepare(slot=slot)
        for count in (0,17):
            with self.assertRaises(ValueError):self.prepare(count=count)
        with self.assertRaises(ValueError):self.c.prepare_point_scan([(0,0)],name=123)
        self.assertEqual(self.wire.frames,[])
        s=self.prepare()
        with self.assertRaises(ValueError):s.move_next_and_wait(timeout_s=0)
        self.assertFalse(any(f[4]==0x43 for f in self.wire.frames))


if __name__=='__main__':unittest.main()
