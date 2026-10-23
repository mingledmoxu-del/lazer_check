import struct
import unittest
from unittest.mock import Mock,MagicMock

from app import MotorController,MotorError
from app.point_bank import PRESETS,upload_frames,run_route_point,decode_route
from app.protocol import Response
from app.test_motor_controller import FakeSerial
from app import test_joint_timing as joint_helpers


class BankSerial(FakeSerial):
    def __init__(self):
        super().__init__();self.token=self.speed=self.count=self.loaded=self.ready=0;self.tolerance=200

    def write(self,frame):
        command=frame[4]
        if command==0x40:self.token,self.speed,self.tolerance,self.count=struct.unpack_from('>IHHB',frame,7);self.loaded=self.ready=0
        if command==0x41:self.loaded+=1
        if command==0x42:self.ready=1
        if command in (0x40,0x41,0x42,0x44):self.custom_responses[command]=struct.pack('>IHHBBB',self.token,self.speed,self.tolerance,self.count,self.loaded,self.ready)
        if command==0x43:self.custom_responses[command]=struct.pack('>II',50,60)
        return super().write(frame)


class PointBankTests(unittest.TestCase):
    def test_preset_coordinates_and_upload_no_run_frame(self):
        points=PRESETS['四点测量（含起终原点）']
        self.assertEqual(points,((0,0),(6.9,0),(16.1,-9.7),(24.8,0),(16.1,9.7),(0,0)))
        frames=upload_frames(points,slot=1,token=123,speed_rpm=.25)
        self.assertEqual([f[4] for f in frames],[0x40]+[0x41]*6+[0x42])
        self.assertEqual(struct.unpack('>IBii',frames[3][7:20]),(123,2,16100,-9700))

    def test_validate_all_points_before_sending_begin(self):
        wire=BankSerial();c=MotorController('TEST',serial_factory=lambda *_a,**_kw:wire).connect()
        try:
            with self.assertRaises(ValueError):c.preload_route(((0,0),(float('nan'),0)))
            self.assertEqual(wire.frames,[])
        finally:c.close(stop=False)
        for slot,token,index in ((0,1,0),(1,0,0),(1,1,16),(1,1,-1)):
            with self.assertRaises(ValueError):run_route_point(slot,token,index)

    def test_sdk_upload_then_run_only_one_index(self):
        wire=BankSerial();rows=[]
        c=MotorController('TEST',serial_factory=lambda *_a,**_kw:wire,timing_callback=rows.append).connect()
        try:
            record=c.preload_route(PRESETS['四点测量（含起终原点）'],route_id=2,speed_rpm=.25,token=123)
            self.assertTrue(record['ready']);self.assertEqual(record['speed_rpm'],.25)
            self.assertFalse(any(f[4] in (0x1A,0x43) for f in wire.frames))
            before=len(wire.frames);result=c.run_preloaded_point(2,3)
            self.assertEqual(len(wire.frames),before+1)
            self.assertEqual(wire.frames[-1][4],0x43)
            self.assertEqual(wire.frames[-1][5:7],b'\x00\x02')
            self.assertEqual(struct.unpack('>IB',wire.frames[-1][7:12]),(123,3))
            self.assertEqual(result['targets_deg'],(24.8,0))
            self.assertEqual([r['axis'] for r in rows if r['event']=='motion_ack'],[1,2])
            self.assertTrue(c.get_preloaded_route(2)['ready'])
        finally:c.close(stop=False)

    def test_run_without_known_token_does_not_send(self):
        c=MotorController('TEST');c._request=Mock()
        with self.assertRaises(MotorError):c.run_preloaded_point(1,0)
        c._request.assert_not_called()
        with self.assertRaises(ValueError):decode_route(b'bad')

    def test_event_api_reuses_completion_and_route_tolerance_without_extra_stop(self):
        wire=BankSerial();c=MotorController('TEST',serial_factory=lambda *_a,**_kw:wire).connect()
        try:
            c.preload_route(((0,0),(6.9,0)),speed_rpm=.25,token=123,tolerance_deg=.2)
            c.wait_for_axes=Mock(return_value=('axis1-fresh','axis2-fresh'))
            before=len(wire.frames);done=c.move_preloaded_point_and_wait(1,1)
            self.assertEqual(len(wire.frames),before+1)
            c.wait_for_axes.assert_called_once_with((6.9,0),tolerance_deg=.2,timeout_s=120)
            self.assertTrue(done['completed']);self.assertEqual(done['positions'],('axis1-fresh','axis2-fresh'))
        finally:c.close(stop=False)

    def test_event_api_reuses_completion_and_route_tolerance_without_extra_stop(self):
        wire=BankSerial();c=MotorController('TEST',serial_factory=lambda *_a,**_kw:wire).connect()
        try:
            c.preload_route(((0,0),(6.9,0)),speed_rpm=.25,token=123,tolerance_deg=.2)
            c.wait_for_axes=Mock(return_value=('axis1-fresh','axis2-fresh'))
            before=len(wire.frames);done=c.move_preloaded_point_and_wait(1,1)
            self.assertEqual(len(wire.frames),before+1)
            c.wait_for_axes.assert_called_once_with((6.9,0),tolerance_deg=.2,timeout_s=120)
            self.assertTrue(done['completed']);self.assertEqual(done['positions'],('axis1-fresh','axis2-fresh'))
        finally:c.close(stop=False)

    def test_gui_seal_only_enables_point_selection_without_running(self):
        p=joint_helpers.JointTimingTests().panel();p.phase='uploading';p.awaiting=(1,0x42)
        p.upload_info=dict(route_id=1,token=123,points=((0,0),(6.9,0)),speed_rpm=.25,tolerance_deg=.2)
        p.upload_text='0,0\n6.9,0';p.upload_speed='0.25'
        p.plan=Mock();p.plan.get.return_value=p.upload_text
        p.speed=Mock();p.speed.get.return_value='0.25'
        p.tolerance=Mock();p.tolerance.get.return_value='0.2';p.upload_tolerance='0.2'
        p.route_id=Mock();p.route_id.get.return_value='1'
        p.point_combo=MagicMock();p.selected_point=Mock()
        p.handle(Response(1,0x42,0,1,struct.pack('>IHHBBB',123,25,200,2,2,1),b''))
        self.assertIsNone(p.phase);self.assertEqual(p.loaded_route['token'],123)
        self.assertEqual(len(p.jobs),0)

    def test_gui_trigger_uses_bank_index_and_waits_without_next_point(self):
        helper=joint_helpers.JointTimingTests();p=helper.panel()
        p.phase=None;p.loaded_route=dict(route_id=1,token=123,points=((0,0),(6.9,0)),speed_rpm=.25,tolerance_deg=.2)
        p.selected_point=Mock();p.selected_point.get.return_value='1: (6.9°, 0°)'
        p.app.business_panel=Mock();p.app.business_panel.available.return_value=True
        p.app._lock_auto_axis=Mock()
        p.run_point();helper.read(p)
        self.assertEqual(len(p.jobs),1);self.assertEqual(p.jobs[0][4],0x43)
        p.jobs.clear()
        for axis in (1,2):p.segment['axes'][axis]['sent']=0
        helper.read(p,6.9,0,0x41,0x41)
        self.assertIsNone(p.phase);self.assertEqual(len(p.jobs),0)


if __name__=='__main__':unittest.main()
