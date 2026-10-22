import math
import struct
import unittest
from unittest.mock import patch, Mock
from types import SimpleNamespace
from collections import deque
from app import MotorController
from app.test_motor_controller import FakeSerial
from app.protocol import (build_dual_limits,build_dual_home,verify_crc,
    CMD_READ_DUAL_LIMIT,CMD_CONFIG_DUAL_LIMIT,CMD_HOME_DUAL_LIMIT,CMD_MOVE_OUTPUT_RELATIVE)


class DualLimitTests(unittest.TestCase):
    def test_home_limit_rejection_reads_inputs_and_keeps_specific_diagnostic(self):
        from debug_panel import DebugPanel
        from test_limit_mode import Value
        from protocol import Response, CMD_READ_FEEDBACK
        p=DebugPanel.__new__(DebugPanel)
        p.app=SimpleNamespace(auto_renew=True,auto_motor=1,home_running=False,monitor_until=0,
            _lock_auto_axis=Mock(),root=Mock(),result_text=Value(""))
        p.jobs=deque();p.after_stop=None;p.goto_target=None;p.records=deque()
        p.live=Value("");p.input_text=Value("");p.actual_inputs={}
        p.actual_dual_inputs=(2,0,255,1)
        p.handle(Response(1,CMD_HOME_DUAL_LIMIT,8,0x4449,b"",b""))
        self.assertEqual(p.jobs[0][0][4],CMD_READ_FEEDBACK)
        self.assertIsNone(p.app.auto_motor)
        data=struct.pack(">iiiHHHiHBBBBih",0,0,0,3,10000,0,0,2,5,5,0,0,0,0)
        p.handle(Response(1,CMD_READ_FEEDBACK,0,0,data,b""))
        self.assertIn("DI2、DI0",p.live.get())
        self.assertIn("DI0原始电平=1，DI2原始电平=1",p.live.get())
        self.assertIn("复位未启动",p.live.get())
        self.assertTrue(all(item[0][4]==CMD_READ_FEEDBACK for item in p.jobs))

    def test_panel_preset_waits_for_stop_and_config_failure_cancels_search(self):
        from debug_panel import DebugPanel
        from test_limit_mode import Value
        from protocol import Response,CMD_READ_FEEDBACK
        p=DebugPanel.__new__(DebugPanel)
        p.app=SimpleNamespace(_stop=Mock(),_send=Mock(return_value=True),port=True,pending=None,
            auto_renew=False,home_running=False,result_text=Value(""),root=Mock())
        p.window=None;p.ready=Mock(return_value=True);p.error=Mock();p.live=Value("")
        p.jobs=deque();p.after_stop=None;p.goto_target=None
        # No editable polarity/direction inputs: the measured preset is fixed.
        p.dual={"speed":Value("0.8")}
        with patch("debug_panel.messagebox.askyesno",return_value=True):p.dual_raised_home()
        p.app._stop.assert_called_once();p.app._send.assert_not_called()
        frames=[item[0] for item in p.after_stop[2][1]]
        self.assertEqual([f[4] for f in frames],[CMD_CONFIG_DUAL_LIMIT,CMD_READ_DUAL_LIMIT,CMD_HOME_DUAL_LIMIT])
        self.assertEqual(frames[-1][5:7],b'\x44\x49')
        self.assertEqual(struct.unpack(">biii",frames[-1][7:-2]),(-1,800,45000,120000))
        self.assertEqual(struct.unpack(">BBBBbBii",frames[0][7:-2]),(1,2,0,0,-1,3,-29000,35000))
        p.stopped()
        self.assertEqual(p.app._send.call_args.args[0][4],CMD_READ_FEEDBACK)
        p.actual_inputs={};p.input_text=Value("");p.records=deque()
        data=struct.pack(">iiiHHHiHBBBBih",0,0,0,3,10000,0,0,2,63,0,0,0,0,0)
        p.handle(Response(1,CMD_READ_FEEDBACK,0,0,data,b""))
        self.assertEqual(p.jobs[0][0][4],CMD_CONFIG_DUAL_LIMIT)
        p.handle(Response(1,CMD_CONFIG_DUAL_LIMIT,1,0,b"",b""))
        self.assertFalse(p.jobs)

    def test_existing_reference_returns_to_35_without_config_or_homing(self):
        from debug_panel import DebugPanel
        from test_limit_mode import Value
        from protocol import Response,CMD_READ_FEEDBACK
        for angle in (0,10000,35007):
            p=DebugPanel.__new__(DebugPanel)
            p.app=SimpleNamespace(home_running=False,_send=Mock(return_value=True),
                result_text=Value(""),root=Mock(),speed_rpm=Value("0.6"))
            p.jobs=deque();p.goto_target=None;p.records=deque();p.actual_inputs={}
            p.live=Value("");p.input_text=Value("");p.error=Mock()
            p.raised_request=(0.8/6,[(b"must not configure","unused")])
            data=struct.pack(">iiiHHHiHBBBBih",100,100,0,3,10000,0,angle,7,63,0,0,0,100,0)
            p.handle(Response(1,CMD_READ_FEEDBACK,0,0,data,b""))
            self.assertFalse(p.jobs);self.assertIsNone(p.raised_request)
            if angle==35007:p.app._send.assert_not_called()
            else:
                frame=p.app._send.call_args.args[0]
                self.assertEqual(frame[4],CMD_MOVE_OUTPUT_RELATIVE)
                rpm,delta=struct.unpack(">Hi",frame[7:-2])
                self.assertEqual(rpm,13) # selected output rpm, protocol 0.01 rpm units
                self.assertEqual(delta,35000-angle)
                p.cancel();self.assertIsNone(p.goto_target)

    def test_stop_cancels_reference_check_without_queuing_homing(self):
        from debug_panel import DebugPanel
        p=DebugPanel.__new__(DebugPanel);p.jobs=deque();p.after_stop=None;p.goto_target=None
        p.raised_request=(1.0,[(b"config","unused")])
        p.cancel()
        self.assertIsNone(p.raised_request);self.assertFalse(p.jobs)

    def test_default_channels_and_unmeasured_angles(self):
        frame=build_dual_limits()
        self.assertTrue(verify_crc(frame))
        self.assertEqual(struct.unpack(">BBBBbBii",frame[7:-2]),(1,2,0,1,1,0,0,0))

    def test_angles_and_inverted_direction(self):
        frame=build_dual_limits(horizontal_deg=1.5,raised_deg=28,positive_is_up=False,active_high=False)
        self.assertEqual(struct.unpack(">BBBBbBii",frame[7:-2]),(1,2,0,0,-1,3,1500,28000))
        for direction in (-1,1):
            frame=build_dual_home(direction)
            self.assertTrue(verify_crc(frame))
            self.assertEqual(struct.unpack(">biii",frame[7:-2]),(direction,300,40000,120000))

    def test_invalid_calibration_rejected_before_io(self):
        for kwargs in (dict(horizontal_channel=0),dict(horizontal_deg=math.nan),dict(raised_deg=36),dict(horizontal_deg=-30),
                       dict(horizontal_deg=20,raised_deg=10),dict(positive_is_up=None)):
            with self.assertRaises(ValueError):build_dual_limits(**kwargs)
        for direction in (0,True,2):
            with self.assertRaises(ValueError):build_dual_home(direction)
        for kwargs in (dict(speed_deg_s=10),dict(max_travel_deg=71),dict(timeout_s=math.inf)):
            with self.assertRaises(ValueError):build_dual_home(1,**kwargs)

    def test_sdk_config_and_home_keep_reference_search_separate_from_zero_move(self):
        serial=FakeSerial()
        serial.custom_responses[CMD_READ_DUAL_LIMIT]=struct.pack(">BBBBbBiiBB",1,2,0,1,-1,3,1500,28000,2,0)
        controller=MotorController("TEST",timeout=0.5,serial_factory=lambda *_a,**_kw:serial).connect()
        try:
            cfg=controller.configure_axis1_limits(horizontal_deg=1.5,raised_deg=28,positive_is_up=False)
            self.assertEqual(cfg["last_hit"],"raised")
            self.assertEqual(cfg["horizontal_channel"],2)
            self.assertFalse(cfg["positive_is_up"])
            with patch("app.motor_controller.threading.Thread"):
                controller.start_axis1_limit_homing(-1)
            self.assertIn(1,controller._home_axes)
            ops=[f[4] for f in serial.frames]
            self.assertIn(CMD_CONFIG_DUAL_LIMIT,ops)
            self.assertIn(CMD_HOME_DUAL_LIMIT,ops)
            self.assertNotIn(CMD_MOVE_OUTPUT_RELATIVE,ops)
        finally:controller.close(stop=False)

    def test_measured_raised_preset_and_target_key(self):
        serial=FakeSerial()
        serial.custom_responses[CMD_READ_DUAL_LIMIT]=struct.pack(">BBBBbBiiBB",1,2,0,1,-1,3,-29000,35000,0,0)
        controller=MotorController("TEST",timeout=0.5,serial_factory=lambda *_a,**_kw:serial).connect()
        try:
            with patch("app.motor_controller.threading.Thread"):
                controller.home_axis1_raised(active_high=True)
            cfg=next(f for f in serial.frames if f[4]==CMD_CONFIG_DUAL_LIMIT)
            self.assertEqual(struct.unpack(">BBBBbBii",cfg[7:-2]),(1,2,0,1,-1,3,-29000,35000))
            home=next(f for f in serial.frames if f[4]==CMD_HOME_DUAL_LIMIT)
            self.assertEqual(home[5:7],b'\x44\x49')
            self.assertEqual(struct.unpack(">biii",home[7:-2]),(-1,500,45000,120000))
            self.assertNotIn(CMD_MOVE_OUTPUT_RELATIVE,[f[4] for f in serial.frames])
        finally:controller.close(stop=False)
        with self.assertRaises(ValueError):build_dual_home(1,raised_only=True)


if __name__=="__main__":unittest.main()
