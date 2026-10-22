import struct
import unittest
import app.test_motor_controller as fixture
from app import MotorController, MotorError
from app.protocol import (CMD_READ_FEEDBACK,CMD_READ_GUARD,CMD_CONFIG_GUARD,CMD_SET_ZERO,CMD_HOME,
                          build_guard_inputs,build_soft_range,build_guard_policy,build_home,CMD_MOVE_OUTPUT_RELATIVE)

def feedback(flags=1,phase=0,mode=3):
    return struct.pack(">iiiHHHiHBBBBih",100000,100010,10,mode,10000,17,15000,flags,0,0,phase,0,79167,0)

class GuardApiTests(unittest.TestCase):
    def setUp(self):
        self.serial=fixture.FakeSerial()
        self.serial.custom_responses={CMD_READ_FEEDBACK:feedback(),CMD_READ_GUARD:struct.pack(">BBBBbBBBiiB",0,255,0,1,-1,1,1,0,0,120000,1)}
        self.controller=MotorController("TEST",timeout=0.5,serial_factory=lambda *_a,**_kw:self.serial)
        self.controller.connect()

    def tearDown(self):self.controller.close(stop=False)

    def test_feedback_keeps_units_separate(self):
        f=self.controller.get_feedback(1)
        self.assertEqual(f.actual_counts,100000)
        self.assertEqual(f.commanded_counts,100010)
        self.assertEqual(f.output_angle_deg,15)
        self.assertEqual(f.counts_per_motor_rev,10000)
        self.serial.custom_responses[CMD_READ_FEEDBACK]=feedback(flags=0,mode=0)
        f=self.controller.get_feedback(1)
        self.assertIsNone(f.output_angle_deg)
        self.assertEqual(f.counts_per_motor_rev,131072)

    def test_guard_config_signed_direction_and_unused_channel(self):
        cfg=self.controller.get_guard_config(1)
        self.assertEqual(cfg.home_direction,-1)
        self.assertIsNone(cfg.positive)
        self.assertEqual(cfg.maximum_deg,120)

    def test_homing_registers_heartbeat_and_terminal_status_removes_it(self):
        self.controller.start_homing(1,max_travel_deg=20)
        self.assertIn(1,self.controller._home_axes)
        self.assertEqual([f[4] for f in self.serial.frames].count(CMD_HOME),1)
        self.serial.custom_responses[CMD_READ_FEEDBACK]=feedback(phase=7)
        self.controller.wait_homing(1,timeout=1)
        self.assertNotIn(1,self.controller._auto_axes)

    def test_zero_requires_stop_before_unit_change(self):
        self.controller.set_current_zero(1)
        ops=[f[4] for f in self.serial.frames]
        self.assertEqual(ops[0],6)
        self.assertIn(CMD_SET_ZERO,ops)

    def test_absolute_target_requires_reference(self):
        self.serial.custom_responses[CMD_READ_FEEDBACK]=feedback(flags=0)
        with self.assertRaises(MotorError):self.controller.move_to_angle(1,30)
        self.assertNotIn(CMD_MOVE_OUTPUT_RELATIVE,[f[4] for f in self.serial.frames])

    def test_coordinate_move_confirms_stop_once(self):
        self.controller.move_to_angle(1,30)
        ops=[f[4] for f in self.serial.frames]
        self.assertEqual(ops.count(6),1)
        self.assertEqual(ops.count(CMD_MOVE_OUTPUT_RELATIVE),1)

    def test_mapping_and_policy_frames(self):
        frame=build_guard_inputs(1,0,None,0,True)
        self.assertEqual(frame[7:11],bytes((0,255,0,1)))
        frame=build_guard_policy(1,hardware=True,soft=False,home_direction=-1,confirmed=True,home_at_end=True)
        self.assertEqual(frame[7:9],bytes((13,255)))
        with self.assertRaises(ValueError):build_guard_inputs(1,0,0,1)
        with self.assertRaises(ValueError):build_guard_inputs(1,None,None,6)
        with self.assertRaises(ValueError):build_guard_policy(1,hardware=True,soft=False,confirmed=True)

    def test_range_and_home_bounds(self):
        with self.assertRaises(ValueError):build_soft_range(1,10,90)
        with self.assertRaises(ValueError):build_home(1,speed_deg_s=10)
        with self.assertRaises(ValueError):build_home(1,max_travel_deg=1,backoff_deg=2)
        self.assertEqual(len(build_home(1)),25)

if __name__=="__main__":unittest.main()
