import math
import struct
import unittest
from unittest.mock import patch

from app import MotorController, MotorError, CommunicationError, CommandRejected
from app.protocol import (CMD_SET_REFERENCE, CMD_SET_ZERO, CMD_READ_FEEDBACK, CMD_READ_AXES,
                          CMD_MOVE_OUTPUT_RELATIVE, build_set_reference, verify_crc)
from app.test_motor_controller import FakeSerial


class ReferenceSerial(FakeSerial):
    """Encoder counts advance only when the test moves them, not on command acceptance."""
    def __init__(self):
        super().__init__()
        self.counts={1:100000,2:200000};self.origins={1:0,2:0};self.reference={1:25000,2:0}
        self.valid={1:False,2:False};self.age=0;self.bad_echo=False;self.bad_readback=False

    def angle(self,axis):
        delta=(self.counts[axis]-self.origins[axis])*360000
        return (1 if delta>=0 else -1)*(abs(delta)//500000)+self.reference[axis]

    def write(self,frame):
        axis,command=frame[3],frame[4]
        if command in (CMD_SET_REFERENCE,CMD_SET_ZERO) and self.reject_command!=command:
            angle=struct.unpack_from(">i",frame,7)[0] if command==CMD_SET_REFERENCE else (25000 if axis==1 else 0)
            self.origins[axis]=self.counts[axis];self.reference[axis]=angle;self.valid[axis]=True
            self.custom_responses[command]=struct.pack(">i",angle+int(self.bad_echo)) if command==CMD_SET_REFERENCE else b"\0\0"
        if command==CMD_READ_FEEDBACK:
            angle=self.angle(axis) if self.valid[axis] else 0
            self.custom_responses[command]=struct.pack(">iiiHHHiHBBBBih",self.counts[axis],999999,0,3,10000,17,
                angle+(1000 if self.bad_readback else 0),int(self.valid[axis]) | (2 if axis==1 else 0),0,0,0,0,self.origins[axis],self.rpm)
        if command==CMD_READ_AXES:
            self.custom_responses[command]=b"".join(struct.pack(">iihHIHBB",
                self.angle(i) if self.valid[i] else 0,self.counts[i],self.rpm,
                int(self.valid[i]) | (2 if i==1 else 0),self.age,self.alarm,0,0) for i in (1,2))
        return super().write(frame)


class ReferenceApiTests(unittest.TestCase):
    def setUp(self):
        self.serial=ReferenceSerial()
        self.motors=MotorController("TEST",timeout=0.1,serial_factory=lambda *_a,**_kw:self.serial).connect()
        sleep=patch("app.motor_controller.time.sleep");sleep.start();self.addCleanup(sleep.stop)

    def tearDown(self):self.motors.close(stop=False)

    def test_capture_zero_and_read_actual_forward_reverse_travel(self):
        reference=self.motors.set_current_angle(1,0)
        self.assertEqual(reference.output_angle_deg,0)
        frame=next(f for f in self.serial.frames if f[4]==CMD_SET_REFERENCE)
        self.assertEqual(struct.unpack_from(">H",frame,5)[0],0x5246);self.assertTrue(verify_crc(frame))
        for counts,angle in ((106945,5),(120834,15),(116667,12)):
            self.serial.counts[1]=counts
            self.assertEqual(self.motors.get_current_angle(1),angle)
        self.motors.move_relative(1,1)
        self.assertEqual(self.motors.get_current_angle(1),12)  # No feedback movement yet.

    def test_reference_capture_itself_never_sends_motion(self):
        self.motors.set_current_angle(1,0)
        self.assertNotIn(CMD_MOVE_OUTPUT_RELATIVE,[f[4] for f in self.serial.frames])

    def test_axis2_arbitrary_reference_and_multi_turn_angle(self):
        self.motors.set_current_reference(2,angle_deg=150)
        self.serial.counts[2]=1200000
        self.assertEqual(self.motors.get_current_angle(2),870)

    def test_legacy_reference_still_captures_axis1_as_twenty_five(self):
        self.assertEqual(self.motors.set_current_reference(1).output_angle_deg,25)
        self.assertEqual(self.motors.set_current_reference(1,0).output_angle_deg,0)

    def test_unknown_or_stale_angle_is_not_zero(self):
        with self.assertRaises(MotorError):self.motors.get_current_angle(1)
        self.motors.set_current_angle(1,0);self.serial.age=1001
        with self.assertRaises(MotorError):self.motors.get_current_angle(1)

    def test_bad_reference_rejected_before_serial_io(self):
        for axis,angle in ((1,36),(2,math.nan),(2,36001),(0,0)):
            with self.assertRaises(ValueError):self.motors.set_current_angle(axis,angle)
        self.assertFalse(self.serial.frames)
        self.assertEqual(struct.unpack_from(">i",build_set_reference(2,-12.345),7)[0],-12345)

    def test_old_firmware_does_not_fall_back_to_twenty_five(self):
        self.serial.reject_command=CMD_SET_REFERENCE;self.serial.reject_status=7
        with self.assertRaises(CommandRejected):self.motors.set_current_angle(1,0)
        self.assertNotIn(CMD_SET_ZERO,[f[4] for f in self.serial.frames]);self.assertFalse(self.serial.valid[1])

    def test_echo_and_readback_mismatch_are_reported(self):
        self.serial.bad_readback=True
        with self.assertRaises(MotorError):self.motors.set_current_angle(1,0)
        self.serial.bad_readback=False;self.serial.bad_echo=True
        with self.assertRaises(CommunicationError):self.motors.set_current_angle(1,0)
        self.assertNotIn(CMD_MOVE_OUTPUT_RELATIVE,[f[4] for f in self.serial.frames])


if __name__=="__main__":unittest.main()
