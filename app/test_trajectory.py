import math
import struct
import unittest
from unittest.mock import patch

from app import MotorController, TrajectoryPoint, joint_ellipse, MotorError, CommandRejected
from app.trajectory import validate_points, decode_status
from app.protocol import (CMD_TRAJECTORY_BEGIN, CMD_TRAJECTORY_POINT, CMD_TRAJECTORY_SEAL,
    CMD_TRAJECTORY_START, CMD_TRAJECTORY_STATUS, CMD_TRAJECTORY_STOP, CMD_TRAJECTORY_KEEPALIVE,
    CMD_STOP, CMD_READ_AXES, CMD_TRAJECTORY_CLEAR, build_trajectory_begin, build_trajectory_point, verify_crc)
from app.test_motor_controller import FakeSerial


class TrajectorySerial(FakeSerial):
    def __init__(self):
        super().__init__();self.session=0;self.state=0;self.count=0;self.loaded=0;self.fault=0

    def write(self, frame):
        command=frame[4];session=struct.unpack(">H",frame[5:7])[0]
        if command==CMD_TRAJECTORY_BEGIN:
            self.session=session;self.state=1;self.count=struct.unpack(">H",frame[7:9])[0];self.loaded=0
        elif command==CMD_TRAJECTORY_POINT:self.loaded+=1
        elif command==CMD_TRAJECTORY_SEAL:self.state=2
        elif command==CMD_TRAJECTORY_START:self.state=3
        elif command==CMD_TRAJECTORY_STOP:self.state=8
        elif command==CMD_TRAJECTORY_CLEAR and self.reject_command!=command:
            self.session=self.state=self.fault=self.count=self.loaded=0
        if CMD_TRAJECTORY_BEGIN<=command<=CMD_TRAJECTORY_KEEPALIVE or command==CMD_TRAJECTORY_CLEAR:
            self.custom_responses[command]=struct.pack(">HBBHHIIIIiiii",self.session,self.state,self.fault,
                self.count,self.loaded,0,5000,250,180,25000,0,0,0)
        return super().write(frame)


class TrajectoryTests(unittest.TestCase):
    def test_joint_example_closed_and_millisecond_normalized(self):
        points=joint_ellipse(amplitude1=2,amplitude2=3,period_s=5)
        self.assertEqual(len(points),33)
        self.assertEqual(points[0],TrajectoryPoint(0,2,0))
        self.assertEqual(points[-1],TrajectoryPoint(5,2,0))
        self.assertTrue(all(0.05<=b.time_s-a.time_s for a,b in zip(points,points[1:])))

    def test_invalid_points_are_rejected(self):
        for points in ((TrajectoryPoint(1,0,0),TrajectoryPoint(2,0,0)),
                       (TrajectoryPoint(0,0,0),TrajectoryPoint(0.01,0,0)),
                       (TrajectoryPoint(0,math.nan,0),TrajectoryPoint(2,0,0))):
            with self.assertRaises(ValueError):validate_points(points)
        with self.assertRaises(ValueError):joint_ellipse(cycles=5)

    def test_frame_units_bounds_and_crc(self):
        frame=build_trajectory_begin(17,3)
        self.assertEqual(frame[4],CMD_TRAJECTORY_BEGIN)
        self.assertEqual(struct.unpack(">HHiiii",frame[7:-2]),(3,250,20000,30000,5000,500))
        self.assertTrue(verify_crc(frame))
        point=build_trajectory_point(17,1,250,-25,5)
        self.assertEqual(struct.unpack(">HIii",point[7:-2]),(1,250,-25000,5000))
        with self.assertRaises(ValueError):build_trajectory_begin(0,3)
        with self.assertRaises(ValueError):build_trajectory_begin(1,3,cycle_ms=20)

    def test_status_length_and_fields(self):
        with self.assertRaises(ValueError):decode_status(b"bad")
        raw=struct.pack(">HBBHHIIIIiiii",1,3,0,3,3,1000,5000,270,190,25000,-1000,500,-300)
        s=decode_status(raw)
        self.assertTrue(s.active);self.assertEqual(s.axis1_deg,25);self.assertEqual(s.error2_deg,-0.3)


class TrajectoryApiTests(unittest.TestCase):
    def setUp(self):
        self.serial=TrajectorySerial()
        self.api=MotorController("TEST",timeout=0.5,serial_factory=lambda *_a,**_kw:self.serial)
        self.api.connect()
        self.sleep=patch("app.motor_controller.time.sleep",return_value=None);self.sleep.start();self.addCleanup(self.sleep.stop)
        self.thread=patch("app.motor_controller.threading.Thread.start");self.thread.start();self.addCleanup(self.thread.stop)

    def tearDown(self):self.api.close(stop=False)

    def prepare(self):
        return self.api.prepare_trajectory((TrajectoryPoint(0,25,0),TrajectoryPoint(2.5,26,1),TrajectoryPoint(5,25,0)),session=17)

    def test_prepare_does_not_start(self):
        s=self.prepare()
        self.assertEqual(s.state,2)
        commands=[frame[4] for frame in self.serial.frames]
        self.assertNotIn(CMD_TRAJECTORY_START,commands)
        self.assertEqual(commands.count(CMD_STOP),2)
        self.assertEqual(commands.count(CMD_TRAJECTORY_POINT),3)

    def test_start_heartbeat_group_stop_never_restarts(self):
        self.prepare();self.api.start_trajectory(17)
        self.api._renew_continuous();self.assertEqual(self.serial.frames[-1][4],CMD_TRAJECTORY_KEEPALIVE)
        self.api.stop(2)
        self.assertEqual(self.serial.frames[-1][4],CMD_TRAJECTORY_STOP)
        count=len(self.serial.frames);self.api._renew_continuous()
        self.assertEqual(len(self.serial.frames),count)

    def test_terminal_status_cancels_heartbeat(self):
        self.prepare();self.api.start_trajectory(17);self.serial.state=6
        result=self.api.wait_trajectory(17,timeout=1)
        self.assertEqual(result.state,6);self.assertIsNone(self.api._trajectory_session)

    def test_unsupported_firmware_probe_does_not_stop_or_upload(self):
        self.serial.reject_command=CMD_TRAJECTORY_STATUS;self.serial.reject_status=7
        with self.assertRaises(CommandRejected):self.prepare()
        self.assertEqual([f[4] for f in self.serial.frames],[CMD_TRAJECTORY_STATUS])

    def test_bad_input_does_not_write(self):
        with self.assertRaises(ValueError):self.api.prepare_trajectory([])
        self.assertEqual(self.serial.frames,[])

    def test_cancelled_trajectory_is_not_completion(self):
        self.prepare();self.api.start_trajectory(17);self.serial.state=8
        with self.assertRaises(MotorError):self.api.wait_trajectory(17,timeout=1)

    def test_acknowledging_fault_invalidates_old_session_and_does_not_move(self):
        self.prepare();self.serial.state=7;self.serial.fault=7
        self.serial.frames.clear()
        result=self.api.clear_trajectory_fault()
        self.assertEqual((result.state,result.fault,result.session),(0,0,0))
        self.assertEqual([f[4] for f in self.serial.frames],[CMD_TRAJECTORY_STATUS,CMD_TRAJECTORY_CLEAR])

    def test_clear_rejects_running_or_preloaded_task(self):
        self.prepare();self.serial.frames.clear()
        with self.assertRaises(MotorError):self.api.clear_trajectory_fault()
        self.assertEqual([f[4] for f in self.serial.frames],[CMD_TRAJECTORY_STATUS])
        self.serial.state=3;self.serial.frames.clear()
        with self.assertRaises(MotorError):self.api.clear_trajectory_fault()
        self.assertEqual([f[4] for f in self.serial.frames],[CMD_TRAJECTORY_STATUS])

    def test_old_firmware_does_not_fake_successful_fault_clear(self):
        self.prepare();self.serial.state=7;self.serial.fault=7
        self.serial.reject_command=CMD_TRAJECTORY_CLEAR;self.serial.reject_status=7
        with self.assertRaises(CommandRejected):self.api.clear_trajectory_fault()
        self.assertEqual((self.serial.state,self.serial.fault),(7,7))

    def test_business_acceleration_five_is_encoded_in_wire_header(self):
        self.serial.custom_responses[CMD_READ_AXES]=b"".join(
            struct.pack(">iihHIHBB",0,0,0,flags,0,0,0,0) for flags in (3,1))
        points=(TrajectoryPoint(0,0,0),TrajectoryPoint(1,2,2),TrajectoryPoint(2,0,0))
        self.api.prepare_business_trajectory(points,max_speed=2,max_accel=5)
        begin=next(f for f in self.serial.frames if f[4]==CMD_TRAJECTORY_BEGIN)
        limits=struct.unpack(">HHiiii",begin[7:-2])
        self.assertEqual(limits[1],0x8000|100);self.assertEqual(limits[2:4],(2000,5000))
        self.assertNotIn(0x15,[f[4] for f in self.serial.frames])


if __name__=="__main__":unittest.main()
