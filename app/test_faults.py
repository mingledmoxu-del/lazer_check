import re
import struct
import unittest
from dataclasses import asdict, replace
from pathlib import Path
from unittest.mock import Mock, patch

from app import (MotorController, MotorError, CommunicationError, DriveAlarm,
                 CommandRejected, get_fault_advice, explain_error)
from app.faults import _TRAJECTORY, _GUARD, _STATUS, _DRIVE
from app.protocol import (STATUS_TEXT, Response, CMD_TRAJECTORY_STATUS, CMD_READ_AXES,
                          CMD_CONFIG_GUARD, CMD_READ_GUARD, CMD_READ_FEEDBACK, CMD_READ_ALARM)
from app.trajectory import FAULTS, AxisPosition
from app.test_reference_api import ReferenceSerial


class GuardSerial(ReferenceSerial):
    def __init__(self):
        super().__init__()
        self.valid = {1: True, 2: True}
        self.reference = {1: 0, 2: 0}
        self.fault = 1; self.inputs = 0; self.trajectory_state = 0
        self.hardware = True; self.soft = True; self.ignore_clear = False

    def write(self, frame):
        command = frame[4]
        if command == CMD_CONFIG_GUARD and not self.ignore_clear:
            self.fault = 0
        result = super().write(frame)
        if self.mute or self.mute_command == command:
            return result
        # Supply realistic responses after the shared fixture records the frame.
        if command == CMD_READ_GUARD:
            data = struct.pack(">BBBBbBBBiiB", 0, 1, 2, 1, -1, 1, int(self.hardware), int(self.soft), -35000, 35000, 0)
        elif command == CMD_READ_FEEDBACK:
            data = struct.pack(">iiiHHHiHBBBBih", self.counts[frame[3]], 0, 0, 3, 10000, 17,
                self.angle(frame[3]), int(self.valid[frame[3]]) | (2 if self.soft else 0), 0, self.inputs, 0, self.fault,
                self.origins[frame[3]], self.rpm)
        elif command == CMD_TRAJECTORY_STATUS:
            data = struct.pack(">HBBHHIIIIiiii", 1, self.trajectory_state, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        else:
            return result
        from app.protocol import append_crc
        self.buffer.clear()
        payload = bytes((frame[3], command, 0)) + frame[5:7] + data
        self.buffer.extend(append_crc(b"\xAA\x55" + bytes((len(payload)+2,)) + payload))
        return result


class FaultCatalogTests(unittest.TestCase):
    def test_all_firmware_fault_namespaces_have_explicit_guidance(self):
        root = Path(__file__).resolve().parents[1] / "LightCheck/Communication/Inc"
        trajectory = (root / "trajectory.h").read_text(encoding="utf-8")
        enum = re.search(r"enum\s*\{\s*TJ_OK(.*?)\};", trajectory, re.S).group(0)
        self.assertEqual(len(re.findall(r"\bTJ_[A-Z_]+\b", enum)), len(_TRAJECTORY))
        guard = (root / "axis_guard.h").read_text(encoding="utf-8")
        enum = re.search(r"enum\s*\{\s*GUARD_OK(.*?)\};", guard, re.S).group(0)
        self.assertEqual(len(re.findall(r"\bGUARD_[A-Z_]+\b", enum)), len(_GUARD))
        protocol = (root / "pc_protocol.h").read_text(encoding="utf-8")
        codes = {int(x) for x in re.findall(r"PC_STATUS_\w+\s*=\s*(\d+)U", protocol)}
        self.assertEqual(codes, set(_STATUS))
        self.assertEqual(set(_TRAJECTORY), set(range(len(FAULTS))))
        self.assertEqual(set(_STATUS), set(STATUS_TEXT))
        for domain, codes in (("trajectory", _TRAJECTORY), ("guard", _GUARD), ("status", _STATUS)):
            for code in codes:
                advice = get_fault_advice(domain, code)
                self.assertTrue(advice.steps)
                self.assertNotIn("未识别", advice.title)
                for api in advice.apis:
                    self.assertTrue(callable(getattr(MotorController, api, None)), api)

    def test_exception_codes_do_not_cross_namespaces(self):
        rejected = CommandRejected(Response(2, 0x25, 7, 0, b"", b""))
        self.assertEqual(rejected.recovery.domain, "status")
        self.assertIn("版本", rejected.recovery.steps[0])
        self.assertIn("跟随", get_fault_advice("trajectory", 7).title)
        self.assertEqual(explain_error(DriveAlarm(2, 9)).motor_id, 2)
        self.assertTrue(explain_error(CommunicationError("timeout")).execution_unknown)
        for code in (1, 2, 3, 5):
            self.assertTrue(get_fault_advice("status", code).execution_unknown)
        self.assertEqual(explain_error(ValueError("accel")).code, "parameter")
        self.assertEqual(MotorError("停稳超时").recovery.code, "condition")

    def test_unknown_codes_have_no_automatic_recovery(self):
        for domain in ("trajectory", "guard", "status", "drive", "other"):
            advice = get_fault_advice(domain, 234, detail="raw reply")
            self.assertTrue(advice.steps)
            self.assertEqual(asdict(advice)["detail"], "raw reply")
        self.assertIn("未识别代码", get_fault_advice("drive", 234).steps[0])

    def test_manual_driver_codes_have_specific_remedies_and_real_apis(self):
        expected = {1,2,3,4,5,6,7,8,10,11,13,14,18,19,20,21,23,28,29,
                    38,39,40,41,42,46,47,50,51,52,53,54,55,56,57,58,59}
        self.assertEqual(set(_DRIVE), expected)
        for code in expected:
            advice = get_fault_advice("drive", code)
            self.assertNotIn("未识别", advice.title)
            self.assertTrue(advice.steps[0])
            for name in advice.apis:
                self.assertTrue(callable(getattr(MotorController, name, None)))
        self.assertIn("厂家", get_fault_advice("drive", 53).steps[0])
        self.assertIn("不是编码器复位", get_fault_advice("drive", 59).steps[0])

    def test_report_preserves_partial_results_and_unusable_position(self):
        api = MotorController("TEST")
        api.get_trajectory_status = Mock(side_effect=MotorError("unsupported"))
        pos = AxisPosition(None, 0, 0, 0, 2000, 9, 3, 2)
        api.get_axis_positions = Mock(return_value=(pos, replace(pos, alarm_code=0, guard_fault=0)))
        report = api.get_fault_report()
        self.assertFalse(report.complete)
        self.assertIsNone(report.trajectory)
        self.assertEqual({x.domain for x in report.issues}, {"sdk", "drive", "guard", "trajectory"})
        self.assertEqual(len(report.axes), 2)


class FaultRecoveryApiTests(unittest.TestCase):
    def setUp(self):
        self.serial = GuardSerial()
        # Align the artificial encoder counters with the established reference.
        self.serial.origins = dict(self.serial.counts)
        self.api = MotorController("TEST", timeout=0.03, serial_factory=lambda *_a, **_kw: self.serial).connect()
        sleep = patch("app.motor_controller.time.sleep", return_value=None)
        sleep.start(); self.addCleanup(sleep.stop)
        self.addCleanup(lambda: self.api.close(stop=False))

    def test_clear_guard_preserves_reference_policy_and_sends_no_motion(self):
        origins = dict(self.serial.origins)
        self.assertEqual(self.api.clear_guard_fault(1).fault, 0)
        self.assertEqual(self.serial.origins, origins)
        frames = self.serial.frames
        writes = [f for f in frames if f[4] == CMD_CONFIG_GUARD]
        self.assertEqual(len(writes), 1)
        self.assertEqual(writes[0][5:9], b"\0\3\7\xff")
        self.assertTrue(all(f[4] in (CMD_TRAJECTORY_STATUS, 6, 7, CMD_READ_ALARM,
                                    CMD_READ_GUARD, CMD_READ_FEEDBACK, CMD_CONFIG_GUARD) for f in frames))

    def assert_refused_without_ack(self):
        with self.assertRaises(MotorError):
            self.api.clear_guard_fault(1)
        self.assertFalse(any(f[4] == CMD_CONFIG_GUARD for f in self.serial.frames))

    def test_active_limit_refuses_ack(self):
        self.serial.inputs = 1
        self.assert_refused_without_ack()

    def test_unbounded_axis2_keeps_its_nonzero_reference(self):
        self.serial.soft = False
        self.serial.reference[2] = 150000
        actual = self.api.clear_guard_fault(2)
        self.assertEqual(actual.output_angle_deg, 150)
        self.assertFalse(actual.flags & 2)
        self.assertEqual(self.serial.reference[2], 150000)
        self.assertEqual(self.serial.origins[2], self.serial.counts[2])

    def test_invalid_reference_refuses_ack(self):
        self.serial.valid[1] = False
        self.assert_refused_without_ack()

    def test_drive_alarm_refuses_ack(self):
        self.serial.alarm = 9
        self.assert_refused_without_ack()

    def test_out_of_range_refuses_ack(self):
        self.serial.counts[1] += 100000
        self.assert_refused_without_ack()

    def test_preloaded_or_active_task_refuses_ack(self):
        for state in (1, 2, 3, 4, 5):
            with self.subTest(state=state):
                self.serial.frames.clear(); self.serial.trajectory_state = state
                self.assert_refused_without_ack()

    def test_execution_clear_uses_explicit_ack_and_preserves_reference(self):
        self.serial.fault = 8
        before = dict(self.serial.origins)
        result = self.api.clear_guard_fault(1)
        self.assertEqual(result.fault, 0)
        self.assertEqual(self.serial.origins, before)
        config_frames = [f for f in self.serial.frames if f[4] == CMD_CONFIG_GUARD]
        self.assertEqual(len(config_frames), 1)
        self.assertEqual(config_frames[0][5:7], b'\x00\x04')

    def test_execution_clear_readback_failure_is_reported(self):
        self.serial.fault = 8; self.serial.ignore_clear = True
        with self.assertRaisesRegex(MotorError, "尚未清除"):
            self.api.clear_guard_fault(1)

    def test_failed_clear_is_not_reported_as_success(self):
        self.serial.ignore_clear = True
        with self.assertRaisesRegex(MotorError, "未清除"):
            self.api.clear_guard_fault(1)

    def test_configuration_readback_change_is_rejected(self):
        real = self.api.set_guard_policy
        def changed(*args, **kwargs):
            return replace(real(*args, **kwargs), soft_enabled=False)
        self.api.set_guard_policy = changed
        with self.assertRaisesRegex(MotorError, "读回发生变化"):
            self.api.clear_guard_fault(1)

    def test_limit_retrigger_after_ack_is_rejected(self):
        real = self.api.set_guard_policy
        def retrigger(*args, **kwargs):
            config = real(*args, **kwargs)
            self.serial.inputs = 1
            return config
        self.api.set_guard_policy = retrigger
        with self.assertRaisesRegex(MotorError, "再次触发"):
            self.api.clear_guard_fault(1)

    def test_moving_motor_cannot_be_acknowledged(self):
        self.serial.rpm = 1
        # Short timeout exercises the actual two-zero-observation stop gate.
        real = self.api.stop
        self.api.stop = lambda axis: real(axis, timeout=0.001)
        self.assert_refused_without_ack()

    def test_read_only_report_never_resets_or_moves(self):
        self.serial.fault = 0
        report = self.api.get_fault_report()
        self.assertTrue(report.complete)
        self.assertEqual(report.issues, ())
        self.assertEqual([f[4] for f in self.serial.frames], [CMD_TRAJECTORY_STATUS, CMD_READ_AXES])

    def test_broken_connection_report_is_incomplete_and_does_not_retry_motion(self):
        self.serial.mute_command = CMD_TRAJECTORY_STATUS
        report = self.api.get_fault_report()
        self.assertFalse(report.complete)
        self.assertTrue(report.issues[0].execution_unknown)
        self.assertEqual(len(self.serial.frames), 1)


if __name__ == "__main__":
    unittest.main()
