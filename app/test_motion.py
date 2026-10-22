import unittest

from motion import convert_output_motion


class MotionConversionTests(unittest.TestCase):
    def test_five_to_one_gearbox(self):
        motion = convert_output_motion(10.0, 150.0, 10000, 5.0)
        self.assertEqual(motion.motor_speed_rpm, 50)
        self.assertEqual(motion.motor_angle_deg, 750.0)
        self.assertEqual(motion.pulses, 20833)
        self.assertEqual(motion.actual_output_speed_rpm, 10.0)

    def test_safe_defaults(self):
        motion = convert_output_motion(0.2, 5.0, 10000, 50.0)
        self.assertEqual(motion.motor_speed_rpm, 10)
        self.assertEqual(motion.motor_angle_deg, 250.0)
        self.assertEqual(motion.pulses, 6944)

    def test_motor_speed_limit_is_enforced_after_gearing(self):
        with self.assertRaises(ValueError):
            convert_output_motion(101.0, 10.0, 10000, 50.0)

    def test_invalid_ratio_is_rejected(self):
        with self.assertRaises(ValueError):
            convert_output_motion(10.0, 10.0, 10000, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
