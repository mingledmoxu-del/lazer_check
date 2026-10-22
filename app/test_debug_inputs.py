import unittest
from debug_panel import format_inputs


class InputDisplayTests(unittest.TestCase):
    def test_unknown_mapping_does_not_claim_trigger(self):
        text = format_inputs(63, 63)
        self.assertIn("未读取映射", text)
        self.assertNotIn("：触发", text)

    def test_unassigned_channels_are_not_limits(self):
        text = format_inputs(62, 62, (255, 255, 0, 1))
        self.assertIn("DI0: raw=0 原点：未触发", text)
        self.assertIn("DI1: raw=1 未使用", text)

    def test_low_active_uses_firmware_state(self):
        text = format_inputs(61, 2, (255, 255, 1, 0))
        self.assertIn("DI1: raw=0 原点：触发", text)
        text = format_inputs(63, 0, (255, 255, 1, 0))
        self.assertIn("DI1: raw=1 原点：未触发", text)


if __name__ == "__main__":
    unittest.main()
