import unittest
from protocol import command, parse_line

class ProtocolTests(unittest.TestCase):
    def test_round_trip(self):
        self.assertEqual(command(7, "RUN", rpm=-50), "@C 7 RUN rpm=-50\n")
        r = parse_line("@R 7 OK result=0 state=3")
        self.assertEqual((r.kind, r.rid, r.status, r.fields["state"]), ("R", 7, "OK", "3"))
    def test_log_and_bad_frame_are_ignored(self):
        self.assertIsNone(parse_line("STATE=3 speed=0"))
        self.assertIsNone(parse_line("@R nope OK"))
    def test_status_frame(self):
        r = parse_line("@S 0 FAIL valid=0 age_ms=999999")
        self.assertEqual(r.kind, "S"); self.assertEqual(r.fields["valid"], "0")

if __name__ == "__main__": unittest.main()
