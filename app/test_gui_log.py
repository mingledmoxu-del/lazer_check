import unittest
from unittest.mock import Mock

from main import SimpleServoApp


class GuiLogTests(unittest.TestCase):
    def app(self):
        a=SimpleServoApp.__new__(SimpleServoApp)
        a.root=Mock();a.log=Mock();a.log.index.return_value='100.0'
        a._log_queue=[];a._log_after=None
        return a

    def test_protocol_logging_does_not_redraw_widget_inline(self):
        a=self.app()
        for n in range(20):a._log(f'TX {n}')
        a.log.insert.assert_not_called();a.log.see.assert_not_called()
        a.root.after.assert_called_once()
        a._flush_log()
        a.log.insert.assert_called_once();a.log.see.assert_called_once()
        self.assertIn('TX 19',a.log.insert.call_args.args[1])
        self.assertEqual(a._log_queue,[])
        a._log('next');self.assertEqual(a.root.after.call_count,2)

    def test_long_session_bounds_display_size(self):
        a=self.app();a.log.index.return_value='3001.0'
        a._log('new');a._flush_log()
        a.log.delete.assert_called_once_with('1.0','1002.0')


if __name__=='__main__':unittest.main()
