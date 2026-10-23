"""Offline unit tests; no pip, camera or Arduino calls."""
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import livekit_install_test as lk


class LivekitInstallOfflineTest(unittest.TestCase):
    def test_compatibility_old_glibc(self):
        with patch.object(lk.sys, 'platform', 'linux'), patch.object(lk, 'glibc_version', return_value=(2, 27)):
            self.assertFalse(lk.supported_binary_platform())

    def test_compatibility_new_glibc(self):
        with patch.object(lk.sys, 'platform', 'linux'), patch.object(lk, 'glibc_version', return_value=(2, 31)):
            self.assertTrue(lk.supported_binary_platform())

    def test_failed_install_keeps_robot_clean(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            env = base/'livekit-test'
            (env/'bin').mkdir(parents=True)
            (env/'bin'/'python').touch()
            with patch.object(lk, 'RESULT_DIR', base), patch.object(lk, 'TEST_ENV', env), patch.object(lk, 'STATUS_FILE', base/'status.json'), \
                    patch.object(lk, '_run', return_value=1) as runner, patch.object(lk, 'supported_binary_platform', return_value=False):
                self.assertEqual(lk.test_install(), 2)
                self.assertEqual(len(runner.call_args_list), 1)
                argv = runner.call_args.args[0]
                self.assertIn('--only-binary=:all:', argv)
                self.assertNotIn('--user', argv)
                self.assertIn('livekit==1.1.21', argv)
                d = json.loads((base/'status.json').read_text())
                self.assertEqual(d['status'], 'install_failed')
                self.assertNotIn('LIVEKIT_API_SECRET', json.dumps(d))

    def test_success_probes_real_import(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            env = base/'livekit-test'
            (env/'bin').mkdir(parents=True)
            (env/'bin'/'python').touch()
            with patch.object(lk, 'RESULT_DIR', base), patch.object(lk, 'TEST_ENV', env), patch.object(lk, 'STATUS_FILE', base/'status.json'), \
                    patch.object(lk, '_run', side_effect=[0, 0]) as runner, patch.object(lk, 'supported_binary_platform', return_value=True):
                self.assertEqual(lk.test_install(), 0)
                self.assertEqual(len(runner.call_args_list), 2)
                self.assertIn('from livekit import rtc, api;', runner.call_args_list[-1].args[0][-1])
                self.assertEqual(json.loads((base/'status.json').read_text())['status'], 'ready_isolated')


if __name__ == '__main__':
    unittest.main()
