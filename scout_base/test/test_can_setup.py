import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'launch/can_setup.py'
spec = importlib.util.spec_from_file_location('can_setup', SCRIPT)
can = importlib.util.module_from_spec(spec)
spec.loader.exec_module(can)


class CanSetupTests(unittest.TestCase):
    def setUp(self):
        self.link = {'flags': ['UP'], 'linkinfo': {'info_kind': 'can', 'info_data': {
            'state': 'ERROR-ACTIVE', 'bittiming': {'bitrate': 500000}}}}
        self.calls = []
        self.error = False

    def ip(self, command, **kwargs):
        self.calls.append(command)
        if 'show' in command:
            return subprocess.CompletedProcess(command, 0, json.dumps([copy.deepcopy(self.link)]), '')
        if self.error:
            return subprocess.CompletedProcess(command, 1, '', 'Operation not permitted')
        if command[-1] == 'down':
            self.link['flags'] = []
        if command[-1] == 'up':
            self.link['flags'] = ['UP']
            self.link['linkinfo']['info_data']['state'] = 'ERROR-ACTIVE'
        if 'bitrate' in command:
            self.link['linkinfo']['info_data']['bittiming']['bitrate'] = int(command[-1])
        return subprocess.CompletedProcess(command, 0, '', '')

    def test_up_does_not_reconfigure_or_claim_robot_ready(self):
        with patch.object(can.subprocess, 'run', self.ip):
            can.ensure_can('can7')
        self.assertTrue(all('show' in c for c in self.calls))
        self.assertTrue(all('can7' in c for c in self.calls))

    def test_bring_up_configures_requested_interface_at_500k(self):
        self.link['flags'] = []
        self.link['linkinfo']['info_data']['bittiming']['bitrate'] = 250000
        with patch.object(can.subprocess, 'run', self.ip), patch.object(can.os, 'geteuid', return_value=1000):
            can.ensure_can('can7')
        changes = [c for c in self.calls if 'set' in c]
        self.assertEqual(len(changes), 3)
        self.assertTrue(all(c[:3] == ['sudo', '-n', 'ip'] and 'can7' in c for c in changes))
        self.assertIn('500000', changes[1])
        self.assertEqual(changes[-1][-1], 'up')

    def test_vcan_needs_no_bitrate(self):
        self.link['linkinfo']['info_kind'] = 'vcan'
        self.link['linkinfo']['info_data'] = {}
        with patch.object(can.subprocess, 'run', self.ip):
            can.ensure_can('vcan7')
        self.assertFalse(any('bitrate' in c for c in self.calls))

    def test_bus_off_is_recovered(self):
        self.link['linkinfo']['info_data']['state'] = 'BUS-OFF'
        with patch.object(can.subprocess, 'run', self.ip):
            can.ensure_can('can7')
        self.assertEqual([c[-1] for c in self.calls if 'set' in c], ['down', '500000', 'up'])

    def test_incorrect_interface_type(self):
        self.link['linkinfo']['info_kind'] = 'ether'
        with patch.object(can.subprocess, 'run', self.ip):
            with self.assertRaisesRegex(RuntimeError, 'not a CAN interface'):
                can.ensure_can('eth0')

    def test_missing_interface_timeout(self):
        with patch.object(can.subprocess, 'run', return_value=subprocess.CompletedProcess([], 1, '', 'missing')):
            with self.assertRaisesRegex(RuntimeError, 'unavailable|timed out'):
                can.ensure_can('wrongcan', timeout=0.02)

    def test_no_permission(self):
        self.link['flags'] = []
        self.error = True
        with patch.object(can.subprocess, 'run', self.ip):
            with self.assertRaisesRegex(RuntimeError, 'bring-up failed.*Operation not permitted'):
                can.ensure_can('can7')

    def test_configure_can_disabled(self):
        self.link['flags'] = []
        with patch.object(can.subprocess, 'run', self.ip):
            with self.assertRaisesRegex(RuntimeError, 'configuration disabled'):
                can.ensure_can('can7', configure=False)
        self.assertFalse(any('set' in c for c in self.calls))

    def test_command_timeout_and_missing_ip(self):
        for error in (subprocess.TimeoutExpired('ip', 1), FileNotFoundError('ip')):
            with patch.object(can.subprocess, 'run', side_effect=error):
                with self.assertRaisesRegex(RuntimeError, 'CAN can7'):
                    can.ensure_can('can7')

    def test_invalid_name_and_timeout(self):
        for interface, timeout in [('', 1), ('../can0', 1), ('c' * 16, 1), ('can7', 0)]:
            with self.assertRaises(RuntimeError):
                can.ensure_can(interface, timeout)


if __name__ == '__main__':
    unittest.main()
