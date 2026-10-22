"""Integration tests with the compiled ROS node, real Zenoh and a mock CAN bus."""
import os
import errno
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest


@unittest.skipUnless(os.environ.get('SCOUT_BASE_TEST_NODE') and os.environ.get('SCOUT_BASE_TEST_CAN_MOCK'),
                     'set SCOUT_BASE_TEST_NODE and SCOUT_BASE_TEST_CAN_MOCK to compiled artifacts')
class NodeStartupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.env = dict(os.environ, ROS_LOG_DIR=self.temp.name,
                        RMW_IMPLEMENTATION='rmw_zenoh_cpp', ZENOH_ROUTER_CHECK_ATTEMPTS='1',
                        ZENOH_CONFIG_OVERRIDE='connect/endpoints=[];scouting/multicast/enabled=false',
                        ROS_DOMAIN_ID='217')
        self.env.pop('SCOUT_TEST_SIGNATURE', None)
        self.env.pop('SCOUT_TEST_FAIL_SECOND_CONNECT', None)
        self.env.pop('LD_PRELOAD', None)
        self.env.pop('SCOUT_TEST_SEND_ERRNO', None)
        self.env.pop('SCOUT_TEST_DROP_FILE', None)

    def tearDown(self):
        self.temp.cleanup()

    def command(self, *parameters):
        args = [os.environ['SCOUT_BASE_TEST_NODE'], '--ros-args']
        for parameter in parameters:
            args += ['-p', parameter]
        return args

    def execute(self, *parameters, mock=False, **env):
        if mock:
            env['LD_PRELOAD'] = os.environ['SCOUT_BASE_TEST_CAN_MOCK']
        result = subprocess.run(self.command(*parameters), env=dict(self.env, **env),
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
        self.assertNotIn('Resource deadlock', result.stdout)
        self.assertNotIn('terminate called', result.stdout)
        return result

    def test_missing_and_incorrect_can_return_failure_with_zenoh(self):
        for interface in ('scout_missing', 'lo'):
            result = self.execute(f'port_name:={interface}')
            self.assertEqual(result.returncode, 1, result.stdout)
            self.assertIn('Cannot open CAN interface', result.stdout)
            self.assertIn('scout_base initialization failed', result.stdout)

    def test_no_frames_timeout_differs_from_unknown_frames(self):
        silent = self.execute('port_name:=mockcan', 'protocol_timeout_sec:=1', mock=True)
        self.assertEqual(silent.returncode, 1, silent.stdout)
        self.assertIn('no CAN frames received', silent.stdout)
        unknown = self.execute('port_name:=mockcan', 'protocol_timeout_sec:=1', mock=True,
                               SCOUT_TEST_SIGNATURE='0x123')
        self.assertEqual(unknown.returncode, 1, unknown.stdout)
        self.assertIn('frames, but no supported protocol signature', unknown.stdout)

    def test_second_connection_failure_returns_failure(self):
        result = self.execute('port_name:=mockcan', mock=True,
                              SCOUT_TEST_SIGNATURE='0x241', SCOUT_TEST_FAIL_SECOND_CONNECT='1')
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('Failed to connect to the robot CAN bus', result.stdout)
        self.assertNotIn('SCOUT_BASE_READY', result.stdout)

    def test_fatal_send_exits_nonzero_for_both_models(self):
        for omni in ('false', 'true'):
            with self.subTest(omni=omni):
                result = self.execute('port_name:=mockcan', f'is_omni_wheel:={omni}', mock=True,
                                      SCOUT_TEST_SIGNATURE='0x241',
                                      SCOUT_TEST_SEND_ERRNO=str(errno.ENETDOWN))
                self.assertEqual(result.returncode, 1, result.stdout)
                self.assertIn('fatal send failure', result.stdout)
                self.assertIn('Fatal CAN transport failure', result.stdout)

    def test_transient_send_keeps_node_alive(self):
        env = dict(self.env, LD_PRELOAD=os.environ['SCOUT_BASE_TEST_CAN_MOCK'],
                   SCOUT_TEST_SIGNATURE='0x241', SCOUT_TEST_SEND_ERRNO=str(errno.ENOBUFS))
        output = Path(self.temp.name) / 'transient.log'
        with output.open('w') as stream:
            node = subprocess.Popen(self.command('port_name:=mockcan'), env=env,
                                    stdout=stream, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    text = output.read_text()
                    if 'SCOUT_BASE_READY' in text and 'transient send error' in text:
                        break
                    if node.poll() is not None:
                        self.fail(text)
                    time.sleep(0.02)
                self.assertIn('transient send error', output.read_text())
                self.assertIn('SCOUT_BASE_READY', output.read_text())
                time.sleep(0.05)
                self.assertIsNone(node.poll(), output.read_text())
                node.terminate()
                self.assertEqual(node.wait(timeout=5), 0, output.read_text())
                self.assertNotIn('Fatal CAN transport failure', output.read_text())
            finally:
                if node.poll() is None:
                    node.kill()
                    node.wait()

    def test_runtime_can_loss_exits_nonzero_for_both_models(self):
        for omni in ('false', 'true'):
            with self.subTest(omni=omni):
                drop_file = Path(self.temp.name) / f'drop_{omni}'
                output = Path(self.temp.name) / f'runtime_{omni}.log'
                env = dict(self.env, LD_PRELOAD=os.environ['SCOUT_BASE_TEST_CAN_MOCK'],
                           SCOUT_TEST_SIGNATURE='0x241', SCOUT_TEST_DROP_FILE=str(drop_file))
                with output.open('w') as stream:
                    node = subprocess.Popen(self.command('port_name:=mockcan',
                                                         f'is_omni_wheel:={omni}'), env=env,
                                            stdout=stream, stderr=subprocess.STDOUT)
                    try:
                        deadline = time.monotonic() + 5
                        while 'SCOUT_BASE_READY' not in output.read_text() and time.monotonic() < deadline:
                            if node.poll() is not None:
                                self.fail(output.read_text())
                            time.sleep(0.02)
                        self.assertIn('SCOUT_BASE_READY', output.read_text())
                        drop_file.touch()
                        self.assertEqual(node.wait(timeout=5), 1, output.read_text())
                        self.assertIn('fatal receive failure', output.read_text())
                        self.assertIn('Fatal CAN transport failure', output.read_text())
                        self.assertNotIn('Resource deadlock', output.read_text())
                    finally:
                        if node.poll() is None:
                            node.kill()
                            node.wait()

    def test_middleware_initialization_exception_returns_failure(self):
        result = self.execute(ZENOH_SESSION_CONFIG_URI='/no-such-scout-test-config.json5')
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('scout_base failed (CAN/ROS middleware)', result.stdout)

    def test_invalid_parameters_return_failure(self):
        result = self.execute('protocol_timeout_sec:=0')
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('protocol_timeout_sec must be positive', result.stdout)

    def test_real_router_and_node_shutdown(self):
        with socket.socket() as reserved:
            reserved.bind(('127.0.0.1', 0))
            port = reserved.getsockname()[1]
        endpoint = f'tcp/127.0.0.1:{port}'
        router_env = dict(self.env, ZENOH_CONFIG_OVERRIDE=(
            f'listen/endpoints=["{endpoint}"];connect/endpoints=[];scouting/multicast/enabled=false'))
        router = subprocess.Popen(['/opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd'],
                                  env=router_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            deadline = time.monotonic() + 5
            while True:
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=0.2):
                        break
                except OSError:
                    if time.monotonic() >= deadline or router.poll() is not None:
                        self.fail('local Zenoh router did not start')
                    time.sleep(0.02)
            env = dict(self.env, LD_PRELOAD=os.environ['SCOUT_BASE_TEST_CAN_MOCK'],
                       SCOUT_TEST_SIGNATURE='0x241', ZENOH_CONFIG_OVERRIDE=(
                           f'connect/endpoints=["{endpoint}"];scouting/multicast/enabled=false'))
            output = Path(self.temp.name) / 'connected.log'
            with output.open('w') as stream:
                node = subprocess.Popen(self.command('port_name:=mockcan'), env=env,
                                        stdout=stream, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 5
                    while 'SCOUT_BASE_READY' not in output.read_text() and time.monotonic() < deadline:
                        if node.poll() is not None:
                            self.fail(output.read_text())
                        time.sleep(0.02)
                    self.assertIn('SCOUT_BASE_READY', output.read_text())
                    node.terminate()
                    self.assertEqual(node.wait(timeout=5), 0, output.read_text())
                finally:
                    if node.poll() is None:
                        node.kill()
                        node.wait()
            self.assertNotIn('Unable to connect to a Zenoh router', output.read_text())
            self.assertNotIn('context is invalid', output.read_text())
            self.assertIsNone(router.poll())
        finally:
            router.terminate()
            router.wait(timeout=5)

    def test_launch_preflight_failure_returns_nonzero(self):
        result = subprocess.run(
            ['ros2', 'launch', 'scout_base', 'scout_mini_base.launch.py',
             'port_name:=lo', 'can_timeout_sec:=0.1'], env=self.env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('not a CAN interface', result.stdout)
        self.assertNotIn('process started with pid', result.stdout)

    def test_launch_node_failure_returns_nonzero(self):
        # Preflight sees a virtual UP link; the SDK uses only local socketpairs.
        bindir = Path(self.temp.name) / 'bin'
        bindir.mkdir()
        ip = bindir / 'ip'
        ip.write_text("#!/usr/bin/env python3\nprint('[{\"flags\":[\"UP\"],\"linkinfo\":{\"info_kind\":\"vcan\"}}]')\n")
        ip.chmod(0o755)
        env = dict(self.env, PATH=str(bindir) + os.pathsep + self.env['PATH'])
        result = subprocess.run(
            ['ros2', 'launch', '--launch-prefix',
             'env LD_PRELOAD=' + os.environ['SCOUT_BASE_TEST_CAN_MOCK'],
             'scout_base', 'scout_mini_base.launch.py',
             'port_name:=mockcan', 'protocol_timeout_sec:=1'], env=env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn('no CAN frames received', result.stdout)
        self.assertIn('scout_base exited unexpectedly with code 1', result.stdout)

    def test_sigint_and_sigterm_during_detection_and_running(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            for ready in (False, True):
                with self.subTest(sig=sig, ready=ready):
                    env = dict(self.env, LD_PRELOAD=os.environ['SCOUT_BASE_TEST_CAN_MOCK'])
                    if ready:
                        env['SCOUT_TEST_SIGNATURE'] = '0x241'
                    output = Path(self.temp.name) / 'node.log'
                    with output.open('w') as stream:
                        omni = 'true' if ready and sig == signal.SIGTERM else 'false'
                        process = subprocess.Popen(self.command('port_name:=mockcan',
                                                                 f'is_omni_wheel:={omni}'), env=env,
                                                   stdout=stream, stderr=subprocess.STDOUT)
                        try:
                            marker = 'SCOUT_BASE_READY' if ready else 'Start listening to port:'
                            deadline = time.monotonic() + 5
                            while marker not in output.read_text() and time.monotonic() < deadline:
                                if process.poll() is not None:
                                    self.fail(output.read_text())
                                time.sleep(0.02)
                            self.assertIn(marker, output.read_text())
                            process.send_signal(sig)
                            self.assertEqual(process.wait(timeout=5), 0, output.read_text())
                        finally:
                            if process.poll() is None:
                                process.kill()
                                process.wait()
                    text = output.read_text()
                    self.assertNotIn('Resource deadlock', text)
                    self.assertNotIn('context is invalid', text)
                    self.assertNotIn('terminate called', text)


if __name__ == '__main__':
    unittest.main()
