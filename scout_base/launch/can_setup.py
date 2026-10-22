#!/usr/bin/env python3
"""Prepare a SocketCAN link without transmitting robot commands."""
import json
import os
import math
import subprocess
import time


def ensure_can(interface, timeout=10.0, configure=True):
    if not interface or len(interface.encode()) > 15 or '/' in interface:
        raise RuntimeError(f'Invalid CAN interface: {interface!r}')
    if not math.isfinite(timeout) or timeout <= 0:
        raise RuntimeError('CAN timeout must be positive')
    deadline = time.monotonic() + timeout

    def ip(*args, mutate=False):
        command = ['ip', *args]
        if mutate and os.geteuid() != 0:
            command = ['sudo', '-n', *command]
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(f'CAN {interface}: configuration timed out after {timeout}s')
        try:
            return subprocess.run(command, capture_output=True, text=True,
                                  timeout=remaining, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f'CAN {interface}: {exc}') from exc

    def inspect():
        result = ip('-j', '-details', 'link', 'show', 'dev', interface)
        if result.returncode:
            return None
        try:
            return json.loads(result.stdout)[0]
        except (ValueError, IndexError, TypeError) as exc:
            raise RuntimeError(f'CAN {interface}: invalid ip link response') from exc

    link = inspect()
    while link is None and time.monotonic() < deadline:
        time.sleep(min(0.1, max(0, deadline - time.monotonic())))
        link = inspect()
    if link is None:
        raise RuntimeError(f'CAN {interface}: unavailable after {timeout}s; check interface name/device')
    info = link.get('linkinfo', {})
    kind = info.get('info_kind')
    if kind not in ('can', 'vcan'):
        raise RuntimeError(f'CAN {interface}: not a CAN interface (type={kind})')
    data = info.get('info_data', {})
    bitrate = data.get('bittiming', {}).get('bitrate')
    healthy = data.get('state') not in ('BUS-OFF', 'STOPPED')
    needs_config = 'UP' not in link.get('flags', []) or not healthy
    needs_bitrate = kind == 'can' and bitrate != 500000
    if needs_config or needs_bitrate:
        if not configure:
            raise RuntimeError(f'CAN {interface}: DOWN, BUS-OFF or bitrate != 500000; configuration disabled')
        commands = [('link', 'set', 'dev', interface, 'down')]
        if kind == 'can':
            commands.append(('link', 'set', 'dev', interface, 'type', 'can', 'bitrate', '500000'))
        commands.append(('link', 'set', 'dev', interface, 'up'))
        for command in commands:
            result = ip(*command, mutate=True)
            if result.returncode:
                raise RuntimeError(f'CAN {interface}: bring-up failed: {result.stderr.strip()} '
                                   '(requires NET_ADMIN or noninteractive sudo)')
    link = inspect()
    data = (link or {}).get('linkinfo', {}).get('info_data', {})
    if (not link or 'UP' not in link.get('flags', []) or
            data.get('state') in ('BUS-OFF', 'STOPPED') or
            (kind == 'can' and data.get('bittiming', {}).get('bitrate') != 500000)):
        raise RuntimeError(f'CAN {interface}: failed to reach UP at bitrate 500000')
    print(f'CAN {interface}: UP ({kind}, bitrate=' +
          ('virtual' if kind == 'vcan' else '500000') +
          '); waiting for robot frames during protocol detection', flush=True)


def prepare_can(context):
    from launch.substitutions import LaunchConfiguration
    interface = LaunchConfiguration('port_name').perform(context)
    timeout = float(LaunchConfiguration('can_timeout_sec').perform(context))
    configure = LaunchConfiguration('configure_can').perform(context).lower() == 'true'
    ensure_can(interface, timeout, configure)
    return []


def on_base_exit(event, context):
    from launch.actions import EmitEvent
    from launch.events import Shutdown
    if event.returncode != 0:
        raise RuntimeError(f'scout_base exited unexpectedly with code {event.returncode}')
    return [EmitEvent(event=Shutdown(reason='scout_base stopped'))]
