#!/usr/bin/env python3
"""Own only the combined GUI, isolated planner preview and read-only video bridge."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import urllib.request

WS = Path(__file__).resolve().parents[1]
ROOT = WS / 'runtime/conbine_preview'
UNIT = 'autolabor-conbine-preview.service'


def command(args, check=True):
    return subprocess.run(args, check=check, text=True, capture_output=True, timeout=40)


def run():
    if WS != Path('/home/slam/robot_j6m_ws_conbine'):
        raise RuntimeError('Combined workspace ownership mismatch')
    ROOT.mkdir(parents=True, exist_ok=True)
    with (ROOT / 'session.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        session = ROOT / ('session-%d-%d' % (time.time_ns(), os.getpid()))
        session.mkdir()
        endpoint = session / 'frames.sock'
        stop = threading.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.set())
        children = []
        streams = []
        try:
            def spawn(name, args, env=None):
                stream = (session / (name + '.log')).open('a')
                streams.append(stream)
                child = subprocess.Popen(args, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                         start_new_session=True)
                children.append(child)
                return child
            bridge = spawn('frame_bridge', [sys.executable, '-B',
                str(WS / 'scripts/conbine_preview_relay.py'), 'host', '--socket', str(endpoint)])
            deadline = time.monotonic() + 5
            while not endpoint.exists() and time.monotonic() < deadline and bridge.poll() is None:
                stop.wait(.05)
            if not endpoint.exists():
                raise RuntimeError('Read-only frame bridge did not start')
            env = dict(os.environ, CONBINE_FRAME_SOCKET=str(endpoint))
            gui = spawn('isolated_preview', ['/bin/bash', str(WS / 'scripts/navigation_tuning_preview.sh'), 'run'], env)
            record = dict(mode='real_surround_and_isolated_navigation', parent_pid=os.getpid(),
                          namespace_process_pid=gui.pid, bridge_pid=bridge.pid,
                          source='http://127.0.0.1:18781', source_owned=False,
                          socket=str(endpoint), session=str(session), robot_navigation_connected=False,
                          started_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
            (ROOT / 'session.json').write_text(json.dumps(record, indent=2) + '\n')
            while not stop.wait(.25):
                if gui.poll() is not None:
                    if gui.returncode:
                        raise RuntimeError('Combined preview exited; see ' + str(session))
                    break
                if bridge.poll() is not None:
                    raise RuntimeError('Frame bridge exited')
        finally:
            for child in reversed(children):
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGTERM)
                        child.wait(timeout=5)
            for stream in streams:
                stream.close()


def main(action):
    existing = command(['systemctl', '--user', 'show', UNIT, '-p', 'ExecStart', '--value'], False).stdout
    if existing.strip() and str(Path(__file__).resolve()) not in existing:
        raise RuntimeError('Combined service belongs to another workspace')
    if action == 'run':
        return run()
    if action == 'start':
        if command(['systemctl', '--user', 'is-active', '--quiet', UNIT], False).returncode == 0:
            return main('status')
        command(['systemd-run', '--user', '--unit=' + UNIT, '--collect',
                 '--property=WorkingDirectory=' + str(WS), '--property=KillMode=mixed',
                 '--property=TimeoutStopSec=50',
                 '--setenv=DISPLAY=' + os.environ.get('DISPLAY', ':0'),
                 '--setenv=XAUTHORITY=' + os.environ.get('XAUTHORITY', '/run/user/1000/gdm/Xauthority'),
                 sys.executable, '-B', str(Path(__file__).resolve()), 'run'])
        print('Started', UNIT)
    elif action == 'stop':
        command(['systemctl', '--user', 'stop', UNIT])
    else:
        print(command(['systemctl', '--user', 'show', UNIT, '-p', 'ActiveState,SubState,MainPID,ExecStart'], False).stdout)
        try:
            with urllib.request.urlopen('http://127.0.0.1:18781/state', timeout=1) as response:
                state = json.load(response)
            print('Camera source:', {p: v['fresh'] for p, v in state['ports'].items()})
        except (OSError, ValueError):
            print('Camera source unavailable; merged Qt will show waiting status')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['start', 'stop', 'status', 'run'])
    main(parser.parse_args().action)
