#!/usr/bin/env python3
"""Static real-camera Qt acceptance; no goals, control messages or system changes."""
import json
from datetime import datetime
import os
from pathlib import Path
import re
import subprocess
import time
import xmlrpc.client
import rosgraph
import rospy
from sensor_msgs.msg import Image

LAB = Path(__file__).resolve().parents[1]
OUTPUT = LAB / 'logs' / ('qt_integration_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.json')
MASTER = 'http://127.0.0.1:11571'
RAW = '/bpu_preview_zed/zed_node/rgb/image_rect_color'


def wait_for(predicate, seconds=15):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(0.2)
    raise TimeoutError('Condition did not become true within %.1f s' % seconds)


def main():
    if os.environ.get('ROBOT_OPTIMIZED_SANDBOX') != '1' or os.environ.get('ROS_MASTER_URI') != MASTER:
        raise RuntimeError('Only the private stationary fixture is allowed')
    rospy.init_node('qt_preview_readonly_audit', disable_signals=True)
    master = rosgraph.Master('/qt_preview_readonly_audit')
    state = json.loads((LAB / 'qt_bridge_current.json').read_text())
    directory = Path(state['directory'])
    if not state['active'] or directory.resolve().parent != LAB / 'live' or state['master'] != MASTER:
        raise RuntimeError('No active isolated bridge')
    report = {'bridge_run': state['run_id'], 'passed': False}
    latest_times = []
    subscriber = rospy.Subscriber(RAW, Image, lambda msg: latest_times.append(
        (time.monotonic(), msg.header.stamp.to_sec())), queue_size=1)
    def records():
        lines = (directory / 'results.jsonl').read_text().splitlines()
        return [json.loads(line) for line in lines if line.endswith('}')]
    def graph():
        return master.getSystemState()
    def image_subs():
        return dict(graph()[1]).get('/fod/bpu_preview/image', [])
    def raw_subs():
        return dict(graph()[1]).get(RAW, [])
    def action(kind):
        subprocess.run(['python3', str(LAB / 'tools/qt_window_action.py'), kind,
                        hex(window), str(pid)], check=True)
    window = None
    try:
        wait_for(lambda: len(records()) >= 4, 55)
        uri = master.lookupNode('/autolabor_operator_gui')
        pid = xmlrpc.client.ServerProxy(uri).getPid('/qt_preview_readonly_audit')[2]
        tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
        for line in tree.splitlines():
            if 'Autolabor 无人车操作与诊断台' not in line:
                continue
            candidate = int(re.search(r'0x[0-9a-fA-F]+', line)[0], 16)
            owner = subprocess.check_output(['xprop', '-id', hex(candidate), '_NET_WM_PID'], text=True)
            if owner.strip().split('=')[-1].strip() == str(pid):
                window = candidate
                break
        if window is None:
            raise RuntimeError('Owned Qt window not found')
        report['gui_pid'], report['window'] = pid, hex(window)
        action('show')
        wait_for(lambda: image_subs() == ['/autolabor_operator_gui'])
        report['active_graph'] = graph()
        assert image_subs() == ['/autolabor_operator_gui'], image_subs()
        assert '/j6m_bpu_qt_bridge' in raw_subs()
        assert '/cmd_vel' not in dict(graph()[0]), 'Unexpected chassis command publisher'
        time.sleep(1)
        subprocess.run(['gnome-screenshot', '-w', '-f', str(OUTPUT.with_name(OUTPUT.stem + '_live.png'))],
                       check=True, env=dict(os.environ, GSETTINGS_BACKEND='memory'))
        action('minimize')
        wait_for(lambda: not image_subs())
        wait_for(lambda: '/j6m_bpu_qt_bridge' not in raw_subs())
        count_before = len(records())
        time.sleep(7)
        count_after = len(records())
        report['hidden_counts'] = [count_before, count_after]
        assert count_after == count_before, 'Inference continued without a Qt viewer'
        check = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4',
                                'root@192.168.10.100',
                                "pgrep -af '^(/usr/bin/)?python3 -u /map/robot_j6m_optimized_20260905/bpu_lab_20260905/tools/bpu_preview_worker.py " + state['run_id'] + "_'"],
                               stdout=subprocess.PIPE, text=True, timeout=8)
        report['hidden_remote_worker'] = check.stdout
        assert check.returncode == 1 and not check.stdout, 'Remote worker still present'
        report['hidden_graph'] = graph()
        action('show')
        wait_for(lambda: len(records()) >= count_after + 3, 20)
        report['resumed_count'] = len(records())
        report['resumed_generation'] = records()[-1]['generation']
        # Stop only the exact experimental bridge; leave the live camera and Qt up.
        (directory / 'STOP').touch()
        wait_for(lambda: not json.loads((LAB / 'qt_bridge_current.json').read_text())['active'])
        time.sleep(4)
        report['bridge_stop_graph'] = graph()
        recent = [item for item in latest_times if item[0] >= time.monotonic() - 3]
        assert len(recent) >= 8, 'Raw camera did not continue after BPU stop'
        assert master.lookupNode('/autolabor_operator_gui') == uri
        assert '/j6m_bpu_qt_bridge' not in dict(graph()[0]).get('/fod/bpu_preview/image', [])
        report['camera_frames_last_3s_after_bridge_stop'] = len(recent)
        report['camera_mean_hz'] = (len(latest_times) - 1) / (latest_times[-1][0] - latest_times[0][0])
        report['camera_max_interval_s'] = max(b[0] - a[0] for a, b in zip(latest_times, latest_times[1:]))
        action('show')
        time.sleep(1)
        subprocess.run(['gnome-screenshot', '-w', '-f', str(OUTPUT.with_name(OUTPUT.stem + '_stopped.png'))],
                       check=True, env=dict(os.environ, GSETTINGS_BACKEND='memory'))
        report['passed'] = True
    except Exception as error:
        report['error'] = repr(error)
        raise
    finally:
        subscriber.unregister()
        OUTPUT.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print(json.dumps({key: value for key, value in report.items() if not key.endswith('_graph')},
                         ensure_ascii=False, indent=2), flush=True)
        if not report['passed'] and window is not None:
            action('show')


if __name__ == '__main__':
    main()
