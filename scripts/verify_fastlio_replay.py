#!/usr/bin/env python3
"""Compare original/candidate FAST-LIO against the same recorded sensor input.

Only a loopback test master, FAST-LIO and rosbag are started. No chassis or goals.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import xmlrpc.client
import yaml
from benchmark_cloud_enhancer import stop, stats


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = Path(args.output)
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:11473', ROS_IP='127.0.0.1')
    env.pop('ROS_HOSTNAME', None)
    os.environ.update(env)
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 11473)) == 0:
            raise RuntimeError('test port already in use')
    import rosbag
    import rospy
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import UInt64
    from nav_msgs.msg import Odometry
    fixture = output.with_suffix('.input.bag')
    with rosbag.Bag(args.bag, 'r') as source, rosbag.Bag(str(fixture), 'w') as target:
        end = source.get_start_time() + 15
        for topic, msg, stamp in source.read_messages(topics=['/livox/lidar', '/livox/imu'], end_time=rospy.Time.from_sec(end)):
            target.write(topic, msg, stamp)
    logs = []
    def launch(command, suffix):
        log = output.with_suffix(suffix).open('w')
        logs.append(log)
        return subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
    master = launch(['roscore', '-p', '11473'], '.master.log')
    worker = player = None
    trials = {}
    try:
        server = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        deadline = time.monotonic() + 15
        while True:
            try:
                server.getPid('/replay_verifier')
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(.1)
        config = yaml.safe_load((root / 'src/localization_fastlio/FAST_LIO/config/mid360.yaml').read_text())
        for key, value in config.items():
            server.setParam('/replay_verifier', '/' + key, value)
        for key, value in {'use_sim_time':True, 'feature_extract_enable':False, 'point_filter_num':3,
                           'max_iteration':3, 'filter_size_surf':.5, 'filter_size_map':.5,
                           'cube_side_length':1000., 'runtime_pos_log_enable':False,
                           'publish/path_en':False}.items():
            server.setParam('/replay_verifier', '/' + key, value)
        rospy.init_node('replay_verifier', disable_signals=True)
        for name, binary in [('baseline', args.baseline), ('candidate', args.candidate)]:
            clouds, counts, odoms = [], [], []
            subscribers = [
                rospy.Subscriber('/cloud_registered_body', PointCloud2,
                                 lambda m: clouds.append([m.header.stamp.to_sec(), m.width * m.height]), queue_size=100),
                rospy.Subscriber('/fast_lio/body_point_count', UInt64,
                                 lambda m: counts.append(m.data), queue_size=100),
                rospy.Subscriber('/Odometry', Odometry,
                                 lambda m: odoms.append([m.header.stamp.to_sec(), m.pose.pose.position.x,
                                                        m.pose.pose.position.y, m.pose.pose.position.z,
                                                        m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                                                        m.pose.pose.orientation.z, m.pose.pose.orientation.w]), queue_size=100),
            ]
            worker = launch([binary, '__name:=laserMapping'], '.' + name + '.log')
            time.sleep(2)
            if worker.poll() is not None:
                raise RuntimeError('FAST-LIO failed to start')
            player = launch(['rosbag', 'play', str(fixture), '--clock', '--delay=1'], '.' + name + '.player.log')
            cpu_before, _ = stats(worker.pid)
            start = time.monotonic()
            peak_rss = 0.
            while player.poll() is None:
                if time.monotonic() - start > 40 or worker.poll() is not None:
                    raise RuntimeError('replay failed or timed out')
                _, rss = stats(worker.pid)
                peak_rss = max(peak_rss, rss)
                time.sleep(.2)
            time.sleep(1)
            cpu, _ = stats(worker.pid)
            trials[name] = dict(clouds=clouds, counts=counts, odoms=odoms,
                                cpu_seconds=cpu-cpu_before, peak_rss_mib=peak_rss)
            stop(worker)
            worker = None
            for subscriber in subscribers:
                subscriber.unregister()
            print('{}: {} body clouds, {} point-count samples, {} odometry samples'.format(
                name, len(clouds), len(counts), len(odoms)), flush=True)
        baseline, candidate = trials['baseline'], trials['candidate']
        count_match = candidate['counts'] == [c[1] for c in candidate['clouds']]
        cloud_match = baseline['clouds'] == candidate['clouds']
        odom_max_error = None
        if len(baseline['odoms']) == len(candidate['odoms']) and baseline['odoms']:
            odom_max_error = max(abs(a-b) for first, second in zip(baseline['odoms'], candidate['odoms'])
                                 for a,b in zip(first, second))
        result = dict(trials=trials, point_counts_match_every_cloud=count_match,
                      clouds_match=cloud_match, maximum_odometry_component_difference=odom_max_error,
                      accepted=count_match and cloud_match and odom_max_error is not None and odom_max_error < 1e-5)
        output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps({k:v for k,v in result.items() if k != 'trials'}))
        if not result['accepted']:
            raise RuntimeError('replay equivalence failed; inspect output')
    finally:
        if player:
            stop(player)
        if worker:
            stop(worker)
        rospy.signal_shutdown('test complete')
        stop(master)
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
