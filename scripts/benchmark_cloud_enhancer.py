#!/usr/bin/env python3
"""ROS replay benchmark, isolated loopback master; never launches a device/control node."""
import argparse
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client


def stats(pid):
    values = Path('/proc/{}/stat'.format(pid)).read_text().rsplit(') ', 1)[1].split()
    ticks = int(values[11]) + int(values[12])
    rss = int(values[21]) * os.sysconf('SC_PAGE_SIZE') / 1048576.0
    return ticks / os.sysconf('SC_CLK_TCK'), rss


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=8)
    parser.add_argument('--repeats', type=int, default=3)
    args = parser.parse_args()
    result_path = Path(args.output)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:11471', ROS_IP='127.0.0.1')
    env.pop('ROS_HOSTNAME', None)
    os.environ.update(env)
    # Refuse to attach to an existing graph, even on the benchmark port.
    import socket
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 11471)) == 0:
            raise RuntimeError('benchmark port already in use')
    import rosbag
    import rospy
    from sensor_msgs.msg import PointCloud2, LaserScan
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    samples = {}
    with rosbag.Bag(args.bag, 'r') as bag:
        for topic, message, _ in bag.read_messages(topics=['/cloud_registered_body', '/dual_lidar/scan']):
            samples.setdefault(topic, message)
            if len(samples) == 2:
                break
    cloud = samples['/cloud_registered_body']
    scan = samples['/dual_lidar/scan']
    master_log = result_path.with_suffix('.master.log').open('w')
    master = subprocess.Popen(['roscore', '-p', '11471'], env=env, stdout=master_log, stderr=subprocess.STDOUT)
    trials = []
    proc = None
    try:
        deadline = time.monotonic() + 15
        while True:
            try:
                xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/benchmark')
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(.1)
        rospy.init_node('cloud_benchmark', disable_signals=True)
        cloud_pub = rospy.Publisher('/benchmark/cloud', PointCloud2, queue_size=2)
        scan_pub = rospy.Publisher('/benchmark/scan', LaserScan, queue_size=2)
        tf_pub = StaticTransformBroadcaster()
        tf = TransformStamped()
        tf.header.frame_id = 'body'
        tf.child_frame_id = 'base_link'
        tf.header.stamp = rospy.Time.now()
        tf.transform.translation.x = -.211
        tf.transform.translation.y = -.02329
        tf.transform.translation.z = -.95588
        tf.transform.rotation.w = 1.
        tf_pub.sendTransform(tf)
        for rate in (10, 100):
            for subscribed in (True, False):
                for repeat in range(args.repeats):
                    # Reverse each pair on alternate repetitions to reduce order bias.
                    variants = [('baseline', args.baseline), ('candidate', args.candidate)]
                    if repeat % 2:
                        variants.reverse()
                    for label, binary in variants:
                        received = []
                        expected_hash = []
                        def callback(msg):
                            received.append(time.monotonic())
                            if not expected_hash:
                                msg.header.stamp = rospy.Time()
                                msg.header.seq = 0
                                data = io.BytesIO()
                                msg.serialize(data)
                                expected_hash.append(hashlib.sha256(data.getvalue()).hexdigest())
                        subscriber = rospy.Subscriber('/benchmark/output', PointCloud2, callback, queue_size=2) if subscribed else None
                        log = result_path.with_name('{}-{}-{}-{}-{}.log'.format(result_path.stem, label, rate, subscribed, repeat)).open('w')
                        proc = subprocess.Popen([
                            binary, '__name:=cloud_benchmark_enhancer',
                            '_mid_cloud_topic:=/benchmark/cloud', '_lidar_scan_topic:=/benchmark/scan',
                            '_output_topic:=/benchmark/output', '_queue_size:=2',
                        ], env=env, stdout=log, stderr=subprocess.STDOUT)
                        deadline = time.monotonic() + 10
                        while cloud_pub.get_num_connections() == 0 or scan_pub.get_num_connections() == 0:
                            if proc.poll() is not None or time.monotonic() > deadline:
                                raise RuntimeError('enhancer failed to connect')
                            time.sleep(.05)
                        # Allow publisher/subscriber negotiation and static TF reception.
                        for _ in range(10):
                            scan.header.stamp = cloud.header.stamp = rospy.Time.now()
                            scan_pub.publish(scan)
                            cloud_pub.publish(cloud)
                            time.sleep(.1)
                        received.clear()
                        expected_hash.clear()
                        before_cpu, _ = stats(proc.pid)
                        start = time.monotonic()
                        sent = 0
                        peak_rss = 0.
                        while time.monotonic() - start < args.duration:
                            scan.header.stamp = cloud.header.stamp = rospy.Time.now()
                            scan_pub.publish(scan)
                            cloud_pub.publish(cloud)
                            sent += 1
                            _, rss = stats(proc.pid)
                            peak_rss = max(peak_rss, rss)
                            time.sleep(max(0, start + sent / rate - time.monotonic()))
                        cpu, rss = stats(proc.pid)
                        elapsed = time.monotonic() - start
                        result = dict(variant=label, rate=rate, subscribed=subscribed, repeat=repeat,
                                      seconds=elapsed, sent=sent, received=len(received),
                                      cpu_percent_one_core=(cpu-before_cpu)/elapsed*100,
                                      rss_mib=rss, peak_rss_mib=peak_rss,
                                      output_sha256=expected_hash[0] if expected_hash else None)
                        trials.append(result)
                        print(json.dumps(result), flush=True)
                        stop(proc)
                        proc = None
                        log.close()
                        if subscriber:
                            subscriber.unregister()
                        time.sleep(.2)
        output = dict(host=os.uname().nodename, cloud_points=cloud.width*cloud.height,
                      cloud_bytes=len(cloud.data), scan_bins=len(scan.ranges),
                      input_bag=args.bag, trials=trials)
        result_path.write_text(json.dumps(output, indent=2) + '\n')
        hashes = {t['output_sha256'] for t in trials if t['subscribed']}
        if len(hashes) != 1 or None in hashes:
            raise RuntimeError('baseline and candidate outputs differ')
    finally:
        if proc:
            stop(proc)
        rospy.signal_shutdown('benchmark completed')
        stop(master)
        master_log.close()


if __name__ == '__main__':
    main()
