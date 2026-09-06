#!/usr/bin/env python3
"""Bidirectional custom-message probe on a separate master, no hardware nodes."""
import argparse
import json
import os
from pathlib import Path
import socket
import statistics
import subprocess
import threading
import time
import uuid
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', action='store_true')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    os.environ['ROS_MASTER_URI'] = 'http://192.168.10.100:11476'
    os.environ['ROS_IP'] = '192.168.10.100' if args.server else '192.168.10.50'
    os.environ.pop('ROS_HOSTNAME', None)
    import rospy
    from autolabor_fod_msgs.msg import FodDetectionArray
    from livox_ros_driver2.msg import CustomMsg
    process = None
    log = None
    try:
        if args.server:
            with socket.socket() as probe:
                if probe.connect_ex(('127.0.0.1', 11476)) == 0:
                    raise RuntimeError('probe master port occupied')
            log = open(args.output + '.master.log', 'w')
            process = subprocess.Popen(['roscore', '-p', '11476'], stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 15
            while True:
                try:
                    xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')
                    break
                except OSError:
                    if time.monotonic() > deadline:
                        raise
                    time.sleep(.1)
        rospy.init_node('isolated_cross_host_' + ('server' if args.server else 'client'), disable_signals=True)
        if args.server:
            reply = rospy.Publisher('/isolated_probe/reply', CustomMsg, queue_size=10)
            received = []
            def request(message):
                received.append(message.header.frame_id)
                response = CustomMsg()
                response.header = message.header
                reply.publish(response)
            subscription = rospy.Subscriber('/isolated_probe/request', FodDetectionArray, request, queue_size=10)
            deadline = time.monotonic() + 30
            while len(set(received)) < 5 and time.monotonic() < deadline:
                time.sleep(.1)
            time.sleep(1)
            result = dict(unique_requests=len(set(received)), all_five=len(set(received)) == 5)
        else:
            request = rospy.Publisher('/isolated_probe/request', FodDetectionArray, queue_size=10)
            replies = {}
            def response(message):
                replies.setdefault(message.header.frame_id, time.monotonic())
            subscription = rospy.Subscriber('/isolated_probe/reply', CustomMsg, response, queue_size=10)
            deadline = time.monotonic() + 15
            while request.get_num_connections() == 0:
                if time.monotonic() > deadline:
                    raise RuntimeError('J6M probe did not connect')
                time.sleep(.1)
            time.sleep(1)
            rtt = []
            for _ in range(5):
                token = str(uuid.uuid4())
                message = FodDetectionArray()
                message.header.frame_id = token
                message.header.stamp = rospy.Time.now()
                start = time.monotonic()
                request.publish(message)
                while token not in replies:
                    if time.monotonic() - start > 3:
                        raise RuntimeError('custom message round trip timed out')
                    time.sleep(.001)
                rtt.append((replies[token] - start) * 1000)
                time.sleep(.2)
            result = dict(round_trips=5, rtt_ms=rtt, median_rtt_ms=statistics.median(rtt),
                          request_md5=FodDetectionArray._md5sum, reply_md5=CustomMsg._md5sum)
        Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result))
    finally:
        rospy.signal_shutdown('cross-host probe complete')
        if process:
            process.terminate()
            process.wait(timeout=10)
        if log:
            log.close()


if __name__ == '__main__':
    main()
