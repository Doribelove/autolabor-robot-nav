#!/usr/bin/env python3
"""Real sensors and navigation on a dedicated ROS master; no chassis device."""
import collections
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client

root = Path('/tmp')
stopping = threading.Event()
children = []
for sig in (signal.SIGTERM, signal.SIGINT):
    signal.signal(sig, lambda *_: stopping.set())

def spawn(name, args):
    stream = (root/'logs'/(name+'.log')).open('a')
    child = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    children.append((name, child, stream))
    return child

def write_state(value):
    temp = root/'status.tmp'
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temp.replace(root/'status.json')

try:
    spawn('master', ['roscore', '-p', '11583'])
    deadline = time.monotonic()+20
    while time.monotonic() < deadline and not stopping.is_set():
        try:
            if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/conbine_startup')[0] == 1:
                break
        except OSError:
            pass
        stopping.wait(.1)
    else:
        raise RuntimeError('ROS master failed to start')
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import Imu, LaserScan, PointCloud2
    from std_msgs.msg import String
    from livox_ros_driver2.msg import CustomMsg
    from autolabor_coverage.msg import CoverageStatus
    rospy.init_node('conbine_live_monitor', disable_signals=True)
    rospy.set_param('/conbine/session', json.loads((root/'session.json').read_text()))
    arrivals = collections.defaultdict(lambda: collections.deque(maxlen=1000))
    counts = collections.Counter()
    latest = {}
    lock = threading.RLock()
    def callback(topic, msg):
        with lock:
            arrivals[topic].append(time.monotonic())
            counts[topic] += 1
            if isinstance(msg, String):
                latest[topic] = msg.data
            elif isinstance(msg, Twist):
                latest[topic] = [msg.linear.x, msg.linear.y, msg.linear.z,
                                 msg.angular.x, msg.angular.y, msg.angular.z]
            elif topic == '/map':
                latest[topic] = {'width': msg.info.width, 'height': msg.info.height,
                                 'resolution': msg.info.resolution}
            elif isinstance(msg, CoverageStatus):
                latest[topic] = {k:getattr(msg,k) for k in ('state','map_ready','localized','active',
                    'paused','watchdog_motion_enabled','chassis_ready','avoidance_ready','detail')}
    for topic, cls in [('/livox/lidar',CustomMsg),('/livox/imu',Imu),('/Odometry',Odometry),
                       ('/cloud_registered',PointCloud2),('/scan',LaserScan),('/odom',Odometry),
                       ('/map',OccupancyGrid),('/fast_lio/localization_status',String),
                       ('/coverage/status',CoverageStatus),('/cmd_vel_safe',Twist),('/cmd_vel',Twist),
                       ('/nvidia_cmd_vel_watchdog/status',String)]:
        rospy.Subscriber(topic, cls, lambda msg,t=topic: callback(t,msg), queue_size=2, tcp_nodelay=True)
    spawn('navigation', ['roslaunch', '--wait', '/tmp/navigation.launch'])
    started = time.monotonic()
    while not stopping.wait(.5) and not rospy.is_shutdown():
        failures = [n for n,p,_ in children if p.poll() is not None]
        if failures:
            raise RuntimeError('Child exited: '+','.join(failures))
        with lock:
            now=time.monotonic()
            state={'mode':'real_navigation',
                   'motion_enabled':rospy.get_param('/nvidia_cmd_vel_watchdog/motion_enabled',False),
                   'uptime_s':round(now-started,2), 'counts':dict(counts),
                   'age_s':{t:round(now-v[-1],3) for t,v in arrivals.items() if v},
                   'rates_hz':{t:round((len(v)-1)/(v[-1]-v[0]),3) for t,v in arrivals.items() if len(v)>1},
                   'latest':dict(latest), 'persistence_protocol':{
                       'hybrid':rospy.get_param('/move_base/CoverageGlobalPlanner/tuning_persistence_protocol',0),
                       'transit':rospy.get_param('/coverage_manager/tuning_persistence_protocol',0)}}
        write_state(state)
finally:
    for name, child, stream in reversed(children):
        for sig, timeout in ((signal.SIGINT,10),(signal.SIGTERM,3),(signal.SIGKILL,2)):
            if child.poll() is not None:
                break
            os.killpg(child.pid,sig)
            try:
                child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass
        stream.close()
