#!/usr/bin/env python3
"""Actual ROS nodes in synthetic scenes; run only through the namespace wrapper."""
import collections
import argparse
import copy
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
import xmlrpc.client

WS = Path(__file__).resolve().parents[4]
parser = argparse.ArgumentParser()
parser.add_argument('--baseline', type=Path,
                    help='Optional historical manager used to reproduce the old preview bug')
args = parser.parse_args()
ROOT = WS / 'validation/first_entry_simulation_20260907'
RUN = ROOT / datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
RUN.mkdir(parents=True)
(ROOT / 'latest_run.txt').write_text(str(RUN) + '\n')
os.environ['ROS_LOG_DIR'] = str(RUN / 'ros_logs')
os.environ['ROS_HOME'] = str(RUN / 'ros_home')
assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11491'
assert [name for _,name in socket.if_nameindex()] == ['lo'], 'requires isolated network namespace'
processes = []
events_file = open(str(RUN / 'events.jsonl'), 'a', buffering=1)
lock = threading.RLock()
observed = collections.defaultdict(list)
results = []

def plain(msg):
    if isinstance(msg, (list, tuple)):
        return [plain(v) for v in msg]
    if hasattr(msg, '__slots__'):
        return {slot: plain(getattr(msg, slot)) for slot in msg.__slots__}
    return msg

def event(kind, **data):
    with lock:
        events_file.write(json.dumps(dict(ts=time.time(), kind=kind, **data)) + '\n')

def spawn(name, args):
    log = open(str(RUN / (name + '.log')), 'w')
    proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(proc)
    event('process_start', name=name, pid=proc.pid, args=args)
    return proc

def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3)

def wait(predicate, reason, timeout=40):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('timeout: ' + reason)

def receive(msg, topic):
    with lock:
        observed[topic].append((time.time(), copy.deepcopy(msg)))
        if topic != '/coverage/status' or not observed[topic][:-1] or (
                msg.state, msg.detail, msg.batch_active, msg.batch_id) != (
                observed[topic][-2][1].state, observed[topic][-2][1].detail,
                observed[topic][-2][1].batch_active, observed[topic][-2][1].batch_id):
            event('message', topic=topic, msg=plain(msg))

def latest(topic):
    with lock:
        return observed[topic][-1][1] if observed[topic] else None

def check(name, **evidence):
    results.append(dict(name=name, passed=True, **evidence))
    event('check_pass', name=name, **evidence)
    print('PASS', name, json.dumps(evidence), flush=True)

def map_scene(kind, revision=0):
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.info.width, msg.info.height, msg.info.resolution = 320, 200, .1
    msg.info.origin.orientation.w = 1
    grid = [0] * (320*200)
    for y in range(200):
        for x in range(320):
            if x < 3 or x > 316 or y < 3 or y > 196:
                grid[y*320+x] = 100
            if kind in ('detour', 'sealed') and 118 <= x <= 122:
                if kind == 'sealed' or y <= 130:
                    grid[y*320+x] = 100
    if revision:
        grid[170*320+280] = 100
    msg.data = grid
    return msg

def set_scene(kind, revision=0):
    global scene
    prior = latest('/coverage/status')
    old = prior.map_digest if prior else None
    scene = map_scene(kind, revision)
    map_pub.publish(scene)
    wait(lambda: latest('/coverage/status') is not None and latest('/coverage/status').map_ready
         and (old is None or latest('/coverage/status').map_digest != old), 'new map digest')
    wait(lambda: latest('/move_base/global_costmap/costmap') is not None,
         'global costmap')
    time.sleep(.7)
    event('scene', scene=kind, revision=revision, map=plain(scene))

def request(region_shift=0):
    req = StartCoverageBatchRequest()
    req.client_request_id = 'coverage-batch-' + uuid.uuid4().hex
    region = CoverageRegion(id='sim-region', name='synthetic first-entry region')
    region.region.header.frame_id = 'map'
    region.region.header.stamp = rospy.Time.now()
    region.region.polygon.points = [Point32(x=x+region_shift, y=y, z=0)
                                    for x,y in ((19,4),(25,4),(25,9),(19,9))]
    req.regions = [region]
    req.operation_width_m = 1.
    req.overlap_ratio = .15
    req.allow_reverse_transit = True
    req.max_speed_mps = .8
    req.reverse_speed_mps = .3
    req.max_angular_speed_rps = .6
    req.linear_accel_mps2 = 1.
    req.angular_accel_rps2 = .5
    req.direction_change_penalty_sec = .5
    req.segment_handoff_penalty_sec = .5
    req.transit_replan_period_sec = .5
    req.map_digest = latest('/coverage/status').map_digest
    return req

def run_batch(name, expected_preview=True, shift=0, hold_sec=.25):
    req = request(shift)
    start = time.time()
    event('request', scenario=name, request=plain(req))
    response = start_batch(req)
    assert response.accepted, response.message
    wait(lambda: latest('/coverage/status').batch_id == req.client_request_id
         and not latest('/coverage/status').batch_active
         and latest('/coverage/status').state == 'FAILED', 'batch terminal failure', timeout=90)
    status = latest('/coverage/status')
    assert status.detail == 'main motion gate is disabled', status.detail
    time.sleep(hold_sec)
    with lock:
        paths = [msg for ts,msg in observed['/coverage/first_entry_path'] if ts >= start]
    assert any(p.poses for p in paths), 'Navfn did not publish any entry preview'
    if expected_preview:
        first_nonempty = next(i for i,p in enumerate(paths) if p.poses)
        assert all(p.poses for p in paths[first_nonempty:]), 'cleanup briefly erased the preview'
    final = latest('/coverage/first_entry_path')
    assert bool(final.poses) == expected_preview, 'unexpected final preview retention'
    # New subscriber proves the public latch, not just the manager's memory.
    latched = rospy.wait_for_message('/coverage/first_entry_path', NavPath, timeout=3)
    assert bool(latched.poses) == expected_preview, 'new subscriber received wrong latch'
    path = next(p for p in paths if p.poses)
    points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(points, points[1:]))
    start_error = math.hypot(points[0][0]-4, points[0][1]-4)
    assert start_error <= 1.0, start_error
    for x,y in points:
        ix,iy = int(x/.1),int(y/.1)
        assert 0 <= ix < 320 and 0 <= iy < 200 and scene.data[iy*320+ix] < 65, (x,y)
    with lock:
        planned = [p for ts,p in observed['/coverage/planned_path'] if ts >= start and p.poses]
        swaths = [m for ts,array in observed['/coverage/markers'] if ts >= start
                  for m in array.markers if m.ns == 'coverage_swaths' and m.points]
    assert planned, 'no geometry published'
    assert swaths, 'no swath geometry published'
    entrance = swaths[-1].points[0]
    end_error = math.hypot(points[-1][0]-entrance.x,points[-1][1]-entrance.y)
    assert end_error <= .4, 'preview ends away from the first swath entrance'
    (RUN / (name + '_path.json')).write_text(json.dumps(dict(path=plain(path),
        planned_path=plain(planned[-1]), swaths=plain(swaths[-1]),
        request=plain(req), map=plain(scene)), indent=2))
    check(name, published_nonempty=True, final_points=len(final.poses),
          late_subscriber_points=len(latched.poses), duration_sec=round(time.time()-start,3),
          navfn_points=len(points), length_m=round(length,3), start_error_m=round(start_error,3),
          endpoint_error_m=round(end_error,6), hold_after_failure_sec=hold_sec,
          max_y=max(y for x,y in points), goal=list(points[-1]))
    return path

def cancel_preview(name):
    response = cancel()
    assert response.success, response.message
    wait(lambda: latest('/coverage/first_entry_path') is not None
         and not latest('/coverage/first_entry_path').poses, 'cancel clears preview')
    assert not rospy.wait_for_message('/coverage/first_entry_path', NavPath, timeout=3).poses
    check(name)

def start_manager(script, name):
    with lock:
        observed['/coverage/status'].clear()
        observed['/coverage/first_entry_path'].clear()
    proc = spawn(name, [sys.executable, str(script), '__name:=coverage_manager'])
    rospy.wait_for_service('/coverage/start_batch', timeout=20)
    map_pub.publish(scene)
    wait(lambda: latest('/coverage/status') is not None and latest('/coverage/status').localized
         and latest('/coverage/status').map_ready, 'manager map/localization')
    # Each latched visualization publisher establishes its own ROS TCP
    # connection.  The status connection being ready does not prove that the
    # marker connection is ready; let the one-time handshakes settle before a
    # deliberately fast precheck failure publishes ADD followed by DELETEALL.
    time.sleep(.5)
    return proc

try:
    master = spawn('roscore', ['roscore', '-p', '11491'])
    def master_ready():
        try:
            return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/sim_probe')[0] == 1
        except OSError:
            return False
    wait(master_ready, 'private ROS master', timeout=20)
    import rospy
    import yaml
    import tf2_ros
    from geometry_msgs.msg import Point32, TransformStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Path as NavPath, Odometry
    from nav_msgs.srv import GetPlan, GetPlanRequest
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String, Bool
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import MarkerArray
    from move_base_msgs.msg import MoveBaseActionGoal
    from autolabor_canbus_driver.msg import ChassisStatusInfo
    from autolabor_coverage.msg import CoverageStatus, CoverageRegion, PlanHybridTransitionsActionGoal
    from autolabor_coverage.srv import StartCoverageBatch, StartCoverageBatchRequest, CancelCoverageBatch

    rospy.init_node('first_entry_simulation')
    config_dir = WS / 'src/application/autolabor_coverage/config'
    config = yaml.safe_load((config_dir / 'coverage.yaml').read_text())
    config['config_file'] = str(config_dir / 'coverage.yaml')
    config['factory_defaults_file'] = str(config_dir / 'coverage_factory_defaults.yaml')
    rospy.set_param('/coverage_manager', config)
    footprint = [[.62,.45],[.62,-.45],[-.62,-.45],[-.62,.45]]
    for ns in ('global_costmap', 'local_costmap'):
        rospy.set_param('/move_base/' + ns, dict(global_frame='map', robot_base_frame='base_link',
            update_frequency=5., publish_frequency=2., transform_tolerance=1., rolling_window=False,
            footprint=footprint, footprint_padding=.05,
            plugins=[dict(name='static_layer',type='costmap_2d::StaticLayer'),
                     dict(name='inflation',type='costmap_2d::InflationLayer')],
            static_layer=dict(map_topic='/map', first_map_only=False, subscribe_to_updates=True),
            inflation=dict(inflation_radius=1.2, cost_scaling_factor=3.)))
    rospy.set_param('/move_base/base_global_planner', 'autolabor_coverage/CoverageGlobalPlanner')
    rospy.set_param('/move_base/base_local_planner', 'teb_local_planner/TebLocalPlannerROS')
    rospy.set_param('/move_base/CoverageGlobalPlanner_navfn/allow_unknown', False)
    rospy.set_param('/move_base/recovery_behavior_enabled', False)
    rospy.set_param('/move_base/clearing_rotation_allowed', False)
    rospy.set_param('/move_base/TebLocalPlannerROS', dict(odom_topic='/odom', min_turning_radius=1.35,
        wheelbase=.65, max_vel_x=.8, max_vel_x_backwards=.3, max_vel_theta=.6,
        footprint_model=dict(type='polygon',vertices=footprint)))
    topics = [('/map',OccupancyGrid),('/fast_lio/localization_status',String),
              ('/nvidia_cmd_vel_watchdog/status',String),('/scan',LaserScan),
              ('/avoidance/dual_lidar_active',Bool),('/m2_driver/chassis_info',ChassisStatusInfo),
              ('/odom',Odometry),('/Odometry',Odometry)]
    publishers = {t:rospy.Publisher(t,c,queue_size=5,latch=(t=='/map')) for t,c in topics}
    map_pub = publishers['/map']
    scene = map_scene('open')
    map_pub.publish(scene)
    broadcaster = tf2_ros.TransformBroadcaster()
    def tick(_):
        now = rospy.Time.now()
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = 4
        transform.transform.translation.y = 4
        transform.transform.rotation.w = 1
        broadcaster.sendTransform(transform)
        publishers['/fast_lio/localization_status'].publish(String('state=LOCALIZED;backend=synthetic_fixture'))
        publishers['/nvidia_cmd_vel_watchdog/status'].publish(String(json.dumps(dict(
            motion_enabled=False,max_linear_speed=1.6,max_angular_speed=1.))))
        scan = LaserScan()
        scan.header.stamp,scan.header.frame_id = now,'base_link'
        scan.angle_min,scan.angle_max,scan.angle_increment = -math.pi,math.pi,math.pi/180
        scan.range_min,scan.range_max,scan.ranges = .1,30.,[30.]*361
        publishers['/scan'].publish(scan)
        publishers['/avoidance/dual_lidar_active'].publish(Bool(True))
        publishers['/m2_driver/chassis_info'].publish(ChassisStatusInfo())
        odom = Odometry()
        odom.header.stamp,odom.header.frame_id = now,'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 4
        odom.pose.pose.position.y = 4
        odom.pose.pose.orientation.w = 1
        publishers['/odom'].publish(odom)
        publishers['/Odometry'].publish(odom)
    timer = rospy.Timer(rospy.Duration(.05), tick)
    subscriptions = []
    for topic,cls in [('/coverage/status',CoverageStatus),('/coverage/first_entry_path',NavPath),
                      ('/coverage/planned_path',NavPath),('/coverage/markers',MarkerArray),
                      ('/move_base/global_costmap/costmap',OccupancyGrid),
                      ('/move_base/goal',MoveBaseActionGoal),('/cmd_vel',Twist),
                      ('/move_base/CoverageGlobalPlanner/plan_hybrid_transitions/goal',PlanHybridTransitionsActionGoal)]:
        subscriptions.append(rospy.Subscriber(topic,cls,receive,callback_args=topic,queue_size=100))
    spawn('move_base', ['rosrun','move_base','move_base','__name:=move_base'])
    spawn('navigation_pause', [sys.executable,str(WS/'src/platform/autolabor_dual_host/scripts/move_base_pause_bridge.py'),
                               '__name:=navigation_pause'])
    rospy.wait_for_service('/move_base/CoverageGlobalPlanner/make_navfn_plan', timeout=35)
    rospy.wait_for_service('/navigation_pause/set_coverage_owner', timeout=15)
    start_batch = rospy.ServiceProxy('/coverage/start_batch',StartCoverageBatch)
    cancel = rospy.ServiceProxy('/coverage/cancel',Trigger)
    baseline = args.baseline or ROOT / 'baseline_coverage_manager.py'
    current = WS / 'src/application/autolabor_coverage/scripts/coverage_manager.py'
    event('identity', current_sha256=hashlib.sha256(current.read_bytes()).hexdigest(),
          master=os.environ['ROS_MASTER_URI'])
    if args.baseline is not None or baseline.is_file():
        assert hashlib.sha256(baseline.read_bytes()).hexdigest() == '9959a2c17574628a1af012481f77004be9f6e33b62a2341d128d6340439306aa'
        event('baseline_identity', sha256=hashlib.sha256(baseline.read_bytes()).hexdigest())
        manager = start_manager(baseline, 'manager_baseline')
        run_batch('baseline_path_is_erased', expected_preview=False)
        stop(manager)
        time.sleep(.5)
    else:
        event('historical_baseline_not_supplied', note='Current implementation checks still run')
    manager = start_manager(current, 'manager_fixed')
    for index in range(3):
        run_batch('fixed_open_%d' % index, hold_sec=2.1 if index == 2 else .25)
        cancel_preview('explicit_cancel_%d' % index)
    set_scene('detour')
    path = run_batch('fixed_wall_detour')
    assert max(p.pose.position.y for p in path.poses) > 13.0
    check('path_actually_goes_around_wall')
    first_end = copy.deepcopy(path.poses[-1].pose.position)
    path = run_batch('new_batch_replaces_preview',shift=2)
    assert math.hypot(path.poses[-1].pose.position.x-first_end.x,
                      path.poses[-1].pose.position.y-first_end.y) > 1.
    check('new_batch_has_distinct_entry')
    set_scene('detour',revision=1)
    wait(lambda: not latest('/coverage/first_entry_path').poses, 'map change clears retained path')
    check('map_change_invalidates_retained_preview')
    req = request()
    assert start_batch(req).accepted
    canceled = rospy.ServiceProxy('/coverage/cancel_batch',CancelCoverageBatch)(req.client_request_id)
    assert canceled.success
    wait(lambda: not latest('/coverage/status').batch_active and
         latest('/coverage/status').batch_id == req.client_request_id, 'cancel during planning')
    assert not latest('/coverage/first_entry_path').poses
    check('cancel_during_planning_leaves_no_stale_path',state=latest('/coverage/status').state)
    set_scene('sealed')
    query = GetPlanRequest()
    query.start.header.frame_id=query.goal.header.frame_id='map'
    query.start.pose.position.x=query.start.pose.position.y=4.
    query.start.pose.orientation.w=query.goal.pose.orientation.w=1.
    query.goal.pose.position.x,query.goal.pose.position.y=20.,6.
    response = rospy.ServiceProxy('/move_base/CoverageGlobalPlanner/make_navfn_plan',GetPlan)(query)
    assert not response.plan.poses, 'closed wall unexpectedly has a Navfn route'
    check('sealed_map_returns_no_navfn_path')
    assert not observed['/move_base/goal'], 'preview submitted a movement goal'
    assert not observed['/move_base/CoverageGlobalPlanner/plan_hybrid_transitions/goal'], 'unexpected Hybrid goal'
    assert all(not any((msg.linear.x,msg.linear.y,msg.angular.z)) for _,msg in observed['/cmd_vel'])
    check('no_motion_or_hybrid_actions', move_base_goals=0, hybrid_goals=0,
          cmd_vel_samples=len(observed['/cmd_vel']))
    (RUN/'results.json').write_text(json.dumps(dict(passed=True,checks=results),indent=2))
    print('SIMULATION PASSED',str(RUN),flush=True)
except Exception as error:
    import traceback
    traceback.print_exc()
    event('failure',error=str(error))
    (RUN/'results.json').write_text(json.dumps(dict(passed=False,error=str(error),checks=results),indent=2))
    raise
finally:
    for proc in reversed(processes):
        stop(proc)
    event('all_simulation_processes_stopped')
