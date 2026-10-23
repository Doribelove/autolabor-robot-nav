#!/usr/bin/env python3
"""Real move_base/Navfn/TEB/bridge, synthetic poses, network namespace only."""
import copy
import datetime
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid
import xmlrpc.client

WS = Path(__file__).resolve().parents[4]
RUN = WS / 'validation/yaw_free_goal_simulation' / ('yaw_ros_' + datetime.datetime.now().strftime('%H%M%S'))
RUN.mkdir(parents=True)
assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11494'
assert [name for _, name in socket.if_nameindex()] == ['lo']
os.environ['ROS_LOG_DIR'] = str(RUN / 'ros_logs')
os.environ['ROS_HOME'] = str(RUN / 'ros_home')
processes, checks, commands, plans, echoed, statuses = [], [], [], [], {}, {}
debug = {}

def spawn(name, command):
    process = subprocess.Popen(command, stdout=open(RUN/(name+'.log'), 'w'),
                               stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(process)

def wait(predicate, description, timeout=20):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('timeout: '+description)

def check(name, **details):
    checks.append(dict(name=name, **details))
    print('PASS', name, details, flush=True)

try:
    spawn('roscore', ['roscore', '-p', '11494'])
    def master_ready():
        try:
            return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0] == 1
        except OSError:
            return False
    wait(master_ready, 'master')
    import rospy
    import tf2_ros
    from actionlib_msgs.msg import GoalID, GoalStatusArray
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from move_base_msgs.msg import MoveBaseActionGoal
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
    from autolabor_coverage.msg import CoverageStatus
    from autolabor_coverage.srv import SetOrdinaryGoalPolicy
    from std_msgs.msg import String
    rospy.init_node('yaw_free_simulation')
    for ns in ('global_costmap', 'local_costmap'):
        rospy.set_param('/move_base/'+ns, dict(global_frame='map', robot_base_frame='base_link',
            update_frequency=15., publish_frequency=5., transform_tolerance=.5,
            rolling_window=False, robot_radius=.25,
            plugins=[dict(name='static_layer', type='costmap_2d::StaticLayer')],
            static_layer=dict(map_topic='/map', track_unknown_space=True)))
    rospy.set_param('/move_base/base_global_planner', 'autolabor_coverage/CoverageGlobalPlanner')
    rospy.set_param('/move_base/base_local_planner', 'teb_local_planner/TebLocalPlannerROS')
    for key, val in dict(planner_frequency=3., controller_frequency=15., controller_patience=12.,
                         oscillation_timeout=0., recovery_behavior_enabled=False).items():
        rospy.set_param('/move_base/'+key, val)
    rospy.set_param('/move_base/TebLocalPlannerROS', dict(min_turning_radius=1.35, wheelbase=.65,
        odom_topic='/odom', enable_homotopy_class_planning=False, motion_direction_mode=0,
        max_vel_x=.3, max_vel_x_backwards=.2, max_vel_theta=.5,
        max_global_plan_lookahead_dist=2., global_plan_viapoint_sep=.8,
        global_plan_overwrite_orientation=True, xy_goal_tolerance=.15, yaw_goal_tolerance=.10,
        trans_stopped_vel=.03, theta_stopped_vel=.03))
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    coverage_pub = rospy.Publisher('/coverage/status', CoverageStatus, queue_size=1)
    request_pub = rospy.Publisher('/navigation_goal/action_request', MoveBaseActionGoal, queue_size=1)
    heartbeat_pub = rospy.Publisher('/navigation_goal/ai_heartbeat', GoalID, queue_size=1)
    cancel_pub = rospy.Publisher('/navigation_goal/cancel_request', GoalID, queue_size=1)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 120
    grid.info.resolution = .1
    grid.info.origin.orientation.w = 1
    grid.data = [0]*(120*120)
    map_pub.publish(grid)
    broadcaster = tf2_ros.TransformBroadcaster()
    robot = dict(x=5., y=5., yaw=math.pi/2, speed=0., odom_age=0.)
    active = ['']
    def tick(_):
        stamp = rospy.Time.now()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y = robot['x'], robot['y']
        t.transform.rotation.z, t.transform.rotation.w = math.sin(robot['yaw']/2), math.cos(robot['yaw']/2)
        broadcaster.sendTransform(t)
        odom = Odometry()
        odom.header = copy.deepcopy(t.header)
        odom.header.stamp -= rospy.Duration(robot['odom_age'])
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = robot['x'], robot['y']
        odom.pose.pose.orientation = copy.deepcopy(t.transform.rotation)
        odom.twist.twist.linear.x = robot['speed']
        odom_pub.publish(odom)
        coverage = CoverageStatus()
        coverage.header.stamp, coverage.state = stamp, 'IDLE'
        coverage_pub.publish(coverage)
        if active[0]:
            heartbeat_pub.publish(GoalID(id=active[0]))
    timer = rospy.Timer(rospy.Duration(.025), tick)
    def status(msg):
        for item in msg.status_list:
            statuses[item.goal_id.id] = item.status
    subscribers = [rospy.Subscriber('/move_base/status', GoalStatusArray, status),
        rospy.Subscriber('/navigation_pause/status', String, lambda m: debug.update(bridge=m.data)),
        rospy.Subscriber('/move_base/current_goal', PoseStamped, lambda m: debug.update(current_goal=str(m))),
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, lambda m: echoed.update({m.goal_id.id:m})),
        rospy.Subscriber('/cmd_vel', Twist, lambda m: commands.append((time.monotonic(), m))),
        rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', RosPath, plans.append)]
    spawn('move_base', ['rosrun','move_base','move_base','__name:=move_base'])
    rospy.wait_for_service('/move_base/CoverageGlobalPlanner/set_ordinary_goal_policy', timeout=40)
    import dynamic_reconfigure.client
    tuning = dynamic_reconfigure.client.Client('/move_base/CoverageGlobalPlanner', timeout=5)
    original_tuning = tuning.get_configuration(timeout=5)
    changed = tuning.update_configuration(dict(hybrid_motion_step=.25, hybrid_max_expansions=60000))
    assert changed['hybrid_motion_step'] == .25 and changed['hybrid_max_expansions'] == 60000
    check('Hybrid runtime tuning acknowledged', motion_step=changed['hybrid_motion_step'])
    corrected = tuning.update_configuration(dict(hybrid_steering_samples=6))
    assert corrected['hybrid_steering_samples'] == 5
    check('Hybrid even steering sample count corrected to valid odd count')
    finite = tuning.update_configuration(dict(hybrid_motion_step=float('nan')))
    assert finite['hybrid_motion_step'] == .25
    check('Hybrid nonfinite tuning preserves actual runtime value')
    tuning.update_configuration({key: value for key, value in original_tuning.items() if key != 'groups'})
    check('Hybrid tuning restored before navigation regression')
    # The public repository exports this bridge without the complete platform
    # catkin package, so start the checked-in script directly in both source
    # and standalone layouts.
    spawn('bridge', [sys.executable,
        str(WS/'src/platform/autolabor_dual_host/scripts/move_base_pause_bridge.py'),
        '__name:=navigation_pause'])
    wait(lambda: request_pub.get_num_connections() > 0, 'bridge')
    time.sleep(1.)
    policy_service = rospy.ServiceProxy('/move_base/CoverageGlobalPlanner/set_ordinary_goal_policy', SetOrdinaryGoalPolicy)
    for bad in ('old_stamp', 'zero_stamp', 'wrong_frame', 'invalid_quaternion'):
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.header.stamp = rospy.Time.now()
        target.pose.position.x = target.pose.position.y = 5.
        target.pose.orientation.w = 1.
        if bad == 'old_stamp': target.header.stamp -= rospy.Duration(5.)
        if bad == 'zero_stamp': target.header.stamp = rospy.Time()
        if bad == 'wrong_frame': target.header.frame_id = 'odom'
        if bad == 'invalid_quaternion': target.pose.orientation.w = 0.
        result = policy_service('sweeper-ai-'+uuid.uuid4().hex, target, True)
        assert not result.success, bad
    check('invalid_goal_policies_rejected')
    target = PoseStamped()
    target.header.frame_id = 'map'
    target.header.stamp = rospy.Time.now()
    target.pose.position.x = target.pose.position.y = 5.
    target.pose.orientation.w = 1.
    assert policy_service('sweeper-ai-'+uuid.uuid4().hex, target, True).success
    target.header.stamp -= rospy.Duration(.05)
    assert not policy_service('sweeper-ai-'+uuid.uuid4().hex, target, True).success
    check('delayed_older_policy_cannot_replace_newer_goal_policy')
    def submit(yaw_free, x=5., y=5., yaw=0.):
        action = MoveBaseActionGoal()
        action.header.stamp = action.goal_id.stamp = action.goal.target_pose.header.stamp = rospy.Time.now()
        action.goal_id.id = 'sweeper-ai-'+uuid.uuid4().hex
        target = action.goal.target_pose
        target.header.frame_id = 'map'
        target.header.seq = 0xA14F0001 if yaw_free else 0
        target.pose.position.x, target.pose.position.y = x, y
        target.pose.orientation.z, target.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        active[0] = action.goal_id.id
        request_pub.publish(action)
        wait(lambda: active[0] in echoed, 'bridge action echo')
        return action.goal_id.id
    def cancel(identifier):
        cancel_pub.publish(GoalID(id=identifier))
        wait(lambda: statuses.get(identifier) in (2,3,4,5,8), 'terminal cancellation')
        active[0] = ''
        time.sleep(.4)

    first = submit(True)
    wait(lambda: statuses.get(first) == 3, 'yaw-free 90 degree arrival')
    check('free_goal_succeeds_at_xy_with_90_degree_heading_error')
    cancel(first)
    fixed = submit(False)
    time.sleep(1.2)
    assert statuses.get(fixed) != 3
    check('same_xy_new_fixed_yaw_goal_does_not_inherit_free_policy', state=statuses.get(fixed))
    cancel(fixed)
    aligned = submit(False, yaw=robot['yaw'])
    wait(lambda: statuses.get(aligned) == 3, 'explicit matched yaw')
    check('explicit_matching_yaw_succeeds')
    cancel(aligned)
    robot['speed'] = .2
    time.sleep(.2)
    moving = submit(True)
    start = time.monotonic()
    time.sleep(1.)
    assert statuses.get(moving) != 3
    recent = [m for t,m in commands if t > start+.3]
    assert recent and all(m.linear.x == 0 and m.angular.z == 0 for m in recent)
    robot['speed'] = 0.
    wait(lambda: statuses.get(moving) == 3, 'physical stop')
    check('free_goal_brakes_and_waits_for_physical_stop')
    cancel(moving)
    robot['odom_age'] = 2.
    time.sleep(.2)
    stale = submit(True)
    time.sleep(1.)
    assert statuses.get(stale) != 3
    robot['odom_age'] = 0.
    wait(lambda: statuses.get(stale) == 3, 'fresh stopped odometry')
    check('stale_zero_odometry_cannot_complete_free_goal')
    cancel(stale)
    robot['yaw'] = math.pi
    time.sleep(.2)
    reverse = submit(True)
    wait(lambda: statuses.get(reverse) == 3, '180 degree yaw-free arrival')
    check('free_goal_succeeds_at_xy_with_180_degree_heading_error')
    cancel(reverse)
    robot['yaw'] = math.pi/2
    time.sleep(.2)
    outside = submit(True, y=7.)
    time.sleep(1.)
    assert statuses.get(outside) != 3
    assert any(p.poses and all((x.header.seq & 0xC0000000) == 0x80000000 for x in p.poses) for p in plans)
    check('free_goal_still_requires_xy_and_marker_survives_replanning')
    # Change only the synthetic pose, then verify arbitrary final yaw at the
    # same live GoalID. This is a pose-injection test, not a driving trial.
    robot['y'], robot['yaw'] = 7., math.pi
    wait(lambda: statuses.get(outside) == 3, 'arrival at original xy with arbitrary yaw')
    check('same_goal_completes_after_xy_arrival_with_different_heading')
    cancel(outside)
    (RUN/'results.json').write_text(json.dumps(dict(passed=True, checks=checks), indent=2))
except Exception as exc:
    (RUN/'results.json').write_text(json.dumps(dict(passed=False, error=str(exc), checks=checks), indent=2))
    raise
finally:
    debug.update(statuses=statuses, echoed={k:str(v) for k,v in echoed.items()},
                 plans=[str(p) for p in plans[-2:]], commands=[str(m) for t,m in commands[-5:]])
    (RUN/'debug.json').write_text(json.dumps(debug, indent=2))
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3)
    print('Evidence:', RUN, flush=True)
