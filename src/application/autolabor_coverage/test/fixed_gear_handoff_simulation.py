#!/usr/bin/env python3
"""Real move_base/global-planner/TEB hand-offs in a network-isolated graph.

The robot pose is synthetic and held stationary. This verifies action metadata
and command direction, not physical tracking or steering convergence.
"""
import copy
import datetime
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

WS = Path(__file__).resolve().parents[4]
ROOT = WS / 'validation/fixed_gear_handoff_20260911'
RUN = ROOT / datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
RUN.mkdir(parents=True)
(ROOT / 'latest_run.txt').write_text(str(RUN) + '\n')
assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11494'
assert [name for _, name in socket.if_nameindex()] == ['lo']
os.environ['ROS_LOG_DIR'] = str(RUN / 'ros_logs')
os.environ['ROS_HOME'] = str(RUN / 'ros_home')
processes, checks, logs, commands, local_plans = [], [], [], [], []


def spawn(name, command):
    proc = subprocess.Popen(command, stdout=open(RUN / (name + '.log'), 'w'),
                            stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(proc)
    return proc


def wait_for(predicate, description, timeout=25):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('timeout: ' + description)


def check(name, **details):
    checks.append(dict(name=name, passed=True, **details))
    print('PASS', name, json.dumps(details), flush=True)


try:
    spawn('roscore', ['roscore', '-p', '11494'])

    def master_ready():
        try:
            return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0] == 1
        except OSError:
            return False

    wait_for(master_ready, 'isolated master')
    import actionlib
    import rospy
    import tf2_ros
    from dynamic_reconfigure.client import Client
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
    from nav_msgs.srv import GetPlan, GetPlanRequest
    from rosgraph_msgs.msg import Log
    from autolabor_coverage.msg import EnforcedPath
    from autolabor_coverage.srv import SetEnforcedPath, SetEnforcedPathRequest

    rospy.init_node('fixed_gear_handoff_simulation')
    footprint = [[.52, .35], [.52, -.35], [-.52, -.35], [-.52, .35]]
    for ns in ('global_costmap', 'local_costmap'):
        rospy.set_param('/move_base/' + ns, dict(
            global_frame='map', robot_base_frame='base_link',
            update_frequency=10., publish_frequency=10., transform_tolerance=1.,
            rolling_window=False, footprint=footprint,
            plugins=[dict(name='static_layer', type='costmap_2d::StaticLayer')],
            static_layer=dict(map_topic='/map', track_unknown_space=True)))
    rospy.set_param('/move_base/base_global_planner', 'autolabor_coverage/CoverageGlobalPlanner')
    rospy.set_param('/move_base/base_local_planner', 'teb_local_planner/TebLocalPlannerROS')
    for name, value in dict(planner_frequency=5., controller_frequency=10.,
                            controller_patience=8., oscillation_timeout=0.,
                            recovery_behavior_enabled=False,
                            make_plan_clear_costmap=False,
                            make_plan_add_unreachable_goal=False).items():
        rospy.set_param('/move_base/' + name, value)
    rospy.set_param('/move_base/TebLocalPlannerROS', dict(
        min_turning_radius=1.35, wheelbase=.65, odom_topic='/odom',
        enable_homotopy_class_planning=False, motion_direction_mode=1,
        max_vel_x=.8, max_vel_x_backwards=.3, max_vel_theta=.6,
        max_global_plan_lookahead_dist=2., global_plan_viapoint_sep=.3,
        global_plan_overwrite_orientation=False, allow_init_with_backwards_motion=True,
        xy_goal_tolerance=.04, yaw_goal_tolerance=.10,
        weight_kinematics_forward_drive=1000.,
        footprint_model=dict(type='polygon', vertices=footprint)))
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    lease_pub = rospy.Publisher('/coverage/enforced_path', EnforcedPath, queue_size=1)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 160
    grid.info.resolution = .1
    grid.info.origin.orientation.w = 1
    grid.data = [0] * (160 * 160)
    map_pub.publish(grid)
    broadcaster = tf2_ros.TransformBroadcaster()
    active_path = [None]

    def tick(_):
        stamp = rospy.Time.now()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = transform.transform.translation.y = 5.
        transform.transform.rotation.w = 1
        broadcaster.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = odom.pose.pose.position.y = 5.
        odom.pose.pose.orientation.w = 1
        odom_pub.publish(odom)
        if active_path[0] is not None:
            msg = copy.deepcopy(active_path[0])
            msg.header.stamp = msg.path.header.stamp = stamp
            lease_pub.publish(msg)

    timer = rospy.Timer(rospy.Duration(.03), tick)
    subscriptions = [
        rospy.Subscriber('/rosout', Log, lambda msg: logs.append(msg.msg)),
        rospy.Subscriber('/cmd_vel', Twist, commands.append),
        rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', RosPath, local_plans.append),
    ]
    spawn('move_base', ['rosrun', 'move_base', 'move_base', '__name:=move_base'])
    rospy.wait_for_service('/move_base/CoverageGlobalPlanner/set_enforced_path', timeout=35)
    rospy.wait_for_service('/move_base/make_plan', timeout=10)
    arm = rospy.ServiceProxy('/move_base/CoverageGlobalPlanner/set_enforced_path', SetEnforcedPath)
    query = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    configure = Client('/move_base/TebLocalPlannerROS', timeout=10)
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    assert client.wait_for_server(rospy.Duration(10))

    def pose(x):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y = x, 5.
        msg.pose.orientation.w = 1
        return msg

    sequences = []
    for name, gear, length, sweep in (
            ('forward_transit', 1, 2., False),
            ('same_gear_new_action', 1, 2.1, False),
            ('short_reverse_cusp', -1, .3, False),
            ('forward_sweep', 1, 2., True),
            ('same_sweep_resume', 1, 2., True)):
        configure.update_configuration(dict(motion_direction_mode=gear,
                                            max_vel_x_backwards=.3 if gear < 0 else 0.))
        request = SetEnforcedPathRequest()
        request.coverage_active = True
        msg = request.enforced_path
        msg.plan_id = 'same-plan-for-all-actions'
        msg.segment_index = 1
        msg.path_generation = 1
        msg.header.frame_id = msg.path.header.frame_id = 'map'
        msg.header.stamp = msg.path.header.stamp = rospy.Time.now()
        msg.active = sweep
        msg.planner_mode = (EnforcedPath.MODE_ENFORCED_SWEEP if sweep
                            else EnforcedPath.MODE_HYBRID_TRANSIT)
        msg.expected_gear = EnforcedPath.GEAR_AUTO if sweep else gear
        count = round(length / .1)
        msg.path.poses = [pose(5. + gear * length * n / count) for n in range(count + 1)]
        profile = request.transit_profile
        profile.allow_reverse = gear < 0
        profile.max_forward_speed_mps, profile.max_reverse_speed_mps = .8, .3
        profile.max_angular_speed_rps = .6
        profile.linear_accel_mps2, profile.angular_accel_rps2 = 1., .5
        profile.goal_position_tolerance_m, profile.goal_yaw_tolerance_rad = .04, .10
        profile.replan_period_sec = .5
        active_path[0] = None
        assert arm(request).success
        active_path[0] = msg
        plan_request = GetPlanRequest(start=pose(5.), goal=msg.path.poses[-1], tolerance=0.)
        first = query(plan_request).plan.poses
        assert len(first) >= 2
        sequence = first[-1].header.seq
        assert sequence & 0xC0000000 == 0xC0000000
        assert all(p.header.seq == sequence for p in first)
        assert sequence not in sequences, 'new installation reused a previous action marker'
        sequences.append(sequence)
        cmd_begin, plan_begin, log_begin = len(commands), len(local_plans), len(logs)
        client.send_goal(MoveBaseGoal(target_pose=msg.path.poses[-1]))
        text = 'fixed-gear coverage action {}'.format(sequence)
        wait_for(lambda: any(text in line for line in logs[log_begin:]), 'TEB receives ' + name)
        wait_for(lambda: len(local_plans) >= plan_begin + 8, 'TEB control cycles for ' + name)
        time.sleep(.4)  # Include several global planner and topic lease refreshes.
        assert all(p.poses[-1].header.seq == sequence for p in local_plans[plan_begin:] if p.poses)
        resets = sum(text in line for line in logs[log_begin:])
        assert resets == 1, (name, 'repeated action reset', resets)
        sample = commands[cmd_begin:]
        assert sample and any(gear * c.linear.x > .001 for c in sample), 'no directional output: ' + name
        assert all(gear * c.linear.x >= -1.e-6 for c in sample), 'wrong gear output: ' + name
        assert all(abs(c.angular.z) <= abs(c.linear.x) / 1.35 + 1.e-6 for c in sample)
        client.cancel_goal()
        assert client.wait_for_result(rospy.Duration(3)), 'exact synthetic goal did not cancel'
        time.sleep(.15)
        # move_base intentionally refuses its make_plan service during an
        # active action. Query only after that exact synthetic action ends.
        again = query(plan_request).plan.poses
        assert again and again[-1].header.seq == sequence, 'refresh changed action marker'
        check(name, sequence=sequence, action_resets=resets, command_samples=len(sample))
    check('network_isolation', interfaces=[name for _, name in socket.if_nameindex()],
          real_robot_connected=False)
    (RUN / 'results.json').write_text(json.dumps(dict(passed=True, checks=checks), indent=2))
    print('SIMULATION PASSED', RUN, flush=True)
except Exception as error:
    (RUN / 'results.json').write_text(json.dumps(dict(passed=False, error=str(error), checks=checks), indent=2))
    raise
finally:
    (RUN / 'observed_logs.json').write_text(json.dumps(logs, indent=2))
    for proc in reversed(processes):
        if proc.poll() is not None:
            continue
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3)
