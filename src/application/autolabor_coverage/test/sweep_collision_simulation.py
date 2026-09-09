#!/usr/bin/env python3
"""Exercise the actual coverage plugin through make_plan in an isolated graph."""
import argparse
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
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('--horizon', type=float)
parser.add_argument('--expect-legacy', action='store_true')
args = parser.parse_args()
WS = Path(__file__).resolve().parents[4]
if args.horizon is None:
    args.horizon = yaml.safe_load((WS / 'src/application/autolabor_coverage/config/coverage_global_planner.yaml').read_text())['sweep_collision_check_horizon']
ROOT = WS / 'validation/sweep_collision_simulation'
RUN = ROOT / datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
RUN.mkdir(parents=True)
(ROOT / 'latest_run.txt').write_text(str(RUN) + '\n')
os.environ['ROS_LOG_DIR'] = str(RUN / 'ros_logs')
os.environ['ROS_HOME'] = str(RUN / 'ros_home')
assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:11493'
assert [name for _, name in socket.if_nameindex()] == ['lo']
processes, checks, commands, goals = [], [], [], []

def spawn(name, command):
    proc = subprocess.Popen(command, stdout=open(RUN / (name + '.log'), 'w'),
                            stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(proc)
    return proc

def wait_for(predicate, description, timeout=25):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('timeout: ' + description)

def check(name, **details):
    checks.append(dict(name=name, passed=True, **details))
    print('PASS', name, json.dumps(details), flush=True)

try:
    spawn('roscore', ['roscore', '-p', '11493'])
    def master_ready():
        try:
            return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0] == 1
        except OSError:
            return False
    wait_for(master_ready, 'master')
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.srv import GetPlan, GetPlanRequest
    from move_base_msgs.msg import MoveBaseActionGoal
    from autolabor_coverage.msg import EnforcedPath, PlanHybridTransitionsActionGoal
    from autolabor_coverage.srv import SetEnforcedPath, SetEnforcedPathRequest
    rospy.init_node('sweep_collision_simulation')
    footprint = [[.52,.35],[.52,-.35],[-.52,-.35],[-.52,.35]]
    for ns in ('global_costmap', 'local_costmap'):
        rospy.set_param('/move_base/' + ns, dict(
            global_frame='map', robot_base_frame='base_link', update_frequency=10.,
            publish_frequency=10., transform_tolerance=1., rolling_window=False,
            footprint=footprint, footprint_padding=.10, always_send_full_costmap=True,
            plugins=[dict(name='static_layer', type='costmap_2d::StaticLayer')],
            static_layer=dict(map_topic='/map', first_map_only=False,
                              subscribe_to_updates=True, track_unknown_space=True)))
    rospy.set_param('/move_base/base_global_planner', 'autolabor_coverage/CoverageGlobalPlanner')
    rospy.set_param('/move_base/base_local_planner', 'teb_local_planner/TebLocalPlannerROS')
    rospy.set_param('/move_base/make_plan_clear_costmap', False)
    rospy.set_param('/move_base/make_plan_add_unreachable_goal', False)
    rospy.set_param('/move_base/recovery_behavior_enabled', False)
    rospy.set_param('/move_base/clearing_rotation_allowed', False)
    rospy.set_param('/move_base/CoverageGlobalPlanner/sweep_collision_check_horizon', args.horizon)
    rospy.set_param('/move_base/TebLocalPlannerROS', dict(
        min_turning_radius=1.35, wheelbase=.65,
        footprint_model=dict(type='polygon', vertices=footprint)))
    publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width, grid.info.height, grid.info.resolution = 300, 100, .1
    grid.info.origin.orientation.w = 1
    grid.data = [0] * (300 * 100)
    publisher.publish(grid)
    robot_x = 3.
    broadcaster = tf2_ros.TransformBroadcaster()
    def tick(_):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = robot_x
        transform.transform.translation.y = 5.
        transform.transform.rotation.w = 1
        broadcaster.sendTransform(transform)
    timer = rospy.Timer(rospy.Duration(.03), tick)
    subscriptions = [rospy.Subscriber('/cmd_vel', Twist, commands.append),
                     rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, goals.append),
                     rospy.Subscriber('/move_base/CoverageGlobalPlanner/plan_hybrid_transitions/goal',
                                      PlanHybridTransitionsActionGoal, goals.append)]
    move_base = spawn('move_base', ['rosrun', 'move_base', 'move_base', '__name:=move_base'])
    if not math.isfinite(args.horizon) or args.horizon <= 0:
        wait_for(lambda: move_base.poll() is not None, 'invalid parameter rejects initialization')
        assert move_base.returncode != 0
        check('invalid_horizon_rejected', value=str(args.horizon))
    else:
        rospy.wait_for_service('/move_base/CoverageGlobalPlanner/set_enforced_path', timeout=35)
        rospy.wait_for_service('/move_base/make_plan', timeout=10)
        arm = rospy.ServiceProxy('/move_base/CoverageGlobalPlanner/set_enforced_path', SetEnforcedPath)
        plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        def pose(x, y=5., yaw=0.):
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x, msg.pose.position.y = x, y
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            return msg
        def scene(cells):
            grid.header.stamp = rospy.Time.now()
            grid.data = [0] * (300 * 100)
            for x,y,value in cells:
                grid.data[int(y/.1)*300+int(x/.1)] = value
            publisher.publish(grid)
            # Wait for an entire published costmap to reflect the new map.
            def updated():
                msg = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout=3)
                return list(msg.data) == list(grid.data)
            wait_for(updated, 'costmap update')
        def query(name, expected, start=3., endpoint=23., y=5., reverse=False):
            global robot_x
            robot_x = start
            req = SetEnforcedPathRequest()
            req.coverage_active = True
            msg = req.enforced_path
            msg.header.frame_id = msg.path.header.frame_id = 'map'
            msg.header.stamp = msg.path.header.stamp = rospy.Time.now()
            msg.plan_id = name
            msg.planner_mode = EnforcedPath.MODE_ENFORCED_SWEEP
            msg.active = True
            direction = -1 if reverse else 1
            xs = [start + direction * .1 * n for n in range(round(abs(endpoint-start)/.1)+1)]
            msg.path.poses = [pose(x, y, math.pi if reverse else 0.) for x in xs]
            assert arm(req).success
            request = GetPlanRequest(start=msg.path.poses[0], goal=msg.path.poses[-1], tolerance=0.)
            response = plan(request)
            # move_base's service can return the partial vector left by a
            # failed planner call. Only a complete plan to the requested end
            # counts as acceptance, as required by the coverage contract.
            accepted = bool(response.plan.poses) and abs(
                response.plan.poses[-1].pose.position.x-endpoint) < 1e-6
            assert accepted == expected, (name, len(response.plan.poses), expected)
            if expected:
                assert abs(response.plan.poses[-1].pose.position.x-endpoint) < 1e-6, 'path was truncated'
            check(name, accepted=accepted, output_points=len(response.plan.poses))
        scene([(18.,5.,100)])
        query('far_obstacle_15m', not args.expect_legacy)
        if not args.expect_legacy:
            query('moving_horizon_detects_same_obstacle', False, start=18.-args.horizon+.2)
            scene([(3.+args.horizon-.2,5.,100)])
            query('near_obstacle_blocks', False)
            scene([(3.+args.horizon+.9,5.,100)])
            query('outside_horizon_and_footprint_is_deferred', True)
            scene([(3.+args.horizon+.3,5.,100)])
            query('footprint_overhang_at_horizon_blocks', False)
            scene([(3.5,5.4,100)])
            query('off_center_footprint_collision_blocks', False)
            scene([(3.,5.,100)])
            query('current_footprint_collision_blocks', False)
            scene([(18.,5.,-1)])
            query('unknown_space_still_rejected', False)
            scene([])
            query('map_boundary_still_rejected', False, endpoint=31.)
            query('short_clear_line_keeps_endpoint', True, endpoint=4.)
            scene([(3.+args.horizon+.9,5.,100)])
            query('changed_start_projects_forward_window', False, start=4.)
            scene([(8.,5.,100)])
            query('opposite_world_direction_far_obstacle_deferred', True, start=23., endpoint=3., reverse=True)
            # Recorded field geometry, translated to this grid: a 19.1 m line
            # with one lethal cell 15.1 m ahead and 0.19 m off its center.
            scene([(18.1,5.19,100)])
            query('field_geometry_far_cell_deferred', True, endpoint=22.1)
    assert not goals, 'planner service test emitted action goals'
    assert all(not any((m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)) for m in commands)
    check('no_motion_goals_or_nonzero_commands', goals=len(goals), command_samples=len(commands))
    (RUN/'results.json').write_text(json.dumps(dict(passed=True, horizon=args.horizon,
        expect_legacy=args.expect_legacy, checks=checks), indent=2))
    print('SIMULATION PASSED', RUN, flush=True)
except Exception as error:
    (RUN/'results.json').write_text(json.dumps(dict(passed=False, error=str(error), checks=checks), indent=2))
    raise
finally:
    for proc in reversed(processes):
        if proc.poll() is not None:
            continue
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3)
