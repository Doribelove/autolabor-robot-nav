"""ROS/Gazebo adapter for the dependency-free long-running training environment."""

import math
import random
import threading
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Empty as EmptyService

from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.reward_cost import FeedbackSample, WindowEvents
from teb_rl_tuner.state_builder import ScanAngularMetadata
from teb_rl_tuner.teb_parameter_client import TebParameterClient
from teb_rl_tuner.training_environment import (
    ActivationPoll,
    FeedbackWindow,
    ObservationInput,
    ParameterWriteReceipt,
    SafeParameterDecision,
)


FEATURE_ORDER = (
    "footprint_clearance", "obstacle_density", "approximate_ttc",
    "goal_distance", "goal_bearing_sin", "goal_bearing_cos",
    "path_cross_track_error", "path_heading_error", "linear_velocity",
    "angular_velocity", "linear_acceleration", "planner_valid",
    "sensor_valid", "tf_valid", "localization_valid", "interface_valid",
) + tuple("theta_{}".format(name) for name in EXPECTED_THETA_ORDER)


def _stamp(message: Any) -> float:
    value = message.header.stamp.to_sec()
    return value if value > 0.0 else rospy.Time.now().to_sec()


def _yaw(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _angle_delta(after: float, before: float) -> float:
    return math.atan2(math.sin(after - before), math.cos(after - before))


class GazeboTrainingAdapter:
    """One persistent Gazebo instance supporting repeated reset/step cycles."""

    def __init__(
        self,
        parameter_client: Optional[TebParameterClient],
        scenarios: Sequence[Mapping[str, Any]],
        theta_bounds: Optional[Mapping[str, Sequence[float]]],
        observation_timeout_s: float = 5.0,
        planner_namespace: str = "TebLocalPlannerROS",
        initial_theta: Optional[Mapping[str, float]] = None,
    ) -> None:
        if not scenarios:
            raise ValueError("at least one Gazebo scenario is required")
        self.parameter_client = parameter_client
        self.scenarios = tuple(dict(item) for item in scenarios)
        self.theta_bounds = ({name: tuple(theta_bounds[name]) for name in EXPECTED_THETA_ORDER}
                             if theta_bounds is not None else {})
        self.planner_namespace = str(planner_namespace).strip("/")
        self.observation_timeout_s = float(observation_timeout_s)
        self._condition = threading.Condition()
        self.odom: Optional[Odometry] = None
        self.scan: Optional[LaserScan] = None
        self.global_plan: Optional[Path] = None
        self.local_plan: Optional[Path] = None
        self.local_plan_generation = 0
        self._last_read_odom_stamp = -1.0
        self._last_observation_time = 0.0
        if initial_theta is not None:
            if set(initial_theta) != set(EXPECTED_THETA_ORDER):
                raise ValueError("initial theta must contain the frozen nine-parameter order")
            self._current_theta = {
                name: float(initial_theta[name]) for name in EXPECTED_THETA_ORDER
            }
        else:
            self._current_theta = (
                dict(parameter_client.snapshot) if parameter_client is not None else {}
            )
        self._episode_snapshot = dict(self._current_theta)
        self._last_written_previous = dict(self._current_theta)
        self._last_written = dict(self._current_theta)
        self._previous_velocity = 0.0
        self._previous_sample_stamp: Optional[float] = None
        self._scenario_index = -1
        self._episode_seed = 0
        self.current_scenario: Mapping[str, Any] = {}
        self.goal_xy = (0.0, 0.0)
        self.path_length = 0.0
        self._path_previous_xy: Optional[Tuple[float, float]] = None
        self.minimum_clearance = float("inf")
        self.last_metrics: Dict[str, float] = {}
        self.planner_cycle_count = 0
        self._feedback_last_generation = 0
        self.safety_mode = "NORMAL"
        self.fallback_active = False
        self.emergency_active = False
        self._boundary_quiet_period_s = 0.25
        self._boundary_recovery_quiet_period_s = 1.0
        self._boundary_quiesce_timeout_s = 5.0
        self._boundary_quiesce_count = 0
        self._boundary_quiesce_failure_count = 0
        self._boundary_last_terminal_state: Optional[int] = None
        self._boundary_last_quiet_period_s = 0.0
        self._activation_timeout_barrier_count = 0
        self._activation_timeout_last_config_seq: Optional[int] = None
        self._require_recovery_barrier = False

        self.reset_odom_pub = rospy.Publisher("/m2_driver/reset_odom", Empty, queue_size=1)
        self.brake_pub = rospy.Publisher("/m2_driver/brake_set", Bool, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=100)
        rospy.Subscriber("/scan", LaserScan, self._scan_cb, queue_size=20)
        planner_root = "/move_base/{}".format(self.planner_namespace)
        rospy.Subscriber(planner_root + "/global_plan", Path,
                         self._global_plan_cb, queue_size=10)
        rospy.Subscriber(planner_root + "/local_plan", Path,
                         self._local_plan_cb, queue_size=50)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=15.0)
        rospy.wait_for_service("/move_base/clear_costmaps", timeout=20.0)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", EmptyService)
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base.wait_for_server(rospy.Duration(20.0)):
            raise RuntimeError("move_base action server unavailable")
        rospy.wait_for_message("/odom", Odometry, timeout=10.0)
        rospy.wait_for_message("/scan", LaserScan, timeout=10.0)

    def set_scenarios(self, scenarios: Sequence[Mapping[str, Any]]) -> None:
        if not scenarios:
            raise ValueError("at least one Gazebo scenario is required")
        updated = tuple(dict(item) for item in scenarios)
        # T12 single-factor curriculum repair: model.learn calls this after every
        # episode.  Resetting the index for an unchanged curriculum made every
        # training episode select only the first easy scene.
        if updated == self.scenarios:
            return
        self.move_base.cancel_all_goals()
        self.cmd_pub.publish(Twist())
        self.scenarios = updated
        self._scenario_index = -1

    def _odom_cb(self, message: Odometry) -> None:
        with self._condition:
            if self._path_previous_xy is not None:
                current = (message.pose.pose.position.x, message.pose.pose.position.y)
                distance = math.hypot(
                    current[0] - self._path_previous_xy[0],
                    current[1] - self._path_previous_xy[1],
                )
                if distance < 0.25:
                    self.path_length += distance
                self._path_previous_xy = current
            self.odom = message
            self._condition.notify_all()

    def _scan_cb(self, message: LaserScan) -> None:
        with self._condition:
            self.scan = message
            self._condition.notify_all()

    def _global_plan_cb(self, message: Path) -> None:
        if message.poses:
            with self._condition:
                self.global_plan = message
                self._condition.notify_all()

    def _local_plan_cb(self, message: Path) -> None:
        if message.poses:
            with self._condition:
                self.local_plan = message
                self.local_plan_generation += 1
                self._condition.notify_all()

    def _place(self, name: str, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        state = ModelState(model_name=name, reference_frame="world")
        state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, z
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        response = self.set_model_state(state)
        if not response.success:
            raise RuntimeError("cannot place {}: {}".format(name, response.status_message))

    def _scenario_random(self, scenario: Mapping[str, Any], seed: Optional[int]) -> random.Random:
        explicit = scenario.get("evaluation_seed", scenario.get("seed", seed))
        if explicit is None:
            self._episode_seed += 1
            explicit = self._episode_seed
        stable = sum((index + 1) * ord(char)
                     for index, char in enumerate(str(scenario.get("scene_id", "scene"))))
        return random.Random(int(explicit) * 1000003 + stable)

    def _layout(
        self, name: str, scenario: Mapping[str, Any], generator: random.Random
    ) -> None:
        self._place("front_box", 100.0, 0.0, 0.5)
        self._place("left_wall", 100.0, 10.0, 0.5)
        self._place("right_wall", 100.0, -10.0, 0.5)
        if name == "obstacle":
            obstacle = scenario.get("obstacle", (2.5, 0.0))
            jitter = float(scenario.get("obstacle_jitter_m", 0.0))
            self._place(
                "front_box", float(obstacle[0]) + generator.uniform(-jitter, jitter),
                float(obstacle[1]) + generator.uniform(-jitter, jitter), 0.5,
            )
        elif name == "corridor":
            half_width = float(scenario.get("corridor_half_width_m", 1.0))
            jitter = float(scenario.get("corridor_half_width_jitter_m", 0.0))
            half_width += generator.uniform(-jitter, jitter)
            self._place("left_wall", 3.0, half_width, 0.5)
            self._place("right_wall", 3.0, -half_width, 0.5)
        elif name != "clear":
            raise ValueError("unknown layout {}".format(name))

    def _reset_scene_and_dispatch(self, seed: Optional[int]) -> None:
        self._scenario_index = (self._scenario_index + 1) % len(self.scenarios)
        self.current_scenario = self.scenarios[self._scenario_index]
        generator = self._scenario_random(self.current_scenario, seed)
        self.safety_mode = "NORMAL"
        self.fallback_active = False
        self.emergency_active = False
        self.brake_pub.publish(Bool(data=False))
        self._layout(str(self.current_scenario.get("layout", "clear")),
                     self.current_scenario, generator)
        start = self.current_scenario.get("start", (0.0, 0.0, 0.0))
        self._place("autolabor_m2", float(start[0]), float(start[1]), 0.02, float(start[2]))
        for _ in range(3):
            self.reset_odom_pub.publish(Empty())
            rospy.sleep(0.05)
        self.clear_costmaps()
        rospy.sleep(0.5)
        with self._condition:
            self.path_length = 0.0
            self.minimum_clearance = float("inf")
            self.global_plan = None
            self.local_plan = None
            self._path_previous_xy = (
                self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
            )
            self._previous_velocity = float(self.odom.twist.twist.linear.x)
            self._previous_sample_stamp = rospy.Time.now().to_sec()
            self._last_read_odom_stamp = -1.0
            self.planner_cycle_count = 0
            self._feedback_last_generation = self.local_plan_generation
        goal_values = self.current_scenario["goal"]
        goal_jitter = float(self.current_scenario.get("goal_jitter_m", 0.0))
        self.goal_xy = (
            float(goal_values[0]) + generator.uniform(-goal_jitter, goal_jitter),
            float(goal_values[1]) + generator.uniform(-goal_jitter, goal_jitter),
        )
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.goal_xy[0]
        goal.target_pose.pose.position.y = self.goal_xy[1]
        yaw = float(goal_values[2])
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        # A goal can occasionally arrive while move_base is still processing the
        # preceding cancel/costmap reset.  Treat that as a transient reset fault,
        # not as an experiment episode, and re-dispatch the identical goal.  The
        # scene and seed are deliberately left unchanged across attempts.
        plan_attempts = int(self.current_scenario.get("reset_plan_attempts", 3))
        plan_timeout_s = float(
            self.current_scenario.get("reset_plan_timeout_s", 10.0)
        )
        for attempt in range(1, plan_attempts + 1):
            with self._condition:
                self.local_plan = None
            self.move_base.send_goal(goal)
            deadline = time.monotonic() + plan_timeout_s
            with self._condition:
                while self.local_plan is None and time.monotonic() < deadline:
                    self._condition.wait(0.05)
                if self.local_plan is not None:
                    break
            if attempt < plan_attempts:
                rospy.logwarn(
                    "local plan unavailable after reset attempt %d/%d; "
                    "retrying identical scene and goal",
                    attempt, plan_attempts,
                )
                self.move_base.cancel_all_goals()
                self.cmd_pub.publish(Twist())
                self.clear_costmaps()
                rospy.sleep(0.5)
        if self.local_plan is None:
            raise RuntimeError(
                "local plan unavailable after {} reset attempts".format(plan_attempts)
            )

    def reset(self, seed: Optional[int]) -> None:
        self._quiesce_navigation()
        self._reset_scene_and_dispatch(seed)

    def reset_with_parameter_snapshot(
        self, snapshot: Mapping[str, float], seed: Optional[int]
    ) -> None:
        """Execute the frozen episode-boundary order without an active goal."""

        self._quiesce_navigation()
        self._apply_parameter_snapshot(snapshot)
        self._reset_scene_and_dispatch(seed)

    def capture_parameter_snapshot(self) -> Dict[str, float]:
        self._episode_snapshot = dict(self._current_theta)
        return dict(self._episode_snapshot)

    def _quiesce_navigation(self) -> None:
        """Confirm the previous goal and local planner are quiet before a write.

        TEB consumes its configuration from the move_base planning thread.  A
        snapshot restore while that thread is still optimizing can race the
        dynamic-reconfigure callback.  A terminal action state alone is not
        sufficient because the final local-plan callbacks can arrive after the
        action transition, so both conditions are required for a quiet period.
        """

        if not self.move_base.wait_for_server(rospy.Duration(0.5)):
            self._boundary_quiesce_failure_count = int(getattr(
                self, "_boundary_quiesce_failure_count", 0)) + 1
            raise RuntimeError("move_base action server unavailable at episode boundary")
        self.move_base.cancel_all_goals()
        self.cmd_pub.publish(Twist())
        terminal_states = {
            GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
            GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST,
        }
        timeout_s = float(getattr(self, "_boundary_quiesce_timeout_s", 5.0))
        quiet_period_s = float(getattr(self, "_boundary_quiet_period_s", 0.25))
        if bool(getattr(self, "_require_recovery_barrier", False)):
            quiet_period_s = max(quiet_period_s, float(getattr(
                self, "_boundary_recovery_quiet_period_s", 1.0)))
        deadline = time.monotonic() + timeout_s
        last_generation = self.local_plan_generation
        quiet_since = time.monotonic()
        while True:
            now = time.monotonic()
            generation = self.local_plan_generation
            if generation != last_generation:
                last_generation = generation
                quiet_since = now
            state = self.move_base.get_state()
            if state in terminal_states and now - quiet_since >= quiet_period_s:
                self._boundary_quiesce_count = int(getattr(
                    self, "_boundary_quiesce_count", 0)) + 1
                self._boundary_last_terminal_state = int(state)
                self._boundary_last_quiet_period_s = quiet_period_s
                self._require_recovery_barrier = False
                return
            if now >= deadline:
                self._boundary_quiesce_failure_count = int(getattr(
                    self, "_boundary_quiesce_failure_count", 0)) + 1
                raise RuntimeError(
                    "move_base did not quiesce before episode-boundary "
                    "parameter restore (state={}, local_plan_generation={})".format(
                        state, generation)
                )
            self.cmd_pub.publish(Twist())
            with self._condition:
                self._condition.wait(min(0.05, max(0.0, deadline - now)))

    def _apply_parameter_snapshot(self, snapshot: Mapping[str, float]) -> None:
        if self.parameter_client is None:
            if snapshot:
                raise RuntimeError("fixed planner cannot restore TEB parameters")
            return
        record = self.parameter_client.apply(snapshot)
        self._current_theta = dict(record["readback"])

    def restore_parameter_snapshot(self, snapshot: Mapping[str, float]) -> None:
        self._quiesce_navigation()
        self._apply_parameter_snapshot(snapshot)

    def boundary_audit(self) -> Dict[str, Any]:
        return {
            "protocol": "cancel_terminal_and_plan_quiet_restore_reset_dispatch_v1",
            "quiet_period_s": float(self._boundary_quiet_period_s),
            "recovery_quiet_period_s": float(self._boundary_recovery_quiet_period_s),
            "timeout_s": float(self._boundary_quiesce_timeout_s),
            "quiesce_count": int(self._boundary_quiesce_count),
            "quiesce_failure_count": int(self._boundary_quiesce_failure_count),
            "last_terminal_state": self._boundary_last_terminal_state,
            "last_quiet_period_s": float(self._boundary_last_quiet_period_s),
            "activation_timeout_barrier_count": int(
                self._activation_timeout_barrier_count),
            "activation_timeout_last_config_seq": self._activation_timeout_last_config_seq,
            "recovery_barrier_pending": bool(self._require_recovery_barrier),
        }

    def mark_activation_timeout(self, config_seq: int) -> None:
        self._activation_timeout_barrier_count += 1
        self._activation_timeout_last_config_seq = int(config_seq)
        self._require_recovery_barrier = True

    def current_theta(self) -> Dict[str, float]:
        return dict(self._current_theta)

    def terminal_reason(self) -> str:
        """Return only action-server-confirmed terminal navigation outcomes."""
        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            return "goal"
        if state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST):
            return "planner_failure"
        return ""

    def write_parameters(
        self, theta: Mapping[str, float], config_seq: int
    ) -> ParameterWriteReceipt:
        del config_seq  # TebParameterClient maintains and audits its own sequence too.
        if self.parameter_client is None:
            raise RuntimeError("fixed planner does not expose the TEB parameter interface")
        self._last_written_previous = dict(self._current_theta)
        # Some Gazebo sensors stamp at the end of their update interval, a few
        # milliseconds ahead of /clock observed in this callback. Preserve the
        # causal observation <= request <= ack ordering used for attribution.
        t_request = max(rospy.Time.now().to_sec(), self._last_observation_time)
        if all(abs(float(theta[name]) - float(self._current_theta[name])) <= 1e-12
               for name in EXPECTED_THETA_ORDER):
            self._last_written = dict(self._current_theta)
            return ParameterWriteReceipt(t_request, t_request, dict(self._current_theta))
        record = self.parameter_client.apply(theta)
        t_ack = max(rospy.Time.now().to_sec(), t_request)
        self._current_theta = dict(record["readback"])
        self._last_written = dict(self._current_theta)
        return ParameterWriteReceipt(t_request, t_ack, dict(self._current_theta))

    def poll_activation(
        self, config_seq: int, t_ack: float, timeout_s: float
    ) -> ActivationPoll:
        del config_seq
        plans = []
        generation = self.local_plan_generation
        wall_deadline = time.monotonic() + max(2.0 * timeout_s, 2.0)
        while time.monotonic() < wall_deadline:
            with self._condition:
                if self.local_plan_generation > generation:
                    generation = self.local_plan_generation
                    stamp = _stamp(self.local_plan)
                    plans.append((stamp, bool(self.local_plan.poses)))
                    if stamp > t_ack:
                        return ActivationPoll(tuple(plans), rospy.Time.now().to_sec())
                self._condition.wait(0.01)
            if rospy.Time.now().to_sec() - t_ack >= timeout_s:
                break
        now = max(rospy.Time.now().to_sec(), t_ack + timeout_s)
        return ActivationPoll(tuple(plans), now)

    def _footprint_clearance(self) -> float:
        if self.scan is None:
            return float("inf")
        best = float("inf")
        angle = self.scan.angle_min
        for distance in self.scan.ranges:
            if math.isfinite(distance) and self.scan.range_min <= distance <= self.scan.range_max:
                cosine, sine = abs(math.cos(angle)), abs(math.sin(angle))
                x_exit = float("inf") if cosine < 1e-9 else 0.52 / cosine
                y_exit = float("inf") if sine < 1e-9 else 0.35 / sine
                best = min(best, max(0.0, distance - min(x_exit, y_exit)))
            angle += self.scan.angle_increment
        return best if math.isfinite(best) else self.scan.range_max

    def _path_errors(self) -> Tuple[float, float]:
        if self.odom is None or self.global_plan is None or not self.global_plan.poses:
            return 0.0, 0.0
        x, y = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
        nearest = min(
            self.global_plan.poses,
            key=lambda item: ((item.pose.position.x - x) ** 2 +
                              (item.pose.position.y - y) ** 2),
        )
        cross = math.hypot(nearest.pose.position.x - x, nearest.pose.position.y - y)
        heading = abs(_angle_delta(_yaw(self.odom.pose.pose.orientation),
                                   _yaw(nearest.pose.orientation)))
        return cross, heading

    def _metrics(self) -> Dict[str, float]:
        now = rospy.Time.now().to_sec()
        x, y = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
        yaw = _yaw(self.odom.pose.pose.orientation)
        dx, dy = self.goal_xy[0] - x, self.goal_xy[1] - y
        distance = math.hypot(dx, dy)
        bearing = _angle_delta(math.atan2(dy, dx), yaw)
        velocity = float(self.odom.twist.twist.linear.x)
        angular = float(self.odom.twist.twist.angular.z)
        dt = max(now - (self._previous_sample_stamp or now), 1e-6)
        acceleration = (velocity - self._previous_velocity) / dt
        clearance = self._footprint_clearance()
        cross, heading = self._path_errors()
        rays = [value for value in self.scan.ranges if math.isfinite(value)]
        density = sum(value < 2.0 for value in rays) / float(len(rays)) if rays else 0.0
        ttc = clearance / max(abs(velocity), 0.05)
        self._previous_velocity, self._previous_sample_stamp = velocity, now
        self.minimum_clearance = min(self.minimum_clearance, clearance)
        self.last_metrics = {
            "stamp": now, "goal_distance": distance, "goal_bearing": bearing,
            "linear_velocity": velocity, "angular_velocity": angular,
            "linear_acceleration": acceleration, "clearance": clearance,
            "path_error": cross, "path_heading_error": heading,
            "obstacle_density": density, "ttc": ttc,
        }
        return dict(self.last_metrics)

    def read_observation(self) -> ObservationInput:
        deadline = time.monotonic() + self.observation_timeout_s
        with self._condition:
            while time.monotonic() < deadline:
                ready = self.odom is not None and self.scan is not None and self.local_plan is not None
                odom_stamp = _stamp(self.odom) if self.odom is not None else -1.0
                observation_stamp = (
                    max(_stamp(self.odom), _stamp(self.scan), _stamp(self.local_plan))
                    if ready else -1.0
                )
                # StateBuilder enforces strictly increasing synchronized frame
                # times.  A fresh odometry message alone is insufficient when
                # another stream still owns the maximum timestamp, so wait until
                # the actual synchronized observation time advances as well.
                if (ready and odom_stamp > self._last_read_odom_stamp
                        and observation_stamp > self._last_observation_time):
                    self._last_read_odom_stamp = odom_stamp
                    break
                self._condition.wait(0.01)
            else:
                raise RuntimeError("timed out waiting for a fresh synchronized observation")
        metrics = self._metrics()
        features = {
            "footprint_clearance": metrics["clearance"],
            "obstacle_density": metrics["obstacle_density"],
            "approximate_ttc": metrics["ttc"],
            "goal_distance": metrics["goal_distance"],
            "goal_bearing_sin": math.sin(metrics["goal_bearing"]),
            "goal_bearing_cos": math.cos(metrics["goal_bearing"]),
            "path_cross_track_error": metrics["path_error"],
            "path_heading_error": metrics["path_heading_error"],
            "linear_velocity": metrics["linear_velocity"],
            "angular_velocity": metrics["angular_velocity"],
            "linear_acceleration": metrics["linear_acceleration"],
            "planner_valid": 1.0, "sensor_valid": 1.0, "tf_valid": 1.0,
            "localization_valid": 1.0, "interface_valid": 1.0,
        }
        features.update({"theta_{}".format(name): self._current_theta.get(name, 0.0)
                         for name in EXPECTED_THETA_ORDER})
        stamps = {"scan": _stamp(self.scan), "odom": _stamp(self.odom),
                  "local_plan": _stamp(self.local_plan)}
        self._last_observation_time = max(stamps.values())
        return ObservationInput(
            stamps=stamps,
            ranges=tuple(self.scan.ranges), range_min=self.scan.range_min,
            range_max=self.scan.range_max, features=features,
            validity={"scan": True, "tf": True, "localization": True,
                      "interface": True, "planner": True},
            scan_metadata=ScanAngularMetadata(
                stamp=stamps["scan"],
                frame_id=self.scan.header.frame_id,
                angle_min=self.scan.angle_min,
                angle_max=self.scan.angle_max,
                angle_increment=self.scan.angle_increment,
                ray_count=len(self.scan.ranges),
            ),
        )

    def collect_feedback(self, t_active: float, t_window_end: float) -> FeedbackWindow:
        samples = []
        terminal_reason = ""
        collision = False
        collision_threshold = float(self.current_scenario.get("collision_threshold_m", 0.10))
        near_threshold = float(self.current_scenario.get("near_collision_threshold_m", 0.30))
        wall_deadline = time.monotonic() + max(3.0, 2.0 * (t_window_end - t_active))
        while rospy.Time.now().to_sec() <= t_window_end and time.monotonic() < wall_deadline:
            metrics = self._metrics()
            near_collision = metrics["clearance"] <= near_threshold
            collision = collision or metrics["clearance"] <= collision_threshold
            samples.append(FeedbackSample(
                stamp=metrics["stamp"], goal_distance=metrics["goal_distance"],
                path_error=metrics["path_error"], clearance=metrics["clearance"],
                linear_acceleration=metrics["linear_acceleration"],
                angular_acceleration=0.0, near_collision=near_collision,
                fallback_active=self.fallback_active,
                emergency_active=self.emergency_active,
            ))
            if self.local_plan_generation > self._feedback_last_generation:
                self.planner_cycle_count += self.local_plan_generation - self._feedback_last_generation
                self._feedback_last_generation = self.local_plan_generation
            if collision and len(samples) >= 2:
                terminal_reason = "collision"
                self.move_base.cancel_all_goals()
                self.cmd_pub.publish(Twist())
                break
            rospy.sleep(0.05)
        if len(samples) < 2:
            raise RuntimeError("reward window did not contain two feedback samples")
        state = self.move_base.get_state()
        # A goal can complete just after the fixed reward window.  Wait for the
        # action result without extending reward attribution, so the next RL
        # step never writes parameters after move_base has stopped planning.
        close_to_goal = bool(samples and samples[-1].goal_distance <= 0.60)
        if state in (GoalStatus.PENDING, GoalStatus.ACTIVE) and close_to_goal:
            # move_base can log GOAL Reached before its action result becomes
            # observable. Only near the goal, allow enough time for that result;
            # reward attribution still ends at t_window_end.
            grace_end = rospy.Time.now().to_sec() + 3.0
            wall_grace_end = time.monotonic() + 3.5
            while (rospy.Time.now().to_sec() < grace_end and
                   time.monotonic() < wall_grace_end):
                state = self.move_base.get_state()
                if state not in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                    break
                rospy.sleep(0.02)
        if collision:
            terminal_reason = "collision"
        elif state == GoalStatus.SUCCEEDED:
            terminal_reason = "goal"
        elif state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST):
            terminal_reason = "planner_failure"
        normalized_delta = []
        for name in EXPECTED_THETA_ORDER:
            if name not in self.theta_bounds:
                normalized_delta.append(0.0)
                continue
            low, high = self.theta_bounds[name]
            normalized_delta.append(
                2.0 * (self._last_written[name] - self._last_written_previous[name]) /
                float(high - low)
            )
        return FeedbackWindow(
            samples=tuple(samples), events=WindowEvents(
                collision=collision, goal=terminal_reason == "goal"
            ),
            theta_delta_normalized=tuple(normalized_delta),
            terminal_reason=terminal_reason,
        )

    def set_safety_state(
        self, mode: str, fallback_active: bool, emergency_active: bool
    ) -> None:
        self.safety_mode = str(mode)
        self.fallback_active = bool(fallback_active)
        self.emergency_active = bool(emergency_active)

    def request_stop(self, reason: str) -> None:
        rospy.logwarn("training environment stop requested: %s", reason)
        self.move_base.cancel_all_goals()
        self.cmd_pub.publish(Twist())

    def close(self) -> None:
        self.request_stop("adapter_close")
        if self.parameter_client is None:
            return
        # Process-exit restoration follows the same quiet-before-write rule as
        # an episode boundary. TebParameterClient.close performs the one and
        # only startup-snapshot restore; do not issue a duplicate write first.
        self._quiesce_navigation()
        self.parameter_client.close()


class TrainingSafetyAdapter:
    """Bridge T05 safety/fallback primitives to TrainingEnvironment."""

    def __init__(self, safety_filter: Any, fallback_policy: Any,
                 directional_emergency: bool = False,
                 corridor_warning_theta: Optional[Mapping[str, float]] = None,
                 corridor_max_delta: Optional[Mapping[str, float]] = None) -> None:
        self.safety_filter = safety_filter
        self.fallback_policy = fallback_policy
        self.directional_emergency = bool(directional_emergency)
        self.corridor_warning_theta = (
            None if corridor_warning_theta is None else dict(corridor_warning_theta))
        self.corridor_max_delta = (
            None if corridor_max_delta is None else dict(corridor_max_delta))
        self._corridor_active = False
        self.last_decision = None
        self.last_fallback = None

    def reset(self, seed: Optional[int] = None) -> None:
        del seed
        if hasattr(self.safety_filter, "reset"):
            self.safety_filter.reset()
        self.last_decision = None
        self.last_fallback = None
        self._corridor_active = False

    def filter(
        self, projected: Mapping[str, float], current: Mapping[str, float],
        frame: Any, now: float,
    ) -> SafeParameterDecision:
        features = frame.named_features
        health = {name: True for name in
                  ("sensor", "tf", "localization", "parameter_interface", "planner")}
        emergency_distance = None
        if self.directional_emergency:
            sector_count = len(frame.vector) - len(frame.named_features)
            if frame.scan_metadata is not None:
                indices = frame.scan_metadata.sector_indices(
                    sector_count, 0.0, math.radians(20.0))
                forward_ranges = tuple(frame.vector[index] for index in indices)
            else:
                # V1 compatibility only. GazeboTrainingAdapter now always
                # supplies validated angular metadata; V2 state contracts reject
                # its absence instead of assuming a 360-degree index layout.
                center = sector_count // 2
                half_width = max(1, int(math.ceil(sector_count * 20.0 / 360.0)))
                forward_ranges = frame.vector[
                    max(0, center - half_width):min(
                        sector_count, center + half_width + 1)
                ]
            emergency_distance = (
                float("nan") if not forward_ranges else
                max(0.0, min(forward_ranges) - 0.52)
            )
        decision = self.safety_filter.update(
            features["footprint_clearance"], abs(features["linear_velocity"]), now, health,
            emergency_obstacle_distance=emergency_distance,
        )
        self.fallback_policy.confirm_applied_safe(current)
        fallback = self.fallback_policy.decide(decision.mode, projected, True)
        fallback_theta = dict(fallback.theta)
        corridor_detected = (
            self.directional_emergency and decision.mode.value == "WARNING" and
            emergency_distance is not None and emergency_distance > 0.60 and
            features["footprint_clearance"] <= 0.45
        )
        self._corridor_active = self._corridor_active or corridor_detected
        corridor_warning = (
            self._corridor_active and decision.mode.value == "WARNING" and
            self.corridor_warning_theta is not None
        )
        if corridor_warning:
            # A side-wall corridor needs low speed but a feasible obstacle
            # distance. Applying the generic monotonic "more clearance" rule
            # can make the TEB optimization infeasible and induce oscillation.
            for name, target in self.corridor_warning_theta.items():
                limit = float(self.corridor_max_delta[name])
                delta = max(-limit, min(limit, float(target) - float(current[name])))
                fallback_theta[name] = float(current[name]) + delta
            fallback_theta["max_vel_theta"] = min(
                fallback_theta["max_vel_theta"],
                fallback_theta["max_vel_x"] / 1.2,
            )
        self.last_decision = decision
        self.last_fallback = fallback
        return SafeParameterDecision(
            theta=fallback_theta,
            request_stop=fallback.request_stop,
            reasons=tuple(decision.reasons) + tuple(fallback.reasons) +
                    (("warning:corridor_feasibility_profile",) if corridor_warning else ()),
        )
