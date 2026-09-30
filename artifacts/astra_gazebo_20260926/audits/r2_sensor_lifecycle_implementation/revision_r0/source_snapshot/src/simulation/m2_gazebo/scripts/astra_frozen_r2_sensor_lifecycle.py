#!/usr/bin/env python3
"""Independent sensor-only R2 lifecycle; original inference/control body inherited.

The fixed shared path is received and audited, NOT consumed by this algorithm.
No simulator services, truth subscriptions, physical safety or primary claim.
"""
import hashlib
import importlib.util
import inspect
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlparse

WORKSPACE = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(WORKSPACE/'scripts/astra_gazebo'))
from frozen_r2_sensor_lifecycle import (METHOD, FROZEN_CHECKPOINT_SHA256, MAX_AGE_NS, TERMINALS,
    LifecycleFault, SharedTerminal, SensorIngress, SharedLifecycle,
    StreamingTrace, authenticate_frozen, quaternion, finite,
    verify_static_union, json_diagnostic)


def load_frozen(workspace):
    bindings = authenticate_frozen(workspace)
    path = Path(workspace)/'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py'
    spec = importlib.util.spec_from_file_location('astra_sensor_lifecycle_frozen_r2', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    authenticate_frozen(workspace)
    for name, relative in (
        ('v5_e2e_scope_rl.benchmark_runtime', 'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py'),
        ('v5_e2e_scope_rl.runtime_adapter', 'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py')):
        if Path(sys.modules[name].__file__).resolve() != (Path(workspace)/relative).resolve():
            raise LifecycleFault('imported frozen runtime came from another workspace')
    for source in (inspect.getfile(module.r1.SensorActor), module.r1.execution_core.__file__):
        source = Path(source).resolve()
        if not str(source).startswith(str(Path(workspace).resolve()/'src/application')+'/'):
            raise LifecycleFault('actor/execution dependency came from another workspace')
        bindings.append(dict(path=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    return module, bindings


class CheckedTF:
    """Validate TF evidence without altering the inherited scan conversion."""
    def __init__(self, actual, rospy):
        self.actual, self.rospy = actual, rospy
    def lookup_transform(self, target, source, stamp, timeout):
        transform = self.actual.lookup_transform(target, source, stamp, timeout)
        actual_ns = transform.header.stamp.to_nsec()
        now = self.rospy.Time.now().to_nsec()
        requested_ns = stamp.to_nsec()
        if transform.header.frame_id.lstrip('/') != target or transform.child_frame_id.lstrip('/') != source:
            raise LifecycleFault('TF frame/reference mismatch')
        if actual_ns <= 0 or actual_ns > now or now-actual_ns > MAX_AGE_NS:
            raise LifecycleFault('TF source age/lead invalid')
        if requested_ns and actual_ns != requested_ns:
            raise LifecycleFault('TF lookup changed requested source stamp')
        t = transform.transform.translation
        finite((t.x, t.y, t.z)); quaternion(transform.transform.rotation)
        return transform


def make_episode_class(r2, publish, lifecycle, ingress, lock, inputs_ready):
    # Symbols below are the frozen R1 implementations, including the checkpoint
    # loader. The constructor's actor/runtime block is compared by AST in tests.
    r1 = r2.r1
    rospy, tf2_ros, torch, yaml = r1.rospy, r1.tf2_ros, r1.torch, r1.yaml
    SensorActor = r1.SensorActor
    DirectionalBenchmarkRuntimeAdapter = r1.DirectionalBenchmarkRuntimeAdapter
    sha256, METHODS = r1.sha256, r1.METHODS
    class SensorLifecycleR2(r2.R2BenchmarkEpisode):
        def __init__(self):
            self.stage = str(rospy.get_param("~stage", "PCT-BENCH-DEVELOPMENT"))
            self.split = str(rospy.get_param("~split", "development"))
            if self.split not in ("development", "pilot", "formal_round_1", "formal_round_2"):
                raise RuntimeError("unknown benchmark split: " + self.split)
            self.method = rospy.get_param("~method")
            if self.method not in METHODS:
                raise RuntimeError("unknown benchmark method: " + str(self.method))
            self.scene_id = rospy.get_param("~scene_id")
            self.seed = int(rospy.get_param("~seed"))
            self.report_path = rospy.get_param("~report_path")
            self.scene_manifest_path = rospy.get_param("~scene_manifest")
            self.actor_checkpoint_path = rospy.get_param("~actor_checkpoint_path", "")
            self.spawn_world_x = float(rospy.get_param("~spawn_world_x", -5.4))
            self.policy_start_world_x = float(rospy.get_param("~policy_start_world_x", -4.0))
            self.goal_world_x = float(rospy.get_param("~goal_world_x", 5.0))
            self.goal_tolerance_m = float(rospy.get_param("~goal_tolerance_m", 0.25))
            self.timeout_s = float(rospy.get_param("~timeout_s", 22.0))
            self.activation_time_s = float(rospy.get_param("~activation_time_s", 5.0))
            self.warmup_speed_mps = float(rospy.get_param("~warmup_speed_mps", 0.8))
            self.warmup_gate_tolerance_m = float(rospy.get_param("~warmup_gate_tolerance_m", 0.030))
            self.lookahead_m = float(rospy.get_param("~lookahead_m", 0.70))
            self.combined_radius_m = float(rospy.get_param("~combined_radius_m", 1.15))
            with open(self.scene_manifest_path, "r", encoding="utf-8") as stream:
                self.scene = yaml.safe_load(stream)
            if self.scene.get("scene_id") != self.scene_id or int(self.scene.get("seed")) != self.seed:
                raise RuntimeError("scene manifest identity mismatch")
            self.scene = {k: self.scene[k] for k in ("scene_id", "seed", "corridor_half_width_m", "static_circles", "static_boxes", "r2_static_input_adapter") if k in self.scene}
            self.ego_model = "autolabor_m2"
    
            actor = None
            self.actor_checkpoint_sha256 = None
            if self.method in ("pct_scope_rl", "pct_shadow"):
                if not self.actor_checkpoint_path:
                    raise RuntimeError("PCT actor checkpoint absent")
                checkpoint = torch.load(
                    self.actor_checkpoint_path, map_location="cpu", weights_only=False
                )
                actor = SensorActor.from_actor_checkpoint(checkpoint)
                actor.eval()
                self.actor_checkpoint_sha256 = sha256(self.actor_checkpoint_path)
            torch.set_num_threads(2)
            wall_half_width = self.scene.get("corridor_half_width_m")
            if wall_half_width is None:
                wall_half_width = 20.0
            self.runtime = DirectionalBenchmarkRuntimeAdapter(
                actor=actor,
                start_x_m=self.policy_start_world_x,
                goal_x_m=self.goal_world_x,
                goal_tolerance_m=self.goal_tolerance_m,
                wall_half_width_m=float(wall_half_width),
                static_circles=self.scene.get("static_circles", []),
                static_boxes=self.scene.get("static_boxes", []),
                shield_inflation_m=0.33,
                inference_deadline_ms=20.0,
                maximum_input_age_s=0.20,
                goal_speed_envelope=True,
                safety_response_delay_s=0.35,
                certified_stop_dwell_s=1.50,
            )
    
            self.odom = None
            self.odom_received_s = None
            self.scan = None
            self.scan_received_s = None
            self.last_runtime_scan_stamp_s = None
            self.contact_state_count = None
            self.command_count = 0
            self.command_stamps = []
            self.scan_stamps = []
            self.odom_stamps = []
            self.local_plan_count = 0
            self.minimum_scan_m = float("inf")
            self.collecting = False
            self.tf_success_count = 0
            self.tf_failure_count = 0
            self.maximum_scan_age_s = 0.0
            self.maximum_odom_age_s = 0.0
            self.maximum_tf_age_s = 0.0
            self.decision_trace = []
            self.control_trace = []
            self.scan_transform_success_count = 0
            self.scan_transform_failure_count = 0
            self.raw_tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
            self.tf_buffer = CheckedTF(self.raw_tf_buffer, rospy)
            self.tf_listener = tf2_ros.TransformListener(self.raw_tf_buffer)
    
            self.candidate_variant = str(rospy.get_param("~candidate_variant", "r2_candidate_v060"))
            if self.method != 'pct_scope_rl' or self.candidate_variant != 'r2_candidate_v060':
                raise LifecycleFault('exact frozen learned R2 variant required')
            self.candidate_target_speeds_mps = r2.apply_candidate_variant(self.runtime, self.candidate_variant)
            if (self.spawn_world_x, self.policy_start_world_x, self.goal_world_x, self.timeout_s, self.lookahead_m,
                self.goal_tolerance_m, self.activation_time_s, self.warmup_speed_mps, self.warmup_gate_tolerance_m) != (-5.4, -4., 5., 22., .70, .25, 5., .8, .03):
                raise LifecycleFault('unsupported frozen R2 frame/protocol')
            expected = rospy.get_param('~actor_checkpoint_sha256')
            if expected != FROZEN_CHECKPOINT_SHA256 or self.actor_checkpoint_sha256 != expected:
                raise LifecycleFault('frozen actor checkpoint SHA mismatch')
            self.command_pub = rospy.Publisher('/astra/method_cmd', r1.Twist, queue_size=1000)
            self.decision_trace = StreamingTrace('decision', lambda row: publish('trace', row))
            self.control_trace = StreamingTrace('control', lambda row: publish('trace', row))
            self.metric_odom_ns = None
            self.command_sequence = 0
            self.command_generation_protocol = 'publication_ros_time_and_gate_identity_not_driver_acceptance'
            self.subscribers = [rospy.Subscriber('/odom', r1.Odometry, self._odom, queue_size=100),
                                rospy.Subscriber('/scan', r1.LaserScan, self._scan, queue_size=100)]

        def _ingress(self, kind, message):
            with lock:
                try:
                    # Terminal evidence remains immutable after collector pause.
                    if lifecycle.terminal is not None:
                        return
                    ingress.enqueue(kind, message, rospy.Time.now().to_nsec(), lifecycle.active)
                except Exception as exc:
                    lifecycle.fault(str(exc)); self.emit_zero(); self.status('fault', False)

        def _odom(self, message):
            self._ingress('odom', message)

        def _scan(self, message):
            self._ingress('scan', message)

        def accept_scan(self, message):
            try:
                self.tf_buffer.lookup_transform('odom', 'laser_link', message.header.stamp, rospy.Duration(0))
            except Exception:
                return False
            previous = self.last_runtime_scan_stamp_s
            # Exact parent resampling, world transform, float .20s history rule.
            r1.BenchmarkEpisode._scan(self, message)
            if self.last_runtime_scan_stamp_s != previous:
                ingress.appended(message.header.stamp.to_nsec())
            return True

        def drain(self):
            with lock:
                lifecycle.check()
                if lifecycle.terminal is not None:
                    return
                now = rospy.Time.now().to_nsec()
                ingress.drain('odom', now, lambda msg: r1.BenchmarkEpisode._odom(self, msg))
                ingress.drain('scan', now, self.accept_scan)

        def status(self, state, ready=False):
            if lifecycle.terminal is not None:
                state, ready = lifecycle.terminal['phase'], False
            publish('status', dict(run_id=lifecycle.run_id, seed=lifecycle.seed, method=METHOD,
                state=state, ready=ready, reason=lifecycle.fault_reason,
                gate_source_ns=None if lifecycle.gate is None else lifecycle.gate['source_ns'],
                accepted_runtime_scan_ns=list(ingress.accepted_scan_ns),
                route_use='audit_only', full_polyline_consumed=False, late_faults=list(lifecycle.late_faults)))

        def ready(self):
            self.drain()
            now = rospy.Time.now().to_nsec()
            health = self.input_health(now/1e9)
            return bool(ingress.first_positive_ns is not None and now-ingress.first_positive_ns >= 20_000_000
                and ingress.history_ready() and lifecycle.coordinator_seen and inputs_ready()
                and not health.failures(.20))

        def wait_ready(self):
            deadline = time.monotonic()+90
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                with lock:
                    if self.ready():
                        self.status('ready', True); return
                    self.status('bootstrap')
                time.sleep(.002)
            raise LifecycleFault('six real sensor histories/common inputs not ready')

        def warmup(self):
            deadline = time.monotonic()+180
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                with lock:
                    ready = self.ready()
                    if lifecycle.terminal is not None:
                        raise SharedTerminal(lifecycle.terminal)
                    if lifecycle.gate is not None and rospy.Time.now().to_nsec() >= lifecycle.gate['source_ns'] and ready:
                        lifecycle.active = True
                        self.status('active', True)
                        return dict(shared_gate=lifecycle.gate, inherited_elapsed_origin='execute_sensor entry, diagnostic')
                    self.status('ready' if ready else 'bootstrap', ready)
                time.sleep(.002)
            raise LifecycleFault('shared gate not reached')

        def robot_world_pose(self):
            if self.odom is None:
                raise LifecycleFault('source odometry unavailable')
            x, y, yaw = self.policy_pose()
            return dict(x_m=x, y_m=y, yaw_rad=yaw)

        def actor_world_poses(self):
            return None

        def conservative_clearance(self, _pose=None):
            return None

        def common_metrics_step(self, previous_pose, minimum_clearance, path_length):
            self.drain()
            pose = self.robot_world_pose()
            stamp = self.odom.header.stamp.to_nsec()
            if stamp != self.metric_odom_ns:
                step = math.hypot(pose['x_m']-previous_pose['x_m'], pose['y_m']-previous_pose['y_m'])
                if step <= .25:
                    path_length += step
                self.metric_odom_ns = stamp
            return pose, minimum_clearance, path_length

        def goal_state(self, pose):
            with lock:
                lifecycle.check()
                position_error = math.hypot(pose['x_m']-self.goal_world_x, pose['y_m'])
                yaw_error = abs(r1.execution_core.angle_delta(0., pose['yaw_rad']))
                if lifecycle.terminal is not None:
                    if lifecycle.terminal['phase'] == 'complete':
                        return True, position_error, yaw_error
                    raise SharedTerminal(lifecycle.terminal)
                return False, position_error, yaw_error

        def emit_zero(self):
            self.command_pub.publish(r1.Twist())

        def publish_command(self, speed=0., yaw_rate=0.):
            with lock:
                lifecycle.check()
                if lifecycle.terminal is not None:
                    # Unwind inherited post-episode ROS sleeps before collector
                    # pause. Trace rows already streamed when appended.
                    self.emit_zero(); raise SharedTerminal(lifecycle.terminal)
                if not lifecycle.active or lifecycle.gate is None:
                    self.emit_zero(); raise LifecycleFault('method command before gate acknowledgement')
                now = rospy.Time.now().to_nsec()
                if now < lifecycle.gate['source_ns']:
                    self.emit_zero(); raise LifecycleFault('command clock precedes gate')
                finite((speed, yaw_rate))
                r1.BenchmarkEpisode.publish_command(self, speed, yaw_rate)
                self.command_sequence += 1
                publish('command_event', dict(run_id=lifecycle.run_id, seed=lifecycle.seed, method=METHOD,
                    sequence=self.command_sequence, publication_source_ns=now,
                    gate_source_ns=lifecycle.gate['source_ns'], v=float(speed), w=float(yaw_rate),
                    time_authority=self.command_generation_protocol))
    return SensorLifecycleR2


def main():
    import rospy
    import numpy as np
    from std_msgs.msg import String
    from nav_msgs.msg import OccupancyGrid, Path as RosPath
    from common_route import load_bundle
    rospy.init_node('astra_frozen_r2_sensor_lifecycle')
    uri = urlparse(os.environ.get('ROS_MASTER_URI', ''))
    if uri.scheme != 'http' or uri.hostname not in ('127.0.0.1', 'localhost', '::1') or uri.username or uri.password:
        raise LifecycleFault('loopback simulation master required')
    if rospy.get_param('/use_sim_time', False) is not True or rospy.get_param('/m2_gazebo/simulation_only', False) is not True:
        raise LifecycleFault('simulation markers required')
    if Path(os.environ['THESIS_WS']).resolve() != WORKSPACE:
        raise LifecycleFault('workspace identity mismatch')
    if rospy.get_param('~execution_method') != METHOD:
        raise LifecycleFault('new lifecycle method identity required')
    lifecycle = SharedLifecycle(rospy.get_param('~run_id'), rospy.get_param('~seed'))
    ingress, lock = SensorIngress(), threading.RLock()
    bundle = Path(rospy.get_param('~bundle')).resolve()
    bundle_sha = hashlib.sha256((bundle/'common_route.json').read_bytes()).hexdigest()
    if bundle_sha != rospy.get_param('~bundle_sha256'):
        raise LifecycleFault('common route bundle SHA mismatch')
    grid, route, route_data = load_bundle(bundle)
    if route_data['map_from_odom_xy_yaw'] != [-5.4, 0., 0.] or route_data['start_xy_yaw'] != [-4., 0., 0.] or route_data['goal_xy_yaw'] != [5., 0., 0.]:
        raise LifecycleFault('frozen R2 legacy frame/endpoints required; full route remains audit only')
    actor_path = Path(rospy.get_param('~actor_checkpoint_path')).resolve()
    if rospy.get_param('~actor_checkpoint_sha256') != FROZEN_CHECKPOINT_SHA256 or hashlib.sha256(actor_path.read_bytes()).hexdigest() != FROZEN_CHECKPOINT_SHA256:
        raise LifecycleFault('checkpoint bytes differ before deserialization')
    r2, frozen_bindings = load_frozen(WORKSPACE)
    publishers = {name: rospy.Publisher(topic, String, queue_size=1000, latch=latch) for name,topic,latch in (
        ('status', '/astra/method/status', True),
        ('identity', '/astra/frozen_r2_sensor_lifecycle/identity', True),
        ('input_receipt', '/astra/frozen_r2_sensor_lifecycle/input_receipt', True),
        ('trace', '/astra/frozen_r2_sensor_lifecycle/trace', False),
        ('command_event', '/astra/frozen_r2_sensor_lifecycle/command_event', False),
        ('report', '/astra/frozen_r2_sensor_lifecycle/report', True))}
    transport = dict(map_ready=False, route_ready=False, last_status=None, worker_result=None,
                     worker_done=False, report_written=False)
    instance = [None]
    def publish(kind, row):
        with lock:
            row = dict(row)
            row.setdefault('run_id', lifecycle.run_id)
            row.setdefault('seed', lifecycle.seed)
            row.setdefault('method', METHOD)
            if kind == 'trace':
                current = instance[0]
                row.update(publication_clock_ns=rospy.Time.now().to_nsec(),
                    source_odom_ns=None if current is None or current.odom is None else current.odom.header.stamp.to_nsec(),
                    source_scan_ns=None if current is None or current.scan is None else current.scan.header.stamp.to_nsec(),
                    accepted_runtime_scan_ns=list(ingress.accepted_scan_ns))
            if kind == 'status':
                transport['last_status'] = dict(row)
            publishers[kind].publish(String(data=json.dumps(json_diagnostic(row), separators=(',', ':'), allow_nan=False)))
    def protect(function):
        def callback(message):
            with lock:
                try:
                    function(message)
                except Exception as exc:
                    lifecycle.fault(type(exc).__name__+':'+str(exc))
                    if instance[0] is not None:
                        instance[0].emit_zero()
                        instance[0].status('fault', False)
        return callback
    def on_gate(message):
        lifecycle.on_gate(json.loads(message.data))
    def on_status(message):
        lifecycle.on_status(json.loads(message.data))
        if lifecycle.terminal is not None and instance[0] is not None:
            instance[0].emit_zero()
    def receipt():
        if transport['map_ready'] and transport['route_ready']:
            publish('input_receipt', dict(run_id=lifecycle.run_id, seed=lifecycle.seed, method=METHOD,
                bundle_sha256=bundle_sha, map_image_sha256=grid.image_sha256,
                normalized_blocking_occupancy_sha256=hashlib.sha256(grid.blocking_occupancy.tobytes()).hexdigest(),
                path_points_sha256=route_data['path_points_sha256'], points_xy=route_data['points_xy'],
                frame_id='map', reference='base_link', map_from_odom_xy_yaw=route_data['map_from_odom_xy_yaw'],
                route_use='audit_only', full_polyline_consumed=False, candidate_route_model='legacy_global_x',
                map_consumer='frozen exact-raster-union static_boxes with legacy implicit +/-20m y walls',
                static_prior_consumer_semantics_equal=False))
    def on_map(message):
        info = message.info
        q = info.origin.orientation
        quaternion(q)
        if (message.header.frame_id != 'map' or info.width != grid.width or info.height != grid.height
            or abs(info.resolution-grid.resolution) > 1e-7
            or max(abs(x-y) for x,y in zip((info.origin.position.x,info.origin.position.y,info.origin.position.z), (*grid.origin[:2],0.))) > 1e-9
            or max(abs(q.x),abs(q.y),abs(q.z),abs(q.w-1)) > 1e-9
            or not np.array_equal(np.asarray(message.data,dtype=np.int64).reshape(grid.height,grid.width), grid.blocking_occupancy)):
            raise LifecycleFault('shared map receipt differs from frozen bundle')
        transport['map_ready'] = True; receipt()
    def on_path(message):
        points = [[p.pose.position.x,p.pose.position.y] for p in message.poses]
        if message.header.frame_id != 'map' or points != route_data['points_xy']:
            raise LifecycleFault('shared route receipt differs from frozen bundle')
        for index,p in enumerate(message.poses):
            yaw = quaternion(p.pose.orientation)
            expected = route.goal_yaw if index == len(message.poses)-1 else route.sample(route.arcs[index])[2]
            if p.header.frame_id != 'map' or abs(p.pose.position.z) > 1e-9 or abs(p.pose.orientation.x)+abs(p.pose.orientation.y) > 1e-9 or abs(math.atan2(math.sin(yaw-expected),math.cos(yaw-expected))) > 1e-9:
                raise LifecycleFault('shared route orientation/reference changed')
        transport['route_ready'] = True; receipt()
    inputs_ready = lambda: transport['map_ready'] and transport['route_ready']
    episode_class = make_episode_class(r2, publish, lifecycle, ingress, lock, inputs_ready)
    episode = episode_class(); instance[0] = episode
    union = verify_static_union(episode.scene, grid)
    if episode.seed != lifecycle.seed:
        raise LifecycleFault('scene/sensor lifecycle seed differs')
    subscriptions = [rospy.Subscriber(topic,kind,protect(callback),queue_size=size) for topic,kind,callback,size in (
        ('/astra/navigation/gate',String,on_gate,10),('/astra/navigation/status',String,on_status,100),
        ('/map',OccupancyGrid,on_map,1),('/astra/common_route/path',RosPath,on_path,1))]
    publish('identity', dict(run_id=lifecycle.run_id,seed=lifecycle.seed,method=METHOD,
        frozen_source_bindings=frozen_bindings,checkpoint_sha256=episode.actor_checkpoint_sha256,
        node_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        helper_sha256=hashlib.sha256((WORKSPACE/'scripts/astra_gazebo/frozen_r2_sensor_lifecycle.py').read_bytes()).hexdigest(),
        algorithm_body='inherited original R1 execute_sensor and original R2 apply_candidate_variant',
        algorithm_sources_unchanged=True, old_full_episode_protocol_unchanged=False,
        scene_manifest_sha256=hashlib.sha256(Path(episode.scene_manifest_path).read_bytes()).hexdigest(),
        static_union=union, route_use='audit_only', full_polyline_consumed=False,
        shared_route_consumer_semantics_equal=False, oracle_input=False,
        scan_history_semantics='six real accepted scans; inherited float >=.20s rule and 72-beam resampling',
        elapsed_feature_origin='inherited execute_sensor entry, formal shared gate remains separate',
        physical_safety_certified=False, primary_matrix_ready=False,
        command_generation_protocol=episode.command_generation_protocol))
    def worker():
        try:
            episode.wait_ready()
            # Check actual resolved private output authority immediately before
            # waiting for shared activation; public authority belongs to coordinator.
            import rosgraph
            pubs = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
            if pubs.get(episode.command_pub.resolved_name) != [rospy.get_name()]:
                raise LifecycleFault('private method output publisher not unique')
            episode.warmup()
            result = episode.execute_sensor()
            with lock:
                transport['worker_result'] = dict(legacy_diagnostic_result=result, shared_terminal_authoritative=True)
        except SharedTerminal as end:
            with lock:
                transport['worker_result'] = dict(shared_terminal=end.row, inherited_tail_unwound=True)
        except Exception as exc:
            with lock:
                lifecycle.fault(type(exc).__name__+':'+str(exc))
                episode.emit_zero()
        finally:
            with lock:
                transport['worker_done'] = True
                episode.collecting = False
                episode.emit_zero()
    thread = threading.Thread(target=worker, name='immutable_r2_execute_sensor', daemon=True)
    thread.start()
    last_clock, last_progress, last_publish, last_zero = None, time.monotonic(), 0., None
    try:
        while not rospy.is_shutdown():
            with lock:
                now, wall = rospy.Time.now().to_nsec(), time.monotonic()
                if lifecycle.terminal is None:
                    try:
                        ingress.clock(now)
                        if last_clock is None or now > last_clock:
                            last_progress = wall
                        if lifecycle.active and wall-last_progress > 5:
                            raise LifecycleFault('active source clock stalled')
                    except Exception as exc:
                        lifecycle.fault(str(exc))
                last_clock = now
                terminal_or_fault = lifecycle.terminal is not None or lifecycle.fault_reason is not None
                if (terminal_or_fault or transport['worker_done']) and now != last_zero:
                    episode.emit_zero(); last_zero = now
                if wall-last_publish >= .2:
                    if lifecycle.fault_reason is not None:
                        episode.status('fault', False)
                    elif lifecycle.terminal is not None:
                        episode.status(lifecycle.terminal['phase'], False)
                    elif transport['last_status'] is not None:
                        publish('status', transport['last_status'])
                    last_publish = wall
                if terminal_or_fault and not transport['report_written']:
                    report = dict(schema='astra_frozen_r2_sensor_lifecycle_report_v1',run_id=lifecycle.run_id,seed=lifecycle.seed,method=METHOD,
                        terminal=lifecycle.terminal,lifecycle_fault=lifecycle.fault_reason,
                        late_faults=list(lifecycle.late_faults),source_odom_ns=None if episode.odom is None else episode.odom.header.stamp.to_nsec(),
                        accepted_runtime_scan_ns=list(ingress.accepted_scan_ns), bootstrap_zero_discarded=ingress.zero_discarded,
                        decision_count=len(episode.decision_trace),control_count=len(episode.control_trace),
                        trace_authority='streamed append-time bag topics; does not depend on inherited run/tail',
                        worker_state_at_report='finished' if transport['worker_done'] else 'may_be_waiting_for_ros_clock_in_inherited_Rate',
                        worker_diagnostic=transport['worker_result'],oracle_input=False,
                        route_use='audit_only',full_polyline_consumed=False,formal_metrics_authority='independent shared collector/evaluator',
                        scientific_inclusion=None,contact_state_count=None,physical_safety_certified=False)
                    publish('report',report)
                    output=Path(episode.report_path)
                    output.parent.mkdir(parents=True,exist_ok=True)
                    with output.open('x') as stream:
                        stream.write(json.dumps(json_diagnostic(report),indent=2,allow_nan=False)+'\n')
                    transport['report_written']=True
            time.sleep(.002)
    finally:
        episode.emit_zero()


if __name__ == '__main__':
    main()
