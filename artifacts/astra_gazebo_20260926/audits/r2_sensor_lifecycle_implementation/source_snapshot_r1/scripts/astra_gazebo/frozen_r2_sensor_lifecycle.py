"""Sensor-only lifecycle for the immutable straight-reference R2 algorithm.

No planner/predictor implementation lives here. Shared route receipt is an
audit of availability; the frozen algorithm does not consume its polyline.
"""
from collections import deque
import hashlib
import json
import math
from pathlib import Path

METHOD = 'frozen_r2_v060_sensor_lifecycle_v1'
FROZEN_CHECKPOINT_SHA256 = 'deaf8b9fce438d304bd45c261ff1fa6315ff04aaaa52a36b5974db2ca1d8c6b8'
PROTOCOL = 'astra_common_navigation_v1'
MAX_AGE_NS = 200_000_000
TERMINALS = ('complete', 'timeout', 'stopped', 'fault')
FROZEN = {
    'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py': 'e38bc474aa7d6f28b6f61e2e9d26394c31dd7f12a8b4de53da71962d089b7d73',
    'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py': '488d84b266795c348abba90a917a5ef017044e74a50443c7265a12eb4b5ed9fe',
    'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py': 'af73770b60d3f3c4d1cc97e2ca4ff848b7ce0f70476a397d7eded5b230af35e9',
    'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py': '3ae49e46e95e1f09055c6d47f01f3fa9ae4c047f3c54cc6e457d2a4a114ef0f7',
}

class LifecycleFault(RuntimeError):
    pass

class SharedTerminal(Exception):
    def __init__(self, row):
        self.row = dict(row)
        super().__init__(str(row['phase']))

def authenticate_frozen(workspace):
    result = []
    for relative, expected in FROZEN.items():
        path = Path(workspace) / relative
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != expected:
            raise LifecycleFault('frozen source changed: ' + relative)
        result.append({'path': str(path.resolve()), 'sha256': actual})
    return result

def finite(values):
    if not all(math.isfinite(float(v)) for v in values):
        raise LifecycleFault('nonfinite sensor geometry')

def quaternion(q):
    values = q.x, q.y, q.z, q.w
    finite(values)
    if abs(sum(v*v for v in values)-1) > 1e-5:
        raise LifecycleFault('nonunit sensor quaternion')
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def validate_odom(message):
    if message.header.frame_id.lstrip('/') != 'odom' or message.child_frame_id.lstrip('/') != 'base_link':
        raise LifecycleFault('odometry frame/reference mismatch')
    p, t = message.pose.pose, message.twist.twist
    finite((p.position.x, p.position.y, p.position.z, t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z))
    quaternion(p.orientation)

def validate_scan(message):
    if message.header.frame_id.lstrip('/') != 'laser_link':
        raise LifecycleFault('laser frame mismatch')
    finite((message.angle_min, message.angle_max, message.angle_increment, message.range_min, message.range_max))
    if len(message.ranges) < 2 or message.angle_increment <= 0 or not 0 <= message.range_min < message.range_max:
        raise LifecycleFault('invalid scan geometry')
    if any(math.isnan(float(v)) or float(v) < 0 for v in message.ranges):
        raise LifecycleFault('invalid scan range')
    # Positive infinity is a real Gazebo no-return measurement; do not invent
    # returns or change the inherited resampling convention.

class SensorIngress:
    """Strict source order, pending future packets, no stamp substitution."""
    def __init__(self):
        self.queues = {'odom': deque(), 'scan': deque()}
        self.seen = {'odom': None, 'scan': None}
        self.last_clock_ns = None
        self.first_positive_ns = None
        self.accepted_scan_ns = deque(maxlen=6)
        self.zero_discarded = {'odom': 0, 'scan': 0}

    def clock(self, now):
        if type(now) is not int or now < 0:
            raise LifecycleFault('invalid source clock')
        if self.last_clock_ns is not None and now < self.last_clock_ns:
            raise LifecycleFault('source clock regressed')
        self.last_clock_ns = now
        if now > 0 and self.first_positive_ns is None:
            self.first_positive_ns = now

    def enqueue(self, kind, message, now, active=False):
        self.clock(now)
        stamp = message.header.stamp.to_nsec()
        if stamp == 0 and not active and self.seen[kind] is None:
            self.zero_discarded[kind] += 1
            return False
        if type(stamp) is not int or stamp <= 0 or (self.seen[kind] is not None and stamp <= self.seen[kind]):
            raise LifecycleFault(kind + ' nonpositive/repeated/regressing source stamp')
        if not -MAX_AGE_NS <= now-stamp <= MAX_AGE_NS:
            raise LifecycleFault(kind + ' source lead/age exceeds 200 ms')
        (validate_odom if kind == 'odom' else validate_scan)(message)
        if len(self.queues[kind]) >= 100:
            raise LifecycleFault(kind + ' ingress queue overflow')
        self.seen[kind] = stamp
        self.queues[kind].append(message)
        return True

    def drain(self, kind, now, accept):
        self.clock(now)
        queue = self.queues[kind]
        while queue:
            message = queue[0]
            stamp = message.header.stamp.to_nsec()
            if stamp > now:
                break
            if now-stamp > MAX_AGE_NS:
                raise LifecycleFault(kind + ' expired pending source/TF')
            if accept(message) is False:
                break
            queue.popleft()

    def appended(self, stamp):
        if self.accepted_scan_ns and stamp <= self.accepted_scan_ns[-1]:
            raise LifecycleFault('runtime scan history did not advance')
        self.accepted_scan_ns.append(stamp)

    def history_ready(self):
        return len(self.accepted_scan_ns) == 6

class SharedLifecycle:
    def __init__(self, run_id, seed):
        if not isinstance(run_id, str) or not run_id or type(seed) is not int or seed <= 0:
            raise LifecycleFault('run identity required')
        self.run_id, self.seed = run_id, seed
        self.gate = None
        self.coordinator_seen = False
        self.terminal = None
        self.fault_reason = None
        self.late_faults = []
        self.active = False

    def identity(self, row):
        if row.get('schema') != PROTOCOL or (row.get('run_id'), row.get('seed'), row.get('method')) != (self.run_id, self.seed, METHOD) or type(row.get('seed')) is not int:
            raise LifecycleFault('shared protocol identity mismatch')

    def on_gate(self, row):
        self.identity(row)
        stamp = row.get('source_ns')
        activation = row.get('activation_source_ns')
        if type(stamp) is not int or stamp <= 0 or type(activation) is not int or not 5_000_000_000 <= activation <= 5_200_000_000 or activation > stamp or type(row.get('deadline_source_ns')) is not int or row.get('deadline_source_ns') != stamp+22_000_000_000:
            raise LifecycleFault('shared gate source/deadline protocol changed')
        if row.get('goal_pose') != [5., 0., 0.]:
            raise LifecycleFault('shared terminal base goal changed')
        pose, twist = row.get('pose_map'), row.get('body_twist')
        if not isinstance(pose, list) or len(pose) != 3 or not isinstance(twist, list) or len(twist) != 3:
            raise LifecycleFault('shared actual gate pose/twist absent')
        finite(pose+twist)
        if abs(pose[0]+4) > .03 or abs(pose[1]) > .03 or abs(pose[2]) > .03 or abs(twist[0]-.8) > .04:
            raise LifecycleFault('shared gate differs from admitted legacy frame')
        if self.gate is not None and row != self.gate:
            raise LifecycleFault('shared gate changed')
        self.gate = dict(row)

    def on_status(self, row):
        self.identity(row)
        self.coordinator_seen = True
        phase = row.get('phase')
        if phase in TERMINALS:
            stamp = row.get('terminal_source_ns')
            if type(stamp) is not int or stamp < 0 or not row.get('terminal_time_authority'):
                raise LifecycleFault('terminal source authority absent')
            if phase in ('complete', 'timeout') and (self.gate is None or stamp < self.gate['source_ns']):
                raise LifecycleFault('terminal precedes gate')
            if phase == 'complete' and stamp >= self.gate['deadline_source_ns']:
                raise LifecycleFault('complete after shared deadline')
            if phase == 'timeout' and stamp != self.gate['deadline_source_ns']:
                raise LifecycleFault('timeout source deadline changed')
            if self.terminal is None:
                self.terminal = dict(row)
            elif phase != self.terminal['phase'] or stamp != self.terminal['terminal_source_ns']:
                self.late_faults.append('coordinator terminal changed after first terminal')
        elif phase not in ('bootstrap', 'warmup', 'activating', 'active'):
            raise LifecycleFault('unknown coordinator phase')

    def fault(self, reason):
        if self.terminal is not None:
            self.late_faults.append(str(reason))
        elif self.fault_reason is None:
            self.fault_reason = str(reason)

    def check(self):
        if self.fault_reason is not None:
            raise LifecycleFault(self.fault_reason)
        if self.terminal is not None and self.terminal['phase'] == 'fault':
            raise LifecycleFault('shared fault:' + str(self.terminal.get('fault')))

def verify_static_union(scene, grid):
    """Audit exact cell-aligned rectangle union; does not alter frozen inputs."""
    import numpy as np
    if scene.get('static_circles', []) or scene.get('corridor_half_width_m') is not None:
        raise LifecycleFault('exact common raster rectangle prior required')
    counts = np.zeros((grid.height, grid.width), dtype=np.int32)
    for box in scene.get('static_boxes', []):
        if set(box) != {'x', 'y', 'length', 'width'}:
            raise LifecycleFault('legacy box has unsupported semantics')
        x, y, length, width = (float(box[k]) for k in ('x', 'y', 'length', 'width'))
        finite((x, y, length, width))
        if length <= 0 or width <= 0:
            raise LifecycleFault('nonpositive static rectangle')
        bounds = [(x-length/2-grid.origin[0])/grid.resolution, (x+length/2-grid.origin[0])/grid.resolution,
                  (y-width/2-grid.origin[1])/grid.resolution, (y+width/2-grid.origin[1])/grid.resolution]
        indexes = list(map(round, bounds))
        if max(abs(a-b) for a,b in zip(bounds,indexes)) > 1e-7:
            raise LifecycleFault('static rectangle is not raster aligned')
        x0,x1,y0,y1 = indexes
        if not 0 <= x0 < x1 <= grid.width or not 0 <= y0 < y1 <= grid.height:
            raise LifecycleFault('static rectangle outside shared map')
        counts[y0:y1,x0:x1] += 1
    if not np.array_equal(counts, (grid.blocking_occupancy != 0).astype(np.int32)):
        raise LifecycleFault('static raster union mismatch or overlap')
    return dict(rectangle_count=len(scene['static_boxes']), blocked_cell_count=int((counts != 0).sum()), cell_union_mismatch=0, overlap_cells=0)

def json_diagnostic(value):
    """Missing/nonfinite diagnostic numbers are null, never fabricated zero."""
    if isinstance(value, dict):
        return {str(k): json_diagnostic(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_diagnostic(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value

class StreamingTrace(list):
    def __init__(self, kind, emit):
        super().__init__()
        self.kind, self.emit = kind, emit
    def append(self, row):
        super().append(row)
        self.emit({'kind': self.kind, 'index': len(self)-1, 'legacy_row': json_diagnostic(row),
                   'actor_truth_available': False, 'clearance_source': 'separate_evaluator_only'})
