#!/usr/bin/env python3
"""M02 input contract extension; frozen R2 implementation remains unchanged.

~input_sync_mode=legacy preserves the inherited callback and command behavior.
The synchronized mode is for PCT/analytic/shadow, whose command publisher is
owned by this node. It does not pretend to gate move_base's separate publisher.
"""
import functools
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rospy
import tf2_ros
import yaml

from v5_e2e_scope_rl.astra_input_sync import TimestampedScanQueue, TransformPending
from v5_e2e_scope_rl.benchmark_runtime import RuntimeInputHealth


def _load_r2():
    path = Path(os.environ['THESIS_WS'])/'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py'
    spec = importlib.util.spec_from_file_location('astra_frozen_r2_episode', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


r2 = _load_r2()


class AstraSyncEpisode(r2.R2BenchmarkEpisode):
    def __init__(self):
        self.input_sync_mode = str(rospy.get_param('~input_sync_mode','legacy'))
        if self.input_sync_mode not in ('legacy','synchronized'):
            raise ValueError('input_sync_mode must be legacy or synchronized')
        if self.input_sync_mode == 'synchronized' and rospy.get_param('~method') in ('actual_teb','dwa'):
            raise ValueError('M02 does not gate the external move_base publisher; use legacy for TEB/DWA')
        if not rospy.get_param('/use_sim_time',False) or not rospy.get_param('/m2_gazebo/simulation_only',False):
            raise RuntimeError('Astra episode requires isolated simulation markers')
        self._sync_lock = threading.RLock()
        self._scan_queue = TimestampedScanQueue()
        self._seen_epoch = 0
        self._node_fault = None
        self._sync_timer = None
        self._command_gated_count = 0
        self._watchdog_stop_count = 0
        self._bootstrap_real_history_wait_wall_s = 0.0
        self._last_sync_health = None
        self._maximum_committed_scan_age_s = 0.0
        super().__init__()
        if self.input_sync_mode == 'synchronized':
            # The timer and callbacks cannot mutate tracker/history during a
            # candidate build. Wrapping only serializes existing computations.
            for name in ('suggest','revalidate','build','commit_executed','terminal_command',
                         'candidate_safety_diagnostics','execution_audit','tracker_audit'):
                original = getattr(self.runtime,name)
                setattr(self.runtime,name,self._serialized(original))
            self._sync_timer = rospy.Timer(rospy.Duration(.005),self._sync_tick,reset=True)
        rospy.on_shutdown(self._write_sync_sidecar)

    def _serialized(self, function):
        @functools.wraps(function)
        def locked(*args,**kwargs):
            with self._sync_lock:
                return function(*args,**kwargs)
        return locked

    def _reset_runtime_epoch_if_needed(self):
        if self._seen_epoch == self._scan_queue.epoch:
            return
        self._seen_epoch = self._scan_queue.epoch
        self.runtime.scan_history = np.full((6,72),self.runtime.maximum_range_m,dtype=np.float32)
        self.runtime._clear_dynamic_track()
        self.runtime.latest_scan_pose_world = None
        self.runtime.latest_scan_stamp_s = None
        self.last_runtime_scan_stamp_s = None

    def _resolve_scan_transform(self, message, stamp_s):
        # Explicit timestamp only: zero/latest lookup is prohibited here.
        if message.header.stamp.to_nsec() <= 0:
            raise ValueError('zero scan timestamp')
        try:
            transform = self.tf_buffer.lookup_transform('odom',str(message.header.frame_id).lstrip('/'),
                                                       message.header.stamp,rospy.Duration(0))
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException) as error:
            raise TransformPending(str(error))
        if transform.header.stamp.to_nsec() != message.header.stamp.to_nsec():
            raise ValueError('TF returned a different timestamp than requested scan')
        return transform

    def _commit_scan(self, message, transform, stamp_s):
        scan = self.runtime.resample_scan(message.ranges,message.angle_min,message.angle_increment,message.range_max)
        pose = (float(transform.transform.translation.x)+self.spawn_world_x,
                float(transform.transform.translation.y),
                r2.r1.execution_core.quaternion_yaw(transform.transform.rotation))
        self.runtime.append_scan_with_pose(scan,pose,stamp_s)
        self.last_runtime_scan_stamp_s = stamp_s
        self.scan_transform_success_count += 1

    def _drain_locked(self):
        before = self._scan_queue.terminal_failure_count
        self._scan_queue.drain(rospy.Time.now().to_sec(),self._resolve_scan_transform,self._commit_scan)
        self._reset_runtime_epoch_if_needed()
        self.scan_transform_failure_count += self._scan_queue.terminal_failure_count-before

    def _scan(self, message):
        if self.input_sync_mode == 'legacy':
            return super()._scan(message)
        with self._sync_lock:
            stamp = message.header.stamp.to_sec()
            self.scan, self.scan_received_s = message, stamp
            self.scan_stamps.append(stamp)
            finite = [float(v) for v in message.ranges if math.isfinite(float(v))]
            if self.collecting and finite:
                self.minimum_scan_m = min(self.minimum_scan_m,min(finite))
            before = self._scan_queue.terminal_failure_count
            if str(message.header.frame_id).lstrip('/') != 'laser_link':
                self._node_fault = 'unexpected_scan_frame'
                self.scan_transform_failure_count += 1
                return
            self._scan_queue.submit(stamp,message,rospy.Time.now().to_sec())
            self._reset_runtime_epoch_if_needed()
            self.scan_transform_failure_count += self._scan_queue.terminal_failure_count-before
            self._drain_locked()

    def _sync_tick(self, _event):
        with self._sync_lock:
            try:
                self._drain_locked()
                health = self._input_health_locked(record=False)
                if self.collecting and health.failures(.20):
                    # This node owns the sensor-policy command chain only.
                    super().publish_command(0.0,0.0)
                    self._watchdog_stop_count += 1
            except Exception as error:
                self._node_fault = 'sync_timer_error:'+type(error).__name__
                super().publish_command(0.0,0.0)
                rospy.logerr_throttle(1.0,'Astra input sync fault: %s',error)

    def _input_health_locked(self, record=True):
        # Inherited callers pass a loop-start time. Read now under the lock so
        # normal timer progress cannot be mistaken for a backward clock jump.
        now = rospy.Time.now().to_sec()
        before = self._scan_queue.terminal_failure_count
        sync = self._scan_queue.health(now)
        self._reset_runtime_epoch_if_needed()
        self.scan_transform_failure_count += self._scan_queue.terminal_failure_count-before
        self._last_sync_health = sync
        committed_age = sync['latest_committed_scan_age_s']
        committed_age = float('inf') if committed_age is None else committed_age
        raw_age = float('inf') if self.scan_received_s is None else now-self.scan_received_s
        # The inherited _odom substitutes receipt time for a zero header stamp.
        # Synchronized health must reject that missing source timestamp.
        odom_stamp = self.odom.header.stamp.to_sec() if self.odom is not None else 0.0
        odom_age = float('inf') if odom_stamp <= 0.0 else now-odom_stamp
        # Negative raw ages stay invalid; do not clamp a future stamp to zero.
        scan_age = raw_age if raw_age < 0 else max(raw_age,committed_age)
        valid = sync['healthy'] and self._node_fault is None
        scan_frame = str(self.scan.header.frame_id).lstrip('/') if self.scan is not None else ''
        odom_frame = str(self.odom.header.frame_id).lstrip('/') if self.odom is not None else ''
        health = RuntimeInputHealth(scan_age,odom_age,committed_age,
                                    scan_frame_ok=scan_frame=='laser_link',
                                    odometry_frame_ok=odom_frame=='odom',transform_ok=valid)
        if record:
            if valid:
                self.tf_success_count += 1
            else:
                self.tf_failure_count += 1
            for value,name in ((scan_age,'maximum_scan_age_s'),(odom_age,'maximum_odom_age_s'),
                               (committed_age,'maximum_tf_age_s'),(committed_age,'_maximum_committed_scan_age_s')):
                if math.isfinite(value):
                    setattr(self,name,max(getattr(self,name),value))
        return health

    def input_health(self, now_s):
        if self.input_sync_mode == 'legacy':
            return super().input_health(now_s)
        with self._sync_lock:
            self._drain_locked()
            return self._input_health_locked()

    def publish_command(self, speed=0.0, yaw_rate=0.0):
        if self.input_sync_mode == 'legacy':
            return super().publish_command(speed,yaw_rate)
        with self._sync_lock:
            self._drain_locked()
            if (speed != 0.0 or yaw_rate != 0.0) and self._input_health_locked(record=False).failures(.20):
                speed = yaw_rate = 0.0
                self._command_gated_count += 1
            return super().publish_command(speed,yaw_rate)

    def wait_ready(self):
        super().wait_ready()
        if self.input_sync_mode == 'legacy':
            return
        started = time.monotonic()
        while not rospy.is_shutdown() and time.monotonic()-started < 35.0:
            with self._sync_lock:
                self._drain_locked()
                health = self._scan_queue.health(rospy.Time.now().to_sec())
                if self._scan_queue.fault_reason or self._node_fault:
                    raise RuntimeError('input synchronization bootstrap fault')
                if health['healthy']:
                    self._bootstrap_real_history_wait_wall_s = time.monotonic()-started
                    return
                super().publish_command(0.0,0.0)
            rospy.rostime.wallsleep(.01)
        raise RuntimeError('six timestamped real scans did not become ready within bounded bootstrap')

    def synchronization_report(self):
        with self._sync_lock:
            return {'mode':self.input_sync_mode,'queue':self._scan_queue.report(),
                    'bootstrap_contract':'synchronized requires six real source-stamp samples; legacy inherits six maximum-range placeholder rows',
                    'bootstrap_real_history_wait_wall_s':self._bootstrap_real_history_wait_wall_s,
                    'maximum_committed_scan_age_s':self._maximum_committed_scan_age_s,
                    'command_gated_count':self._command_gated_count,'watchdog_zero_command_count':self._watchdog_stop_count,
                    'last_sync_health':self._last_sync_health,'node_fault':self._node_fault,
                    'command_gate_scope':'sensor-policy publisher owned by this node; no move_base publisher control',
                    'health_ceiling_s':.20,'timer_period_s':.005}

    def _write_sync_sidecar(self):
        if not hasattr(self,'report_path'):
            return
        path = Path(self.report_path+'.input_sync.json')
        path.parent.mkdir(parents=True,exist_ok=True)
        temporary = path.with_name(path.name+'.tmp')
        temporary.write_text(json.dumps(self.synchronization_report(),indent=2,allow_nan=False)+'\n')
        os.replace(str(temporary),str(path))

    def run(self):
        try:
            report = super().run()
            report['astra_input_sync'] = self.synchronization_report()
            if self.input_sync_mode == 'synchronized':
                contract_ok = self._scan_queue.terminal_failure_count == 0 and self._node_fault is None
                report['astra_input_sync_contract_pass'] = contract_ok
                report['message_contract_pass'] = report['message_contract_pass'] and contract_ok
                report['execution_integrity_pass'] = report['execution_integrity_pass'] and contract_ok
            temporary = self.report_path+'.astra_sync.tmp'
            with open(temporary,'w',encoding='utf-8') as stream:
                yaml.safe_dump(report,stream,sort_keys=False,allow_unicode=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary,self.report_path)
            return report
        finally:
            if self._sync_timer is not None:
                self._sync_timer.shutdown()
            self._write_sync_sidecar()


def main():
    rospy.init_node('astra_sync_episode')
    try:
        AstraSyncEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr('Astra synchronized episode failed: %s: %s',type(error).__name__,error)
        return 2


if __name__=='__main__':
    sys.exit(main())
