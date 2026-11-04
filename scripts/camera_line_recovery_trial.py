#!/usr/bin/env python3
"""Attended line trial using the existing single-target service, no velocity publisher.

Default invocation only describes the plan. --execute starts physical recovery;
the operator must have confirmed the FOD placement and cleared the whole route.
This coordinator is not loaded by ordinary navigation or the single-target UI.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import threading
import time


def line_progress(origin, pose):
    x, y, yaw = origin
    dx, dy = pose[0] - x, pose[1] - y
    return (math.cos(yaw)*dx + math.sin(yaw)*dy,
            -math.sin(yaw)*dx + math.cos(yaw)*dy)


class CycleEvidence:
    """Do not count a previous latched COMPLETE or an unclean actuator exit."""
    def __init__(self, started):
        self.started = started
        self.active_seen = False
        self.crossing_completed = False
        self.target_ids = set()

    def mode(self, value, received):
        if received >= self.started and value.get('state') in ('ENTERING_FOD', 'FOD_ACTIVE'):
            self.active_seen = True

    def visual(self, value, received):
        if received < self.started or not self.active_seen:
            return
        if value.get('target_locked'):
            self.target_ids.add(value.get('target_track_id'))
        if value.get('state') == 'COMPLETE':
            self.crossing_completed = True

    def finished(self, mode, received):
        return (received >= self.started and self.active_seen and self.crossing_completed
                and mode.get('state') == 'FOD_COMPLETE_STOP'
                and not mode.get('sweep_cleanup_pending', True)
                and not mode.get('sweep_started_for_recovery', True)
                and mode.get('sweep_cleanup_confirmed', False)
                and abs(mode.get('measured_linear_x', 1.0)) <= .01)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--audit-dir', type=Path)
    parser.add_argument('--reverse-each', action='store_true')
    args = parser.parse_args()
    plan = dict(targets=3, forward_extent_m=12.0, line_center_tolerance_m=.5,
                visual_speed_mps=.6, reverse_each=args.reverse_each,
                after_three='stop; do not resume an old GPS route',
                meaning='three completed crossing cycles, physical pickup needs onsite confirmation')
    if not args.execute:
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        return
    if args.audit_dir is None:
        parser.error('--execute requires a new --audit-dir')

    # Importing this file for offline tests cannot initialize ROS or move anything.
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    args.audit_dir.mkdir(parents=True, exist_ok=True)
    events = (args.audit_dir/'sequence_events.jsonl').open('x', buffering=1)
    rospy.init_node('camera_line_recovery_trial', anonymous=True, disable_signals=True)
    lock = threading.RLock()
    latest, current, completed = {}, [None], []

    def event(name, value):
        data = dict(event=name, system_ns=time.time_ns(), ros_ns=rospy.Time.now().to_nsec(),
                    monotonic=time.monotonic(), data=value)
        with lock:
            events.write(json.dumps(data, ensure_ascii=False)+'\n')
        print(json.dumps(data, ensure_ascii=False), flush=True)

    def status(message, key):
        value, received = json.loads(message.data), time.monotonic()
        with lock:
            previous = latest.get(key, {})
            latest[key], latest[key+'_received'] = value, received
            if current[0] is not None:
                getattr(current[0], key)(value, received)
        if previous.get('state') != value.get('state'):
            event(key+'_state', value)

    def odom(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with lock:
            latest['pose'] = (p.x, p.y, yaw)
            latest['speed'] = message.twist.twist.linear.x

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt('operator interrupted this trial')

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    subs = [rospy.Subscriber('/fod_navigation_mode/status', String, lambda m: status(m, 'mode'), queue_size=30),
            rospy.Subscriber('/fod_visual_servo/status', String, lambda m: status(m, 'visual'), queue_size=30),
            rospy.Subscriber('/odom', Odometry, odom, queue_size=20)]
    control = rospy.ServiceProxy('/fod_navigation_mode/set_direct_fod_enabled', SetBool)
    verification = rospy.ServiceProxy('/fod_navigation_mode/set_verification_enabled', SetBool)
    restore_verification, changed_verification, attempted, result = None, False, False, 'incomplete'
    origin = None
    try:
        mode = json.loads(rospy.wait_for_message('/fod_navigation_mode/status', String, timeout=5).data)
        route = json.loads(rospy.wait_for_message('/gps/long_range/status', String, timeout=5).data)
        odom(rospy.wait_for_message('/odom', Odometry, timeout=5))
        if (route.get('active') or mode.get('state') not in ('GPS_ACTIVE', 'FOD_COMPLETE_STOP', 'FOD_ABORTED')
                or abs(mode.get('measured_linear_x', 1.0)) > .01
                or mode.get('sweep_cleanup_pending')):
            raise RuntimeError('A current task or motion is present; preserve it, do not start this trial.')
        origin = latest['pose']
        restore_verification = mode['verification_enabled']
        event('plan', dict(plan, origin_odom=origin))
        if restore_verification != args.reverse_each:
            changed_verification = True
            response = verification(args.reverse_each)
            if not response.success:
                raise RuntimeError(response.message)
        for number in range(1, 4):
            with lock:
                forward, lateral = line_progress(origin, latest['pose'])
            if forward >= 11.5 or abs(lateral) > .5:
                raise RuntimeError('Insufficient remaining line for the next recovery; keep stopped.')
            with lock:
                current[0] = CycleEvidence(time.monotonic())
            event('cycle_start_request', dict(number=number, forward_m=forward))
            attempted = True
            response = control(True)
            event('cycle_start_response', dict(number=number, success=response.success, message=response.message))
            if not response.success:
                raise RuntimeError(response.message)
            while not rospy.is_shutdown():
                with lock:
                    mode = dict(latest.get('mode', {}))
                    received = latest.get('mode_received', 0)
                    forward, lateral = line_progress(origin, latest['pose'])
                    speed = latest['speed']
                    done = current[0].finished(mode, received)
                    identities = sorted(i for i in current[0].target_ids if i)
                # Only this explicitly requested line trial is bounded here.
                # Reserve measured-speed braking travel before its 12 m end.
                if forward >= 12.0-max(.20, max(0., speed)*.80) or abs(lateral) > .5:
                    raise RuntimeError('Line extent reached (forward %.3f, lateral %.3f m); stop this trial.' % (forward, lateral))
                if received >= current[0].started and mode.get('state') in ('FOD_ABORTED', 'FAULT_STOP'):
                    raise RuntimeError(mode.get('reason', mode['state']))
                if done:
                    entry = dict(number=number, target_ids=identities, forward_m=forward,
                                 lateral_m=lateral, verification=mode.get('verification'),
                                 physical_pickup='pending onsite observation')
                    completed.append(entry)
                    event('cycle_crossing_completed', entry)
                    break
                time.sleep(.05)
            else:
                raise RuntimeError('ROS shutdown before this trial completed.')
        result = 'three_crossings_completed_and_stopped'
    except (Exception, KeyboardInterrupt) as exc:
        result = 'stopped_incomplete'
        event('trial_error', str(exc))
    finally:
        if attempted and result != 'three_crossings_completed_and_stopped':
            try:
                response = control(False)
                event('stop_response', dict(success=response.success, message=response.message))
            except Exception as exc:
                event('stop_service_error', str(exc))
        if changed_verification:
            try:
                response = verification(restore_verification)
                event('restore_single_target_verification', dict(success=response.success, message=response.message))
            except Exception as exc:
                event('restore_verification_error', str(exc))
        summary = dict(result=result, plan=plan, completed=completed, origin_odom=origin,
                       final_pose=latest.get('pose'), final_mode=latest.get('mode'))
        event('trial_result', summary)
        (args.audit_dir/'sequence_result.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        for sub in subs:
            sub.unregister()
        events.close()


if __name__ == '__main__':
    main()
