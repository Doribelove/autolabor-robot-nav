#!/usr/bin/env python3
"""Task replay, manual takeover and completion regressions. No ROS master needed."""
from collections import deque
import copy
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch

from fod_platform_tasks import MissionRunner, RosNavigationBackend, TaskCanceled, mission_from_record


def packet(identity='MISSION-1', **extra):
    doc = dict(missionId=identity, vehicleId='FOD-102', type='normal',
               waypoints=[dict(lat=30.1, lon=104.1, action='go')])
    doc.update(extra)
    return dict(valid=True, system_ns=time.time_ns(), document=doc)


class FakeBackend:
    def __init__(self):
        self.calls, self.on_wait = [], None
        self.available = True

    def idle(self): return self.available
    def prepare(self, recovery, check):
        check()
        self.calls.append(('prepare', recovery))
    def set_speed(self, speed): self.calls.append(('speed', speed))
    def dispatch(self, point):
        self.calls.append(('goal', point['latitude']))
        return {}
    def wait_result(self, point, receipt, recovery, check, progress):
        if self.on_wait: self.on_wait()
        check()
        return dict(result='RECOVERY_FLOW_COMPLETED' if recovery else 'GPS_ARRIVED')
    def stop(self): self.calls.append(('stop',))
    def restore_speed(self): self.calls.append(('restore',))


class TaskTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'journal.json'
        self.backend = FakeBackend()
        self.runner = MissionRunner(self.backend, 'FOD-102', self.path, lambda status:None)

    def test_only_addressed_supported_tasks_execute(self):
        for changes in ({'vehicleId':'100'}, {'vehicleId':None}, {'type':'emergency'},
                        {'testOnly':True}, {'missionId':'COMM-TEST-FOD-102-001'},
                        {'missionId':'LOCAL-LINK-CHECK-123'}, {'missionId':''},
                        {'waypoints':[]}, {'waypoints':[dict(lat=104, lon=30)]},
                        {'waypoints':[dict(lat=30, lon=104, speed=-1)]},
                        {'waypoints':[dict(lat=30, lon=104, speed=0)]},
                        {'waypoints':[dict(lat=30, lon=104, action='turn')]}):
            with self.subTest(changes=changes), self.assertRaises((ValueError, TypeError)):
                mission_from_record(packet(**changes), 'FOD-102')
        self.assertFalse(mission_from_record(packet(), 'FOD-102')['recovery'])
        self.assertTrue(mission_from_record(packet(TARGETS=[dict(LAT=30, LON=104)]), 'FOD-102')['recovery'])

    def test_points_run_in_order_once(self):
        record = packet(waypoints=[dict(lat=30+i*.01, lon=104) for i in range(3)])
        self.runner.submit(record)
        self.runner.submit(record)
        self.runner.run_next()
        self.assertEqual([p[1] for p in self.backend.calls if p[0]=='goal'], [30,30.01,30.02])
        self.assertEqual(self.runner.snapshot()['state'], 'COMPLETED')
        self.assertEqual(len(self.runner.snapshot()['results']), 3)
        again = MissionRunner(self.backend, 'FOD-102', self.path, lambda status:None)
        record['system_ns'] = time.time_ns()
        self.assertIn('重复', again.submit(record))
        self.assertFalse(again.run_next())

    def test_conflicting_id_never_overwrites_original(self):
        self.runner.submit(packet())
        with self.assertRaises(ValueError):
            self.runner.submit(packet(waypoints=[dict(lat=31, lon=104)]))
        self.assertEqual(self.runner.records['MISSION-1']['points'][0]['latitude'], 30.1)

    def test_restart_does_not_resume_incomplete_task(self):
        self.runner.submit(packet())
        restarted = MissionRunner(self.backend, 'FOD-102', self.path, lambda status:None)
        self.assertEqual(restarted.records['MISSION-1']['state'], 'INTERRUPTED')
        self.assertFalse(restarted.run_next())

    def test_busy_local_task_is_not_interrupted(self):
        self.backend.available = False
        self.runner.submit(packet())
        self.assertFalse(self.runner.run_next())
        self.assertEqual(self.backend.calls, [])
        self.runner.cancel()
        self.assertEqual(self.runner.snapshot()['state'], 'CANCELED')

    def test_remote_stop_is_not_blocked_by_queued_navigation(self):
        self.backend.available = False
        self.runner.submit(packet())
        self.runner.submit(packet('STOP-1',waypoints=[dict(lat=30,lon=104,action='stop',speed=0)]))
        self.assertTrue(self.runner.run_next())
        self.assertEqual(self.runner.records['MISSION-1']['state'], 'CANCELED')
        self.assertIn(('stop',),self.backend.calls)
        self.assertFalse(any(c[0]=='goal' for c in self.backend.calls))

    def test_manual_navigation_does_not_inherit_old_remote_result(self):
        self.runner.submit(packet())
        self.runner.run_next()
        self.assertTrue(self.runner.snapshot()['owns_task_status'])
        self.runner.cancel('local goal',takeover=True)
        self.assertFalse(self.runner.snapshot()['owns_task_status'])

    def test_manual_goal_prevents_remaining_points_without_canceling_new_goal(self):
        self.runner.submit(packet(waypoints=[dict(lat=30, lon=104),dict(lat=31, lon=104)]))
        self.runner.submit(packet('MISSION-2'))
        self.backend.on_wait = lambda:self.runner.cancel('manual goal', takeover=True)
        self.runner.run_next()
        self.assertEqual(self.runner.snapshot()['state'], 'CANCELED')
        self.assertNotIn(('stop',), self.backend.calls)
        self.assertEqual(len([c for c in self.backend.calls if c[0]=='goal']), 1)
        self.assertEqual(self.runner.records['MISSION-2']['state'], 'CANCELED')

    def test_operator_cancel_stops_own_task_and_clears_queue(self):
        self.runner.submit(packet())
        self.runner.submit(packet('MISSION-2'))
        self.backend.on_wait = lambda:self.runner.cancel()
        self.runner.run_next()
        self.assertIn(('stop',), self.backend.calls)
        self.assertEqual(self.runner.snapshot()['state'], 'CANCELED')
        self.assertEqual(self.runner.snapshot()['pending_count'], 0)

    def test_failure_does_not_send_next_point(self):
        self.runner.submit(packet(type='recovery', waypoints=[dict(lat=30,lon=104),dict(lat=31,lon=104)]))
        def fail(): raise RuntimeError('清扫机构收尾失败')
        self.backend.on_wait = fail
        self.runner.run_next()
        self.assertEqual(self.runner.snapshot()['state'], 'FAILED')
        self.assertEqual(len([c for c in self.backend.calls if c[0]=='goal']), 1)
        self.assertIn(('prepare', True), self.backend.calls)


class BackendTest(unittest.TestCase):
    def setUp(self):
        self.b = RosNavigationBackend.__new__(RosNavigationBackend)
        self.b.ros = Obj(is_shutdown=lambda:False, get_name=lambda:'/fod_platform_link')
        self.b.lock = threading.RLock()
        self.b.route, self.b.mode = {}, {}
        self.b.route_version = self.b.mode_version = 0
        self.b.route_events, self.b.mode_events = deque(), deque()
        self.point = dict(latitude=30.1, longitude=104.1)
        self.receipt = dict(route_version=0, mode_version=0, old_route_id='old', sent_at=time.monotonic())
        self.base_route = dict(route_id='new', final_latitude=30.1, final_longitude=104.1, active=True, state='ACTIVE_FINAL')

    def route(self, **changes):
        r = dict(self.base_route, **changes)
        self.b._route(Obj(data=json.dumps(r)))

    def mode(self, **changes):
        self.b._mode(Obj(data=json.dumps(changes)))

    def run_until(self, stages, recovery=True):
        stages = iter(stages)
        def advance(*args):
            try: next(stages)()
            except StopIteration: raise AssertionError('never reached expected terminal state')
        with patch('fod_platform_tasks.time.sleep', side_effect=advance):
            return self.b.wait_result(self.point, self.receipt, recovery, lambda:None, lambda text:None)

    def test_normal_waits_for_new_matching_goal(self):
        self.route(route_id='old', state='COMPLETE', active=False)
        self.mode(state='GPS_ACTIVE')
        result = self.run_until([lambda:self.route(state='COMPLETE', active=False)], False)
        self.assertEqual(result['route_id'], 'new')

    def test_gps_arrival_and_cleanup_pending_are_not_recovery_completion(self):
        self.route(state='COMPLETE', active=False)
        self.mode(state='GPS_ACTIVE', recovery_armed=True)
        stages = [lambda:self.mode(state='FOD_VERIFY'),
                  lambda:self.mode(state='FOD_COMPLETE_STOP', sweep_cleanup_pending=True, sweep_cleanup_confirmed=False),
                  lambda:self.mode(state='FOD_COMPLETE_STOP', sweep_cleanup_pending=False, sweep_cleanup_confirmed=True,
                                   verification={'phase':'PASSED'})]
        result = self.run_until(stages)
        self.assertEqual(result['result'], 'RECOVERY_FLOW_COMPLETED')
        self.assertEqual(result['physical_pickup'], 'unconfirmed')

    def test_fast_fallback_retains_original_acceptance(self):
        self.route()
        self.mode(state='FOD_ACTIVE', recovery_armed=True)
        self.route(route_id='fallback', final_latitude=30.10001, state='COMPLETE', active=False)
        self.mode(state='FOD_COMPLETE_STOP', sweep_cleanup_confirmed=True)
        self.assertEqual(self.run_until([])['route_id'], 'new')

    def test_gps_recovery_with_verification_disabled_still_waits_for_cleanup(self):
        self.route(state='COMPLETE', active=False)
        self.mode(state='GPS_ACTIVE',recovery_armed=True)
        def finished():
            self.mode(state='GPS_ACTIVE',recovery_armed=False,verification_this_run=False,
                      sweep_cleanup_confirmed=True, sweep_cleanup_pending=False)
        self.assertEqual(self.run_until([finished])['result'],'RECOVERY_FLOW_COMPLETED')

    def test_visual_handover_supersedes_original_route(self):
        self.route(state='SUPERSEDED', active=False)
        self.mode(state='ENTERING_GPS_RECOVERY', recovery_armed=True)
        result = self.run_until([lambda:self.mode(state='FOD_COMPLETE_STOP', sweep_cleanup_confirmed=True)])
        self.assertEqual(result['result'], 'RECOVERY_FLOW_COMPLETED')

    def test_rejection_and_fault_are_reported(self):
        self.route(state='REJECTED_NEW_GOAL', reason='禁区')
        with self.assertRaisesRegex(RuntimeError, '禁区'): self.run_until([], False)
        self.route()
        self.mode(state='FOD_ABORTED', reason='机构异常')
        with self.assertRaisesRegex(RuntimeError, '机构异常'): self.run_until([])

    def test_no_acceptance_never_resends(self):
        self.receipt['sent_at'] = time.monotonic()-7
        with self.assertRaisesRegex(RuntimeError, '未重发'): self.run_until([], False)

    def test_manual_takeover_during_prepare_stops_remaining_service_calls(self):
        called = []
        self.b.call = lambda name, value:called.append(name)
        def check():
            if called: raise TaskCanceled('manual')
        with self.assertRaises(TaskCanceled): self.b.prepare(True, check)
        self.assertEqual(called, ['/gps/long_range/clear_route'])

    def test_stop_attempts_fod_cleanup_when_gps_service_has_failed(self):
        called = []
        def call(name, value):
            called.append(name)
            if len(called)==1: raise RuntimeError('GPS unavailable')
        self.b.call = call
        with self.assertRaisesRegex(RuntimeError,'GPS unavailable'): self.b.stop()
        self.assertEqual(called, ['/gps/long_range/clear_route','/fod_navigation_mode/set_fod_enabled'])

    def test_requested_speed_cannot_raise_original_limit_and_restore_preserves_manual_change(self):
        config = {'max_vel_x':1.8}
        self.b.original_speed = self.b.applied_speed = None
        self.b.speed_client = Obj(get_configuration=lambda **kwargs:dict(config),
                                  update_configuration=lambda changes:config.update(changes))
        self.b.set_speed(3)
        self.assertEqual(config['max_vel_x'], 1.8)
        self.b.set_speed(.8)
        self.b.restore_speed()
        self.assertEqual(config['max_vel_x'], 1.8)
        self.b.set_speed(.6)
        config['max_vel_x'] = .3
        self.b.restore_speed()
        self.assertEqual(config['max_vel_x'], .3)


if __name__ == '__main__': unittest.main()
