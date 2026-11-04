"""Remote task sequencing through existing GPS/FOD interfaces (no cmd_vel)."""
import hashlib
import json
import math
import os
from collections import deque
from pathlib import Path
import threading
import time

from fod_platform_protocol import position


def mission_from_record(record, vehicle_id):
    """Only explicitly addressed tasks are executable; old broadcast data stays data."""
    doc = record.get('document')
    if not record.get('valid') or not isinstance(doc, dict):
        raise ValueError('消息没有有效任务对象')
    if str(doc.get('vehicleId', '')) != vehicle_id:
        raise ValueError('不是明确发给本车的任务，仅保留消息')
    identity = doc.get('missionId')
    if not isinstance(identity, str) or not identity.strip():
        raise ValueError('自动任务需要 missionId')
    identity = identity.strip()
    if identity.startswith(('COMM-TEST-', 'LOCAL-LINK-CHECK-')) or doc.get('testOnly') is True:
        raise ValueError('通信测试消息：只接收，不执行车辆动作')
    task_type = doc.get('type', 'normal')
    if task_type not in ('normal', 'return', 'recovery'):
        raise ValueError('本轮未实现该业务类型：'+str(task_type))
    recovery = task_type == 'recovery' or 'TARGETS' in doc or doc.get('recovery') is True
    raw = doc.get('TARGETS') if 'TARGETS' in doc else doc.get('waypoints')
    if not isinstance(raw, list) or not raw:
        raise ValueError('任务必须提供非空 waypoints 或 TARGETS')
    points = []
    for index, item in enumerate(raw):
        point = position(item)
        action = str(item.get('action', 'go'))
        if action not in ('go', 'park', 'stop'):
            raise ValueError('暂不支持动作：'+action)
        speed = item.get('speed')
        if speed is not None:
            if isinstance(speed, bool):
                raise ValueError('speed 必须是数字')
            speed = float(speed)
            if not math.isfinite(speed) or speed < 0 or (speed == 0 and action not in ('stop', 'park')):
                raise ValueError('行驶速度必须为正；停车请使用 stop')
        if action == 'stop' and len(raw) != 1:
            raise ValueError('立即停车使用独立 stop 任务；路径终点停车使用 park')
        if action == 'park' and index != len(raw)-1:
            raise ValueError('park 必须是路径终点')
        point.update(action=action, speed=speed)
        points.append(point)
    return dict(mission_id=identity, vehicle_id=vehicle_id, points=points, recovery=recovery,
                digest=hashlib.sha256(json.dumps(doc, sort_keys=True, separators=(',', ':')).encode()).hexdigest())


class TaskCanceled(Exception):
    pass


class MissionRunner:
    """One worker per vehicle, with a small persisted task journal for redelivery."""
    def __init__(self, backend, vehicle_id, journal_path, notify, enabled=True):
        self.backend, self.vehicle_id, self.notify = backend, vehicle_id, notify
        self.path = Path(journal_path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.pending, self.current, self.last = [], None, None
        self.enabled = enabled
        self.canceled, self.shutdown = threading.Event(), threading.Event()
        self.foreign_takeover = False
        self.heartbeat_owned = False
        self.started_ns = time.time_ns()
        self.records = json.loads(self.path.read_text()) if self.path.exists() else {}
        for record in self.records.values():
            if record.get('state') in ('QUEUED', 'RUNNING', 'CANCELING'):
                record.update(state='INTERRUPTED', reason='进程结束，未自动续发旧任务')
        self._save()

    def _save(self):
        tmp = self.path.with_suffix('.tmp')
        with tmp.open('w') as stream:
            stream.write(json.dumps(self.records, ensure_ascii=False, indent=2))
            stream.flush()
            os.fsync(stream.fileno())
        tmp.replace(self.path)

    def snapshot(self):
        with self.lock:
            task = self.current or self.last or {}
            return dict(task, enabled=self.enabled, vehicle_id=self.vehicle_id,
                        pending_count=len(self.pending), active=self.current is not None,
                        owns_task_status=self.heartbeat_owned)

    def _announce(self, state, reason, **details):
        with self.lock:
            task = self.current or self.last
            if task is not None:
                task.update(state=state, reason=reason, **details)
                self.records[task['mission_id']].update(task)
                self._save()
            self.notify(self.snapshot())

    def submit(self, record):
        if not self.enabled:
            return '自动执行未启用，仅接收'
        if record.get('system_ns', 0) < self.started_ns:
            return '历史接收记录，不补执行'
        task = mission_from_record(record, self.vehicle_id)
        with self.lock:
            known = self.records.get(task['mission_id'])
            if known:
                if known['digest'] != task['digest']:
                    raise ValueError('相同 missionId 对应不同任务内容，请使用新编号')
                return '重复任务已记录，不重复执行：'+known['state']
            if task['points'][0]['action'] == 'stop':
                self.cancel('收到本车远端停车任务')
            task.update(state='QUEUED', reason='已接收，等待当前任务结束', point_index=0,
                        point_count=len(task['points']), results=[])
            self.records[task['mission_id']] = dict(task)
            self._save()
            self.pending.append(task)
            if self.current is None:
                self.last = task
            self.notify(self.snapshot())
        return '已加入自动任务：'+task['mission_id']

    def cancel(self, reason='操作员取消远端任务', takeover=False):
        with self.lock:
            if takeover or (self.current is None and not self.pending):
                self.heartbeat_owned = False
            for task in self.pending:
                task.update(state='CANCELED', reason=reason)
                self.records[task['mission_id']].update(state='CANCELED', reason=reason)
            self.pending.clear()
            self.foreign_takeover = takeover
            self.canceled.set()
            self._save()
            if self.current is not None:
                self._announce('CANCELING', reason)
            else:
                self.notify(self.snapshot())

    def _check_cancel(self):
        if self.canceled.is_set() or self.shutdown.is_set():
            raise TaskCanceled('人工接管，不覆盖新的本地任务' if self.foreign_takeover else '远端任务已取消')

    def run_next(self):
        with self.lock:
            if self.current or not self.pending:
                return False
            stop_only = self.pending[0]['points'][0]['action'] == 'stop'
            if not stop_only and not self.backend.idle():
                return False
            self.current = self.pending.pop(0)
            self.canceled.clear()
            self.foreign_takeover = False
            self.heartbeat_owned = True
        started = False
        try:
            self._announce('RUNNING', '开始执行远端任务')
            for index, point in enumerate(self.current['points']):
                self._check_cancel()
                self._announce('RUNNING', '准备当前点', point_index=index+1, target=point)
                started = True
                if point['action'] == 'stop':
                    self.backend.stop()
                    self.current['results'].append(dict(point_index=index+1, result='STOPPED'))
                    continue
                self.backend.prepare(self.current['recovery'], self._check_cancel)
                self._check_cancel()
                self.backend.set_speed(point.get('speed'))
                self._check_cancel()
                receipt = self.backend.dispatch(point)
                result = self.backend.wait_result(point, receipt, self.current['recovery'], self._check_cancel,
                                                  lambda text:self._announce('RUNNING', text))
                self.current['results'].append(dict(point_index=index+1, **result))
            self._check_cancel()
            self._announce('COMPLETED', '任务流程执行完毕；FOD 实物入仓以现场证据为准')
        except TaskCanceled as error:
            if started and not self.foreign_takeover:
                try:
                    self.backend.stop()
                except Exception as stop_error:
                    error = RuntimeError(str(error)+'；收尾：'+str(stop_error))
            self._announce('CANCELED', str(error))
        except Exception as error:
            if started and not self.foreign_takeover:
                try:
                    self.backend.stop()
                except Exception as stop_error:
                    error = RuntimeError(str(error)+'；收尾：'+str(stop_error))
            self._announce('FAILED', str(error))
        finally:
            self.backend.restore_speed()
            with self.lock:
                self.last, self.current = self.current, None
                self.notify(self.snapshot())
        return True

    def run(self):
        while not self.shutdown.is_set():
            if not self.run_next():
                self.shutdown.wait(.2)


class RosNavigationBackend:
    def __init__(self):
        import rospy
        from actionlib_msgs.msg import GoalID
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import String
        self.ros = rospy
        self.lock = threading.RLock()
        self.route, self.mode, self.route_version, self.mode_version = {}, {}, 0, 0
        self.mode_events = deque(maxlen=200)
        self.route_events = deque(maxlen=200)
        self.point_mode_floor = 0
        self.foreign_callback = lambda:None
        self.cancel_callback = lambda:None
        self.speed_client, self.original_speed, self.applied_speed = None, None, None
        self.goals = rospy.Publisher('/gps/goal_fix', NavSatFix, queue_size=2)
        self.subs = [rospy.Subscriber('/gps/long_range/status', String, self._route, queue_size=20),
                     rospy.Subscriber('/fod_navigation_mode/status', String, self._mode, queue_size=30),
                     rospy.Subscriber('/gps/goal_fix', NavSatFix, self._goal, queue_size=10),
                     rospy.Subscriber('/move_base/cancel', GoalID, self._cancel, queue_size=10)]

    def _route(self, message):
        with self.lock:
            self.route = json.loads(message.data)
            self.route_version += 1
            self.route_events.append((self.route_version, dict(self.route)))

    def _mode(self, message):
        with self.lock:
            self.mode = json.loads(message.data)
            self.mode_version += 1
            self.mode_events.append((self.mode_version, self.mode.get('state'), self.mode.get('recovery_armed', False)))

    def _goal(self, message):
        caller = (getattr(message, '_connection_header', None) or {}).get('callerid', '')
        if caller and caller not in (self.ros.get_name(), '/fod_navigation_mode'):
            self.foreign_callback()

    def _cancel(self, message):
        caller = (getattr(message, '_connection_header', None) or {}).get('callerid', '')
        if not message.id and caller and caller not in (self.ros.get_name(), '/fod_navigation_mode', '/gps_long_range_goal_manager'):
            self.cancel_callback()

    def idle(self):
        with self.lock:
            return ((self.mode.get('state') == 'FOD_COMPLETE_STOP' or
                     (self.mode.get('state') == 'GPS_ACTIVE' and not self.route.get('active', False)))
                    and not self.mode.get('recovery_armed', False)
                    and not self.mode.get('sweep_cleanup_pending', False))

    def call(self, name, enabled=None):
        from std_srvs.srv import Trigger, SetBool
        service_type = Trigger if enabled is None else SetBool
        if not self.ros.is_shutdown():
            self.ros.wait_for_service(name, timeout=4)
        client = self.ros.ServiceProxy(name, service_type)
        response = client() if enabled is None else client(enabled)
        if not response.success:
            raise RuntimeError(name+': '+response.message)

    def prepare(self, recovery, check_cancel=lambda:None):
        with self.lock:
            self.point_mode_floor = self.mode_version
        for name, value in (('/gps/long_range/clear_route', None),
                            ('/fod_navigation_mode/prepare_gps_goal', None),
                            ('/gps/long_range/set_paused', False),
                            ('/fod_navigation_mode/set_fod_enabled', recovery)):
            check_cancel()
            self.call(name, value)

    def dispatch(self, point):
        from sensor_msgs.msg import NavSatFix, NavSatStatus
        with self.lock:
            receipt = dict(route_version=self.route_version, mode_version=self.point_mode_floor,
                           old_route_id=self.route.get('route_id'), sent_at=time.monotonic())
        message = NavSatFix()
        message.header.stamp = self.ros.Time.now()
        message.header.frame_id = 'gps'
        message.status.status = NavSatStatus.STATUS_FIX
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude, message.longitude = point['latitude'], point['longitude']
        if self.goals.get_num_connections() == 0:
            raise RuntimeError('GPS 目标接口没有订阅者')
        self.goals.publish(message)
        return receipt

    def wait_result(self, point, receipt, recovery, check_cancel, progress):
        accepted, recovering, last_text = False, False, ''
        route_id = None
        while not self.ros.is_shutdown():
            check_cancel()
            with self.lock:
                route, mode = dict(self.route), dict(self.mode)
                fresh_route = self.route_version > receipt['route_version']
                fresh_mode = self.mode_version > receipt['mode_version']
                route_events = [r for v, r in self.route_events if v > receipt['route_version']]
                began_recovery = any(v > receipt['mode_version'] and (armed or s in (
                    'ENTERING_FOD', 'FOD_ACTIVE', 'ENTERING_GPS_RECOVERY', 'GPS_FOD_RECOVERY', 'FOD_VERIFY'))
                    for v, s, armed in self.mode_events)
            state, ms = route.get('state'), mode.get('state')
            if fresh_route and state == 'REJECTED_NEW_GOAL':
                raise RuntimeError('GPS 拒绝目标：'+route.get('reason', ''))
            if not accepted:
                for candidate in route_events:
                    matching = (candidate.get('route_id') and candidate.get('route_id') != receipt['old_route_id']
                                and abs(candidate.get('final_latitude', 999)-point['latitude']) < 1e-8
                                and abs(candidate.get('final_longitude', 999)-point['longitude']) < 1e-8)
                    if matching:
                        accepted, route_id = True, candidate['route_id']
                        break
            if recovery and fresh_mode and accepted and began_recovery:
                recovering = True
            if fresh_mode and ms in ('FOD_ABORTED', 'FAULT_STOP'):
                raise RuntimeError(mode.get('reason', ms))
            handed_to_recovery = recovering and ms in (
                'ENTERING_FOD', 'FOD_ACTIVE', 'ENTERING_GPS_RECOVERY', 'GPS_FOD_RECOVERY',
                'FOD_VERIFY', 'FOD_COMPLETE_STOP')
            if (accepted and route.get('route_id') == route_id
                    and state in ('ABORTED', 'CANCELED', 'SUPERSEDED', 'KEEPOUT_BLOCKED')
                    and not (handed_to_recovery and state in ('CANCELED', 'SUPERSEDED'))):
                raise RuntimeError(route.get('reason', state))
            if not recovery and accepted and route.get('route_id') == route_id and state == 'COMPLETE':
                return dict(result='GPS_ARRIVED', route_id=route_id)
            gps_only_recovery_done = (ms == 'GPS_ACTIVE' and mode.get('verification_this_run') is False
                                      and not mode.get('recovery_armed') and route.get('route_id') == route_id
                                      and state == 'COMPLETE')
            if (recovery and recovering and fresh_mode and (ms == 'FOD_COMPLETE_STOP' or gps_only_recovery_done)
                    and not mode.get('sweep_cleanup_pending') and not mode.get('sweep_started_for_recovery')
                    and mode.get('sweep_cleanup_confirmed')):
                return dict(result='RECOVERY_FLOW_COMPLETED', verification=mode.get('verification'),
                            physical_pickup='unconfirmed', route_id=route_id)
            if not accepted and time.monotonic()-receipt['sent_at'] > 6:
                raise RuntimeError('没有收到本目标的 GPS 接受状态，未重发目标')
            text = mode.get('reason', ms or '') if recovering else route.get('reason', state or '等待 GPS 接受')
            if text != last_text:
                progress(text)
                last_text = text
            time.sleep(.1)
        raise TaskCanceled('ROS 已关闭')

    def stop(self):
        errors = []
        for name, value in (('/gps/long_range/clear_route', None),
                            ('/fod_navigation_mode/set_fod_enabled', False)):
            try:
                self.call(name, value)
            except Exception as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError('；'.join(errors))

    def set_speed(self, requested):
        if requested is None or requested == 0:
            self.restore_speed()
            return
        from dynamic_reconfigure.client import Client
        if self.speed_client is None:
            self.speed_client = Client('/move_base/TebLocalPlannerROS', timeout=3)
        config = self.speed_client.get_configuration(timeout=3)
        if self.original_speed is None:
            self.original_speed = config['max_vel_x']
        self.applied_speed = min(float(requested), self.original_speed)
        self.speed_client.update_configuration({'max_vel_x':self.applied_speed})

    def restore_speed(self):
        try:
            if self.original_speed is not None:
                now = self.speed_client.get_configuration(timeout=2)['max_vel_x']
                if abs(now-self.applied_speed) < 1e-6:
                    self.speed_client.update_configuration({'max_vel_x':self.original_speed})
        except Exception as error:
            self.ros.logwarn('Remote task speed restore: %s', error)
        finally:
            self.original_speed = self.applied_speed = None
