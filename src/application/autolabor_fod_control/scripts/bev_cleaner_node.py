#!/usr/bin/env python3
"""Surround BEV spot cleaning through the existing ordinary goal bridge.

No velocity publisher, global planner reconfiguration or brush actuator command.
Only an explicit start request creates goals. Detection runs independently.
"""
import copy
import json
import math
import threading
import time
import urllib.request
import uuid

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from autolabor_fod_control.bev_cleaning import (
    VisibleTargets, forward_crossing, path_length, sweep_candidates, transform_point,
)
from autolabor_fod_control.cleaning_geometry import footprint_is_free


class BevCleaner:
    TERMINAL = {GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.ABORTED,
                GoalStatus.REJECTED, GoalStatus.RECALLED}

    def __init__(self):
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.shutdown = threading.Event()
        self.targets = VisibleTargets()
        self.active = False
        self.round_id = ''
        self.state, self.reason = 'IDLE', '等待开始视野清扫'
        self.perception_reason = '等待环视 BEV 检测'
        self.frame = None
        self.last_identity = None
        self.source_key = None
        self.goal_id, self.expected = '', None
        self.retired_requests = {}
        self.action_states = {}
        self.nav, self.nav_receipt = {}, 0.
        self.grid, self.grid_receipt = None, 0.
        self.selected = None
        self.passed_count = 0
        self.skipped = set()
        self.skip_reasons = {}
        self.unfinished = []
        self.route = []
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.url = rospy.get_param('~detections_url', 'http://127.0.0.1:18884/detections')
        self.rig_pose = rospy.get_param('~rig_to_base_xyyaw', [])
        if self.rig_pose and (len(self.rig_pose) != 3 or
                not all(math.isfinite(float(v)) for v in self.rig_pose)):
            raise ValueError('rig_to_base_xyyaw must be [x_m, y_m, yaw_rad] or []')
        self.offset = float(rospy.get_param('~cleaning_reference_offset_m', .52))
        self.tolerance = float(rospy.get_param('~cleaning_tolerance_m', .10))
        # Front attachments may extend the reference beyond the old 1 m cap.
        if not math.isfinite(self.offset) or self.offset < 0 or not .01 <= self.tolerance <= .10:
            raise ValueError('Invalid cleaning reference geometry')
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(15.))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=2)
        self.status_pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.observations_pub = rospy.Publisher('~observations', String, queue_size=1)
        self.path_pub = rospy.Publisher('~planned_path', Path, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber('/navigation_pause/status', String, self._nav_cb, queue_size=5),
            rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self._goal_cb, queue_size=10),
            rospy.Subscriber('/move_base/status', GoalStatusArray, self._status_cb, queue_size=5),
            rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid,
                             self._grid_cb, queue_size=1),
            rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate,
                             self._grid_update_cb, queue_size=5),
        ]
        self.start_service = rospy.Service('~start', Trigger, self._start)
        self.cancel_service = rospy.Service('~cancel', Trigger, self._cancel)
        self.timer = rospy.Timer(rospy.Duration(.25), self._publish)
        rospy.on_shutdown(self._shutdown)
        threading.Thread(target=self._perception, daemon=True).start()

    def _nav_cb(self, msg):
        try:
            value = json.loads(msg.data)
            with self.lock:
                self.nav, self.nav_receipt = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def _status_cb(self, msg):
        with self.lock:
            self.action_states = {s.goal_id.id: s.status for s in msg.status_list}

    def _goal_cb(self, msg):
        with self.lock:
            p = self.expected
            actual = msg.goal.target_pose
            if getattr(msg, '_connection_header', {}).get('callerid') != '/navigation_pause':
                return
            retired = self.retired_requests.get(actual.header.seq)
            if retired is not None and actual.header.frame_id == self.map_frame and actual.pose == retired.pose:
                if self.action_states.get(msg.goal_id.id) not in self.TERMINAL:
                    self.cancel_pub.publish(GoalID(id=msg.goal_id.id))
                return
            if (p is None or getattr(msg, '_connection_header', {}).get('callerid') != '/navigation_pause'
                    or actual.header.seq != p.header.seq or actual.header.frame_id != self.map_frame
                    or actual.pose != p.pose):
                return
            self.goal_id = msg.goal_id.id
            if self.stop.is_set():
                self._cancel_exact()

    def _grid_cb(self, msg):
        with self.lock:
            self.grid, self.grid_receipt = copy.deepcopy(msg), time.monotonic()

    def _grid_update_cb(self, msg):
        with self.lock:
            g = self.grid
            if (g is None or msg.header.frame_id != self.map_frame
                    or len(msg.data) != msg.width*msg.height
                    or msg.x+msg.width > g.info.width or msg.y+msg.height > g.info.height):
                return
            values = list(g.data)
            for row in range(msg.height):
                at = (msg.y+row)*g.info.width+msg.x
                values[at:at+msg.width] = msg.data[row*msg.width:(row+1)*msg.width]
            g.data, g.header.stamp = values, msg.header.stamp
            self.grid_receipt = time.monotonic()

    @staticmethod
    def _pose_tuple(transform):
        t, q = transform.transform.translation, transform.transform.rotation
        values = (t.x, t.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('定位变换包含无效数值')
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return t.x, t.y, yaw

    def _robot(self, stamp=None):
        tf = self.tf.lookup_transform(self.map_frame, self.base_frame,
                                     stamp or rospy.Time(0), rospy.Duration(.2))
        if stamp is None and not -.2 <= (rospy.Time.now()-tf.header.stamp).to_sec() <= .5:
            raise RuntimeError('当前位置不可用')
        return self._pose_tuple(tf)

    def _localize(self, frame):
        stamp = rospy.Time.from_sec(float(frame['observed_epoch_s']))
        rig = frame['ground_frame']
        if self.rig_pose:
            base = self._robot(stamp)
            origin = transform_point(base, self.rig_pose[:2])
            pose = (*origin, base[2]+float(self.rig_pose[2]))
        else:
            try:
                pose = self._pose_tuple(self.tf.lookup_transform(
                    self.map_frame, rig, stamp, rospy.Duration(.15)))
            except tf2_ros.TransformException:
                raise RuntimeError('等待安装外参：BEV 原点相对 base_link 的 x/y/yaw')
        result = []
        for obj in frame['ground_objects']:
            x, y = transform_point(pose, (obj['x'], obj['y']))
            if all(math.isfinite(v) for v in (x, y)):
                result.append(dict(x=x, y=y, label=obj['label']))
        return result

    def _perception(self):
        while not self.shutdown.is_set() and not rospy.is_shutdown():
            try:
                with urllib.request.urlopen(self.url, timeout=1.) as response:
                    raw = response.read(512*1024+1)
                if len(raw) > 512*1024:
                    raise ValueError('检测报文过大')
                frame = json.loads(raw)
                if frame.get('schema') != 'conbine.bev.cleaning.observations.v1':
                    raise ValueError('BEV 检测协议不匹配')
                identity = (frame['source_id']['session'], frame['source_id']['sequence'])
                observed = float(frame['observed_epoch_s'])
                if not -.2 <= time.time()-observed <= 2.:
                    raise ValueError('BEV 源图像已过期')
                if not isinstance(frame['ground_objects'], list) or len(frame['ground_objects']) > 100:
                    raise ValueError('BEV 目标数量无效')
                with self.lock:
                    if identity == self.last_identity:
                        continue
                    key = (identity[0], frame['config_sha256'])
                    if self.source_key is not None and key != self.source_key:
                        self.targets = VisibleTargets()
                        if self.active:
                            self.stop.set()
                    self.source_key, self.last_identity = key, identity
                    self.frame = frame
                positions = self._localize(frame)
                with self.lock:
                    self.targets.update(positions, observed)
                    cameras = frame.get('configured_cameras', [])
                    prefix = '{}路{} BEV'.format(len(cameras), '环视' if 'rear' in cameras else '相机') if cameras else 'BEV'
                    self.perception_reason = '{} 在线，当前可定位 {} 个目标'.format(prefix, len(positions))
                self.observations_pub.publish(String(data=json.dumps(dict(
                    frame, map_frame=self.map_frame, map_objects=positions), allow_nan=False)))
            except tf2_ros.TransformException:
                with self.lock:
                    self.perception_reason = 'BEV到底盘平面外参已配置，等待导航定位TF'
            except Exception as exc:
                with self.lock:
                    self.perception_reason = str(exc)
            finally:
                self.shutdown.wait(.15)

    def _navigation_available(self):
        with self.lock:
            n = self.nav
            if time.monotonic()-self.nav_receipt > 1.:
                raise RuntimeError('导航状态尚不可用')
            foreign = (set(n.get('active_ordinary_goal_ids', [])) |
                       set(n.get('active_explicit_goal_ids', []))) - {self.goal_id}
            if n.get('paused') or n.get('coverage_active') or n.get('coverage_owner_claimed') or foreign:
                raise RuntimeError('导航暂停或有其他任务正在执行')

    def _start(self, _request):
        with self.lock:
            if self.active:
                return TriggerResponse(False, '视野清扫已在执行')
            try:
                self._navigation_available()
                if self.nav.get('active_ordinary_goal_ids') or self.nav.get('active_explicit_goal_ids'):
                    raise RuntimeError('等待上次导航目标结束')
                if not self.targets.visible(time.time()):
                    raise RuntimeError(self.perception_reason+'；暂无可定位的当前目标')
                self._robot()
            except Exception as exc:
                return TriggerResponse(False, str(exc))
            self.stop.clear()
            self.round_id = uuid.uuid4().hex
            self.active, self.passed_count = True, 0
            self.targets.begin_round(time.time())
            self.skipped.clear()
            self.skip_reasons.clear()
            self.unfinished = []
            self.state, self.reason = 'PLANNING', '已记录待扫目标，正在规划'
            rospy.loginfo('BEV round %s started with %d map targets',
                          self.round_id, len(self.targets.pending()))
            threading.Thread(target=self._run, daemon=True).start()
        self._publish()
        return TriggerResponse(True, '开始逐步清扫当前 BEV 视野内的垃圾')

    def _cancel_exact(self):
        if self.goal_id and self.action_states.get(self.goal_id) not in self.TERMINAL:
            self.cancel_pub.publish(GoalID(id=self.goal_id))

    def _cancel(self, _request):
        with self.lock:
            self.stop.set()
            self._cancel_exact()
        return TriggerResponse(True, '已请求停止本次视野清扫')

    def _assert_running(self):
        if self.stop.is_set() or self.shutdown.is_set() or rospy.is_shutdown():
            raise RuntimeError('视野清扫已停止')
        self._navigation_available()

    def _pose(self, p):
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = self.map_frame, rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y = p[:2]
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(p[2]/2), math.cos(p[2]/2)
        return msg

    def _plan(self, start, end):
        self._assert_running()
        request = GetPlanRequest(start=self._pose(start), goal=self._pose(end), tolerance=0.)
        outcome, ready = [], threading.Event()
        def query():
            try:
                outcome.append(rospy.ServiceProxy('/move_base/make_plan', GetPlan)(request).plan)
            except Exception as exc:
                outcome.append(exc)
            finally:
                ready.set()
        threading.Thread(target=query, daemon=True).start()
        deadline = time.monotonic()+2.
        while not ready.wait(.05):
            self._assert_running()
            if time.monotonic() > deadline:
                raise RuntimeError('路径规划查询超时')
        if isinstance(outcome[0], Exception):
            raise RuntimeError(str(outcome[0]))
        plan = outcome[0]
        # move_base/GetPlan may leave Path.header empty while each returned
        # PoseStamped explicitly names the global frame. Validate those poses
        # before filling the container header; never guess an unknown frame.
        if plan.header.frame_id not in ('', self.map_frame) or len(plan.poses) < 2:
            return None
        poses = [(p.pose.position.x, p.pose.position.y) for p in plan.poses]
        if (any(p.header.frame_id != self.map_frame for p in plan.poses)
                or not all(math.isfinite(v) for xy in poses for v in xy)
                or math.hypot(poses[0][0]-start[0], poses[0][1]-start[1]) > .3
                or math.hypot(poses[-1][0]-end[0], poses[-1][1]-end[1]) > .15):
            return None
        if not plan.header.frame_id:
            plan.header.frame_id = self.map_frame
            plan.header.stamp = plan.poses[0].header.stamp
        return plan

    def _sweep_free(self, start, end):
        with self.lock:
            if (self.grid is None or self.grid.header.frame_id != self.map_frame
                    or time.monotonic()-self.grid_receipt > 3.):
                raise RuntimeError('等待导航代价地图')
            grid = copy.deepcopy(self.grid)
        shape = rospy.get_param('/move_base/global_costmap/footprint', [])
        if isinstance(shape, str):
            shape = json.loads(shape)
        padding = float(rospy.get_param('/move_base/global_costmap/footprint_padding', .01))
        if not math.isfinite(padding) or padding < 0:
            raise RuntimeError('导航外廓参数无效')
        shape = [(float(x)+math.copysign(padding, x), float(y)+math.copysign(padding, y))
                 for x, y in shape]
        n = max(1, math.ceil(math.hypot(end[0]-start[0], end[1]-start[1]) /
                             max(.01, min(.05, grid.info.resolution/2))))
        return all(footprint_is_free(grid, (start[0]+(end[0]-start[0])*i/n,
                    start[1]+(end[1]-start[1])*i/n, start[2]), shape) for i in range(n+1))

    def _choose(self):
        robot = self._robot()
        with self.lock:
            targets = copy.deepcopy(self.targets.pending())
        targets = sorted((t for t in targets if t.identity not in self.skipped),
                         key=lambda t: math.hypot(t.x-robot[0], t.y-robot[1]))[:6]
        choices = []
        for target in targets:
            self._assert_running()
            best = None
            for start, end in sweep_candidates(robot, (target.x, target.y), self.offset):
                if not self._sweep_free(start, end):
                    continue
                transit = self._plan(robot, start)
                if transit is None:
                    continue
                sweep = self._plan(start, end)
                if sweep is None:
                    continue
                # A detour around the target is not a cleaning pass.
                points = [(p.pose.position.x, p.pose.position.y, start[2]) for p in sweep.poses]
                if not any(forward_crossing(a, b, (target.x, target.y), self.offset, self.tolerance)
                           for a, b in zip(points, points[1:])):
                    continue
                score = path_length([(p.pose.position.x, p.pose.position.y) for p in transit.poses])
                score += path_length(points)
                if best is None or score < best[0]:
                    best = (score, target, start, end, transit, sweep)
            if best:
                choices.append(best)
            else:
                self._skip(target, '未找到可达且无碰撞的接近及清扫路径')
        with self.lock:
            # Planning can take several camera frames. Refresh each candidate's
            # latest map position, retaining its identity even during a dropout.
            pending = {t.identity: t for t in self.targets.pending()}
            choices = [c for c in choices if c[1].identity in pending and
                       math.hypot(c[1].x-pending[c[1].identity].x,
                                  c[1].y-pending[c[1].identity].y) <= self.tolerance]
            self.route = [dict(id=c[1].identity, x=c[1].x, y=c[1].y, cost_m=c[0])
                          for c in sorted(choices, key=lambda c: c[0])]
        return min(choices, key=lambda c: c[0]) if choices else None

    def _skip(self, target, reason):
        with self.lock:
            self.skipped.add(target.identity)
            self.skip_reasons[target.identity] = reason
        rospy.loginfo('BEV target %d unresolved at (%.3f, %.3f): %s',
                      target.identity, target.x, target.y, reason)

    def _navigate(self, pose, target=None):
        self._assert_running()
        message = self._pose(pose)
        message.header.seq = int(uuid.uuid4().hex[:7], 16)+1
        previous, previous_at, crossed = None, None, False
        with self.lock:
            self.goal_id, self.expected = '', message
            self.goal_pub.publish(message)
        started = time.monotonic()
        while time.monotonic()-started < 120.:
            with self.lock:
                goal_id = self.goal_id
                status = self.action_states.get(goal_id)
            if goal_id:
                self._assert_running()
            elif self.stop.is_set() or self.shutdown.is_set() or rospy.is_shutdown():
                raise RuntimeError('视野清扫已停止')
            if not goal_id and time.monotonic()-started > 3.:
                raise RuntimeError('导航未接受本次清扫目标')
            if target is not None:
                current = self._robot()
                now = time.monotonic()
                if previous is not None and now-previous_at <= .3:
                    crossed |= forward_crossing(previous, current, (target.x, target.y),
                                                self.offset, self.tolerance)
                previous, previous_at = current, now
            if status == GoalStatus.SUCCEEDED:
                # Let the existing bridge retire retained-goal replay before
                # issuing the next waypoint (status subscriber ordering differs).
                with self.lock:
                    retired = (goal_id not in self.nav.get('active_ordinary_goal_ids', []) and
                               not self.nav.get('retained_reissue_allowed'))
                if retired:
                    return crossed
            if status in (GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                          GoalStatus.RECALLED, GoalStatus.LOST):
                raise RuntimeError('本次清扫导航结束，状态 {}'.format(status))
            self.stop.wait(.05)
        raise RuntimeError('本次清扫导航超时')

    def _reobserve(self, target):
        """Refine from fresh detections, otherwise keep the recorded map target."""
        arrived = time.time()
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            self._assert_running()
            with self.lock:
                matches = [t for t in self.targets.visible(time.time())
                           if t.observed >= arrived and
                           math.hypot(t.x-target.x, t.y-target.y) <= self.targets.radius]
                if matches:
                    return copy.deepcopy(min(matches, key=lambda t:
                        math.hypot(t.x-target.x, t.y-target.y)))
            self.stop.wait(.05)
        self._assert_running()
        with self.lock:
            remembered = self.targets.targets.get(target.identity)
            if remembered is None:
                raise RuntimeError('待扫目标记录已重置')
            self.reason = '目标暂时漏检，按最后记录的地图位置继续清扫'
            result = copy.deepcopy(remembered)
        rospy.loginfo('BEV target %d not redetected; using remembered map position (%.3f, %.3f), age %.1fs',
                      result.identity, result.x, result.y, time.time()-result.observed)
        return result

    def _run(self):
        final_state, final_reason = 'COMPLETE', '本轮记录的待扫目标已全部获得轨迹通过证据'
        try:
            for _ in range(100):
                self._assert_running()
                with self.lock:
                    self.state, self.reason = 'PLANNING', '从已记录的待扫目标中选择下一处垃圾'
                    self.selected = None
                choice = self._choose()
                if choice is None:
                    with self.lock:
                        remaining = [t for t in self.targets.pending()
                                     if t.identity not in self.skipped]
                    if remaining:
                        continue
                    if self.skipped:
                        final_reason = '本轮部分完成；仍有目标不可达或未获得清扫通过证据'
                    break
                _, target, start, end, transit, sweep = choice
                with self.lock:
                    self.selected = target
                    self.state, self.reason = 'APPROACHING', '接近下一处已记录的垃圾位置'
                rospy.loginfo('BEV round %s approaching target %d at (%.3f, %.3f), last seen %.1fs ago',
                              self.round_id, target.identity, target.x, target.y, time.time()-target.observed)
                display = Path()
                display.header = transit.header
                display.poses = transit.poses+sweep.poses
                self.path_pub.publish(display)
                self._navigate(start)
                self._assert_running()
                with self.lock:
                    self.state, self.reason = 'REOBSERVING', '接近后重新观察垃圾位置'
                current_target = self._reobserve(target)
                if math.hypot(current_target.x-target.x, current_target.y-target.y) > self.tolerance:
                    continue  # The next plan uses the newly observed location.
                target = current_target
                with self.lock:
                    self.selected = target
                if not self._sweep_free(start, end):
                    self._skip(target, '接近后清扫路径被代价地图阻挡')
                    continue
                with self.lock:
                    remembered = time.time()-target.observed > self.targets.ttl
                    self.state = 'SWEEPING'
                    self.reason = ('目标暂时漏检，按记录位置执行前进清扫段' if remembered else
                                   '沿前进路径经过垃圾，检查实际清扫轨迹')
                crossed = self._navigate(end, target)
                with self.lock:
                    if crossed:
                        self.targets.complete(target)
                        self.passed_count += 1
                        rospy.loginfo('BEV target %d trajectory passed; %d map targets remain',
                                      target.identity, len(self.targets.pending()))
                    else:
                        self._skip(target, '导航结束但实际刷子轨迹未通过目标')
                    self.goal_id, self.expected = '', None
            else:
                final_reason = '本轮已规划 100 次，仍有未处理目标'
        except Exception as exc:
            final_state, final_reason = ('CANCELED' if self.stop.is_set() else 'FAILED'), str(exc)
        finally:
            with self.lock:
                self._cancel_exact()
                if self.expected is not None:
                    self.retired_requests[self.expected.header.seq] = self.expected
                self.expected, self.goal_id = None, ''
                # At most 200 requests in a normal round; bound old tokens.
                if len(self.retired_requests) > 1024:
                    self.retired_requests.pop(next(iter(self.retired_requests)))
                self.unfinished = [dict(id=t.identity, x=t.x, y=t.y, label=t.label,
                    observed=t.observed, reason=self.skip_reasons.get(t.identity, final_reason))
                    for t in self.targets.pending()]
                if final_state == 'COMPLETE' and self.unfinished:
                    final_state = 'PARTIAL'
                self.state, self.reason = final_state, final_reason
                self.active, self.selected, self.route = False, None, []
                self.targets.end_round(time.time())
                rospy.loginfo('BEV round %s ended %s: passed=%d unresolved=%d reasons=%s',
                              self.round_id, final_state, self.passed_count,
                              len(self.unfinished), self.skip_reasons)
                # Retired request tokens still cancel a late bridge acceptance.
                self.stop.set()
            path = Path()
            path.header.frame_id = self.map_frame
            self.path_pub.publish(path)
            self._publish()

    def _publish(self, _event=None):
        with self.lock:
            t = self.selected
            frame = self.frame
            now = time.time()
            pending = [dict(id=p.identity, x=p.x, y=p.y, label=p.label, observed=p.observed)
                       for p in self.targets.pending()] if self.active else self.unfinished
            missing_count = sum(now-p['observed'] > self.targets.ttl for p in pending)
            reason = self.reason
            if self.active or pending:
                reason += '；待处理 {} 个（暂未检出 {} 个）'.format(len(pending), missing_count)
            try:
                self._navigation_available()
                navigation_available, navigation_detail = True, '现有导航可接收本任务'
            except RuntimeError as exc:
                navigation_available, navigation_detail = False, str(exc)
            payload = dict(state=self.state, reason=reason, active=self.active, round_id=self.round_id,
                camera_names=frame.get('configured_cameras', []) if frame else [],
                camera_count=len(frame.get('configured_cameras', [])) if frame else 0,
                navigation_available=navigation_available, navigation_detail=navigation_detail,
                perception=self.perception_reason, passed_count=self.passed_count,
                skipped_count=len(self.skipped), visible_count=len(self.targets.visible(time.time())),
                pending_count=len(pending), remembered_count=missing_count,
                pending_targets=pending, skipped_reasons=self.skip_reasons,
                selected=dict(id=t.identity, x=t.x, y=t.y, label=t.label) if t else None,
                route=self.route, goal_id=self.goal_id,
                rig_to_base_configured=bool(self.rig_pose),
                metric_accuracy_verified=False, pickup_verified=False,
                completion_meaning='实际前进清扫参考轨迹经过目标；拾取效果未验证',
                observations_epoch_s=frame['observed_epoch_s'] if frame else None,
                ground_objects=frame['ground_objects'] if frame and
                    time.time()-frame['observed_epoch_s'] <= 2. else [])
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))

    def _shutdown(self):
        self.shutdown.set()
        self.stop.set()
        with self.lock:
            self._cancel_exact()


if __name__ == '__main__':
    rospy.init_node('bev_cleaner')
    BevCleaner()
    rospy.spin()
