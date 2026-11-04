"""Optional reverse-and-look demonstration, independent of normal recovery.

PASSED means no stable detection in the inspected image band, not a bin sensor.
All times are monotonic or source image/odometry stamps, never loop counts.
"""
import math


TERMINAL = {"PASSED", "FOD_REMAINS", "INCONCLUSIVE", "CANCELED"}


def rear_corridor_clear(scan, footprint, lookahead=.35):
    """Check the swept body rectangle, using the existing base_link scan."""
    if scan.header.frame_id.lstrip('/') != 'base_link' or not scan.ranges:
        return False, '复查倒退缺少 base_link 雷达扫描'
    xmin = min(p[0] for p in footprint)
    xmax = max(p[0] for p in footprint)
    ymin = min(p[1] for p in footprint) - .05
    ymax = max(p[1] for p in footprint) + .05
    rear_samples = 0
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        if math.cos(angle) < -.5 and (scan.range_min <= distance <= scan.range_max or distance == math.inf):
            rear_samples += 1
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        if xmin-lookahead <= x <= xmax and ymin <= y <= ymax:
            return False, '复查倒退路径有雷达障碍，保持停车'
    return (True, '') if rear_samples else (False, '复查倒退缺少后方雷达数据')


def overlap(a, b):
    x1, y1 = max(a.x, b.x), max(a.y, b.y)
    x2, y2 = min(a.x+a.width, b.x+b.width), min(a.y+a.height, b.y+b.height)
    intersection = max(0., x2-x1)*max(0., y2-y1)
    union = a.width*a.height+b.width*b.height-intersection
    return intersection/union if union > 0 else 0.


class RecoveryVerification:
    distance_m = 1.5
    speed_mps = .25
    stable_sec = .3
    observe_sec = 2.0
    frame_gap_sec = .3

    def __init__(self):
        self.phase = 'IDLE'
        self.reason = '垃圾回收判定演示未启动'
        self.progress_m = 0.
        self.peak_stable_sec = 0.
        self.stable_track_id = None
        self.frame_count = 0
        self.image_count = 0
        self.first_frame = self.last_frame = None
        self.last_frame_receipt = None
        self.last_image = self.last_image_receipt = None
        self.image_error = None
        self.tracks = []
        self.observe_started = None
        self.observe_ros_started = None
        self.start_pose = self.previous_pose = None
        self.reverse_started = None
        self.stopped_since = None

    def finish(self, phase, reason):
        self.phase, self.reason = phase, reason
        return 0.

    def prepare(self):
        self.phase = 'PREPARING'
        self.reason = '原回收行程已结束；停车、停清扫并收杆后倒退复查'

    def begin_reverse(self, pose, now):
        self.start_pose = self.previous_pose = pose
        self.reverse_started = now
        self.phase, self.reason = 'REVERSING', f'正在倒退 {self.distance_m:g} 米，完成后停车观察'

    def motion(self, pose, speed, now, ros_now, stopped_threshold=.03, stop_confirm=.5):
        if self.phase not in ('REVERSING', 'STOPPING', 'OBSERVING'):
            return 0.
        if pose is None or speed is None or not all(math.isfinite(v) for v in (*pose[:3], pose[4], speed)):
            return self.finish('INCONCLUSIVE', '倒退复查里程计无效，无法判定')
        if pose[3] != self.start_pose[3]:
            return self.finish('INCONCLUSIVE', '倒退复查里程计坐标系变化')
        previous = self.previous_pose
        dt = pose[4]-previous[4]
        step = math.hypot(pose[0]-previous[0], pose[1]-previous[1])
        if dt < 0 or step > max(.10, max(0., dt)*.7):
            return self.finish('INCONCLUSIVE', f'倒退复查里程计跳变，不能用跳变充当 {self.distance_m:g} 米位移')
        self.previous_pose = pose
        dx, dy = pose[0]-self.start_pose[0], pose[1]-self.start_pose[1]
        yaw = self.start_pose[2]
        self.progress_m = -(dx*math.cos(yaw)+dy*math.sin(yaw))
        lateral = abs(-dx*math.sin(yaw)+dy*math.cos(yaw))
        yaw_error = abs(math.atan2(math.sin(pose[2]-yaw), math.cos(pose[2]-yaw)))
        if lateral > .20 or yaw_error > math.radians(20):
            return self.finish('INCONCLUSIVE', '倒退偏离原方向，无法确认原区域已重新进入视野')
        if self.phase == 'REVERSING':
            if now-self.reverse_started > 15.:
                return self.finish('INCONCLUSIVE', f'倒退未完成 {self.distance_m:g} 米，复查结束且保持停车')
            if self.progress_m >= self.distance_m-.015:
                self.phase, self.reason = 'STOPPING', '倒退到位，确认停车后检查相机'
                return 0.
            # Reduce speed near the endpoint without falling below M2's ~0.19 m/s deadband.
            return -.20 if self.distance_m-self.progress_m < .20 else -self.speed_mps
        if self.phase == 'STOPPING':
            if now-self.reverse_started > 18.:
                return self.finish('INCONCLUSIVE', '倒退后未确认停车，无法判定')
            if abs(speed) > stopped_threshold:
                self.stopped_since = None
            elif self.stopped_since is None:
                self.stopped_since = now
            elif now-self.stopped_since >= stop_confirm:
                self.phase, self.reason = 'OBSERVING', '停车观察 2 秒：检查连续至少 0.3 秒的 FOD'
                self.observe_started, self.observe_ros_started = now, ros_now
            return 0.
        if abs(speed) > stopped_threshold:
            return self.finish('INCONCLUSIVE', '复查观察期间车辆移动，无法判定')
        elapsed = now-self.observe_started
        if elapsed < self.observe_sec:
            return 0.
        if (self.image_error or self.frame_count < 3 or self.image_count < 3
                or self.first_frame is None or self.last_frame-self.first_frame < self.observe_sec
                or now-self.last_frame_receipt > self.frame_gap_sec
                or now-self.last_image_receipt > self.frame_gap_sec):
            if elapsed < self.observe_sec+1. and not self.image_error:
                return 0.
            return self.finish('INCONCLUSIVE', self.image_error or '有效相机/检测数据未覆盖完整观察窗口，无法判定')
        if self.peak_stable_sec + 1e-8 >= self.stable_sec:
            return self.finish('FOD_REMAINS', '发现稳定至少 0.3 秒的 FOD，演示判定存在残留')
        return self.finish('PASSED', f'倒退 {self.distance_m:g} 米后未再检出稳定 FOD，演示判定回收完成（非入仓证明）')

    def image(self, stamp, now, valid=True):
        if self.phase != 'OBSERVING' or stamp <= self.observe_ros_started:
            return
        if self.last_image is not None and stamp <= self.last_image:
            return
        if not valid:
            self.image_error = '复查相机画面无效或全黑/全白，无法判定'
            return
        if self.last_image is not None and stamp-self.last_image > self.frame_gap_sec:
            self.image_error = '复查相机画面断流，无法判定'
        self.last_image, self.last_image_receipt = stamp, now
        self.image_count += 1

    def detections(self, stamp, now, detections):
        if self.phase != 'OBSERVING' or stamp <= self.observe_ros_started:
            return
        if self.last_frame is not None and stamp <= self.last_frame:
            return  # Replayed/held frames cannot accumulate 0.3 seconds.
        if self.last_frame is not None and stamp-self.last_frame > self.frame_gap_sec:
            self.image_error = '复查检测数据断流，无法判定'
            self.tracks = []
        if self.first_frame is None:
            self.first_frame = stamp
        updated, used = [], set()
        for detection in detections:
            candidates = [(overlap(detection, old), i, since) for i, (old, since) in enumerate(self.tracks)
                          if i not in used and old.class_name == detection.class_name]
            matched = max(candidates, default=(0., -1, stamp))
            if matched[0] >= .25:
                used.add(matched[1]); since = matched[2]
            else:
                since = stamp
            duration = stamp-since
            if duration >= self.peak_stable_sec:
                self.peak_stable_sec, self.stable_track_id = duration, detection.track_id
            updated.append((detection, since))
        self.tracks = updated
        self.last_frame, self.last_frame_receipt = stamp, now
        self.frame_count += 1

    def status(self):
        return dict(phase=self.phase, reason=self.reason, reverse_distance_m=self.distance_m,
                    reverse_speed_mps=self.speed_mps, reverse_progress_m=self.progress_m,
                    observe_sec=self.observe_sec, stable_threshold_sec=self.stable_sec,
                    max_stable_sec=self.peak_stable_sec, stable_track_id=self.stable_track_id,
                    detection_frames=self.frame_count, image_frames=self.image_count,
                    result=self.phase if self.phase in TERMINAL else None,
                    meaning='camera reinspection demonstration; not physical bin confirmation')
