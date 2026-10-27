#!/usr/bin/env python3
"""Forward only valid, fresh MID360 scans; never replace missing sensor data."""
import math
import threading
import time

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def validate_scan(scan, now, max_age=.5, future_tolerance=.2, frame='base_link'):
    if scan.header.frame_id.lstrip('/') != frame:
        return False, 'MID360 scan frame mismatch'
    if (not scan.ranges or not math.isfinite(scan.angle_increment)
            or scan.angle_increment <= 0 or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_max) or scan.angle_max <= scan.angle_min):
        return False, 'MID360 scan angular geometry is invalid'
    if (not math.isfinite(scan.range_min) or not math.isfinite(scan.range_max)
            or scan.range_min < 0 or scan.range_max <= scan.range_min):
        return False, 'MID360 scan range bounds are invalid'
    expected=(scan.angle_max-scan.angle_min)/scan.angle_increment
    if abs(expected-(len(scan.ranges)-1)) > 2:
        return False, 'MID360 scan sample count is inconsistent'
    stamp=scan.header.stamp.to_sec()
    if not math.isfinite(stamp) or stamp <= 0 or now-stamp > max_age:
        return False, 'MID360 scan timestamp is stale'
    if now-stamp < -future_tolerance:
        return False, 'MID360 scan timestamp is in the future'
    if any(math.isnan(r) or r == -math.inf or (math.isfinite(r) and r < 0)
           for r in scan.ranges):
        return False, 'MID360 scan contains invalid ranges'
    return True, 'fresh MID360-only scan'


class Mid360ScanGuard:
    def __init__(self):
        self.timeout=float(rospy.get_param('~scan_timeout',.5))
        self.future=float(rospy.get_param('~future_tolerance',.2))
        if not all(math.isfinite(v) and v > 0 for v in (self.timeout,self.future)):
            raise ValueError('MID360 scan time limits must be finite and positive')
        self.lock=threading.RLock();self.last=None;self.receipt=0.
        self.publisher=rospy.Publisher('/scan',LaserScan,queue_size=1)
        self.active=rospy.Publisher('/avoidance/mid360_active',Bool,queue_size=1,latch=True)
        self.mode=rospy.Publisher('/avoidance/source_mode',String,queue_size=1,latch=True)
        self.subscriber=rospy.Subscriber('/mid360/scan',LaserScan,self.callback,queue_size=5,tcp_nodelay=True)
        self.timer=rospy.Timer(rospy.Duration(.1),self.tick)
        self.active.publish(Bool(data=False));self.mode.publish(String(data='mid360'))

    def callback(self,scan):
        valid,detail=validate_scan(scan,rospy.Time.now().to_sec(),self.timeout,self.future)
        if not valid:
            # A rejected sample does not renew the last valid sample's lease.
            rospy.logwarn_throttle(2.,detail)
            return
        with self.lock:
            self.last=scan;self.receipt=time.monotonic()
            self.publisher.publish(scan)  # Preserve original header/stamp/ranges.
            self.active.publish(Bool(data=True));self.mode.publish(String(data='mid360'))

    def tick(self,_):
        with self.lock:
            fresh=bool(self.last is not None and time.monotonic()-self.receipt<=self.timeout)
            if fresh:fresh=validate_scan(self.last,rospy.Time.now().to_sec(),self.timeout,self.future)[0]
            self.active.publish(Bool(data=fresh));self.mode.publish(String(data='mid360'))


if __name__=='__main__':
    rospy.init_node('mid360_scan_guard')
    Mid360ScanGuard()
    rospy.spin()
