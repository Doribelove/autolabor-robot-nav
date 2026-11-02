#!/usr/bin/env python3
"""Generate synthetic TF + decoded centers to test ROS plumbing, NOT accuracy."""
import argparse,json,math
from pathlib import Path
import rospy,rosbag
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
p=argparse.ArgumentParser();p.add_argument('--bag',required=True);a=p.parse_args()
Path(a.bag).parent.mkdir(parents=True,exist_ok=True)
with rosbag.Bag(a.bag,'w') as b:
 for i in range(20):
  stamp=rospy.Time.from_sec(100+i*.2);yaw=i*.03;x=i*.025;y=-i*.015
  t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='map';t.child_frame_id='base_link';t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2)
  b.write('/tf',TFMessage([t]),stamp)
  # One fixed map object (2,1), seen from moving/rotating base; no camera model.
  dx,dy=2-x,1-y;bx=math.cos(yaw)*dx+math.sin(yaw)*dy;by=-math.sin(yaw)*dx+math.cos(yaw)*dy
  ns=stamp.to_nsec();d=dict(stamp_ns=ns,frame_id='base_link',source_stamps_ns={n:ns for n in ('front','left','right','rear')},clock_domain='ros_epoch_ns',clock_verified=True,position_std_m=.05,centers=[dict(x=bx,y=by,confidence=.9)],model_bpu_ms=None,fixture=True)
  b.write('/fod/bev/decoded',String(data=json.dumps(d)),stamp+rospy.Duration(.03))
print(a.bag)
