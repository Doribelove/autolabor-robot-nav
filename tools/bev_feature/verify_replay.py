#!/usr/bin/env python3
import argparse,json,subprocess,time
from pathlib import Path
import rospy
from autolabor_fod_msgs.msg import FodGroundObservationArray
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--output',required=True);a=p.parse_args()
rospy.init_node('verify_bev_replay',anonymous=True)
points=[];markers=[];lat=[]
rospy.Subscriber('/fod/bev/map_observations',FodGroundObservationArray,lambda m:points.append(m),queue_size=100)
rospy.Subscriber('/fod/bev/markers',MarkerArray,lambda m:markers.append(m),queue_size=100)
rospy.Subscriber('/fod/bev/latency',String,lambda m:lat.append(json.loads(m.data)),queue_size=100)
time.sleep(1)
p=subprocess.run(['rosbag','play','--clock','--delay=1','-r','0.8',a.bag],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=30,check=True)
time.sleep(1)
before=len(points)
pub=rospy.Publisher('/fod/bev/decoded',String,queue_size=10)
time.sleep(.5)
base=dict(stamp_ns=104000000000,frame_id='base_link',clock_domain='ros_epoch_ns',clock_verified=True,
          source_stamps_ns={n:104000000000 for n in ('front','left','right','rear')},
          position_std_m=.05,centers=[dict(x=2.,y=1.,confidence=.9)])
invalid=[dict(base,clock_verified=False),
         dict(base,source_stamps_ns=dict(base['source_stamps_ns'],front=103950000000)),
         dict(base,frame_id='missing_camera_frame'),
         dict(base,stamp_ns=100000000000,source_stamps_ns={n:100000000000 for n in base['source_stamps_ns']})]
for bad in invalid:
 pub.publish(String(data=json.dumps(bad)));time.sleep(.35)
rejected=len(points)==before
errors=[((o.point.x-2)**2+(o.point.y-1)**2)**.5 for m in points for o in m.observations]
ids={m.id for a in markers for m in a.markers if m.action==0};counts=[sum(m.action==0 for m in a.markers) for a in markers]
result=dict(fixture='synthetic decoded centers and synthetic moving TF, not camera/accuracy validation',messages=len(points),max_map_error_m=max(errors) if errors else None,track_ids=sorted(ids),max_markers=max(counts) if counts else None,invalid_clock_skew_tf_duplicate_rejected=rejected,latency=lat)
Path(a.output).write_text(json.dumps(result,indent=2))
print(result)
assert len(points)>=18 and max(errors)<1e-5 and len(ids)==1 and max(counts)==1 and rejected
