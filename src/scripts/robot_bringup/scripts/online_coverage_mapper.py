#!/usr/bin/env python3
"""Publish the existing MID360 occupancy mapper incrementally in this SLAM session."""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import String

from fused_scan_mapper import SparseOccupancyMapper, yaw_from_quaternion


def occupancy_snapshot(mapper, padding_m=1.0, flat_region=None):
    """Export only observed evidence; unobserved/uncertain cells stay unknown."""
    if not mapper.cells:
        return None
    keys=np.asarray(list(mapper.cells),dtype=np.int32)
    if flat_region is not None:
        keys=np.vstack((keys,np.floor(np.asarray(flat_region)/mapper.resolution).astype(np.int32)))
    padding=int(math.ceil(padding_m/mapper.resolution))
    lo=keys.min(axis=0)-padding;hi=keys.max(axis=0)+padding
    width,height=(hi-lo+1).tolist()
    data=np.full((height,width),-1,dtype=np.int8)
    for (x,y),value in mapper.cells.items():
        if value<=math.log(.35/.65):data[y-lo[1],x-lo[0]]=0
        elif value>=math.log(.65/.35) and mapper.occupied_observations[(x,y)]>=mapper.min_occupied_observations:
            data[y-lo[1],x-lo[0]]=100
    if flat_region is not None:
        # Only the user-declared flat rectangle supplies this free-space prior.
        # Positive obstacle evidence (including not-yet-persistent hits) wins.
        yy,xx=np.indices(data.shape)
        xx=(xx+lo[0]+.5)*mapper.resolution;yy=(yy+lo[1]+.5)*mapper.resolution
        inside=np.ones(data.shape,dtype=bool)
        for a,b in zip(flat_region,np.roll(flat_region,-1,axis=0)):
            inside &= ((b[0]-a[0])*(yy-a[1])-(b[1]-a[1])*(xx-a[0]))>=-1e-9
        prior=inside & (data==-1)
        for (x,y),value in mapper.cells.items():
            if value>0:prior[y-lo[1],x-lo[0]]=False
        data[prior]=0
    return width,height,float(lo[0]*mapper.resolution),float(lo[1]*mapper.resolution),data


class OnlineMapper:
    def __init__(self, output):
        self.output=output;output.mkdir(parents=True,exist_ok=True)
        self.mapper=SparseOccupancyMapper(beam_stride=2,scan_stride=2,max_range=30.,
            free_space_range=8.,base_offset_x=0.,base_offset_y=0.,
            self_crop=(-.52,1.22,-.4,.4))
        self.lock=threading.Lock();self.last_stamp=rospy.Time(0);self.last_save=0.;self.flat_region=None
        self.tf=tf2_ros.Buffer(rospy.Duration(10));self.listener=tf2_ros.TransformListener(self.tf)
        self.publisher=rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True)
        self.status=rospy.Publisher('/online_mapping/status',String,queue_size=1,latch=True)
        self.subscriber=rospy.Subscriber('/mid360/scan',LaserScan,self.scan,queue_size=2)
        self.region_subscriber=rospy.Subscriber('/online_mapping/confirmed_flat_region',PolygonStamped,self.region,queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(1),self.publish)
        rospy.on_shutdown(self.save)

    def region(self, msg):
        if msg.header.frame_id!='map' or len(msg.polygon.points)!=4:return
        points=np.asarray([[p.x,p.y] for p in msg.polygon.points],dtype=float)
        if not np.isfinite(points).all():return
        edges=np.roll(points,-1,axis=0)-points
        if not all(np.cross(edges[i],edges[(i+1)%4])>0 for i in range(4)):return
        with self.lock:self.flat_region=points
        (self.output/'flat_region_prior.json').write_text(json.dumps(dict(
            source='user confirmed flat ground',frame_id='map',vertices=points.tolist(),
            stamp=msg.header.stamp.to_sec(),obstacles_override_prior=True),indent=2)+'\n')

    def scan(self, msg):
        if msg.header.frame_id.lstrip('/')!='base_link':return
        try:
            transform=self.tf.lookup_transform('map','base_link',msg.header.stamp,rospy.Duration(.15))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(5,'online mapper waiting for scan-time pose: %s',exc)
            return
        p=transform.transform.translation;q=transform.transform.rotation
        if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)):return
        with self.lock:
            if self.mapper.integrate_scan(msg,p.x,p.y,yaw_from_quaternion(q)):
                self.last_stamp=msg.header.stamp

    def publish(self, _event):
        with self.lock:
            snapshot=occupancy_snapshot(self.mapper,flat_region=self.flat_region)
            stamp=self.last_stamp;count=self.mapper.integrated_scans
        if snapshot is None:return
        width,height,x,y,data=snapshot
        msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=stamp
        msg.info.resolution=self.mapper.resolution;msg.info.width=width;msg.info.height=height
        msg.info.origin.position.x=x;msg.info.origin.position.y=y;msg.info.origin.orientation.w=1.
        msg.data=data.ravel().tolist();self.publisher.publish(msg)
        detail=dict(mode='online_fastlio',fixed_frame='map',origin='current camera_init',
            integrated_scans=count,width=width,height=height,
            free_cells=int((data==0).sum()),occupied_cells=int((data==100).sum()),
            unknown_cells=int((data==-1).sum()),scan_stamp=stamp.to_sec(),
            no_return_clearing_range_m=self.mapper.free_space_range,
            user_confirmed_flat_region=self.flat_region.tolist() if self.flat_region is not None else None,
            known_prior_loaded=False)
        self.status.publish(String(data=json.dumps(detail)))
        tmp=self.output/'status.json.tmp';tmp.write_text(json.dumps(detail,indent=2)+'\n');tmp.replace(self.output/'status.json')
        if time.monotonic()-self.last_save>=5:
            self.save();self.last_save=time.monotonic()

    def save(self):
        with self.lock:
            if self.mapper.integrated_scans:
                import yaml
                self.mapper.save(str(self.output/'observed_map'))
                width,height,x,y,data=occupancy_snapshot(self.mapper,flat_region=self.flat_region)
                dest=self.output/'map_fused_2d';dest.mkdir(exist_ok=True)
                pixels=np.where(data<0,205,np.where(data==0,254,0)).astype(np.uint8)
                tmp=dest/'map.pgm.tmp';tmp.write_bytes(('P5\n%d %d\n255\n'%(width,height)).encode()+pixels[::-1].tobytes());tmp.replace(dest/'map.pgm')
                tmp=dest/'map.yaml.tmp';tmp.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=self.mapper.resolution,origin=[x,y,0.],negate=0,occupied_thresh=.65,free_thresh=.196)));tmp.replace(dest/'map.yaml')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',required=True)
    args=parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('online_coverage_mapper')
    OnlineMapper(Path(args.output_dir));rospy.spin()
