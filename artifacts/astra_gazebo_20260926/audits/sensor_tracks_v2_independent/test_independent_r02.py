"""Independent synthetic v2 review; no ROS or simulation evidence."""
from pathlib import Path
import sys,copy,math,dataclasses
import numpy as np
import pytest
from shapely.geometry import Polygon,Point
W=Path(__file__).resolve().parents[2]/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import sensor_tracks_v1 as old
import sensor_tracks_v2 as new
from route_sensor_buffer import ScanFrame
from route_candidate_core import FullFootprintEnvelope,RouteConfig
T=1_000_000_000
class Grid:
 image_sha256='a'*64;yaml_sha256='b'*64
 def clearance(self,p):return math.inf

def ready(delay=100_000_000):
 tr=new.SensorTracks(Grid());h=[]
 for i in range(6):
  ns=T+i*200_000_000;frame=ScanFrame(ns,np.ones(72),np.array([[2,0],[2.01,.02],[1.99,0]]),(0.,0.,0.));tr.accept(frame,tf_stamp_ns=ns,now_ns=ns+delay,odom_state={'stamp_ns':ns,'pose_map':(0,0,0),'body_twist':(0,0,0)});h.append((ns,frame))
 c=[{'candidate_id':'stop','states':[{'offset_ns':ns,'base_pose':(0,0,0),'rear_pose':(-.325,0,0),'body_twist':(0,0,0),'center_steering':0.}for ns in (0,1000000000)]}]
 return tr,h,c

@pytest.mark.parametrize('entry',['scalar','batch','request','tracker_scalar','tracker_batch'])
@pytest.mark.parametrize('delta',[-1,0,1])
def test_all_actual_public_apis_exact_acceptance_boundary(entry,delta):
 tr,h,c=ready();s=tr.snapshot;ns=s.accepted_now_ns+delta
 def f():
  return {'scalar':lambda:new.HardOccupancy(s,ns),'batch':lambda:new.BatchHardOccupancy(s,ns),'request':lambda:new.prediction_request(s,h,c,decision_ns=ns),'tracker_scalar':lambda:tr.occupancy(ns),'tracker_batch':lambda:tr.batch_occupancy(ns)}[entry]()
 if delta<0:
  with pytest.raises((ValueError,old.TrackerFault)):f()
 else:assert f()is not None

@pytest.mark.parametrize('entry',['scalar','batch','request','tracker_scalar','tracker_batch'])
def test_no_freshness_extension_after_delayed_acceptance(entry):
 for delta,ok in [(200000000,True),(200000001,False)]:
  tr,h,c=ready();s=tr.snapshot;ns=s.source_ns+delta
  f={'scalar':lambda:new.HardOccupancy(s,ns),'batch':lambda:new.BatchHardOccupancy(s,ns),'request':lambda:new.prediction_request(s,h,c,decision_ns=ns),'tracker_scalar':lambda:tr.occupancy(ns),'tracker_batch':lambda:tr.batch_occupancy(ns)}[entry]
  if ok:assert f()is not None
  else:
   with pytest.raises((ValueError,old.TrackerFault)):f()

def test_original_failure_reproduces_and_valid_request_entire_bytes_identical():
 tr,h,c=ready();s=tr.snapshot;assert old.prediction_request(s,h,c,decision_ns=s.source_ns)
 with pytest.raises(ValueError):new.prediction_request(s,h,c,decision_ns=s.source_ns)
 assert new.prediction_request(s,h,c,decision_ns=s.accepted_now_ns)==old.prediction_request(s,h,c,decision_ns=s.accepted_now_ns)
 assert new.SensorTracks.accept is old.SensorTracks.accept

@pytest.mark.parametrize('count',[31,32,33,2048])
def test_actual_full_footprint_dense_supports_exact_bruteforce(count):
 tr,h,c=ready(delay=0);s=tr.snapshot;rng=np.random.RandomState(count)
 # Synthetic snapshot with separately known arbitrary radii, lost anchor times,
 # baseline and tentative points: direct GEOS reference avoids v1 AABB pruning.
 tracks=tuple(old.Track(i+1,(i+1,),tuple(p),float(r),T, T,None,1, bool(i%2))for i,(p,r)in enumerate(zip(rng.uniform(-8,8,(20,2)),rng.uniform(.01,.3,20))))
 sparse=tuple(((i%64-32,i//64-16),T)for i in range(count));s=dataclasses.replace(s,tracks=tracks,sparse_supports=sparse,baseline_points=((0.,0.),(9998.,9998.)),tentative_map_points=((1.,0.),))
 urdf=W.parent/'runs/passive_sweep_q05_static/robot_description_runtime.xml';foot=FullFootprintEnvelope(urdf.read_text(),RouteConfig());fv=new.ConvexFootprint(foot.vertices);now=s.source_ns+100000000
 poses=rng.uniform([-10,-10,-math.pi],[10,10,math.pi],(67,3));poses[-1]=[9998,9998,.41];offsets=np.arange(67,dtype=np.int64)%9*1000000000
 got=new.BatchHardOccupancy(s,now).distance_lower_bounds(fv,poses,offsets);ref=[]
 for p,o in zip(poses,offsets):
  shape=foot.at(p);target=now+int(o);values=[shape.distance(Point(q))-1.35*(target-s.source_ns)/1e9 for q in s.baseline_points]+[shape.distance(Point(q))-.03-1.35*(target-s.source_ns)/1e9 for q in s.tentative_map_points]
  values += [shape.distance(Point(q.center))-q.radius_m-1.35*(target-q.anchor_ns)/1e9 for q in tracks]
  values += [shape.distance(Point((key[0]+.5)*.2,(key[1]+.5)*.2))-.2/math.sqrt(2)-1.35*(target-stamp)/1e9 for key,stamp in sparse];ref.append(min(values))
 ref=np.asarray(ref);assert np.all(got<=ref+1e-12);np.testing.assert_allclose(got,ref-1e-10,rtol=0,atol=2e-11)
 assert new.BatchHardOccupancy(s,now).support_count==20+count+3

@pytest.mark.parametrize('point',[(.5,0),(.500000000001,0),(0,0),(1,1)])
def test_touch_inside_tiny_outside_against_exact_geos(point):
 tr,h,c=ready(delay=0);s=dataclasses.replace(tr.snapshot,tracks=(),sparse_supports=(),tentative_map_points=(),baseline_points=(point,));foot=new.ConvexFootprint([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]);actual=new.BatchHardOccupancy(s,s.source_ns).distance_lower_bounds(foot,[[0,0,0]],[0])[0];expected=Polygon(foot.vertices).distance(Point(point));assert actual<=expected and abs(actual-(expected-1e-10))<1e-14
