"""Independent synthetic fault/geometry fixtures; no ROS or new simulation evidence."""
from pathlib import Path
import copy,math,sys
import numpy as np
import pytest
from shapely.geometry import box,Point,MultiPoint,Polygon
from shapely.affinity import rotate,translate
W=Path(__file__).resolve().parents[2]/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
from sensor_tracks_v1 import TrackConfig,TrackerFault,nominal_cv
from sensor_tracks_v2 import SensorTracks,prediction_request
from route_sensor_buffer import ScanFrame
from common_route import SharedStaticMap
T=1_000_000_000
class Grid:
 image_sha256='a'*64;yaml_sha256='b'*64
 def __init__(self,shape=None):self.shape=shape
 def clearance(self,p):return math.inf if self.shape is None else self.shape.distance(Point(p))
def cloud(x,y=0):return [(x-.02,y),(x,y+.02),(x+.02,y)]
def frame(t,p=()):return ScanFrame(t,np.ones(72),np.array(p,dtype=float).reshape(-1,2),(0.,0.,0.))
def accept(tr,t,p=(),now=None,odom=None):return tr.accept(frame(t,p),tf_stamp_ns=t,now_ns=t if now is None else now,odom_state={'stamp_ns':t if odom is None else odom,'pose_map':(0.,0.,0.),'body_twist':(1.,.02,0.)})
def ready(delay=0):
 tr=SensorTracks(Grid());history=[]
 for i in range(6):
  t=T+200_000_000*i;f=frame(t,cloud(2+.1*i));tr.accept(f,tf_stamp_ns=t,now_ns=t+delay,odom_state={'stamp_ns':t,'pose_map':(0.,0.,0.),'body_twist':(1.,.02,0.)});history.append((t,f))
 return tr,history
def candidates():
 return [{'candidate_id':'first','states':[{'offset_ns':t,'base_pose':(x,0.,0.),'rear_pose':(x-.325,0.,0.),'body_twist':(1.,.02,0.),'center_steering':0.}for t,x in [(0,0.),(1_000_000_000,1.)]]}]
def direct(occ,shape,h):
 age=(occ.now_ns+h-occ.snapshot.source_ns)/1e9;values=[]
 if occ.baseline is not None:values.append(shape.distance(occ.baseline)-1.35*age)
 if occ.tentative is not None:values.append(shape.distance(occ.tentative)-1.35*age-occ.c.position_error_m)
 values += [shape.distance(p)-r-1.35*(occ.now_ns+h-t)/1e9 for p,r,t in occ.circles]
 return min(values,default=math.inf)

@pytest.mark.parametrize('source_delta,odom_delta,valid',[(0,0,True),(-200_000_000,0,True),(-200_000_001,0,False),(1,0,False),(0,1,False),(0,-200_000_000,True),(0,-200_000_001,False)])
def test_closed_integer_source_odom_boundaries(source_delta,odom_delta,valid):
 tr=SensorTracks(Grid());src=T+source_delta
 if valid:assert accept(tr,src,cloud(2),now=T,odom=src+odom_delta).source_ns==src
 else:
  with pytest.raises(ValueError):accept(tr,src,cloud(2),now=T,odom=src+odom_delta)
  assert tr.snapshot is None and tr.source_ns is None

def test_late_TF_rejected_input_never_updates_support_then_exact_source_retry():
 tr=SensorTracks(Grid());before=accept(tr,T,cloud(2));f=frame(T+100_000_000,cloud(8))
 with pytest.raises(ValueError):tr.accept(f,tf_stamp_ns=f.stamp_ns-1,odom_state={'stamp_ns':f.stamp_ns,'pose_map':(0,0,0),'body_twist':(0,0,0)},now_ns=T+150_000_000)
 assert tr.snapshot is before and tr.source_ns==T
 after=tr.accept(f,tf_stamp_ns=f.stamp_ns,odom_state={'stamp_ns':f.stamp_ns,'pose_map':(0,0,0),'body_twist':(0,0,0)},now_ns=T+200_000_000)
 assert after.source_ns==T+100_000_000 and tr.health(T+300_000_000)==[]
 assert tr.health(T+300_000_001)

def test_static_confirmation_transition_preserves_dynamically_seen_support():
 tr=SensorTracks(Grid(box(2.8,-1,4,1)));old=accept(tr,T,cloud(2.5)).tracks[0]
 for i in range(1,5):s=accept(tr,T+i*100_000_000,cloud(3))
 assert s.tracks==(old,)and not s.baseline_points and not s.tentative_map_points
 assert tr.occupancy(s.source_ns).distance_lower_bound(Point(old.center),0)<=-old.radius_m-.54

def test_ambiguous_loss_then_split_never_drops_prior_reachable_circles():
 tr=SensorTracks(Grid());old=accept(tr,T,cloud(2,-.32)+cloud(2,.32)).tracks
 merged=accept(tr,T+200_000_000,cloud(2,-.1)+cloud(2,.1)).tracks[0]
 assert merged.ambiguous and merged.member_ids==(1,2)
 blank=accept(tr,T+400_000_000,[]);assert blank.tracks==(merged,)
 split=accept(tr,T+600_000_000,cloud(1.7,-.35)+cloud(2.3,.35)).tracks[0]
 assert split.ambiguous and split.velocity_nominal_mps is None
 assert split.radius_m>=math.dist(split.center,merged.center)+merged.radius_m+1.35*.4-1e-12
 assert nominal_cv(tr.snapshot,2_000_000_000)[0]['nominal_center']is None

@pytest.mark.parametrize('count',[31,32,33,128])
def test_AABB_branch_exact_for_concave_rotated_inside_queries_and_negative_bounds(count):
 tr=SensorTracks(Grid());points=[((i%16)*.61,(i//16)*.61)for i in range(count)]
 accept(tr,T,points);accept(tr,T+100_000_000,[]);occ=tr.occupancy(T+150_000_000)
 concave=Polygon([(0,0),(2,0),(2,.2),(.2,.2),(.2,2),(0,2)])
 shapes=[Point(.1,.1),box(-.1,-.1,.3,.3),concave]
 shapes += [translate(rotate(concave,a),xoff=x,yoff=-.2)for a in (17,89,137)for x in(-2,0,4)]
 for shape in shapes:
  for h in(0,1,8000000000):assert math.isclose(occ.distance_lower_bound(shape,h),direct(occ,shape,h),abs_tol=1e-12,rel_tol=1e-13)

def test_real_shared_map_exact_legacy_floor_over_changing_observations():
 bundle=W.parent/'scenes/navigation_n01_clear_r01/common_bundle'
 if not bundle.exists():
  bundle=next((W.parent/'scenes').glob('navigation_n01*/common_bundle'))
 grid=SharedStaticMap((bundle/'map.pgm').read_bytes(),(bundle/'map.yaml').read_bytes());tr=SensorTracks(grid)
 rng=np.random.RandomState(260926)
 for i in range(8):
  points=rng.uniform([-2.,-2.],[6.,2.],size=(60,2));s=accept(tr,T+i*100_000_000,points);occ=tr.occupancy(s.source_ns+100_000_000)
  dynamic=[p for p in points if grid.clearance(p)>.15];baseline=MultiPoint(dynamic)if dynamic else None
  for x in(-3.,0.,3.,6.):
   shape=rotate(box(x-.5,-.35,x+.5,.35),i*13,origin=(x,0))
   for h in(0,1_000_000_000,8_000_000_000):
    old=math.inf if baseline is None else shape.distance(baseline)-1.35*(.1+h/1e9)
    assert occ.distance_lower_bound(shape,h)<=old+1e-12

@pytest.mark.parametrize('mutation',['future_source','fake_same_stamp_ranges','fake_same_stamp_points','extra_oracle_candidate_field','extra_oracle_state_field','future_bucket','duplicate_history'])
def test_prediction_request_causal_hash_and_closed_candidate_fields(mutation):
 tr,h=ready();c=candidates()
 if mutation=='future_source':h[-1]=(h[-1][0],frame(h[-1][1].stamp_ns+1,cloud(9)))
 elif mutation=='fake_same_stamp_ranges':h[-1][1].model_ranges[0]=8.
 elif mutation=='fake_same_stamp_points':h[-1][1].points_map[0,0]=8.
 elif mutation=='extra_oracle_candidate_field':c[0]['actor_truth']=[1,2]
 elif mutation=='extra_oracle_state_field':c[0]['states'][0]['future_actor_velocity']=1.
 elif mutation=='future_bucket':h[-1]=(h[-1][0]+1,h[-1][1])
 elif mutation=='duplicate_history':h[-1]=h[-2]
 with pytest.raises(ValueError):prediction_request(tr.snapshot,h,c,decision_ns=tr.source_ns)

def test_candidate_change_changes_hash_and_no_learned_or_hard_replacement_claim():
 tr,h=ready();c=candidates();a=prediction_request(tr.snapshot,h,c,decision_ns=tr.source_ns);d=copy.deepcopy(c);d[0]['states'][-1].update(base_pose=(2.,0.,0.),rear_pose=(1.675,0.,0.));b=prediction_request(tr.snapshot,h,d,decision_ns=tr.source_ns)
 assert a['request_sha256']!=b['request_sha256']and a['source_cutoff_ns']==b['source_cutoff_ns']==tr.source_ns
 assert not a['learned_interaction']and not a['hard_occupancy_replacement_authorized']and a['sensor_odometry']['body_twist'][1]==.02

def test_saved_snapshot_cannot_be_consumed_before_its_acceptance_time():
 tr,h=ready(delay=100_000_000)
 # Source-time causality alone passes, but the accepted snapshot did not yet
 # exist at this earlier decision: an asynchronous caller must not backdate it.
 with pytest.raises(ValueError):prediction_request(tr.snapshot,h,candidates(),decision_ns=tr.source_ns)
