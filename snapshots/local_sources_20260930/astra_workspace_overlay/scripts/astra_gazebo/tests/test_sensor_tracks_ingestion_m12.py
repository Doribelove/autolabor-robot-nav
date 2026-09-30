"""M12 exact-neighborhood and accepted-state parity, no ROS or learning."""
from pathlib import Path
from dataclasses import asdict
import ast
import hashlib
import math
import sys
import numpy as np
import pytest

P = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(P))
import sensor_tracks_ingestion_m08 as old
import sensor_tracks_ingestion_m12 as new
from sensor_tracks_v1 import TrackConfig
from route_sensor_buffer import ScanFrame
from test_common_route import make_grid


def normalized(snapshot):
    if snapshot is None:return None
    value=asdict(snapshot)
    value['statistics']=[(k,v) for k,v in value['statistics'] if k!='processing_ns']
    return value


def graph_reference(points,gap):
    """Independent all-pairs adjacency/union-find, no KD or production DFS."""
    parent=list(range(len(points)))
    def root(i):
        while parent[i]!=i:i=parent[i]
        return i
    for i in range(len(points)):
        for j in range(i):
            if np.linalg.norm(points[i]-points[j])<=gap:
                a,b=root(i),root(j);parent[max(a,b)]=min(a,b)
    groups={}
    for i in range(len(points)):groups.setdefault(root(i),[]).append(i)
    return [points[ix] for _,ix in sorted(groups.items())]


def same_clusters(points,gap=.3):
    points=np.asarray(points,dtype=float).reshape(-1,2);before=points.tobytes()
    values=[old._clusters(points,gap),new._clusters(points,gap),graph_reference(points,gap)]
    for value in values[1:]:
        assert len(value)==len(values[0])
        for a,b in zip(values[0],value):assert a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()
    assert points.tobytes()==before


def test_accept_body_is_exact_frozen_bytes_and_no_old_global_change():
    raw=(P/'sensor_tracks_ingestion_m08.py').read_bytes()
    assert hashlib.sha256(raw).hexdigest()=='97b739bb8b165a652e44e2695a2298bf5fa0ea87ba3e80a67a40f4b39644ffc2'
    code=raw.decode();candidate=(P/'sensor_tracks_ingestion_m12.py').read_text()
    assert code[code.index('    def accept(self,'):]==candidate[candidate.index('    def accept(self,'):]
    assert new.IngestionSensorTracksM12.accept.__globals__ is not old.IngestionSensorTracks.accept.__globals__
    assert old.IngestionSensorTracks.accept.__globals__['_clusters'] is old._clusters
    # Normalize precisely the two retrieval changes; all traversal/predicate AST remains identical.
    def cluster_ast(text,is_new):
        f=next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name=='_clusters')
        if is_new:
            f.body=[n for n in f.body if not(isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='neighbors')]
            for n in ast.walk(f):
                if isinstance(n,ast.For) and isinstance(n.target,ast.Name) and n.target.id=='j':
                    n.iter=ast.parse('tree.query_ball_point(points[i],gap+1e-9,eps=0.,return_sorted=True)',mode='eval').body
        return ast.dump(f,include_attributes=False)
    assert cluster_ast(code,False)==cluster_ast(candidate,True)


@pytest.mark.parametrize('points',[[],[(1.,2.)],[(1.,2.)]*5,[(i*.5,0.) for i in range(70)],[(i*.25,0.) for i in range(70)],[(i%8*.1,i//8*.1) for i in range(64)]])
def test_empty_single_duplicate_sparse_chain_dense(points):same_clusters(points)


@pytest.mark.parametrize('origin',[(0.,0.),(9999.,-9999.),(-9999.,9999.)])
def test_inclusive_gap_adjacent_floats_and_broad_phase_boundaries(origin):
    for radius in (.3,np.nextafter(.3,0.),np.nextafter(.3,math.inf),.3+1e-12,.3+1e-9,np.nextafter(.3+1e-9,0.),np.nextafter(.3+1e-9,math.inf)):
        for angle in (0.,math.pi/4,math.pi/2):
            p=np.array(origin);q=p+radius*np.array([math.cos(angle),math.sin(angle)])
            same_clusters([p,q,p])


def test_seeded_reference_and_interleaved_components():
    rng=np.random.default_rng(125032)
    for _ in range(24):
        points=rng.uniform(-2,2,(65,2));same_clusters(points)
    # Spatial components interleaved through row/tile boundaries; cache refetches are allowed.
    points=np.array([(i*.1,j*2.) for i in range(12) for j in range(9)])
    same_clusters(points);same_clusters(points[rng.permutation(len(points))])


def test_bulk_rows_exactly_equal_scalar_rows_on_pinned_backend():
    rng=np.random.default_rng(120032)
    points=np.vstack([rng.uniform(-1,1,(129,2)),[[0,0],[.3,0],[np.nextafter(.3,0.),0],[np.nextafter(.3,math.inf),0]]])
    tree=old.cKDTree(points);cache=new._LazyNeighbors32(tree,points,.3+1e-9)
    for i in list(range(len(points)))+list(rng.permutation(len(points)))+list(range(len(points)-1,-1,-1)):
        assert cache(i)==tree.query_ball_point(points[i],.3+1e-9,eps=0.,return_sorted=True)


def test_one_tile_cache_and_discard_before_failed_refetch():
    points=np.zeros((1440,2));calls=[];holder={}
    class Tree:
        fail=False
        def query_ball_point(self,values,radius,**kwargs):
            assert holder['cache'].rows is None
            calls.append((len(values),radius,kwargs))
            if self.fail:raise RuntimeError('synthetic native failure')
            return np.array([list(range(1440)) for _ in values],dtype=object)
    tree=Tree();cache=new._LazyNeighbors32(tree,points,.300000001);holder['cache']=cache
    cache(0);cache(31)
    assert len(calls)==1 and len(cache.rows)==32 and sum(map(len,cache.rows))==46080
    cache(1439);assert len(calls)==2 and len(cache.rows)==32
    cache(0);assert len(calls)==3
    assert all(c[2]=={'eps':0.,'workers':1,'return_sorted':True} for c in calls)
    tree.fail=True
    with pytest.raises(RuntimeError,match='native failure'):cache(32)
    assert cache.rows is None and cache.start==-1


def pair():
    grid,_,_=make_grid(size=400,obstacles=[])
    config=TrackConfig(maximum_episode_span_ns=40_000_000_000)
    return [old.IngestionSensorTracks(grid,config),new.IngestionSensorTracksM12(grid,config)]


def accept(tracker,stamp,points,now=None,tf=None):
    points=np.asarray(points,dtype=float).reshape(-1,2)
    f=ScanFrame(stamp,np.full(72,8.),points,(3.,7.,0.))
    return tracker.accept(f,tf_stamp_ns=stamp if tf is None else tf,
          odom_state={'stamp_ns':stamp,'pose_map':(3.,7.,0.),'body_twist':(.8,.1,.2)},now_ns=stamp if now is None else now)


def test_accept_history_association_ambiguity_and_lost_support_exact():
    trackers=pair();a=np.array([[5.,5.],[5.02,5.],[5.01,5.02]])
    frames=[np.vstack([a,a+[.6,0],[[3.,4.]]]),np.vstack([a+[.1,0],a+[.5,0],[[3.,4.]]]),
            np.vstack([a+[.3,0],[[3.,4.]]]),[],a+[.4,0]]
    for i,points in enumerate(frames):
        states=[accept(t,2_000_000_000+i*100_000_000,points) for t in trackers]
        assert normalized(states[0])==normalized(states[1])
        assert trackers[0].map_streak==trackers[1].map_streak
    assert trackers[0].snapshot.sparse_supports
    assert any(t.ambiguous for t in trackers[0].tracks)


@pytest.mark.parametrize('fault',['future','stale','wrong_tf','replay','nonfinite','capacity'])
def test_validation_and_commit_failure_identical(fault):
    trackers=pair()
    for t in trackers:accept(t,2_000_000_000,[[3.,4.]])
    before=[normalized(t.snapshot) for t in trackers]
    kwargs={'stamp':2_100_000_000,'points':[[3.,4.]]}
    if fault=='future':kwargs['now']=2_099_999_999
    if fault=='stale':kwargs['now']=2_300_000_001
    if fault=='wrong_tf':kwargs['tf']=2_100_000_001
    if fault=='replay':kwargs['stamp']=2_000_000_000
    if fault=='nonfinite':kwargs['points']=[[float('nan'),1.]]
    if fault=='capacity':kwargs['points']=np.zeros((1441,2))
    errors=[]
    for t in trackers:
        try:accept(t,**kwargs)
        except Exception as e:errors.append((type(e).__name__,str(e)))
    assert len(errors)==2 and errors[0]==errors[1]
    assert [normalized(t.snapshot) for t in trackers]==before
