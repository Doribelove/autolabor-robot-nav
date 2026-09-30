"""Exact inherited acceptance and numeric connected-component parity, no ROS."""
from dataclasses import asdict
import inspect
from pathlib import Path
import sys
import numpy as np
import pytest
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
import sensor_tracks_v1 as old
from sensor_tracks_v2 import SensorTracks
from sensor_tracks_ingestion_m08 import IngestionSensorTracks,_clusters
from route_sensor_buffer import ScanFrame
from test_common_route import make_grid


def identical(points):
    a=old._clusters(points,.30);b=_clusters(points,.30)
    assert len(a)==len(b)
    for x,y in zip(a,b):np.testing.assert_array_equal(x,y)


def test_accept_body_is_byte_for_byte_frozen_parent_and_other_methods_are_v2():
    assert inspect.getsource(IngestionSensorTracks.accept)==inspect.getsource(old.SensorTracks.accept)
    for name in ('occupancy','batch_occupancy','health','_clock'):
        assert getattr(IngestionSensorTracks,name)is getattr(SensorTracks,name)


@pytest.mark.parametrize('offset',[0.,-9999.,9999.])
def test_inclusive_nextafter_boundary_cells_duplicates_and_chain(offset):
    for step in (np.nextafter(.3,0.),.3,np.nextafter(.3,1.),.3-1e-12,.3+1e-12):
        pts=np.asarray([(0,0),(step,0),(step*2,0),(step*2,step),(4,4),(4,4),(-step,0)])+offset
        identical(pts)


@pytest.mark.parametrize('shape',['dense_ring','coincident','sparse_lattice','two_crossing_clusters','near_cell_diagonal'])
def test_maximum_or_ambiguous_geometries(shape):
    if shape=='dense_ring':
        a=np.linspace(-np.pi,np.pi,1440);p=np.column_stack((3*np.cos(a),3*np.sin(a)))
    elif shape=='coincident':p=np.full((1440,2),3.)
    elif shape=='sparse_lattice':p=np.asarray([(i*.4,j*.4) for i in range(48) for j in range(30)])
    elif shape=='two_crossing_clusters':p=np.asarray([(i*.08,j*.5) for i in range(80) for j in (-1,1)])
    else:
        p=np.asarray([(i*.3+(-1)**i*1e-14,j*.3+(-1)**j*1e-14) for i in range(20) for j in range(20)])
    identical(p);identical(p[::-1].copy())


def test_complete_snapshot_static_ambiguity_sparse_lost_receipts_match_except_timing():
    grid,_,_=make_grid(size=140,obstacles=[(50,51,50,51)]);a=SensorTracks(grid);b=IngestionSensorTracks(grid)
    clouds=[[(3.,7.),(3.05,7.),(3.1,7.),(4.,7.),(4.05,7.),(4.1,7.),(8.,8.)],
        [(3.5,7.),(3.55,7.),(3.6,7.),(8.1,8.1),(5.05,5.05)],[(5.05,5.05)],[(5.05,5.05)],[]]
    for i,points in enumerate(clouds):
        t=1_000_000_000+i*200_000_000;f=ScanFrame(t,np.full(72,8.),np.asarray(points).reshape(-1,2),(0.,0.,0.))
        kwargs=dict(tf_stamp_ns=t,odom_state=dict(stamp_ns=t,pose_map=(0.,0.,0.),body_twist=(0.,0.,0.)),now_ns=t)
        x=asdict(a.accept(f,**kwargs));y=asdict(b.accept(f,**kwargs))
        for row in (x,y):row['statistics']=tuple((k,v) for k,v in row['statistics'] if k!='processing_ns')
        assert x==y
    assert b.snapshot.tracks and b.snapshot.sparse_supports and b.snapshot.baseline_points==()
    with pytest.raises((ValueError,RuntimeError)):b.batch_occupancy(t-1)


def test_query_dominance_proof_and_full_v2_parity_for_signed_inside_outside():
    from sensor_tracks_query_m08 import DominanceBatchHardOccupancy
    from sensor_tracks_v2 import BatchHardOccupancy,ConvexFootprint
    from test_route_tracks_runtime_m08 import runtime,URDF
    c=runtime();tracker=IngestionSensorTracks(c.grid)
    points=np.asarray([(3.+x*.4,4.+y*.4) for x in range(10) for y in range(10)])
    for t in (1_000_000_000,1_200_000_000):
        frame=ScanFrame(t,np.ones(72),points,(3.,7.,0.));tracker.accept(frame,tf_stamp_ns=t,odom_state=dict(stamp_ns=t,pose_map=(3.,7.,0.),body_twist=(.8,0,0)),now_ns=t)
    exact=BatchHardOccupancy(tracker.snapshot,t);fast=DominanceBatchHardOccupancy(tracker.snapshot,t)
    assert fast.support_count==exact.support_count==200 and len(fast.centers)==100
    assert len(fast.dominance_certificates)==100 and len(fast.all_centers)==200
    for point,witness,slack in fast.dominance_certificates:
        assert witness<100 and point>=100 and slack>1e-9
        assert np.linalg.norm(fast.all_centers[witness]-fast.all_centers[point])+fast.all_radii[point]<fast.all_radii[witness]
    poses=np.asarray([(x,y,yaw) for x in (2.,3.,5.,8.) for y in (3.,5.,7.) for yaw in (0.,.7,2.)])
    offsets=np.arange(len(poses),dtype=np.int64)*100_000_000;foot=ConvexFootprint(c.footprint.vertices)
    a=exact.distance_lower_bounds(foot,poses,offsets);b=fast.distance_lower_bounds(foot,poses,offsets)
    np.testing.assert_allclose(a,b,rtol=0,atol=2e-14)
    assert np.any(a<0) and np.any(a>0)
    assert fast.identity['occupancy_removed']is False
    with pytest.raises(ValueError):DominanceBatchHardOccupancy(tracker.snapshot,t-1)


def test_query_dominance_keeps_equality_points_and_every_persistent_circle():
    from sensor_tracks_query_m08 import DominanceBatchHardOccupancy
    grid,_,_=make_grid(size=140);t=1_000_000_000;tracker=IngestionSensorTracks(grid)
    points=np.asarray([(3.,7.)])
    f=ScanFrame(t,np.ones(72),points,(3.,7.,0.));s=tracker.accept(f,tf_stamp_ns=t,odom_state=dict(stamp_ns=t,pose_map=(3.,7.,0.),body_twist=(0.,0.,0.)),now_ns=t)
    fast=DominanceBatchHardOccupancy(s,t)
    assert fast.support_count==2 and len(fast.centers)==2 and not fast.dominance_certificates
