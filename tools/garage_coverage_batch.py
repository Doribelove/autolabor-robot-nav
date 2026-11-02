#!/usr/bin/env python3
"""Preview a complete garage mission and submit existing continuous-region execution."""
import argparse
import concurrent.futures
import functools
import json
import math
from pathlib import Path
import time
import uuid

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage
import yaml
from autolabor_coverage import coverage_geometry as geometry

GRID = None


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def region_plan(job):
    index, vertices = job
    planner = geometry.CoveragePlanner(GRID, footprint_front=1.32,
        footprint_rear=.62, footprint_half_width=.50)
    try:
        p = planner.plan([geometry.Point(*v) for v in vertices], 1.2, .15)
        return dict(id='garage-sector-%02d'%index, vertices=vertices, angle=p.angle,
            lines=[[[s.start.x,s.start.y],[s.end.x,s.end.y]] for s in p.swaths])
    except ValueError as error:
        return dict(id='garage-sector-%02d'%index, vertices=vertices, error=str(error))


def execute(args):
    import rospy
    from autolabor_coverage.msg import CoverageRegion, CoverageStatus
    from autolabor_coverage.srv import StartCoverageBatch, StartCoverageBatchRequest
    from geometry_msgs.msg import Point32
    rospy.init_node('garage_coverage_batch_start', anonymous=True)
    mission=json.loads((args.output/'batch_mission.json').read_text())
    status=rospy.wait_for_message('/coverage/status',CoverageStatus,timeout=5)
    request=StartCoverageBatchRequest(client_request_id=mission['client_request_id'],
        operation_width_m=1.2,overlap_ratio=.15,map_digest=mission['map_digest'],
        **mission['execution_parameters'])
    for index,region in enumerate(mission['regions'],1):
        item=CoverageRegion(id=region['id'],name='地下车库 %02d/%02d'%(index,len(mission['regions'])))
        item.region.header.frame_id='map';item.region.header.stamp=rospy.Time.now()
        item.region.polygon.points=[Point32(x=x,y=y,z=0) for x,y in region['vertices']]
        request.regions.append(item)
    print('Starting requested continuous garage coverage: %d regions'%len(request.regions),flush=True)
    response=rospy.ServiceProxy('/coverage/start_batch',StartCoverageBatch)(request)
    dump(args.output/'batch_start_result.json',dict(accepted=response.accepted,
        message=response.message,batch_id=response.batch_id,time=time.time()))
    print(response,flush=True)
    if not response.accepted:raise RuntimeError(response.message)


def publish(args):
    import rospy
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker,MarkerArray
    rospy.init_node('garage_coverage_mission_preview',anonymous=True)
    mission=json.loads((args.output/'batch_mission.json').read_text())
    array=MarkerArray()
    for index,r in enumerate(mission['regions']):
        m=Marker();m.header.frame_id='map';m.ns='garage_mission_swaths';m.id=index
        m.type=Marker.LINE_LIST;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=.06
        m.color.r=.1;m.color.g=.8;m.color.b=1.;m.color.a=.7
        for line in r['lines']:
            m.points.extend([Point(x=x,y=y,z=.06) for x,y in line])
        array.markers.append(m)
        label=Marker();label.header.frame_id='map';label.ns='garage_mission_regions';label.id=index
        label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.
        label.pose.position.x=sum(x for x,y in r['vertices'])/4
        label.pose.position.y=sum(y for x,y in r['vertices'])/4;label.pose.position.z=.3
        label.text=str(index+1);label.scale.z=1.;label.color.r=1.;label.color.g=.6;label.color.a=1.
        array.markers.append(label)
    publisher=rospy.Publisher('/coverage/ui_markers',MarkerArray,queue_size=1,latch=True)
    rate=rospy.Rate(.5)
    while not rospy.is_shutdown():publisher.publish(array);rate.sleep()


def plan(args):
    global GRID
    source=args.dataset/'online_map_at_recording_end/map_fused_2d'
    c=yaml.safe_load((source/'map.yaml').read_text());raster=np.asarray(Image.open(source/c['image']))
    pixels=np.flipud(raster);h,w=pixels.shape;resolution=c['resolution'];ox,oy,_=c['origin']
    initial=json.loads((args.output/'initial_state.json').read_text());pose=initial['pose_xyyaw']
    labels,count=ndimage.label(pixels>=250);mx=int((pose[0]-ox)/resolution);my=int((pose[1]-oy)/resolution)
    if not (0<=mx<w and 0<=my<h) or labels[my,mx]==0:
        raise RuntimeError('Current vehicle pose is outside saved known free space')
    connected=labels==labels[my,mx]
    cells=np.full(pixels.shape,-1,dtype=np.int8);cells[pixels<=65]=100;cells[connected]=0
    GRID=geometry.GridMap(w,h,resolution,ox,oy,cells.ravel())
    nx=math.ceil(w*resolution/12.);ny=math.ceil(h*resolution/16.)
    jobs=[];excluded=[]
    for iy in range(ny):
        for ix in range(nx):
            x0=ox+ix*w*resolution/nx;x1=ox+(ix+1)*w*resolution/nx
            y0=oy+iy*h*resolution/ny;y1=oy+(iy+1)*h*resolution/ny
            # Small overlap at artificial partitions prevents a boundary seam.
            x0=max(ox,x0-.15);x1=min(ox+w*resolution,x1+.15)
            y0=max(oy,y0-.15);y1=min(oy+h*resolution,y1+.15)
            jobs.append((iy*nx+ix+1,[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]))
    regions=[];began=time.monotonic()
    with concurrent.futures.ProcessPoolExecutor(max_workers=4) as pool:
        for result in pool.map(region_plan,jobs):
            if 'error' in result:excluded.append(result)
            else:regions.append(result)
            print('Geometry %d/%d, usable regions %d'%(len(regions)+len(excluded),len(jobs),len(regions)),flush=True)
    # Memoize pure transition costs only during this offline preview. The
    # running manager and all collision/control checks remain unchanged.
    geometry._entry_time=functools.lru_cache(maxsize=100000)(geometry._entry_time)
    parameters=geometry.CoverageTimeParameters()
    saved=json.loads((args.output/'request.json').read_text())
    e=saved['execution_parameters']
    parameters=geometry.CoverageTimeParameters(max_forward_speed_mps=e['max_speed_mps'],
        max_reverse_speed_mps=e['reverse_speed_mps'],max_angular_speed_rps=e['max_angular_speed_rps'],
        linear_accel_mps2=e['linear_accel_mps2'],angular_accel_rps2=e['angular_accel_rps2'],
        allow_reverse=e['allow_reverse_transit'],direction_change_penalty_sec=e['direction_change_penalty_sec'],
        segment_handoff_penalty_sec=e['segment_handoff_penalty_sec'],transit_replan_period_sec=e['transit_replan_period_sec'])
    current=geometry.Point(*pose[:2]);yaw=pose[2];ordered=[];estimate=0.;sweep_length=0.
    while regions:
        # Pick a nearby region by known-free travel distance among the nearest
        # four candidates, so a wall is not treated as a direct transit.
        nearest=sorted(regions,key=lambda r:min(math.hypot(x-current.x,y-current.y)
            for line in r['lines'] for x,y in line))[:4]
        choices=[]
        for r in nearest:
            entry=min((geometry.Point(*p) for line in r['lines'] for p in line),
                      key=lambda p:math.hypot(p.x-current.x,p.y-current.y))
            distance=GRID.shortest_known_free_distance(current,entry)
            choices.append((distance if distance is not None else float('inf'),r['id'],r))
        _,_,r=min(choices,key=lambda t:(t[0],t[1]));regions.remove(r)
        swaths=[geometry.Swath(geometry.Point(*a),geometry.Point(*b),
            geometry.rotate_to_scan(geometry.Point(*a),r['angle']).y,math.dist(a,b)) for a,b in r['lines']]
        route,est=geometry.order_swaths(swaths,current,1.02,1.35,current_yaw=yaw,
            time_parameters=parameters,return_estimate=True,time_search_beam_width=128,
            first_entry_connector_distance=lambda a,b:math.hypot(a.x-b.x,a.y-b.y),
            first_entry_distance_slack_m=.3,first_entry_slack_sec=1.)
        r['lines']=[[[s.start.x,s.start.y],[s.end.x,s.end.y]] for s in route]
        r['estimated_seconds']=est.total_time_sec;estimate+=est.total_time_sec
        sweep_length+=sum(s.length for s in route);current=route[-1].end
        yaw=math.atan2(route[-1].end.y-route[-1].start.y,route[-1].end.x-route[-1].start.x)
        ordered.append(r);print('Ordered region %d, %d sweep lines'%(len(ordered),len(route)),flush=True)
    covered=np.zeros((h,w),np.uint8)
    lines=[line for r in ordered for line in r['lines']]
    for a,b in lines:
        vx,vy=np.subtract(b,a);norm=math.hypot(vx,vy);dx=-vy/norm*.6;dy=vx/norm*.6
        corners=[[a[0]+dx,a[1]+dy],[b[0]+dx,b[1]+dy],[b[0]-dx,b[1]-dy],[a[0]-dx,a[1]-dy]]
        points=np.round((np.asarray(corners)-[ox,oy])/resolution-.5).astype(np.int32)
        cv2.fillConvexPoly(covered,points,1)
    mission=dict(client_request_id='coverage-batch-'+uuid.uuid4().hex,
        map_digest=initial['map_digest'],operation_width_m=1.2,overlap_ratio=.15,lane_spacing_m=1.02,
        execution_parameters=e,regions=ordered,excluded_regions=excluded,swath_count=len(lines),
        sweep_length_m=sweep_length,estimated_seconds=estimate,planning_seconds=time.monotonic()-began,
        known_free_area_m2=float(np.count_nonzero(pixels>=250)*resolution**2),
        connected_free_area_m2=float(connected.sum()*resolution**2),
        preview_swept_known_area_m2=float(np.count_nonzero(covered.astype(bool)&connected)*resolution**2),
        preview_description='Full reference sweep sequence. Existing batch executor replans each region from its actual entry pose and computes collision-checked Hybrid/TEB transitions online.')
    dump(args.output/'batch_mission.json',mission)
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig,ax=plt.subplots(figsize=(12,14));ax.imshow(raster,cmap='gray',vmin=0,vmax=255,
        extent=[ox,ox+w*resolution,oy,oy+h*resolution])
    for i,r in enumerate(ordered):
        color=plt.cm.turbo(i/max(1,len(ordered)-1));ax.add_collection(LineCollection(r['lines'],colors=[color],linewidths=.8))
        vertices=np.array(r['vertices']);center=vertices.mean(axis=0)
        ax.text(*center,str(i+1),fontsize=9,color='black',ha='center',bbox=dict(facecolor='white',alpha=.7,pad=1))
    ax.plot(*pose[:2],'r*',markersize=12,label='Current vehicle')
    ax.set_aspect('equal');ax.set_xlabel('map X (m)');ax.set_ylabel('map Y (m)')
    ax.set_title('Garage coverage reference: 1.2 m width / 1.02 m spacing\n%d ordered regions, %d sweep lines; transitions planned online'%(len(ordered),len(lines)))
    ax.legend();fig.tight_layout();fig.savefig(args.output/'complete_coverage_route.png',dpi=160)
    with (args.output/'complete_sweep_sequence.csv').open('w') as f:
        f.write('region_order,line_order,start_x,start_y,end_x,end_y\n')
        for i,r in enumerate(ordered,1):
            for j,(a,b) in enumerate(r['lines'],1):f.write(','.join(map(str,[i,j,*a,*b]))+'\n')
    print(json.dumps({k:v for k,v in mission.items() if k not in ['regions','excluded_regions']},ensure_ascii=False),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--publish',action='store_true')
    args=parser.parse_args()
    if args.execute:execute(args)
    elif args.publish:publish(args)
    else:plan(args)
