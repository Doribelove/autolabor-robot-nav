"""Shared static-map and fixed-route components; imports have no ROS effects.

Consumes static_map_export outputs, never actor poses. The grid-derived route is
a common geometric guide, not an Ackermann trajectory or a swept safety proof.
Frozen R2 does not use this interface and must retain its separate identity.
"""
import argparse
import hashlib
import heapq
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml

from route_world_adapter import RouteFrame, wrap_angle
from axle_reference_adapter import base_pose_to_rear, base_twist_to_rear
from static_map_export import finite_vector, sha_bytes


class SharedStaticMap:
    """Exact normalized static_map_export trinary cells, unknown is blocked."""
    def __init__(self,pgm_bytes,yaml_bytes):
        metadata=yaml.safe_load(yaml_bytes)
        magic,size,maximum,raster=pgm_bytes.split(b'\n',3)
        width,height=(int(v) for v in size.split())
        if magic!=b'P5' or maximum!=b'255' or len(raster)!=width*height:
            raise ValueError('expected canonical exporter P5 raster')
        self.resolution=float(metadata['resolution'])
        self.origin=finite_vector(metadata['origin'],3)
        if self.resolution<=0 or not math.isfinite(self.resolution) or abs(self.origin[2])>1e-12:
            raise ValueError('positive resolution and world-aligned exporter map required')
        if any(metadata.get(k)!=v for k,v in {'negate':0,'occupied_thresh':.65,'free_thresh':.196,'mode':'trinary'}.items()):
            raise ValueError('map threshold convention changed')
        self.pixels=np.frombuffer(raster,dtype=np.uint8).reshape(height,width)[::-1].copy()
        if not set(np.unique(self.pixels))<={0,205,254}:raise ValueError('noncanonical trinary pixels')
        if not np.all(self.pixels[[0,-1],:]==205) or not np.all(self.pixels[:,[0,-1]]==205):
            raise ValueError('explicit unknown boundary required')
        self.height,self.width=height,width
        self.occupancy=np.where(self.pixels==254,0,np.where(self.pixels==0,100,-1)).astype(np.int8)
        # Shared runtime policy: unknown prior is lethal, so a laser clearing
        # update cannot erase its boundary in the downstream StaticLayer.
        self.blocking_occupancy=np.maximum(self.occupancy,0).copy()
        self.blocking_occupancy[self.occupancy<0]=100
        self.image_sha256=sha_bytes(pgm_bytes);self.yaml_sha256=sha_bytes(yaml_bytes)
        # Distance to nearest blocked pixel CENTRE, less its enclosing radius.
        # At arbitrary points subtract point-to-query-cell-centre distance too.
        self.clearance_centres=distance_transform_edt(self.pixels==254)*self.resolution-self.resolution/math.sqrt(2)

    def cell(self,xy):
        x,y=finite_vector(xy,2)
        return math.floor((y-self.origin[1])/self.resolution),math.floor((x-self.origin[0])/self.resolution)

    def centre(self,cell):
        row,col=cell
        return self.origin[0]+(col+.5)*self.resolution,self.origin[1]+(row+.5)*self.resolution

    def clearance(self,xy):
        row,col=self.cell(xy)
        if not(0<=row<self.height and 0<=col<self.width):return -math.inf
        if self.pixels[row,col]!=254:return -self.resolution/math.sqrt(2)
        return float(self.clearance_centres[row,col])-math.dist(xy,self.centre((row,col)))

    def segment_clearance(self,a,b):
        """Conservative lower bound for every point of a line segment."""
        a=finite_vector(a,2);b=finite_vector(b,2);length=math.dist(a,b)
        count=max(1,math.ceil(length/(self.resolution/2)))
        minimum=min(self.clearance((a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count)) for i in range(count+1))
        return minimum-length/(2*count)


class CommonRoute:
    """Ordered map-frame polyline with deterministic earliest-segment ties."""
    def __init__(self,points_xy,goal_yaw_rad=0.0):
        self.points=tuple(finite_vector(p,2) for p in points_xy)
        self.goal_yaw=float(goal_yaw_rad)
        if len(self.points)<2 or not math.isfinite(self.goal_yaw):raise ValueError('finite multi-point route required')
        self.lengths=tuple(math.dist(a,b) for a,b in zip(self.points,self.points[1:]))
        if any(v<=1e-9 for v in self.lengths):raise ValueError('duplicate or degenerate route segment')
        self.arcs=tuple(np.r_[0.,np.cumsum(self.lengths)].tolist());self.length=self.arcs[-1]

    def sample(self,arc_m):
        value=float(arc_m)
        if not math.isfinite(value):raise ValueError('nonfinite route arc')
        value=max(0.,min(self.length,value))
        index=min(len(self.lengths)-1,max(0,int(np.searchsorted(self.arcs,value,side='right'))-1))
        a,b=self.points[index:index+2];phase=(value-self.arcs[index])/self.lengths[index]
        return (a[0]+phase*(b[0]-a[0]),a[1]+phase*(b[1]-a[1]),math.atan2(b[1]-a[1],b[0]-a[0]))

    def project(self,xy):
        x,y=finite_vector(xy,2);best=None
        for index,(a,b,length) in enumerate(zip(self.points,self.points[1:],self.lengths)):
            ux,uy=(b[0]-a[0])/length,(b[1]-a[1])/length
            local=max(0.,min(length,(x-a[0])*ux+(y-a[1])*uy));px,py=a[0]+local*ux,a[1]+local*uy
            distance=math.hypot(x-px,y-py)
            candidate=(distance,index,self.arcs[index]+local,(x-px)*(-uy)+(y-py)*ux,px,py,math.atan2(uy,ux))
            if best is None or candidate[:2]<best[:2]:best=candidate
        return dict(distance_m=best[0],segment=best[1],arc_m=best[2],lateral_m=best[3],x=best[4],y=best[5],heading_rad=best[6])

    def resample(self,spacing_m=.1):
        if not math.isfinite(spacing_m) or spacing_m<=0:raise ValueError('positive route spacing required')
        # Retain all corners exactly: resampling must not silently cut obstacles.
        result=[self.points[0]]
        for a,b,length in zip(self.points,self.points[1:],self.lengths):
            count=max(1,math.ceil(length/spacing_m))
            result.extend((a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count) for i in range(1,count+1))
        return result


def plan_grid_route(grid,start_xy,goal_xy,clearance_radius_m):
    """Deterministic 8-connected A*, then clearance-checked string pulling.

    Radius is an explicit shared geometric-guide clearance threshold, independent
    of each local planner's cost. No start/goal cell is carved free. The choice
    alone does not qualify a compound physical footprint over all steering states.
    """
    radius=float(clearance_radius_m)
    if not math.isfinite(radius) or radius<0:raise ValueError('invalid common radius')
    start_xy=finite_vector(start_xy,2);goal_xy=finite_vector(goal_xy,2)
    start,goal=grid.cell(start_xy),grid.cell(goal_xy)
    # Every diagonal edge remains clear by the 1-Lipschitz distance bound.
    free=grid.clearance_centres>=radius+grid.resolution/math.sqrt(2)
    for cell,xy in ((start,start_xy),(goal,goal_xy)):
        row,col=cell
        if not(0<=row<grid.height and 0<=col<grid.width) or not free[row,col] or grid.segment_clearance(xy,grid.centre(cell))<radius:
            raise ValueError('requested start/goal violates conservative map clearance')
    frontier=[(math.dist(start,goal),0.,start)];costs={start:0.};parents={start:None};closed=set()
    while frontier:
        _,cost,cell=heapq.heappop(frontier)
        if cell in closed:continue
        closed.add(cell)
        if cell==goal:break
        for dr,dc in ((-1,0),(0,-1),(0,1),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)):
            other=(cell[0]+dr,cell[1]+dc)
            if not(0<=other[0]<grid.height and 0<=other[1]<grid.width) or not free[other]:continue
            if dr and dc and not(free[cell[0]+dr,cell[1]] and free[cell[0],cell[1]+dc]):continue
            proposed=cost+math.hypot(dr,dc)
            if proposed<costs.get(other,math.inf):
                costs[other]=proposed;parents[other]=cell
                heapq.heappush(frontier,(proposed+math.dist(other,goal),proposed,other))
    if goal not in closed:raise ValueError('no common static-map route at declared clearance')
    cells=[];cell=goal
    while cell is not None:cells.append(cell);cell=parents[cell]
    points=[start_xy]+[grid.centre(c) for c in reversed(cells)]+[goal_xy]
    points=[point for index,point in enumerate(points) if index==0 or math.dist(point,points[index-1])>1e-9]
    simplified=[points[0]];index=0
    while index<len(points)-1:
        target=len(points)-1
        while target>index+1 and grid.segment_clearance(points[index],points[target])<radius:target-=1
        if grid.segment_clearance(points[index],points[target])<radius:raise ValueError('grid path edge clearance check failed')
        simplified.append(points[target]);index=target
    return simplified,{'expanded_grid_nodes':len(closed),'minimum_segment_clearance_lower_bound_m':min(grid.segment_clearance(a,b) for a,b in zip(simplified,simplified[1:]))}


class CommonRouteInputAdapter:
    """New PCT/analytic consumer interface; never presented as frozen R2."""
    method_suffix='common_input_v1'

    def __init__(self,grid,route,map_from_odom_xy_yaw):
        self.grid,self.route=grid,route
        self.map_from_odom=RouteFrame(*finite_vector(map_from_odom_xy_yaw,3))

    def state(self,odom_base_xyyaw,body_twist_vx_vy_w,lookahead_m=.7,wheelbase_m=.65):
        if not math.isfinite(lookahead_m) or lookahead_m<0:raise ValueError('invalid lookahead')
        base=self.map_from_odom.pose_to_world(finite_vector(odom_base_xyyaw,3))
        projection=self.route.project(base[:2]);target=self.route.sample(projection['arc_m']+lookahead_m)
        return {'map_base_pose':base,'map_rear_pose':base_pose_to_rear(base,wheelbase_m),
                'rear_body_twist':base_twist_to_rear(body_twist_vx_vy_w,wheelbase_m),
                'projection':projection,'remaining_route_m':self.route.length-projection['arc_m'],
                'heading_error_rad':wrap_angle(base[2]-projection['heading_rad']),
                'lookahead_map_pose':target,'lookahead_odom_pose':self.map_from_odom.pose_to_route(target),
                'static_base_centre_clearance_lower_bound_m':self.grid.clearance(base[:2])}

    def candidate_metrics(self,previews_map,footprint_radius_m):
        radius=float(footprint_radius_m)
        if not math.isfinite(radius) or radius<0:raise ValueError('invalid static footprint radius')
        result=[]
        for preview in previews_map:
            if len(preview)<2:raise ValueError('candidate requires at least two states')
            points=[finite_vector(p[:2],2) for p in preview]
            projections=[self.route.project(p) for p in points]
            result.append({'route_progress_m':projections[-1]['arc_m']-projections[0]['arc_m'],
                           'terminal_lateral_m':projections[-1]['lateral_m'],
                           'minimum_static_polyline_clearance_lower_bound_m':min(self.grid.segment_clearance(a,b) for a,b in zip(points,points[1:]))-radius,
                           'scope':'candidate chord geometry only; actual trajectory and dynamic obstacles require separate verifier'})
        return result


def load_bundle(directory):
    directory=Path(directory);data=json.loads((directory/'common_route.json').read_bytes())
    grid=SharedStaticMap((directory/'map.pgm').read_bytes(),(directory/'map.yaml').read_bytes())
    if data['schema']!='astra_common_route_v1' or data['frame_id']!='map' or data['map_image_sha256']!=grid.image_sha256 or data['map_yaml_sha256']!=grid.yaml_sha256:
        raise ValueError('common map/route binding mismatch')
    route=CommonRoute(data['points_xy'],data['goal_xy_yaw'][2])
    if math.dist(route.points[0],finite_vector(data['start_xy_yaw'],3)[:2])>1e-9 or math.dist(route.points[-1],finite_vector(data['goal_xy_yaw'],3)[:2])>1e-9:
        raise ValueError('route endpoints differ from declared fixed request')
    finite_vector(data['map_from_odom_xy_yaw'],3)
    canonical=json.dumps(data['points_xy'],separators=(',',':'),allow_nan=False).encode()
    if sha_bytes(canonical)!=data['path_points_sha256']:raise ValueError('path point identity mismatch')
    return grid,route,data


def build_bundle(export_directory,output_directory,radius=.68):
    source=Path(export_directory);output=Path(output_directory)
    if output.exists():raise FileExistsError('route bundle is immutable')
    pgm=(source/'map.pgm').read_bytes();metadata=(source/'map.yaml').read_bytes()
    request_bytes=(source/'route_request.json').read_bytes();request=json.loads(request_bytes)
    grid=SharedStaticMap(pgm,metadata)
    if request['map_image_sha256']!=grid.image_sha256 or request['map_yaml_sha256']!=grid.yaml_sha256 or request['frame_id']!='map':
        raise ValueError('export request/map mismatch')
    points,audit=plan_grid_route(grid,request['policy_start_xy_yaw'][:2],request['goal_xy_yaw'][:2],radius)
    route=CommonRoute(points,request['goal_xy_yaw'][2]);points=[list(p) for p in route.resample(.1)]
    data={'schema':'astra_common_route_v1','frame_id':'map','map_image_sha256':grid.image_sha256,'map_yaml_sha256':grid.yaml_sha256,
          'route_request_sha256':sha_bytes(request_bytes),'start_xy_yaw':request['policy_start_xy_yaw'],'goal_xy_yaw':request['goal_xy_yaw'],
          'map_from_odom_xy_yaw':request['map_from_odom_xy_yaw'],'points_xy':points,
          'path_points_sha256':sha_bytes(json.dumps(points,separators=(',',':'),allow_nan=False).encode()),
          'route_length_m':route.length,'planning_clearance_radius_m':radius,'planner':'deterministic8connected_Astar_clearance_checked_stringpull',
          'common_policy':'fixed full map-frame route; no online global replanning; internal local-planner pruning remains algorithm-specific',
          'provider_returns_exact_full_polyline':True,'actor_state_used':False,'frozen_R2_consumes_this':False,
          'ackermann_route_feasibility_qualified':False,'continuous_safety_qualified':False,'live_consumption_qualified':False,
          'audit':audit,'scope':'shared static geometric reference, not an executable Ackermann trajectory'}
    output.mkdir(parents=True)
    (output/'map.pgm').write_bytes(pgm);(output/'map.yaml').write_bytes(metadata)
    (output/'common_route.json').write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    return data


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--export',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--clearance-radius',type=float,default=.68)
    args=parser.parse_args();data=build_bundle(args.export,args.output,args.clearance_radius)
    print(json.dumps({k:data[k] for k in ('path_points_sha256','route_length_m','ackermann_route_feasibility_qualified','live_consumption_qualified')}))


if __name__=='__main__':main()
