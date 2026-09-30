import csv
import hashlib
import heapq
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
import yaml
import shapely
from shapely.geometry import Polygon,LineString,Point,MultiPoint
from shapely.ops import unary_union
from shapely.prepared import prep

STAGE=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926')
WS=STAGE/'workspace'
DATA=STAGE/'scenes/development_a01'
spec=importlib.util.spec_from_file_location('gen',WS/'scripts/astra_gazebo/scene_generator.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
SHA=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()


def physical_static(scene):
    boxes=list(scene['static_boxes'])
    if scene['corridor_half_width_m'] is not None:
        h=scene['corridor_half_width_m']
        boxes += [dict(name='corridor_wall_'+str(s),x=0,y=s*(h+.1),length=18,width=.2,height=1,yaw_rad=0) for s in (-1,1)]
    return [(b['name'],Polygon(g.vertices(b['x'],b['y'],b.get('yaw_rad',0),b['length'],b['width']))) for b in boxes]


def footprint(state):
    x,y,yaw=state[:3];delta=state[3] if len(state)>3 else 0.
    pts=g.vertices(x,y,yaw,1.04,.7);c,s=math.cos(yaw),math.sin(yaw)
    radius=.65/math.tan(delta) if abs(delta)>1e-9 else float('inf')
    for front in (False,True):
        for side in (-1,1):
            wx=(.325 if front else -.325);wy=side*.30
            steer=math.atan(.65/(radius-side*.30)) if front and math.isfinite(radius) else 0.
            pts+=g.vertices(x+c*wx-s*wy,y+s*wx+c*wy,yaw+steer,.30,.08)
    return MultiPoint(pts).convex_hull


def static_path(scene, max_nodes=40000):
    obstacles=unary_union([p for _,p in physical_static(scene)])
    # 0.03 m above the 0.33 margin bounds the <0.028 m inter-sample corner displacement.
    forbidden=prep(obstacles.buffer(.36,resolution=16))
    safe=lambda s: not forbidden.intersects(footprint(s))
    endpoints={name:obstacles.distance(footprint(pose)) if not obstacles.is_empty else None
               for name,pose in [('spawn',(-5.4,0,0)),('gate',(-4.,0,0)),('goal',(5.,0,0))]}
    if not safe((-4,0,0)) or not safe((5,0,0)):
        return {'status':'end_pose_not_safe_under_margin','endpoint_physical_clearance_m':endpoints,'path':[]}
    direct=[(-4+i*.025,0,0,0) for i in range(361)]
    if all(safe(s) for s in direct):
        return {'status':'static_forward_ackermann_witness','endpoint_physical_clearance_m':endpoints,'path':direct,'expanded_nodes':0,'path_length_m':9.,'max_steer_rad':0.,'max_steer_rate_radps':0.}
    start=(-4.,0.,0.,0.);queue=[(9.,0,0.,start,0)];serial=0;records={0:(None,[],start)};best={};expanded=0
    def key(s):return (round(s[0]/.15),round(s[1]/.15),round(s[2]/.10),round(s[3]/.1))
    best[key(start)]=0.
    while queue and expanded<max_nodes:
        _,_,cost,state,record=heapq.heappop(queue)
        if cost>best.get(key(state),float('inf'))+1e-8:continue
        expanded+=1
        if math.hypot(state[0]-5,state[1])<=.25 and abs(state[2])<=.25:
            segments=[];rid=record
            while records[rid][0] is not None:
                parent,points,_=records[rid];segments.append(points);rid=parent
            path=[start]+[s for segment in reversed(segments) for s in segment]
            distance=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:]))
            return {'status':'static_forward_ackermann_witness','endpoint_physical_clearance_m':endpoints,'path':path,'expanded_nodes':expanded,'path_length_m':distance,'max_steer_rad':max(abs(s[3]) for s in path),'max_steer_rate_radps':max(abs(b[3]-a[3])/.05 for a,b in zip(path,path[1:]))}
        for target in (0.,-.4,.4,-.2,.2):
            x,y,yaw,delta=state;points=[]
            for _ in range(20):
                delta+=max(-.75*.05,min(.75*.05,(target-delta)*(1-math.exp(-.05/.18))))
                yaw_next=yaw+.4*math.tan(delta)/.65*.05
                x+=.4*math.cos(.5*(yaw+yaw_next))*.05;y+=.4*math.sin(.5*(yaw+yaw_next))*.05;yaw=yaw_next
                step=(x,y,yaw,delta)
                if not (-4.3<=x<=5.6 and abs(y)<=3.8 and abs(yaw)<=1.25) or not safe(step):break
                points.append(step)
            if len(points)!=20:continue
            new=points[-1];nc=cost+.4+.01*abs(target-state[3]);k=key(new)
            if nc>=best.get(k,float('inf')):continue
            best[k]=nc;serial+=1;records[serial]=(record,points,new)
            heuristic=math.hypot(5-x,y)+.2*abs(yaw)
            heapq.heappush(queue,(nc+1.15*heuristic,serial,nc,new,serial))
    return {'status':'no_witness_within_finite_search_not_impossibility','endpoint_physical_clearance_m':endpoints,'path':[],'expanded_nodes':expanded}


def ray_box_occlusion(observer,actor,static):
    x,y=actor['start'];yaw=math.atan2(actor['end'][1]-y,actor['end'][0]-x)
    points=[(x,y)]+g.vertices(x,y,yaw,.72,.72)
    blocked=[]
    for point in points:
        ray=LineString([observer,point]);blocked.append(any(ray.intersects(poly) for _,poly in static))
    return {'blocked_center_ray':blocked[0],'blocked_center_and_corner_ray_fraction':sum(blocked)/len(blocked),
            'scope':'five-ray geometric proxy, not a full 720-beam visibility proof'}


def actor_semantics(scene):
    static=physical_static(scene);route=LineString([(-4.,0.),(5.,0.)]);rows=[]
    for a in scene['actors']:
        start,end=a['start'],a['end'];yaw=math.atan2(end[1]-start[1],end[0]-start[0]);swept=MultiPoint(g.vertices(*start,yaw,.72,.72)+g.vertices(*end,yaw,.72,.72)).convex_hull
        path=LineString([start,end]);conflict_error=path.distance(Point(*a['conflict']))
        conflict_progress=path.project(Point(*a['conflict']))
        speed=a['base_speed_mps'];actor_time=conflict_progress/speed
        ego_time=max(0.,a['conflict'][0]+4)/.8
        blocked=[n for n,p in static if p.intersects(swept)]
        unreactive_min=(float('inf'),None)
        for k in range(441):
            t=.05*k;d=min(path.length,speed*t);pos=path.interpolate(d)
            ego_x=min(5.,-4.+.8*t)
            c=footprint((ego_x,0,0)).distance(Polygon(g.vertices(pos.x,pos.y,yaw,.72,.72)))
            if c<unreactive_min[0]:unreactive_min=(c,t)
        rows.append({'model':a['model_name'],'start_end_length_m':path.length,'conflict_point_distance_to_actor_path_m':conflict_error,
          'actor_full_swept_box_intersections':blocked,'actor_path_minimum_center_distance_to_route_m':path.distance(route),
          'unreactive_time_to_conflict_s':actor_time,'ego_nominal_point_time_s':ego_time,
          'unreactive_arrival_difference_s':actor_time-ego_time,'within_2s_arrival_difference':abs(actor_time-ego_time)<=2,
          'unreactive_min_physical_clearance_m':unreactive_min[0],'unreactive_min_at_s':unreactive_min[1],
          'conservative_conflict_proxy_present':unreactive_min[0]<=.33,
          'initial_gate_occlusion':ray_box_occlusion((-4.,0.),a,static),
          'later_visibility_at_conflict':ray_box_occlusion((a['conflict'][0]-1.,0.),dict(a,start=a['conflict']),static)})
    return rows


def road_width(scene):
    static=physical_static(scene);widths=[]
    for k in range(91):
        x=-4.+k*.1;line=LineString([(x,-8),(x,8)]); intervals=[]
        for _,poly in static:
            hit=line.intersection(poly)
            if not hit.is_empty:intervals.append((hit.bounds[1],hit.bounds[3]))
        lower=max([b for a,b in intervals if b<=0]+[-8.]);upper=min([a for a,b in intervals if a>=0]+[8.])
        blocked=any(a<=0<=b for a,b in intervals)
        widths.append({'x':round(x,3),'centerline_blocked':blocked,'clear_width_about_center_m':0. if blocked else upper-lower})
    return {'minimum_unblocked_center_band_width_m':min([r['clear_width_about_center_m'] for r in widths if not r['centerline_blocked']] or [0]),
            'centerline_blocked_sample_count':sum(r['centerline_blocked'] for r in widths),
            'interpretation':'local cross-section only; an obstacle may require a noncentered bypass','samples':widths}


def sdf_consistency(scene,path):
    world=ET.parse(path).getroot().find('world');issues=[]
    for a in scene['actors']:
        model=world.find("model[@name='%s']"%a['model_name']);plugin=model.find('plugin')
        fields={'base_speed':a['base_speed_mps'],'initial_speed':a['base_speed_mps'],'response_distance':a['response_distance_m'],
                'response_time_margin':a['response_time_margin_s'],'seed':a['response_seed']}
        for k,v in fields.items():
            if abs(float(plugin.findtext(k))-v)>1e-9:issues.append(a['model_name']+':'+k)
        for k in ('start','end','conflict'):
            if any(abs(x-y)>1e-9 for x,y in zip(map(float,plugin.findtext(k).split()),a[k])):issues.append(a['model_name']+':'+k)
    return issues


def main():
    rows=list(csv.DictReader((DATA/'diagnostic_blocks.csv').open()));layout_reports=[];episode_reports=[];witnessdir=STAGE/'audits/scene_witnesses';witnessdir.mkdir(exist_ok=True)
    for row in rows:
        scene=yaml.safe_load(Path(row['manifest_path']).read_text())
        episode_reports.append({'layout_id':scene['layout_id'],'seed':scene['seed'],'replicate':scene['replicate'],
           'sdf_consistency_issues':sdf_consistency(scene,row['world_path']),'actors':actor_semantics(scene)})
        if scene['replicate']!=0:continue
        now=time.monotonic();path=static_path(scene);points=path.pop('path')
        wp=witnessdir/(scene['layout_id']+'.json');wp.write_text(json.dumps({'layout_id':scene['layout_id'],'state_fields':['x_m','y_m','yaw_rad','steering_rad'],'path':points},indent=2)+'\n')
        report={'layout_id':scene['layout_id'],'family':scene['family'],'topology':scene['topology'],'static_ackermann':path,
                'witness_path':str(wp),'witness_sha256':SHA(wp),'road_width':road_width(scene),'wall_s':time.monotonic()-now}
        layout_reports.append(report);print(scene['layout_id'],path['status'],path.get('expanded_nodes'),flush=True)
    allactors=[a for e in episode_reports for a in e['actors']]
    deps=[WS/'scripts/astra_gazebo/scene_generator.py',WS/'scripts/astra_gazebo/route_world_adapter.py',
      WS/'src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp',WS/'src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h',
      WS/'src/simulation/m2_gazebo/src/m2_ackermann_plugin.cpp',WS/'src/simulation/m2_gazebo/include/m2_gazebo/actuator_dynamics.h',
      WS/'src/simulation/m2_gazebo/launch/spawn_m2.launch',WS/'src/simulation/m2_gazebo/urdf/m2.urdf.xacro',
      WS/'src/simulation/m2_gazebo/urdf/m2_base.xacro',WS/'src/simulation/m2_gazebo/urdf/m2_sensors.xacro',
      WS/'src/simulation/m2_gazebo/urdf/m2_gazebo.xacro',WS/'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py',
      STAGE/'contracts/splits.yaml',DATA/'diagnostic_blocks.csv',DATA/'generation_report.json',Path(__file__)]
    report={'status':'offline_scene_readiness_audit_complete_not_live_ready','execution_ready':False,'ros_started':False,'final_seeds_accessed':False,
      'layout_count':18,'episode_manifest_count':36,'static_witness_count':sum(r['static_ackermann']['status']=='static_forward_ackermann_witness' for r in layout_reports),
      'sdf_actor_parameter_mismatch_count':sum(len(r['sdf_consistency_issues']) for r in episode_reports),
      'actor_swept_static_collision_count':sum(bool(a['actor_full_swept_box_intersections']) for a in allactors),
      'unreactive_conservative_conflict_actor_count':sum(a['conservative_conflict_proxy_present'] for a in allactors),
      'occlusion_initial_center_blocked_count':sum(a['initial_gate_occlusion']['blocked_center_ray'] for e in episode_reports if 'occluded_emergence' in e['layout_id'] for a in e['actors']),
      'layouts':layout_reports,'episodes':episode_reports,
      'component_only_adapters':{'route_world':'RouteFrame point/pose/box/path transforms + body command invariance; 3 tests pass; not live wired',
        'gate_delay':'GateRelativeActivation binds delay to first ego route gate crossing and rejects clock reset; not live wired'},
      'live_rng_audit':{'actor_parameters':'sampled values actually encoded in SDF and read by plugin','actor_response_seed':'encoded and read but jitter multiplied by zero in v3_reactive_actor.h; no stochastic behavior',
        'sensor_noise_seed':'separate recorded seed is not bound to Gazebo scanner; live launcher uses one simulation seed','actuator_seed':'spawn uses Gazebo episode seed for dynamics; recorded separate actuator seed is not wired; command_jitter=0 makes jitter inactive',
        'gate_delay':'metadata sampled but SDF start_delay remains absolute5.0 and gate_x=-4.02; no gate-relative delay support'},
      'automatic_proof_scope':['Initial/end 2D footprint clearance','Explicit forward Ackermann static path witness under .40 rad steering/.75 rad_s slew and .33m margin plus .03m sample envelope','Full straight actor swept-box/static intersections','Five-ray initial occlusion proxy and constant-speed interaction proxy','SDF parameter consistency'],
      'pending_live_or_new_version':['Correct any actor/static swept penetration before live qualification; preserve these diagnostic layouts','Exact raycast visibility including all scan beams and actual ego motion','Full common-gate response timing with a versioned plugin','Wire named RNG streams and test reset/call-order invariance','Live source-stamp TF/odom/scan/time and wheel/body settling','Real dynamic interaction under every method; proxy is not navigation/performance evidence','Route/world transformations integrated into planner observation/map/path/evaluator and actor activation; pure functions alone are insufficient'],
      'planning_limitations':['Static witness ignores actors and shows existence only; it does not certify learned candidate coverage or completion within 22s.','Search has finite40000-node computation limit; no witness is not a proof of geometric impossibility.','Witness runs at0.4m/s with steering actuator timeconstant.18s and.75rad/s; steering cap.40 is conservative relative to actual plugin.4964.','Witness uses convex hull of actual horizontal chassis and all four projected wheel cylinders with inner/outer Ackermann steering; roll/pitch/vertical geometry remains a live gate.','Five-ray visibility is a geometric screening test, not a complete scan simulation.'],
      'dependencies':[{'path':str(p),'sha256':SHA(p)} for p in deps],
      'environment':{'python':sys.version,'shapely':shapely.__version__}}
    out=STAGE/'audits/scene_execution_readiness.json';out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ['status','static_witness_count','sdf_actor_parameter_mismatch_count','actor_swept_static_collision_count','unreactive_conservative_conflict_actor_count','occlusion_initial_center_blocked_count']},indent=2))


if __name__=='__main__':main()
