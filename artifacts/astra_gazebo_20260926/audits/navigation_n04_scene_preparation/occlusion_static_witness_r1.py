#!/usr/bin/env python3
"""Supplement only; preserves the insufficient r01 sin^4 witness and all inputs."""
import sys,json,math,hashlib
from pathlib import Path
from shapely.geometry import Polygon
from shapely.ops import unary_union
import yaml
S=Path(__file__).resolve().parents[2];sys.path.insert(0,str(S/'workspace/scripts/astra_gazebo'))
import prepare_navigation_n04_scenes as n
P=S/'scenes/navigation_n04_development_r01/development_a02_development_occluded_emergence_000/replicate_00'
scene=yaml.safe_load((P/'scene_manifest.yaml').read_text());world=(P/'world.sdf').read_text();xml,ss,_=n.static_input(world,scene);obstacles,_=n.static_projections(xml,ss)
robot=n.ReferenceRobot((n.REFERENCE/'robot_description.xml').read_text(),(n.REFERENCE/'conversion/conversion.sdf').read_text())
union=unary_union([Polygon(x.polygon_xy) for x in obstacles])
A=1.6;half=4.5;step=.01;ypmax=1.875*A/half;anglemax=(10*math.sqrt(3)/3)*A/half**2
ypppmax=60*A/half**3;kpmax=ypppmax+3*ypmax*anglemax**2
deltaprime=.65*kpmax/(1-.3*anglemax)**2;pointL=math.sqrt(1+ypmax**2)+1.325*anglemax+.3*deltaprime
minimum=None;worst=None;maxcurv=0.;records=[]
for i in range(901):
    x=-4.325+step*i;u=(x+4.325)/half
    if u<=1:t=u;sign=1
    else:t=2-u;sign=-1
    f=10*t**3-15*t**4+6*t**5;fp=30*t*t*(1-t)**2;fpp=60*t*(1-t)*(1-2*t)
    y=A*f;yp=sign*A/half*fp;ypp=A/half**2*fpp;yaw=math.atan(yp);curvature=ypp/(1+yp*yp)**1.5;maxcurv=max(maxcurv,abs(curvature))
    pose=n.Pose3.from_xyz_rpy((x+.325*math.cos(yaw),y+.325*math.sin(yaw),0),(0,0,yaw))
    parts=robot.project(pose,curvature);polys=[Polygon(p.polygon_xy) for p in parts]
    gap=min(p.distance(union) for p in polys)
    if any(p.intersects(union) for p in polys):gap=0.
    if minimum is None or gap<minimum:minimum=gap;worst={'rear_x_m':x,'base_pose':[pose.xyz[0],pose.xyz[1],yaw],'curvature':curvature}
    records.append({'rear_x':x,'base_x':pose.xyz[0],'base_y':pose.xyz[1],'yaw':yaw,'curvature':curvature,'physical_xy_gap':gap})
lower=minimum-pointL*step/2-1e-5
r={'schema':'astra_n04_static_witness_supplement_v1','parent_r01_inputs_unchanged':True,
 'preserved_previous_witness':{'path':str(P/'geometry_qc.json'),'sha256':n.sha(P/'geometry_qc.json'),'status':'sin^4 static witness physical gap positive but margin insufficient'},
 'curve':'piecewise symmetric quintic smoothstep: rear_x -4.325 to4.675; y=A*(10t^3-15t^4+6t^5), t rises0to1 in4.5m then falls1to0',
 'amplitude_m':A,'endpoint_yaw_and_curvature_zero':True,'sample_count':len(records),'rear_x_step_m':step,
 'derivative_bounds':{'abs_yprime':ypmax,'abs_y2prime':anglemax,'abs_y3prime':ypppmax,'abs_curvature_prime':kpmax,'point_L_per_rear_x':pointL},
 'body_radius_m':1.,'steer_local_radius_m':.3,'shape_radius_audit':robot.radius_audit,
 'curvature_abs_bound':anglemax,'sampled_max_abs_curvature':maxcurv,'central_steer_abs_bound':math.atan(.65*anglemax),'central_steer_limit':.4964,
 'sampled_min_physical_xy_m':minimum,'interval_physical_xy_lower_m':lower,'interval_minus_033_lower_m':lower-.33,'worst_sample':worst,
 'static_geometric_witness_pass':lower>.33 and math.atan(.65*anglemax)<.4964,
 'scope':'ideal static planar Ackermann geometry with nine converted primitive shapes and conservative point interval pad; no dynamic actor or tracking/actuator claim',
 'root_geometry_provenance':{'urdf_sha256':n.sha(n.REFERENCE/'robot_description.xml'),'converted_sdf_sha256':n.sha(n.REFERENCE/'conversion/conversion.sdf')},
 'generator_sha256':n.sha(__file__),'live_qualification':False,'scene_bytes_changed':False}
out=Path(__file__).with_suffix('.json');out.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
Path(__file__).with_suffix('.samples.json').write_text(json.dumps(records,allow_nan=False)+'\n');print(json.dumps(r,indent=2))
