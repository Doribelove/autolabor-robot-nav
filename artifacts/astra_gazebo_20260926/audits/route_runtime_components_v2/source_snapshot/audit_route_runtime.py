#!/usr/bin/env python3
"""Offline-only six-map component benchmark and nominal geometry diagnostics.

Uses saved development input maps, never episode outcomes or actor truth.
No ROS, simulator, execution identities or stochastic experiment seeds.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
from shapely.geometry import Polygon
from common_route import build_bundle,load_bundle
from geometry import Pose3
from route_candidate_core import RouteAwareRuntime,ACTION_NAMES
from route_sensor_buffer import ScanFrame


def actual(pose,velocity=.6):
    return {'stamp_ns':2_000_000_000,'pose_map':pose,'body_twist':(velocity,0.,0.),'roll_pitch':(0.,0.),'joints':{'front_left_steer_joint':0.,'front_right_steer_joint':0.}}


def compound_margin(core,trajectory):
    minimum=math.inf
    for row in trajectory:
        k=math.tan(row['steer'])/.65
        joints={name:0. for name,joint in core.footprint.model.joints.items() if joint.kind!='fixed'}
        joints.update(front_left_steer_joint=math.atan(.65*k/(1-.3*k)),front_right_steer_joint=math.atan(.65*k/(1+.3*k)))
        pose=Pose3.from_xyz_rpy((*row['base'][:2],0),(0,0,row['base'][2]))
        for part in core.footprint.model.project(pose,joints):minimum=min(minimum,Polygon(part.polygon_xy).distance(core.static)-.33)
    return minimum


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--stage',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.output.exists():raise FileExistsError('new component audit output required')
    args.output.mkdir(parents=True)
    urdf=(args.stage/'audits/m12_p02_robot_description.xml').read_text();rows=[];guide_rows=[]
    samples=args.stage/'audits/common_route_components/samples';exports=args.stage/'audits/shared_map_route_export_samples_r1'
    for directory in sorted(samples.iterdir()):
        if not (directory/'common_route.json').exists():continue
        new=args.output/'new_guide_radius_1m'/directory.name
        try:
            bundle=build_bundle(exports/directory.name,new,1.0)
            guide_rows.append({'layout':directory.name,'status':'geometric_guide_created','path':str(new),'path_sha256':bundle['path_points_sha256'],'route_length_m':bundle['route_length_m'],'clearance_lower_bound_m':bundle['audit']['minimum_segment_clearance_lower_bound_m']})
        except ValueError as exc:guide_rows.append({'layout':directory.name,'status':'no_guide_at_1m','reason':str(exc)})
        for source in (directory,new):
            if not source.exists():continue
            grid,route,data=load_bundle(source);core=RouteAwareRuntime(grid,route,urdf)
            pose=tuple(data['start_xy_yaw']);state=actual(pose);now=state['stamp_ns'];history=np.full((6,72),8.)
            # Deliberately synthetic sensor fixture: no actor IDs/positions read.
            cloud=np.asarray([[pose[0]+4+.2*math.cos(a),pose[1]+1.5+.2*math.sin(a)] for a in np.linspace(0,2*math.pi,48,endpoint=False)])
            scenarios=[]
            for cloud_name,points in [('no_unassociated_returns',np.empty((0,2))),('48_unassociated_fixture_returns',cloud)]:
                fresh=ScanFrame(now,np.full(72,8.),points,pose);build_ms=[];command_ms=[];built=None
                for _ in range(12):
                    begin=time.perf_counter_ns();built=core.build(state,fresh,history,0,now);build_ms.append((time.perf_counter_ns()-begin)/1e6)
                    begin=time.perf_counter_ns();core.command(state,fresh,3,now);command_ms.append((time.perf_counter_ns()-begin)/1e6)
                scenarios.append({'cloud':cloud_name,'build_ms':build_ms,'recheck_ms':command_ms,'build_p95_ms':float(np.percentile(build_ms,95)),'build_max_ms':max(build_ms),'recheck_p95_ms':float(np.percentile(command_ms,95)),'recheck_max_ms':max(command_ms),'mask':built['mask']})
            diagnostics=[]
            if 'static_avoidance' in directory.name or 'occluded_emergence' in directory.name:
                for arc in (0.,1.5,3.,4.5,6.,7.5):
                    pose=route.sample(arc);a=actual(pose);fresh=ScanFrame(now,np.full(72,8.),np.empty((0,2)),pose);built=core.build(a,fresh,history,0,now)
                    for action,trajectory in enumerate(built['previews']):
                        diagnostics.append({'route_arc_m':arc,'action':ACTION_NAMES[action],'horizon_s':trajectory[-1]['t'],'backup_mask':built['mask'][action],
                            'hull_static_interpolant_margin_m':core.verify(trajectory,core.predict(fresh),now),'compound_pointwise_margin_m':compound_margin(core,trajectory),
                            'maximum_nominal_curvature_per_m':max(abs(math.tan(r['steer'])/.65) for r in trajectory),'maximum_reference_steer_rad':max(abs(r['steer']) for r in trajectory),
                            'terminal_route_arc_m':core.project(trajectory[-1]['base'][:2])['arc_m']})
            rows.append({'layout':directory.name,'guide_radius_m':data['planning_clearance_radius_m'],'bundle':str(source),'semantics_sha256':core.semantics_sha256,'footprint_radius_m':core.footprint.radius,'timing':scenarios,'geometry_diagnostics':diagnostics})
    all_build=[v for row in rows for scenario in row['timing'] for v in scenario['build_ms']];all_recheck=[v for row in rows for scenario in row['timing'] for v in scenario['recheck_ms']]
    report={'status':'offline_component_diagnostics_complete','live_run':False,'experiment_units_consumed':0,'new_guide_adopted':False,'all_methods_must_share_same_future_adopted_bundle':True,
        'scope':'wall-time component timings on saved maps and synthetic clouds; nominal model geometry, not Gazebo safety/navigation/performance qualification',
        'new_guides':guide_rows,'rows':rows,'aggregate':{'build_p95_ms':float(np.percentile(all_build,95)),'build_max_ms':max(all_build),'recheck_p95_ms':float(np.percentile(all_recheck,95)),'recheck_max_ms':max(all_recheck)},
        'source_sha256':{name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in ('audit_route_runtime.py','route_candidate_core.py','route_sensor_buffer.py','route_actor_adapter.py')}}
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps({'aggregate':report['aggregate'],'new_guides':guide_rows}))

if __name__=='__main__':main()
