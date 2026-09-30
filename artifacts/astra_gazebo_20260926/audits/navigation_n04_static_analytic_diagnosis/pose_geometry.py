"""Six logged actual poses only; no candidate rollout/replay/benchmark."""
from pathlib import Path
import sys,json,hashlib,math
S=Path(__file__).resolve().parents[2];A=Path(__file__).parent;P=S/'workspace/scripts/astra_gazebo';sys.path.insert(0,str(P))
import yaml
from shapely.geometry import box
from route_candidate_core import FullFootprintEnvelope,RouteConfig
R=S/'runs/astra_navigation_n04_b00_astra_analytic_route_v1'
manifest=yaml.safe_load((R/'scene_manifest.yaml').read_text());rows=json.loads((A/'decision_rows.json').read_text())
f=FullFootprintEnvelope((R/'robot_description_runtime.xml').read_text(),RouteConfig())
obstacles=[(f'map_rectangle_{i}',box(b['x']-b['length']/2,b['y']-b['width']/2,b['x']+b['length']/2,b['y']+b['width']/2)) for i,b in enumerate(manifest['static_boxes'])]
result=[]
for i in [8,14,16,20,24,87]:
 r=rows[i];pose=[r['pose_x'],r['pose_y'],r['pose_yaw']];shape=f.at(pose);distances={name:shape.distance(o)-.33 for name,o in obstacles}
 guide_shape=f.at([*r['guide']['nearest_xy'],r['guide']['heading_rad']])
 result.append(dict(index=i,source_ns=r['source_ns'],pose=pose,endpoint_envelope_static_clearance_m=min(distances.values()),nearest_map_rectangle=min(distances,key=distances.get),all_rectangle_clearances_m=distances,upper_extent_y_m=shape.bounds[3],upper_physical_wall_envelope_clearance_m=manifest['parent_corridor_half_width_m']-shape.bounds[3]-.33,route_nearest_heading_aligned_envelope_clearance_m=min(guide_shape.distance(o)-.33 for _,o in obstacles),logged_stop_backup_clearance_m=r['backup_clearances_m'][0]))
out=dict(schema='astra_n04_static_six_actual_pose_geometry_v1',scope='conservative all-steering 16-plane envelope at six logged actual poses, not actual joint compound/ODE sweep; nominal guide pose is descriptive, not an executable recovery path',component_count=f.component_count,footprint_sha256=f.sha256,margin_m=.33,tilt_pad_m=f.tilt_pad,envelope_radius_m=f.radius,upper_raster_inner_y_m=2.7,upper_physical_inner_y_m=manifest['parent_corridor_half_width_m'],physical_vs_raster_inner_edge_m=manifest['parent_corridor_half_width_m']-2.7,rows=result,source_bindings=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in [Path(__file__),P/'route_candidate_core.py',P/'geometry.py',R/'robot_description_runtime.xml',R/'scene_manifest.yaml',A/'decision_rows.json']])
(A/'pose_geometry.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:out[k] for k in ['component_count','tilt_pad_m','physical_vs_raster_inner_edge_m','rows']},indent=2))
