import json,sys,math
from pathlib import Path
import numpy as np
s=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');sys.path.insert(0,str(s/'workspace/scripts/astra_gazebo'))
from common_route import load_bundle
from route_candidate_core import RouteAwareRuntime,ACTION_NAMES
from route_sensor_buffer import ScanFrame
urdf=(s/'audits/m12_p02_robot_description.xml').read_text();results=[]
for directory in sorted((s/'audits/route_runtime_components_v3/new_guide_radius_1m').iterdir()):
 grid,route,data=load_bundle(directory);c=RouteAwareRuntime(grid,route,urdf)
 actual={'stamp_ns':2_000_000_000,'pose_map':tuple(data['start_xy_yaw']),'body_twist':(0.,0.,0.),'roll_pitch':(0.,0.),'joints':{'front_left_steer_joint':0.,'front_right_steer_joint':0.}}
 state=c.state(actual);chosen=0;trace=[]
 for tick in range(440):
  now=2_000_000_000+tick*50_000_000;k=math.tan(state['steer'])/.65
  actual.update(stamp_ns=now,pose_map=state['base'],body_twist=(state['vx'],state['vy']+.325*state['w'],state['w']),joints={'front_left_steer_joint':math.atan(.65*k/(1-.3*k)),'front_right_steer_joint':math.atan(.65*k/(1+.3*k))})
  scan=ScanFrame(now,np.full(72,8.),np.empty((0,2)),state['base'])
  if c.goal_reached(actual):break
  if tick%5==0:built=c.build(actual,scan,np.full((6,72),8.),tick*.05,now);chosen=c.select(built)[0]
  v,w,executed,reason,clearance=c.command(actual,scan,chosen,now)
  state=c.step(c.state(actual),executed,.05);c.previous_action=executed
  trace.append({'t_s':tick*.05,'base':state['base'],'chosen':chosen,'executed':executed,'clearance':clearance,'reason':reason})
 results.append({'layout':directory.name,'goal':c.goal_reached(actual),'duration_s':tick*.05,'terminal_base':state['base'],'goal_base':data['goal_xy_yaw'],'minimum_backup_margin':min(x['clearance'] for x in trace),'stop_ticks':sum(x['executed']==0 for x in trace),'trace':trace})
 print(directory.name,results[-1]['goal'],results[-1]['duration_s'],state['base'],flush=True)
report={'scope':'synthetic closed-loop nominal bicycle integration only; no actual sensor/Gazebo/physical navigation evidence','runtime_core_sha256':__import__('hashlib').sha256(Path(c.__class__.__module__ and s/'workspace/scripts/astra_gazebo/route_candidate_core.py').read_bytes()).hexdigest(),'results':results}
(s/'audits/route_runtime_components_v3/nominal_closed_loop.json').write_text(json.dumps(report,indent=2)+'\n')
