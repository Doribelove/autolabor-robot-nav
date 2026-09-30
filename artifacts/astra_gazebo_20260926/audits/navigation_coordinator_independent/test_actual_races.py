"""Focused fault fixtures execute frozen actual core/node with offline fake transport."""
import hashlib,importlib.util,json,sys,types
from pathlib import Path
from unittest.mock import patch
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'));sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages')
def module(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m
core=module('navigation_coordinator',D/'snapshot_r0/navigation_coordinator.py')
def ready():
 c=core.NavigationCoordinator(core.CoordinatorConfig('fixture',42,'astra_analytic_route_v1'));c.observe_odom(1000000000,(-5.4,0,0),(0,0,0));c.method_status(dict(run_id='fixture',seed=42,method='astra_analytic_route_v1',state='ready',ready=True));c.arm(1000000000);return c
def gated():
 c=ready();c.observe_odom(5000000000,(-5.4,0,0),(0,0,0));c.output(5000000000);c.observe_odom(7000000000,(-4.01,0,0),(.8,0,0));c.observe_odom(7020000000,(-3.994,0,0),(.8,0,0));c.method_status(dict(run_id='fixture',seed=42,method='astra_analytic_route_v1',state='active',ready=True,gate_source_ns=7020000000));return c
def identity(c):return dict(phase=c.phase,source_ns=c.terminal_source_ns,reason=c.terminal_reason,authority=c.terminal_time_authority)
cases=[]
for terminal in ('complete','timeout','stopped'):
 c=gated()
 if terminal=='complete':c.observe_odom(8000000000,(4.8,0,0),(.4,0,0))
 elif terminal=='timeout':c.observe_odom(29020000000,(0,0,0),(.8,0,0))
 else:c.request_stop()
 before=identity(c);c.fault('late_method_or_clock_fault');cases.append(dict(case='late_fault_after_'+terminal,before=before,after=identity(c),original_terminal_overwritten=before!=identity(c)))
c=ready();c.observe_odom(5002000000,(-5.4,0,0),(0,0,0));row=c.output(5000000000);cases.append(dict(case='valid_odom_callback_two_ms_before_local_clock',odom_ns=5002000000,clock_ns=5000000000,result=row,faulted=row['phase']=='fault'))
for order in ('goal_before_clock_poll','goal_after_clock_poll'):
 c=gated();c.observe_odom(28980000000,(4.5,0,0),(.8,0,0));deadline=c.gate['deadline_source_ns']
 if order=='goal_before_clock_poll':c.observe_odom(29000000000,(4.8,0,0),(.8,0,0))
 c.output(deadline)
 if order=='goal_after_clock_poll':c.observe_odom(29000000000,(4.8,0,0),(.8,0,0))
 cases.append(dict(case=order,goal_source_ns=29000000000,declared_deadline_ns=deadline,result=identity(c)))
# Execute the complete actual ROS node. Generated ROS messages are real; only transport/clock are fake.
def node_case(late):
 import genpy
 from nav_msgs.msg import Odometry
 from geometry_msgs.msg import Twist
 from std_msgs.msg import String
 node=module('coordinator_actual_node_'+late,D/'snapshot_r0/astra_navigation_coordinator.py');bundle=S/'scenes/navigation_n01_clear_r03/common_bundle';params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'~run_id':'fixture','~seed':42,'~method':'astra_analytic_route_v1','~bundle':str(bundle),'~bundle_sha256':hashlib.sha256((bundle/'common_route.json').read_bytes()).hexdigest()};pubs={};subs={};services={};state={'ticks':0,'now':0,'wall':100.,'gate_tick':None,'goal_tick':None};instances=[];orig=core.NavigationCoordinator
 def constructor(*a):v=orig(*a);instances.append(v);return v
 def stamp(ns=None):return genpy.Time.from_sec((state['now']if ns is None else ns)/1e9)
 def status(kind,**kw):return String(data=json.dumps(dict(run_id='fixture',seed=42,method='astra_analytic_route_v1',state=kind,ready=True,**kw)))
 class Publisher:
  def __init__(self,topic,kind,**kw):self.resolved_name=topic;self.messages=[];pubs[topic]=self
  def get_num_connections(self):return 2
  def publish(self,m):
   self.messages.append(m)
   if self.resolved_name=='/astra/navigation/gate':
    g=json.loads(m.data);state['gate_tick']=state['ticks'];subs['/astra/method/status'](status('active',gate_source_ns=g['source_ns']))
 def advance(_):
  state['ticks']+=1;state['wall']+=.2 if state['goal_tick']is not None else .01
  if state['goal_tick']is not None:
   if late=='method_fault'and state['ticks']==state['goal_tick']+1:subs['/astra/method/status'](status('fault',reason='late_sensor_callback'))
   return
  state['now']=4800000000+(state['ticks']-1)*20000000;age=max(0.,(state['now']-5000000000)/1e9);x=-5.4+.8*age;v=0. if age==0 else .8
  if state['gate_tick']is not None and state['ticks']>=state['gate_tick']+4:x=4.8;v=.4;state['goal_tick']=state['ticks']
  msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.header.stamp=stamp();msg.pose.pose.position.x=x+5.4;msg.pose.pose.orientation.w=1.;msg.twist.twist.linear.x=v;subs['/odom'](msg)
  if state['ticks']==1:subs['/astra/method/status'](status('ready'));assert services['/astra/navigation/arm'](None).success
  if state['gate_tick']is not None:msg=Twist();msg.linear.x=.6;subs['/astra/method_cmd'](msg)
 fake=types.ModuleType('rospy');fake.init_node=lambda _:None;fake.get_name=lambda:'/astra_navigation_coordinator';fake.get_param=lambda n,d=None:params.get(n,d);fake.Publisher=Publisher;fake.Subscriber=lambda topic,kind,fn,**kw:subs.setdefault(topic,fn);fake.Service=lambda name,kind,fn:services.setdefault(name,fn);fake.Time=types.SimpleNamespace(now=stamp);fake.is_shutdown=lambda:state['goal_tick']is not None and state['ticks']>=state['goal_tick']+(32 if late=='paused_clock_stall'else 3)
 graph=types.ModuleType('rosgraph');graph.Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/cmd_vel_navigation',['/astra_navigation_coordinator']),('/astra/method_cmd',['/astra_route_runtime'])],[],[]));action=types.ModuleType('actionlib');node.time=types.SimpleNamespace(monotonic=lambda:state['wall'],monotonic_ns=lambda:round(state['wall']*1e9),sleep=advance)
 with patch.dict(sys.modules,{'rospy':fake,'rosgraph':graph,'actionlib':action}),patch.dict(__import__('os').environ,{'ROS_MASTER_URI':'http://127.0.0.1:14820','THESIS_WS':str(W)}),patch.object(core,'NavigationCoordinator',constructor):node.main()
 trace=[json.loads(x.data)for x in pubs['/astra/navigation/command_trace'].messages];return dict(case='full_actual_node_'+late,final=identity(instances[0]),terminal_trace=[x for x in trace if x['phase']in('complete','fault','timeout')],published_commands_after_goal_all_zero=all(x['v']==x['w']==0. for x in trace if x['publisher_visible_clock_ns']>=state['now']))
cases += [node_case('method_fault'),node_case('paused_clock_stall')]
assert all(x['original_terminal_overwritten']for x in cases[:3]);assert cases[3]['faulted'];assert cases[4]['result']['phase']=='complete'and cases[5]['result']['phase']=='timeout';assert all(x['final']['phase']=='fault'for x in cases[-2:]);assert all(any(t['phase']=='complete'for t in x['terminal_trace']) for x in cases[-2:])
report={'status':'actual_frozen_code_defects_reproduced_offline','no_ROS':True,'cases':cases,'source_snapshot_manifest':json.loads((D/'snapshot_manifest_r0.json').read_bytes()),'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'command':str(W/'.venv/bin/python')+' '+str(Path(__file__).resolve())};(D/'actual_race_results_r0.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'status':report['status'],'cases':[dict(case=x['case'],phase=x.get('result',x.get('after',x.get('final',{}))).get('phase'))for x in cases]},indent=2))
