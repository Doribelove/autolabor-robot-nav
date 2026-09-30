"""Focused fault fixtures execute frozen actual core/node with offline fake transport."""
import hashlib,importlib.util,json,sys,types
from pathlib import Path
from unittest.mock import patch
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'));sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages')
def module(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m
core=module('navigation_coordinator',D/'snapshot_r2/navigation_coordinator.py')
def ready():
 c=core.NavigationCoordinator(core.CoordinatorConfig('fixture',42,'astra_analytic_route_v1'));c.observe_odom(1000000000,(-5.4,0,0),(0,0,0));c.method_status(dict(run_id='fixture',seed=42,method='astra_analytic_route_v1',state='ready',ready=True));c.arm(1000000000);return c
def gated():
 c=ready();c.observe_odom(5000000000,(-5.4,0,0),(0,0,0));c.output(5000000000);c.observe_odom(7000000000,(-4.01,0,0),(.8,0,0));c.observe_odom(7020000000,(-3.994,0,0),(.8,0,0));c.method_status(dict(run_id='fixture',seed=42,method='astra_analytic_route_v1',state='active',ready=True,gate_source_ns=7020000000));return c
def identity(c):return dict(phase=c.phase,source_ns=c.terminal_source_ns,reason=c.terminal_reason,authority=c.terminal_time_authority)
def node_case(late,source_lead_ns=0):
 import genpy
 from nav_msgs.msg import Odometry
 from geometry_msgs.msg import Twist
 from std_msgs.msg import String
 node=module('coordinator_actual_node_'+late,D/'snapshot_r2/astra_navigation_coordinator.py');bundle=S/'scenes/navigation_n01_clear_r03/common_bundle';params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'~run_id':'fixture','~seed':42,'~method':'astra_analytic_route_v1','~bundle':str(bundle),'~bundle_sha256':hashlib.sha256((bundle/'common_route.json').read_bytes()).hexdigest()};pubs={};subs={};services={};state={'ticks':0,'now':0,'wall':100.,'gate_tick':None,'goal_tick':None};instances=[];orig=core.NavigationCoordinator
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
   if state['ticks']==state['goal_tick']+1:state['now']+=20000000
   if late=='method_fault'and state['ticks']==state['goal_tick']+2:subs['/astra/method/status'](status('fault',reason='late_sensor_callback'))
   return
  state['now']=4800000000+(state['ticks']-1)*20000000;age=max(0.,(state['now']-5000000000)/1e9);x=-5.4+.8*age;v=0. if age==0 else .8
  if state['gate_tick']is not None and state['ticks']>=state['gate_tick']+4:x=4.8;v=.4;state['goal_tick']=state['ticks']
  msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.header.stamp=stamp(state['now']+source_lead_ns);msg.pose.pose.position.x=x+5.4;msg.pose.pose.orientation.w=1.;msg.twist.twist.linear.x=v;subs['/odom'](msg)
  if state['ticks']==1:subs['/astra/method/status'](status('ready'))
  if state['ticks']==(2 if source_lead_ns==2000000 else 1):
   answer=services['/astra/navigation/arm'](None);assert answer.success==(source_lead_ns<=200000000)
  if state['gate_tick']is not None:msg=Twist();msg.linear.x=.6;subs['/astra/method_cmd'](msg)
 fake=types.ModuleType('rospy');fake.init_node=lambda _:None;fake.get_name=lambda:'/astra_navigation_coordinator';fake.get_param=lambda n,d=None:params.get(n,d);fake.Publisher=Publisher;fake.Subscriber=lambda topic,kind,fn,**kw:subs.setdefault(topic,fn);fake.Service=lambda name,kind,fn:services.setdefault(name,fn);fake.Time=types.SimpleNamespace(now=stamp);fake.is_shutdown=lambda:(source_lead_ns>200000000 and state['ticks']>=3)or(state['goal_tick']is not None and state['ticks']>=state['goal_tick']+(32 if late=='paused_clock_stall'else 4))
 graph=types.ModuleType('rosgraph');graph.Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/cmd_vel_navigation',['/astra_navigation_coordinator']),('/astra/method_cmd',['/astra_route_runtime'])],[],[]));action=types.ModuleType('actionlib');node.time=types.SimpleNamespace(monotonic=lambda:state['wall'],monotonic_ns=lambda:round(state['wall']*1e9),sleep=advance)
 with patch.dict(sys.modules,{'rospy':fake,'rosgraph':graph,'actionlib':action}),patch.dict(__import__('os').environ,{'ROS_MASTER_URI':'http://127.0.0.1:14820','THESIS_WS':str(W)}),patch.object(core,'NavigationCoordinator',constructor):node.main()
 trace=[json.loads(x.data)for x in pubs['/astra/navigation/command_trace'].messages];return dict(case='full_actual_node_'+late,source_lead_ns=source_lead_ns,late_faults=instances[0].late_faults,late_fault_count=instances[0].late_fault_count,trace=trace,final=identity(instances[0]),terminal_trace=[x for x in trace if x['phase']in('complete','fault','timeout')],published_commands_after_goal_all_zero=all(x['v']==x['w']==0. for x in trace if x['publisher_visible_clock_ns']>=state['now']))

cases=[]
for terminal in ('complete','timeout','stopped','fault'):
 c=gated()
 if terminal=='complete':c.observe_odom(8000000000,(4.8,0,0),(.4,0,0))
 elif terminal=='timeout':c.observe_odom(29020000000,(0,0,0),(.8,0,0))
 elif terminal=='stopped':c.request_stop()
 else:c.fault('first_fault')
 before=identity(c);c.fault('late_method_fault');c.fault('publisher_clock_wall_stall');c.request_stop();out=c.output(29040000000);passed=before==identity(c)and c.late_fault_count==2 and out['v']==out['w']==0.;cases.append(dict(case='first_'+terminal+'_immutable',pass_=passed,before=before,after=identity(c),late_faults=c.late_faults))
for order in ('goal_before_clock_poll','goal_after_clock_poll'):
 c=gated();c.observe_odom(28980000000,(4.5,0,0),(.8,0,0));deadline=c.gate['deadline_source_ns']
 if order=='goal_before_clock_poll':c.observe_odom(29000000000,(4.8,0,0),(.8,0,0))
 c.output(deadline)
 if order=='goal_after_clock_poll':c.observe_odom(29000000000,(4.8,0,0),(.8,0,0))
 cases.append(dict(case=order,pass_=c.phase=='complete'and c.terminal_source_ns==29000000000,result=identity(c)))
c=gated();c.observe_odom(29000000000,(4.7,0,0),(.8,0,0));before=c.output(29020000000);c.observe_odom(29020000000,(4.7,0,0),(.8,0,0));cases.append(dict(case='timeout_requires_actual_source_watermark',pass_=before['phase']=='active'and before['v']==before['w']==0. and c.phase=='timeout'and c.terminal_source_ns==29020000000,before=before,after=identity(c)))
c=gated();c.observe_odom(29000000000,(4.7,0,0),(.8,0,0));c.output(29200000001);cases.append(dict(case='unchanged200ms_stale_odom_fault',pass_=c.phase=='fault'and c.fault_reason=='actual_odometry_stale_or_future',result=identity(c)))
c=gated();c.method_command(.8,.02,7030000000,100);before=c.output(7030000000);c.fault('active_sensor_failure');after=c.output(7030000000);cases.append(dict(case='active_fault_revokes_output',pass_=before['v']==.8 and after['v']==after['w']==0 and after['phase']=='fault'))
for lead in (0,2000000,200000001):
 for late in ('method_fault','paused_clock_stall'):
  x=node_case(late,lead)
  if lead>200000000:passed=x['final']['phase']=='fault'and all(t['v']==t['w']==0. for t in x['trace'])and 'future transport bound'in x['final']['reason']
  else:passed=x['final']['phase']=='complete'and x['late_fault_count']>0 and all(t['actual_odom_source_ns']is None or t['actual_odom_source_ns']<=t['publisher_visible_clock_ns']for t in x['trace'])and any(t['phase']=='warmup'and t['v']==.8 for t in x['trace'])and all(t['phase']=='complete'for t in x['terminal_trace'])
  x['pass_']=passed;cases.append(x)
report={'status':'independent_actual_code_r2_focused_acceptance_pass'if all(x['pass_']for x in cases)else'independent_actual_code_r2_failed','pass':all(x['pass_']for x in cases),'case_count':len(cases),'cases':cases,'no_ROS':True,'source_snapshot_manifest':json.loads((D/'snapshot_manifest_r2.json').read_bytes()),'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()};(D/'actual_race_results_r2.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'status':report['status'],'count':len(cases),'failures':[x['case']for x in cases if not x['pass_']]},indent=2));assert report['pass']
