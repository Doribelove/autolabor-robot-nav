#!/usr/bin/env python3
"""C++ token/source proof: removing passive telemetry recovers P02 exactly."""
from pathlib import Path
import re,json,hashlib,xml.etree.ElementTree as ET
D=Path(__file__).resolve().parent;S=D.parents[1];G=S/'workspace/src/simulation/m2_gazebo';oldp=G/'src/astra_physical_ackermann_p02_plugin.cpp';newp=G/'src/astra_physical_ackermann_p05_plugin.cpp';old=oldp.read_text();new=newp.read_text()
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def stripcomments(s):return re.sub(r'//[^\n]*|/\*.*?\*/', '',s,flags=re.S)
def tokens(s):return re.findall(r'"(?:\\.|[^"\\])*"|\w+|[^\w\s]',stripcomments(s))
def method(s,name):
 m=re.search(r'  (?:static )?(?:void|bool|double|ros::Time) '+name+r'\([^{}]*\)(?: const)?(?: override)?\s*\{',s);assert m,name
 start=m.start();pos=s.index('{',m.start());depth=1;i=pos+1
 # Braces inside current method strings are balanced, but tokenize to find span robustly.
 while depth:
  if s[i:i+2]=='//':i=s.index('\n',i);continue
  if s[i]=='"':
   i+=1
   while s[i]!='"':
    if s[i]=='\\':i+=1
    i+=1
  elif s[i]=='{':depth+=1
  elif s[i]=='}':depth-=1
  i+=1
 return start,i,s[start:i]
normalized=new.replace('AstraPhysicalAckermannP05Plugin','AstraPhysicalAckermannP02Plugin');projection=normalized
start,end,_=method(projection,'publishCommandIngress');projection=projection[:start]+projection[end:]
projection=projection.replace('    ingress_pub_=driver.advertise<diagnostic_msgs::DiagnosticArray>("command_ingress",1000);\n','').replace(',status_pub_,ingress_pub_;',',status_pub_;').replace('  std::uint32_t ingress_sequence_{0U};\n','')
# Remove the telemetry-only final if/else, then standalone telemetry calls.
projection=re.sub(r'    if \(fault_\) publishCommandIngress\(receipt_time,input_topic,requested_linear,requested_angular,false,"rejected_queue_overflow"\);\n    else publishCommandIngress\(receipt_time,input_topic,requested_linear,requested_angular,true,\n      last_activation_>now\?"accepted_deferred":"accepted_queued",last_activation_\);\n','',projection)
projection=re.sub(r'      publishCommandIngress\([^;]+;\n','',projection)
projection=projection.replace(',"cmd_vel",msg->linear.x,msg->angular.z','').replace(',"ackerman_vel",msg->linear.x,msg->angular.z','')
projection=projection.replace('void enqueue(double speed,double steering,const std::string& input_topic,\n      double requested_linear,double requested_angular)', 'void enqueue(double speed,double steering)')
projection=projection.replace('    if (fault_) {\n      return;\n    }','    if (fault_) return;')
projection=projection.replace('    const auto receipt_time=world_->SimTime();\n    const double now=receipt_time.Double();','    const double now=world_->SimTime().Double();')
assert tokens(old)==tokens(projection), 'projected C++ tokens differ from P02'
names=['Load','read','valid','jointReferenceEnvelopeValid','cmd','ack','enqueue','brake','emergency','reset','bias','control','finiteBaseState','finiteJointState','stamp','validStatusSourceTime','publishPhysicalStatus','publish','publishMetadata','parameters']
methodproof=[]
for name in names:
 a=method(old,name)[2];b=method(projection,name)[2];assert tokens(a)==tokens(b);methodproof.append({'method':name,'projected_tokens_identical':True,'unprojected_body_tokens_identical':tokens(a)==tokens(method(normalized,name)[2])})
def xmlstructure(e):return (e.tag,tuple(sorted(e.attrib.items())),(e.text or '').strip(),tuple(xmlstructure(x) for x in e))
a=ET.parse(D/'p02_expanded.xml').getroot();b=ET.parse(D/'p05_expanded.xml').getroot();newplugin=b.find("gazebo/plugin[@filename='libastra_physical_ackermann_p05_plugin.so']");assert newplugin is not None;newplugin.set('filename','libastra_physical_ackermann_p02_plugin.so');assert newplugin.get('name')=='astra_physical_ackermann_p05';newplugin.set('name','astra_physical_ackermann_p02');assert xmlstructure(a)==xmlstructure(b)
for parent,newfile in [('urdf/m2_physical_p02.urdf.xacro','urdf/m2_physical_p05.urdf.xacro'),('urdf/m2_physical_p02_gazebo.xacro','urdf/m2_physical_p05_gazebo.xacro'),('launch/spawn_m2_physical_p02.launch','launch/spawn_m2_physical_p05.launch')]:
 assert (G/newfile).read_text().replace('physical_p05','physical_p02').replace('ackermann_p05','ackermann_p02')==(G/parent).read_text()
helper=method(new,'publishCommandIngress')[2]
keys=re.findall(r'add\("([^"]+)"',helper);assert keys==['input_topic','requested_linear_x','requested_angular_z','accepted','disposition','backend_fault','activation_time_s']
assert not re.search(r'\b(fault_|received_|last_receipt_|last_activation_|pending_|command_speed_|command_steer_|reference_|geometry_|efforts_)\s*(?:=|\+=|-=|\.)',helper)
assert new.index('ingress_pub_=driver.advertise')<new.index('cmd_sub_=root.subscribe')
assert 'message.header.stamp=ros::Time(receipt_time.sec,receipt_time.nsec);' in helper
(D/'projected_control_source.cpp').write_text(projection)
report={'status':'measurement_only_source_projection_and_URDF_equivalence_pass','new_driver_sha256':sha(newp),'parent_driver_sha256':sha(oldp),'entire_projected_CPP_token_stream_exact':True,'projection_scope':'Erase passive ingress publisher/helper/calls/counter and extra telemetry arguments; collapse Time capture+Double to original one-read expression; remove telemetry-only fault outcome branch. No control statement altered.','methods':methodproof,'expanded_URDF_only_plugin_library_and_instance_name_changed':True,'all_xacro_spawn_changes_only_new_identity_references':True,'ingress_schema':{'topic':'/m2_driver/command_ingress','message_type':'diagnostic_msgs/DiagnosticArray','header_frame_id':'world','header_stamp':'same Gazebo common::Time read used for original last_receipt_ double; exact sec/nsec, no ROS receipt clock','header_seq':'dedicated publication sequence, monotonic per published callback event; ROS transport may populate its publisher sequence','status_name':'astra_physical_command_ingress','hardware_id':'simulation-only','value_keys':keys,'input_topic_values':{'cmd_vel':'/cmd_vel, angular_z yaw rate rad/s','ackerman_vel':'/ackerman_vel, angular_z steering angle rad'},'normal_disposition':'accepted_queued when activation_time<=receipt; accepted_deferred only with later original queue activation','normal_level':0,'normal_accepted':'true','normal_backend_fault':'false','activation_time_s':'Original pending command activation timestamp; not measured force activation. Empty for rejected command.','rejections':['rejected_invalid_command','rejected_existing_fault','rejected_clock_fault','rejected_queue_overflow'],'invalid_header_time':'Logged and omitted, never fabricated; incomplete evidence cannot pass expected event-count gate.'},'no_control_writes_in_telemetry_helper':True,'advertise_before_subscribe':True,'no_build_no_ROS':True,'limits':['Passive ROS publication has finite execution cost; code equivalence does not claim identical stochastic live trajectories.','No new receipt-to-application latency gate or actuator threshold is introduced.','Root independent review and compilation remain pending.'],'sources':[{'path':str(p),'sha256':sha(p)} for p in [oldp,newp,D/'p02_expanded.xml',D/'p05_expanded.xml',Path(__file__)]],'command':'/usr/bin/python3 '+str(Path(__file__).resolve())}
(D/'source_projection_review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(json.dumps({'status':report['status'],'new_driver_sha256':report['new_driver_sha256'],'methods':len(methodproof),'value_keys':keys},indent=2))
