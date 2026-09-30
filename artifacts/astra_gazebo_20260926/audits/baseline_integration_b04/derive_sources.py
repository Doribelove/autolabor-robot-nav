"""Offline source derivation only; inputs frozen, outputs exclusively NEW B04 files."""
from pathlib import Path
import ast,json,hashlib,sys
CHECK='--check' in sys.argv
S=Path(__file__).resolve().parents[2];W=S/'workspace';H=W/'scripts/astra_gazebo';G=W/'src/simulation/m2_gazebo'
def change(s,a,b,count=1):
 if s.count(a)!=count:raise ValueError('unexpected anchor count '+repr(a)+' '+str(s.count(a)))
 return s.replace(a,b)
def write(path,s):
 if CHECK:
  if path.read_text()!=s:raise ValueError('derived bytes differ '+str(path))
  return
 if path.exists():raise FileExistsError(path)
 path.write_text(s)
inputs={}
def read(p):inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();return p.read_text()
q=read(H/'navigation_evidence_n02.py')
q=q.replace('navigation_evidence_n02','navigation_evidence_b04').replace('astra_navigation_collector_n02','astra_navigation_collector_b04').replace('navigation_coordinator_n02','navigation_coordinator_b03')
q=q.replace('actual_teb_common_route','actual_teb_rear_reference_b03').replace('dwa_common_route','dwa_rear_reference_b03').replace('astra_navigation_evidence_n02_v1','astra_navigation_evidence_b04_v1')
q=change(q,'import yaml\n','import yaml\nimport baseline_rear_evidence_b04 as rear\n')
q=change(q,"METHODS=('astra_analytic_route_v1','astra_pct_route_v1','frozen_r2_v060_sensor_lifecycle_v1','actual_teb_rear_reference_b03','dwa_rear_reference_b03')","METHODS=rear.METHODS")
q=change(q,"SERVICE_PHASES=", "ESSENTIAL_BEFORE_ARM=tuple(dict.fromkeys(ESSENTIAL_BEFORE_ARM+rear.REQUIRED_RECORD_TOPICS))\nSERVICE_PHASES=")
q=change(q,"CONTRACT_FIELDS={'schema',", "CONTRACT_FIELDS={'reference_contract','schema',")
q=change(q,"value={'schema':SCHEMA,'stage':stage", "value={'schema':SCHEMA,'reference_contract':json.loads(json.dumps(rear.REFERENCE_CONTRACT)),'stage':stage")
q=change(q,"    if value['schema']!=SCHEMA:raise ValueError('navigation evidence schema differs')", "    if value['schema']!=SCHEMA:raise ValueError('navigation evidence schema differs')\n    if json.dumps(value['reference_contract'],sort_keys=True,allow_nan=False)!=json.dumps(rear.REFERENCE_CONTRACT,sort_keys=True):raise ValueError('fixed B04 rear reference contract differs')\n    rear.static_world_only(read_bound(value['world']).decode())\n    if not set(rear.REQUIRED_RECORD_TOPICS)<=set(value['record_topics']) or not set(rear.REQUIRED_RECORD_TOPICS)<=set(value['required_before_arm']):raise ValueError('complete B04 rear recorder roster required')\n    if not set(rear.REQUIRED_NODES)<=set(value['required_nodes']):raise ValueError('required B04 rear nodes absent')")
q=change(q,"required_sources=[WORKSPACE/'scripts/astra_gazebo/navigation_evidence_b04.py',", "required_sources=[WORKSPACE/'scripts/astra_gazebo/navigation_evidence_b04.py',\n      WORKSPACE/'src/simulation/m2_gazebo/launch/astra_baseline_rear_b04.launch',\n      WORKSPACE/'scripts/astra_gazebo/baseline_rear_evidence_b04.py',\n      WORKSPACE/'scripts/astra_gazebo/baseline_rear_runtime_b03.py',\n      WORKSPACE/'scripts/astra_gazebo/baseline_rear_axle_adapter_b02.py',\n      WORKSPACE/'src/simulation/m2_gazebo/scripts/astra_baseline_rear_adapter_b03.py',\n      WORKSPACE/'src/simulation/m2_gazebo/config/astra_baseline_rear_b03.yaml',")
q=change(q,"    integrity=all(checks.values())", "    rear_reference=rear.audit(raw['rear_reference_evidence'],value,start,end,raw['final_drain']['source_watermark_ns'],raw['planner_full_parameters'])\n    checks['rear_reference_source_and_parameter_integrity']=rear_reference['pass_']\n    checks['rear_reference_observed_graph']=bool(raw['node_roster']) and all(r.get('rear_reference_graph',{}).get('pass_')is True for r in raw['node_roster'])\n    integrity=all(checks.values())")
q=change(q,"'failure_inclusive':True,'task':task", "'failure_inclusive':True,'rear_reference_audit':rear_reference,'task':task")
q=change(q,"    samples=[]\n    with rosbag.Bag", "    samples=[];bag_rear=rear.new_evidence()\n    with rosbag.Bag")
q=change(q,"for _topic,msg,_receipt in bag.read_messages(topics=['/astra/p03/end_links']):", "for _topic,msg,_receipt,connection_header in bag.read_messages(topics=['/astra/p03/end_links']+list(rear.TOPICS),return_connection_header=True):\n            if _topic in rear.TOPICS:\n                msg._connection_header=connection_header\n                rear.capture(bag_rear,_topic,msg,_receipt.to_nsec());continue")
q=change(q,"    checks['rear_reference_source_and_parameter_integrity']=rear_reference['pass_']", "    rear_bag_reference=rear.audit(bag_rear,value,start,end,raw['final_drain']['source_watermark_ns'],raw['planner_full_parameters'])\n    checks['rear_reference_source_and_parameter_integrity']=rear_reference['pass_']\n    checks['rear_reference_bag_integrity']=rear_bag_reference['pass_']\n    checks['rear_reference_bag_collector_source_equality']=rear.bag_source_equality(raw['rear_reference_evidence'],bag_rear,start,end,raw['final_drain']['source_watermark_ns'])")
q=change(q,"'rear_reference_audit':rear_reference,'task':task", "'rear_reference_audit':rear_reference,'rear_reference_bag_audit':rear_bag_reference,'task':task")
write(H/'navigation_evidence_b04.py',q)
c=read(G/'scripts/astra_navigation_collector_n02.py').replace('import navigation_evidence_n02 as q','import navigation_evidence_b04 as q\nimport baseline_rear_evidence_b04 as rear')
c=c.replace("parsed.port!=14820","parsed.port!=14832")
c=c.replace('actual_teb_common_route','actual_teb_rear_reference_b03').replace('dwa_common_route','dwa_rear_reference_b03')
c=change(c,'    from geometry_msgs.msg import Pose','    from nav_msgs.msg import Odometry,Path as RosPath\n    from tf2_msgs.msg import TFMessage\n    from move_base_msgs.msg import MoveBaseActionGoal\n    from geometry_msgs.msg import Pose')
c=change(c,"'errors':[],'images':[],'resource_samples':[],'source_wall_samples':[]}","'errors':[],'images':[],'resource_samples':[],'source_wall_samples':[],\n        'rear_reference_evidence':rear.new_evidence()}")
c=change(c,'    def parse_callback(topic,message):\n',"    def parse_callback(topic,message):\n        if topic in rear.TOPICS:\n            rear.capture(raw['rear_reference_evidence'],topic,message,rospy.Time.now().to_nsec());return\n")
c=change(c,"    if value['camera_topic'] is not None:\n", "    for topic in rear.TOPICS:\n        kind=String if topic in rear.STRING_TOPICS else Odometry if topic in ('/odom','/astra/rear_odom') else TFMessage if topic in ('/tf','/tf_static') else MoveBaseActionGoal if topic=='/move_base/goal' else RosPath\n        subscribers.append(rospy.Subscriber(topic,kind,lambda msg,t=topic:callback(t,msg),queue_size=10000))\n    if value['camera_topic'] is not None:\n")
c=change(c,'    def roster():\n',"    def full_planner_parameters():\n        result=rpc(master_uri,'getParam','/move_base')\n        if type(result)is not dict:raise ValueError('actual move_base parameter namespace absent')\n        rear.planner_parameters(result,value['method']);return result\n    def rear_drained():\n        with lock:return rear.drained(raw['rear_reference_evidence'],start,end,watermark)\n    def roster():\n")
c=change(c,"        raw['node_roster'].append(row);", "        row['rear_reference_graph']=rear.graph_audit(system,require_ready=phase[0]!='bootstrap')\n        row['pass']=row['pass'] and row['rear_reference_graph']['pass_']\n        raw['node_roster'].append(row);")
c=change(c,"bootstrap_planners=actual_planner_parameters();raw['planner_parameters']={'bootstrap':bootstrap_planners}","bootstrap_planners=actual_planner_parameters();raw['planner_parameters']={'bootstrap':bootstrap_planners}\n        raw['planner_full_parameters']={'bootstrap':full_planner_parameters()}")
c=change(c,"'node_processes':bootstrap_processes,'planner_parameters':bootstrap_planners}","'node_processes':bootstrap_processes,'planner_parameters':bootstrap_planners,'planner_full_parameters':raw['planner_full_parameters']['bootstrap']}")
c=change(c,"            raw['recorder_before_arm']=result;return True", "            with lock:\n                if not rear.ready(raw['rear_reference_evidence'],rospy.Time.now().to_nsec()):return False\n            raw['recorder_before_arm']=result;return True")
c=change(c,"        require_roster();terminal_processes=process_roster('terminal');", "        wait_until(rear_drained,drain_deadline[0].remaining(),'rear source receipt join through final50Hz slot')\n        require_roster();terminal_processes=process_roster('terminal');")
c=change(c,"        raw['final_drain']={'source_watermark_ns':watermark", "        raw['planner_full_parameters']['terminal']=full_planner_parameters();drain_deadline[0].check()\n        raw['final_drain']={'source_watermark_ns':watermark")
c=change(c,"'node_processes':terminal_processes,'planner_parameters':terminal_planners}", "'node_processes':terminal_processes,'planner_parameters':terminal_planners,'planner_full_parameters':raw['planner_full_parameters']['terminal']}")
c=change(c,"    passed=(failure is None", "    raw['rear_reference_audit']=rear.audit(raw['rear_reference_evidence'],value,start,end,watermark,raw.get('planner_full_parameters',{}))\n    passed=(raw['rear_reference_audit']['pass_'] and failure is None")
write(G/'scripts/astra_navigation_collector_b04.py',c)
l=read(G/'launch/astra_baseline_rear_b03.launch')
l=change(l,'<arg name="bag_path"/><arg name="bag_topics"/><arg name="collector_script"/>','<arg name="bag_path"/><arg name="bag_topics"/>')
l=change(l,'type="$(arg collector_script)"','type="astra_navigation_collector_b04.py"')
l=change(l,'Source draft only: new collector/stage/actor metadata/release required.','B04 static offline integration: separately reviewed release required before any run.')
write(G/'launch/astra_baseline_rear_b04.launch',l)
outputs=[H/'navigation_evidence_b04.py',G/'scripts/astra_navigation_collector_b04.py',G/'launch/astra_baseline_rear_b04.launch']
for p in outputs:
 if p.suffix=='.py':ast.parse(p.read_text())
if not CHECK:(S/'audits/baseline_integration_b04/derivation.json').write_text(json.dumps({'inputs':inputs,'outputs':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in outputs},'scope':'No B03/N02/N05 bytes modified; static-only B04 evidence integration'},indent=2)+'\n')

if CHECK:print(json.dumps({'derived_outputs_identical':True,'files':3,'frozen_inputs':inputs}))
