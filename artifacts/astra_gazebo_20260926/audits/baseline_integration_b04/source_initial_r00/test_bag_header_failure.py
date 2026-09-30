"""B04 author synthetic actual-message evidence tests; no ROS master or simulation."""
import ast,copy,hashlib,importlib.util,json,math,os,sys,tempfile,types,unittest
from unittest.mock import patch
from pathlib import Path
import numpy as np,yaml
P=Path(__file__).resolve().parents[1];W=P.parents[1];S=W.parent;G=W/'src/simulation/m2_gazebo'
sys.path.insert(0,str(P));sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages')
import genpy
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped,TransformStamped
from tf2_msgs.msg import TFMessage
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String
import baseline_rear_evidence_b04 as e
import baseline_rear_runtime_b03 as b
import navigation_evidence_b04 as q
from common_route import load_bundle
spec=importlib.util.spec_from_file_location('b04_actual_adapter',G/'scripts/astra_baseline_rear_adapter_b03.py');node=importlib.util.module_from_spec(spec);spec.loader.exec_module(node)
BUNDLE=S/'scenes/navigation_n03_dwa_r01/common_bundle';START=7_000_000_000;END=START+40_000_000;WATERMARK=END+2_000_000

def stamp(ns):return genpy.Time(ns//10**9,ns%10**9)
def publish(rows,topic,msg,owner,ns):
    msg._connection_header={'callerid':owner};e.capture(rows,topic,msg,ns)
def fixture(method=e.METHODS[0]):
    rows=e.new_evidence();value={'run_id':'b04_synthetic','seed':1,'method':method,'shared_bundle':q.binding(BUNDLE/'common_route.json')};ident={k:value[k] for k in ('run_id','seed','method')}
    _,route,bundle=load_bundle(BUNDLE);fixed=b.RearFixedRoute(route,bundle)
    identity=dict(ident,schema=b.SCHEMA,bundle_sha256=value['shared_bundle']['sha256'],input='/odom',output='/astra/rear_odom',tf_parent='base_link',tf_child=b.FRAME,fixed_translation_xyz=[-.325,0.,0.],base_oriented_path_sha256=fixed.converted['base_oriented_path_sha256'],rear_oriented_path_sha256=fixed.converted['rear_oriented_path_sha256'],oracle_used=False,physical_safety_qualified=False,tilted_costmap_equivalence_qualified=False,goal_contract=b.goal_tolerance_contract())
    publish(rows,e.PREFIX+'identity',String(data=json.dumps(identity)),e.ADAPTER,START)
    transform=TransformStamped();transform.header.frame_id='base_link';transform.child_frame_id=b.FRAME;transform.transform.translation.x=-.325;transform.transform.rotation.w=1
    publish(rows,'/tf_static',TFMessage(transforms=[transform]),e.ADAPTER,START)
    stream=b.RearOdometryStream()
    for ns in range(START,END+1,e.PERIOD_NS):
        m=Odometry();m.header.stamp=stamp(ns);m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.position.x=2.;m.pose.pose.position.z=.3
        # Nonzero roll and body angular x/y/z exercise all dimensions and adjoints.
        m.pose.pose.orientation.x=math.sin(.012/2);m.pose.pose.orientation.w=math.cos(.012/2)
        m.twist.twist.linear.x=.8;m.twist.twist.linear.y=.2;m.twist.twist.linear.z=.01
        m.twist.twist.angular.x=.01;m.twist.twist.angular.y=.02;m.twist.twist.angular.z=.5
        m.pose.covariance=list(np.diag([.01,.02,.03,.004,.005,.006]).ravel());m.twist.covariance=list(np.diag([.02,.03,.04,.005,.006,.007]).ravel())
        publish(rows,'/odom',m,'/gazebo',ns+2_000_000)
        stream.ingest(node.message_row(m),ns-2_000_000)
        sample,audit=stream.drain(ns)[0]
        publish(rows,'/astra/rear_odom',node.output_message(sample,Odometry,stamp),e.ADAPTER,ns+2_000_000)
        receipt=dict(ident,schema=b.SCHEMA,source_ns=ns,received_ns=sample.received_ns,available_ns=sample.available_ns,output_frame=b.FRAME,raw_quaternion=audit['raw_quaternion'],raw_quaternion_norm_squared=audit['raw_quaternion_norm_squared'],roll_pitch_yaw=audit['roll_pitch_yaw'],rear_body_twist=list(sample.body_twist),actual_rear_vy_retained=True)
        publish(rows,e.PREFIX+'input_receipt',String(data=json.dumps(receipt)),e.ADAPTER,ns+2_000_000)
        status=dict(ident,schema=b.SCHEMA,state='ready',ready=True,reason=None,status_source_ns=ns,latest_odom_source_ns=ns,pending_count=0,accepted=stream.accepted)
        publish(rows,e.PREFIX+'status',String(data=json.dumps(status)),e.ADAPTER,ns+2_000_000)
    path=RosPath();path.header.frame_id='map';path.header.stamp=stamp(START)
    for x,y,yaw in fixed.converted['rear_poses']:
        p=PoseStamped();p.header=copy.deepcopy(path.header);p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);path.poses.append(p)
    publish(rows,e.PREFIX+'served_path',path,e.ADAPTER,START)
    publish(rows,'/move_base/FixedRouteGlobalPlanner/provided_path',copy.deepcopy(path),'/move_base',START)
    goal=MoveBaseActionGoal();goal.goal.target_pose=copy.deepcopy(path.poses[-1]);goal.goal_id.id='fixture_goal_1';publish(rows,'/move_base/goal',goal,e.COORDINATOR,START)
    config=yaml.safe_load((G/'config/astra_baseline_rear_b03.yaml').read_text());config['base_global_planner']='astra_common_route/FixedRouteGlobalPlanner';config['base_local_planner']='teb_local_planner/TebLocalPlannerROS' if method==e.METHODS[0] else 'dwa_local_planner/DWAPlannerROS'
    return rows,value,{'bootstrap':copy.deepcopy(config),'terminal':copy.deepcopy(config)}

def result(rows,value,params):return e.audit(rows,value,START,END,WATERMARK,params)
class ReferenceEvidence(unittest.TestCase):
    def setUp(self):self.rows,self.value,self.params=fixture()
    def reject(self,key,mutate):
        mutate();r=result(self.rows,self.value,self.params);self.assertFalse(r['pass_'],r);self.assertFalse(r['checks'][key],r)
    def test_two_actual_message_baselines(self):
        for method in e.METHODS:
            r,v,p=fixture(method);audit=result(r,v,p);self.assertTrue(audit['pass_'],audit);self.assertTrue(e.ready(r,END));self.assertNotEqual(r['rear'][0]['payload']['linear'][1],0.)
    def test_missing_receipt_partial_batch(self):self.reject('exact_joined_original_rear_receipts_through_watermark',lambda:self.rows['receipts'].pop(1))
    def test_actual_partial_batch_publish_is_incomplete(self):
        stream=b.RearOdometryStream();outputs=[]
        for ns in (START,START+e.PERIOD_NS):
            message=Odometry();message.header.stamp=stamp(ns);message.header.frame_id='odom';message.child_frame_id='base_link';message.pose.pose.orientation.w=1.
            stream.ingest(node.message_row(message),START-1)
        times=iter([START+e.AGE_NS,START+e.AGE_NS,START+e.PERIOD_NS+e.AGE_NS+1])
        with self.assertRaises(ValueError):node.publish_pending(stream,lambda:next(times),outputs.append,Odometry,stamp)
        self.assertEqual(len(outputs),1);self.assertTrue(stream.fault_reason)
        self.rows['rear']=[];self.rows['receipts']=[]
        publish(self.rows,'/astra/rear_odom',outputs[0],e.ADAPTER,START+e.AGE_NS)
        checked=result(self.rows,self.value,self.params)
        self.assertFalse(checked['pass_']);self.assertFalse(checked['checks']['exact_joined_original_rear_receipts_through_watermark'])
    def test_missing_rear(self):self.reject('exact_joined_original_rear_receipts_through_watermark',lambda:self.rows['rear'].pop())
    def test_missing_base(self):self.reject('exact_joined_original_rear_receipts_through_watermark',lambda:self.rows['base'].pop(1))
    def test_duplicate_source(self):self.reject('exact_joined_original_rear_receipts_through_watermark',lambda:self.rows['rear'].append(self.rows['rear'][-1]))
    def test_regressed_source(self):self.reject('exact_joined_original_rear_receipts_through_watermark',lambda:self.rows['rear'].reverse())
    def test_wrong_frame(self):self.reject('full_3d_odometry_covariance_and_source_availability',lambda:self.rows['rear'][0]['payload'].update(child_frame='base_link'))
    def test_wrong_odom_owner(self):self.reject('full_3d_odometry_covariance_and_source_availability',lambda:self.rows['base'][0].update(caller='/oracle'))
    def test_vy_not_clipped(self):self.reject('full_3d_odometry_covariance_and_source_availability',lambda:self.rows['rear'][0]['payload']['linear'].__setitem__(1,0.))
    def test_covariance_transform(self):self.reject('full_3d_odometry_covariance_and_source_availability',lambda:self.rows['rear'][0]['payload']['pose_covariance'].__setitem__(0,100.))
    def test_future_availability(self):self.reject('full_3d_odometry_covariance_and_source_availability',lambda:self.rows['receipts'][0]['payload'].update(available_ns=START-1))
    def test_stale_availability(self):self.reject('full_3d_odometry_covariance_and_source_availability',lambda:self.rows['receipts'][0]['payload'].update(available_ns=START+e.AGE_NS+1))
    def test_source_mutation(self):self.reject('exact_joined_original_rear_receipts_through_watermark',lambda:self.rows['rear'][0]['payload'].update(source_ns=START+1))
    def test_tf_foreign_owner(self):self.reject('unique_correct_static_rear_TF_authority',lambda:self.rows['tf'][0].update(caller='/foreign'))
    def test_tf_wrong_translation(self):self.reject('unique_correct_static_rear_TF_authority',lambda:self.rows['tf'][0]['transform'].__setitem__(0,.325))
    def test_tf_dynamic_duplicate(self):self.reject('unique_correct_static_rear_TF_authority',lambda:self.rows['tf'].append(dict(self.rows['tf'][0],topic='/tf')))
    def test_adapter_fault_preserved(self):self.reject('rear_status_ready_and_never_faulted',lambda:self.rows['status'][-1]['payload'].update(state='fault',reason='partial_batch'))
    def test_identity_wrong_method(self):self.reject('rear_identity_and_path_hashes',lambda:self.rows['identity'][0]['payload'].update(method='actual_teb_common_route'))
    def test_shortened_full_guide(self):self.reject('actual_global_plugin_provided_full_rear_polyline',lambda:self.rows['provided_paths'][0]['payload']['poses'].pop(0))
    def test_unshifted_goal(self):self.reject('single_gated_rear_action_goal',lambda:self.rows['goals'][0]['payload']['position'].__setitem__(0,5.))
    def test_goal_before_gate(self):self.reject('single_gated_rear_action_goal',lambda:self.rows['goals'][0]['payload'].update(source_ns=START-1))
    def test_goal_after_terminal(self):self.reject('single_gated_rear_action_goal',lambda:self.rows['goals'][0]['payload'].update(source_ns=END+1))
    def test_duplicate_goal(self):self.reject('single_gated_rear_action_goal',lambda:self.rows['goals'].append(self.rows['goals'][0]))
    def test_base_frame_wrong_parameter(self):self.reject('effective_rear_planner_parameters_terminal',lambda:self.params['terminal']['local_costmap'].update(robot_base_frame='base_link'))
    def test_internal_goal_not_relaxed(self):self.reject('effective_rear_planner_parameters_terminal',lambda:self.params['terminal']['TebLocalPlannerROS'].update(xy_goal_tolerance=.25))
    def test_private_success_is_not_shared_terminal(self):
        p={k:self.value[k] for k in ('run_id','seed','method')};p.update(action_state=3,receipt_clock_ns=START,shared_phase='active',is_task_terminal_authority=False)
        publish(self.rows,e.PREFIX+'private_action_result',String(data=json.dumps(p)),e.COORDINATOR,START)
        self.assertTrue(result(self.rows,self.value,self.params)['pass_']);self.assertGreater(abs(self.rows['base'][-1]['payload']['position'][0]-10.4),.25)
    def test_private_result_claiming_terminal_rejected(self):
        p={k:self.value[k] for k in ('run_id','seed','method')};p.update(action_state=3,is_task_terminal_authority=True)
        publish(self.rows,e.PREFIX+'private_action_result',String(data=json.dumps(p)),e.COORDINATOR,START)
        self.assertFalse(result(self.rows,self.value,self.params)['pass_'])
    def test_terminal_none_retains_failed_audit(self):self.assertFalse(e.audit(self.rows,self.value,None,None,None,{})['pass_'])
    def test_final_slot_and_interior_completeness(self):
        self.assertTrue(e.drained(self.rows,START,END,WATERMARK));self.assertFalse(e.drained(self.rows,START,END,END+20_000_000))
    def test_static_world_rejects_actor(self):
        self.assertTrue(e.static_world_only((S/'scenes/navigation_n03_dwa_r01/world.sdf').read_text()))
        for text in ('<sdf><world><model name="actor"><static>false</static></model></world></sdf>','<sdf><world><model name="actor"><static>true</static><plugin filename="libastra_event_actor_n05_plugin.so"/></model></world></sdf>'):
            with self.assertRaises(ValueError):e.static_world_only(text)
    def test_unexpanded_sdf_actor_or_include_rejected(self):
        for text in ('<sdf><world><actor name="pedestrian"/></world></sdf>','<sdf><world><include><uri>model://unknown</uri></include></world></sdf>'):
            with self.assertRaises(ValueError):e.static_world_only(text)

class ActualCoordinatorOrder(unittest.TestCase):
    def exercise(self,method,late):
        # Complete frozen node main(), actual ROS message classes, transport only fake.
        sys.path.insert(0,str(Path(__file__).parent))
        import test_baseline_rear_nodes_b03 as old
        params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'~run_id':'fixture','~seed':42,'~method':method,'~bundle':str(old.BUNDLE),'~bundle_sha256':hashlib.sha256((old.BUNDLE/'common_route.json').read_bytes()).hexdigest()}
        fake,state,pubs,subs,services=old.transport_fixture('/astra_navigation_coordinator',params);state.update(gate_tick=None,done=False)
        actual=old.load('b04_actual_coordinator_order',G/'scripts/astra_navigation_coordinator_b03.py');action=types.ModuleType('actionlib');done=[]
        class Client:
            def __init__(self,*a):pass
            def wait_for_server(self):return True
            def send_goal(self,goal,active_cb,done_cb):state['gate_tick']=state['ticks'];done.append(done_cb);active_cb()
            def cancel_all_goals(self):pass
        class Thread:
            def __init__(self,target,daemon):self.target=target
            def start(self):self.target()
        action.SimpleActionClient=Client;graph=types.ModuleType('rosgraph');graph.Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/cmd_vel_navigation',[e.COORDINATOR]),('/astra/method_cmd',['/move_base'])],[],[]))
        from geometry_msgs.msg import Twist
        def advance(_):
            state['ticks']+=1;state['ns']=4_800_000_000+(state['ticks']-1)*20_000_000
            age=max(0.,(state['ns']-5_000_000_000)/1e9);world=-5.4+.8*age;v=0. if age==0 else .8
            delta=None if state['gate_tick'] is None else state['ticks']-state['gate_tick']
            if delta is not None and delta>=6:world=4.8;v=.4
            if delta is not None and delta>=(8 if late else 2) and not state['done']:done[0](3,object());state['done']=True
            m=old.odom(state['ns'],world+5.4,v);m.twist.twist.linear.y=m.twist.twist.angular.z=0.;subs['/odom'](m)
            subs[e.PREFIX+'status'](String(data=json.dumps({'schema':b.SCHEMA,'run_id':'fixture','seed':42,'method':method,'state':'ready','ready':True,'status_source_ns':state['ns'],'latest_odom_source_ns':state['ns']})))
            if state['ticks']==2:self.assertTrue(services['/astra/navigation/arm'](None).success)
            if delta is not None:cmd=Twist();cmd.linear.x=.6;subs['/astra/method_cmd'](cmd)
        fake.is_shutdown=lambda:state['ticks']>=200 or (state['gate_tick'] is not None and state['ticks']>=state['gate_tick']+10)
        with patch.dict(sys.modules,{'rospy':fake,'actionlib':action,'rosgraph':graph}),patch.dict(os.environ,{'ROS_MASTER_URI':'http://127.0.0.1:14832','THESIS_WS':str(W)}),patch('common_route.load_bundle',return_value=old.bundle()),patch.object(actual.threading,'Thread',Thread),patch.object(actual.time,'sleep',advance),patch.object(actual.time,'monotonic',side_effect=lambda:100+state['ticks']*.01),patch.object(actual.time,'monotonic_ns',side_effect=lambda:10**11+state['ticks']*10**7):actual.main()
        private=[json.loads(m.data) for m in pubs[e.PREFIX+'private_action_result'].messages];trace=[json.loads(m.data) for m in pubs['/astra/navigation/command_trace'].messages]
        self.assertEqual(private[0]['shared_phase'],'complete' if late else 'active');self.assertFalse(private[0]['is_task_terminal_authority']);self.assertEqual(trace[-1]['phase'],'complete');self.assertEqual(trace[-1]['v'],0.)
        return trace[-1]['terminal_source_ns']
    def test_teb_private_first_and_base_first(self):self.assertEqual(self.exercise(e.METHODS[0],False),self.exercise(e.METHODS[0],True))
    def test_dwa_private_first_and_base_first(self):self.assertEqual(self.exercise(e.METHODS[1],False),self.exercise(e.METHODS[1],True))

class GraphAndCallbacks(unittest.TestCase):
    def graph(self):
        r=e.graph_audit(([],[],[]));pubs=r['expected_publishers'];subs={'/odom':[e.ADAPTER],'/tf':[e.ADAPTER,'/move_base'],'/tf_static':[e.ADAPTER,'/move_base'],'/astra/rear_odom':['/move_base'],'/scan':['/move_base']};return [list(pubs.items()),list(subs.items()),[(e.PREFIX+'get_plan',[e.ADAPTER])]]
    def test_graph_baseline(self):self.assertTrue(e.graph_audit(self.graph())['pass_'])
    def test_bootstrap_late_planner_advertisements(self):
        graph=self.graph();graph[0]=[(k,v) for k,v in graph[0] if k not in ('/move_base/goal','/move_base/FixedRouteGlobalPlanner/provided_path','/astra/method_cmd')];graph[1]=[(k,[n for n in v if n!='/move_base']) for k,v in graph[1]]
        self.assertTrue(e.graph_audit(graph,False)['pass_']);self.assertFalse(e.graph_audit(graph,True)['pass_'])
    def test_graph_oracle(self):
        graph=self.graph();graph[1].append(('/gazebo/model_states',['/move_base']));self.assertFalse(e.graph_audit(graph)['pass_'])
    def test_graph_wrong_command_owner(self):
        graph=self.graph();graph[0].append(('/cmd_vel',['/foreign']));self.assertFalse(e.graph_audit(graph)['pass_'])
    def test_actual_collector_parse_callback_closure(self):
        tree=ast.parse((G/'scripts/astra_navigation_collector_b04.py').read_text());main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main');functions=[n for n in main.body if isinstance(n,ast.FunctionDef) and n.name in ('parse_callback','callback')]
        raw={'rear_reference_evidence':e.new_evidence(),'errors':[]};store=q.EvidenceStore(raw)
        namespace={'rear':e,'raw':raw,'evidence':store,'rospy':types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:stamp(START)))}
        exec(compile(ast.Module(body=functions,type_ignores=[]),'<actual B04 callback closure>','exec'),namespace)
        msg=Odometry();msg.header.stamp=stamp(START);msg.header.frame_id='odom';msg.child_frame_id='base_link';msg._connection_header={'callerid':'/gazebo'}
        namespace['callback']('/odom',msg);self.assertEqual(len(raw['rear_reference_evidence']['base']),1)
        namespace['callback']('/odom',object());self.assertEqual(len(raw['errors']),1)
        frozen=store.freeze();namespace['callback']('/odom',msg);namespace['callback']('/odom',object());self.assertEqual(raw,frozen)

class ContractBoundary(unittest.TestCase):
    def setUp(self):self.value=json.loads((S/'audits/baseline_integration_b04/launch_preflight_attempt02'/e.METHODS[0]/'contract.json').read_text())
    def test_real_expanded_contract(self):self.assertEqual(q.validate_contract(self.value),self.value)
    def test_recording_receipt_required(self):
        self.value['record_topics'].remove(e.PREFIX+'input_receipt')
        with self.assertRaises(ValueError):q.validate_contract(self.value)
    def test_original_base_topic_required(self):
        self.value['required_before_arm'].remove('/odom')
        with self.assertRaises(ValueError):q.validate_contract(self.value)
    def test_adapter_node_required(self):
        self.value['required_nodes'].remove(e.ADAPTER)
        with self.assertRaises(ValueError):q.validate_contract(self.value)
    def test_private_success_cannot_become_contract_authority(self):
        self.value['reference_contract']['private_action_result_terminal_authority']=True
        with self.assertRaises(ValueError):q.validate_contract(self.value)
    def test_old_method_cannot_silently_use_rear_contract(self):
        self.value['method']='actual_teb_common_route'
        with self.assertRaises(ValueError):q.validate_contract(self.value)

class BagCompleteness(unittest.TestCase):
    def test_actual_synthetic_bag_source_join_and_receiver_clock_separation(self):
        import rosbag
        rows,_,_=fixture();replayed=copy.deepcopy(rows)
        for key in ('base','rear','receipts'):replayed[key]=[]
        with tempfile.TemporaryDirectory(prefix='b04_synthetic_bag_') as tmp:
            path=Path(tmp)/'toy.bag'
            with rosbag.Bag(str(path),'w') as bag:
                for topic,key in (('/odom','base'),('/astra/rear_odom','rear'),(e.PREFIX+'input_receipt','receipts')):
                    for row in rows[key]:
                        p=row['payload']
                        if key=='receipts':m=String(data=json.dumps(p))
                        else:
                            m=Odometry();m.header.stamp=stamp(p['source_ns']);m.header.frame_id=p['parent_frame'];m.child_frame_id=p['child_frame']
                            for name,val in zip('xyz',p['position']):setattr(m.pose.pose.position,name,val)
                            for name,val in zip('xyzw',p['quaternion']):setattr(m.pose.pose.orientation,name,val)
                            for name,val in zip('xyz',p['linear']):setattr(m.twist.twist.linear,name,val)
                            for name,val in zip('xyz',p['angular']):setattr(m.twist.twist.angular,name,val)
                            m.pose.covariance=p['pose_covariance'];m.twist.covariance=p['twist_covariance']
                        header={'callerid':row['caller'],'type':m._type,'md5sum':m._md5sum,'message_definition':m._full_text}
                        bag.write(topic,m,stamp(p['source_ns']+10_000_000),connection_header=header)
            with rosbag.Bag(str(path),'r') as bag:
                for topic,msg,receipt in bag.read_messages():e.capture(replayed,topic,msg,receipt.to_nsec())
        self.assertTrue(e.bag_source_equality(rows,replayed,START,END,WATERMARK))
        self.assertNotEqual(rows['base'][0]['collector_receipt_clock_ns'],replayed['base'][0]['collector_receipt_clock_ns'])
    def test_collector_complete_bag_missing_packet_rejected(self):
        rows,_,_=fixture();bag=copy.deepcopy(rows);bag['receipts'].pop(1)
        self.assertFalse(e.bag_source_equality(rows,bag,START,END,WATERMARK))
    def test_bag_wrong_source_payload_rejected(self):
        rows,_,_=fixture();bag=copy.deepcopy(rows);bag['rear'][0]['payload']['linear'][1]+=.001
        self.assertFalse(e.bag_source_equality(rows,bag,START,END,WATERMARK))
if __name__=='__main__':unittest.main()
