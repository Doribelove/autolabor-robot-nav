"""Complete actual node main() with generated ROS messages and stub transports."""
import hashlib,importlib.util,io,json,math,os,sys,types,unittest
from pathlib import Path
from unittest.mock import patch
P=Path(__file__).resolve().parents[1];W=P.parents[1];G=W/'src/simulation/m2_gazebo';S=W.parent
sys.path.insert(0,str(P));sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages')
import genpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.srv import GetPlanRequest
from std_msgs.msg import String
from common_route import CommonRoute
from baseline_rear_runtime_b03 import SCHEMA,METHODS
BUNDLE=S/'scenes/navigation_n01_clear_r03/common_bundle'

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def bundle():
    return None,CommonRoute([(-4.,0.),(5.,0.)]),{'frame_id':'map','provider_returns_exact_full_polyline':True,'goal_xy_yaw':[5.,0.,0.],'start_xy_yaw':[-4.,0.,0.],'map_from_odom_xy_yaw':[-5.4,0.,0.]}

def odom(ns,x=0.,v=.8):
    m=Odometry();m.header.stamp=genpy.Time(ns//10**9,ns%10**9);m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.position.x=x;m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=v;m.twist.twist.linear.y=.2;m.twist.twist.angular.z=.5;return m


def transport_fixture(node_name,params):
    fake=types.ModuleType('rospy');state={'ns':0,'ticks':0};pubs={};subs={};services={}
    class Time(genpy.Time):
        @staticmethod
        def now():return genpy.Time(state['ns']//10**9,state['ns']%10**9)
    class Publisher:
        def __init__(self,topic,kind,**kw):self.topic=topic;self.messages=[];self.options=kw;pubs[topic]=self
        def publish(self,m):
            out=io.BytesIO();m.serialize(out);assert out.getvalue();self.messages.append(m)
        def get_num_connections(self):return 2
    fake.init_node=lambda _:None;fake.get_name=lambda:node_name;fake.get_param=lambda key,default=None:params.get(key,default)
    fake.Time=Time;fake.Publisher=Publisher;fake.Subscriber=lambda topic,kind,cb,**kw:subs.setdefault(topic,cb)
    fake.Service=lambda name,kind,cb:services.setdefault(name,cb);fake.ServiceException=RuntimeError
    fake.is_shutdown=lambda:state['ticks']>=5
    return fake,state,pubs,subs,services


class FullAdapterNodeTests(unittest.TestCase):
    def exercise(self,lead=0,fault=None):
        params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'~run_id':'fixture','~seed':42,'~method':METHODS[0],'~bundle':str(BUNDLE),'~bundle_sha256':hashlib.sha256((BUNDLE/'common_route.json').read_bytes()).hexdigest()}
        fake,state,pubs,subs,services=transport_fixture('/astra_baseline_rear_adapter_b03',params)
        node=load('complete_adapter_fixture_b03',G/'scripts/astra_baseline_rear_adapter_b03.py');tf=types.ModuleType('tf2_ros');transforms=[];plans=[]
        class Broadcaster:
            def sendTransform(self,t):transforms.append(t);subs['/tf_static'](types.SimpleNamespace(transforms=[t],_connection_header={'callerid':'/astra_baseline_rear_adapter_b03'}))
        tf.StaticTransformBroadcaster=Broadcaster
        graph=types.ModuleType('rosgraph');graph.Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/odom',['/gazebo']),('/astra/rear_odom',['/astra_baseline_rear_adapter_b03'])],[],[]))
        def advance(_):
            state['ticks']+=1;state['ns']=1_000_000_000+20_000_000*(state['ticks']-1)
            source=state['ns']+lead
            if fault=='duplicate' and state['ticks']==2:source=1_000_000_000+lead
            if fault=='foreign_tf' and state['ticks']==2:subs['/tf'](types.SimpleNamespace(transforms=[transforms[0]],_connection_header={'callerid':'/foreign'}))
            subs['/odom'](odom(source))
            if state['ticks']==3 and fault is None:
                request=GetPlanRequest();request.start.header.frame_id=request.goal.header.frame_id='map'
                request.start.pose.position.x=-4.325;request.goal.pose.position.x=4.675;request.start.pose.orientation.w=request.goal.pose.orientation.w=1.
                plans.append(services['/astra/baseline_rear/get_plan'](request))
        with patch.dict(sys.modules,{'rospy':fake,'tf2_ros':tf,'rosgraph':graph}),patch.dict(os.environ,{'ROS_MASTER_URI':'http://127.0.0.1:14820'}),patch('common_route.load_bundle',return_value=bundle()),patch.object(node.time,'sleep',advance),patch.object(node.time,'monotonic',side_effect=lambda:100+state['ticks']*.05):node.main()
        statuses=[json.loads(m.data) for m in pubs['/astra/baseline_rear/status'].messages]
        self.assertEqual(len(transforms),1);self.assertEqual(transforms[0].child_frame_id,'rear_axle_kinematic')
        self.assertNotIn('/cmd_vel_navigation',pubs)
        if fault:
            self.assertTrue(any(m['state']=='fault' for m in statuses));self.assertTrue(all(m['state']=='fault' for m in statuses[next(i for i,m in enumerate(statuses) if m['state']=='fault'):]))
            self.assertLessEqual(len(pubs['/astra/rear_odom'].messages),1)
        else:
            self.assertTrue(any(m['ready'] for m in statuses));self.assertEqual(plans[0].plan.poses[-1].pose.position.x,4.675)
            self.assertTrue(all(m.header.stamp.to_nsec()%20_000_000==lead for m in pubs['/astra/rear_odom'].messages))
            self.assertTrue(all(abs(m.twist.twist.linear.y-.0375)<1e-12 for m in pubs['/astra/rear_odom'].messages))
    def test_complete_node_straight_transport(self):self.exercise()
    def test_complete_node_future_clock_queue(self):self.exercise(2_000_000)
    def test_complete_node_duplicate_permanent_fault(self):self.exercise(fault='duplicate')
    def test_complete_node_foreign_tf_owner_fault(self):self.exercise(fault='foreign_tf')


class FullCoordinatorNodeTests(unittest.TestCase):
    def exercise(self,method,lead=0,fault=False):
        params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'~run_id':'fixture','~seed':42,'~method':method,'~bundle':str(BUNDLE),'~bundle_sha256':hashlib.sha256((BUNDLE/'common_route.json').read_bytes()).hexdigest()}
        fake,state,pubs,subs,services=transport_fixture('/astra_navigation_coordinator',params);state['gate_tick']=None
        node=load('complete_coordinator_fixture_b03',G/'scripts/astra_navigation_coordinator_b03.py');arms=[];goals=[];cancels=[]
        action=types.ModuleType('actionlib')
        class Client:
            def __init__(self,*args):pass
            def wait_for_server(self):return True
            def send_goal(self,goal,active_cb,done_cb):
                goals.append(goal);state['gate_tick']=state['ticks'];self.goal=goal
                assert goal.target_pose.pose.position.x==4.675
                active_cb();done_cb(3,object())  # Deliberate success before actual base goal.
            def cancel_all_goals(self):cancels.append(state['ns'])
        action.SimpleActionClient=Client
        graph=types.ModuleType('rosgraph');graph.Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/cmd_vel_navigation',['/astra_navigation_coordinator']),('/astra/method_cmd',['/move_base'])],[],[]))
        class Thread:
            def __init__(self,target,daemon):self.target=target
            def start(self):self.target()
        def advance(_):
            state['ticks']+=1;state['ns']=4_800_000_000+(state['ticks']-1)*20_000_000
            age=max(0.,(state['ns']-5_000_000_000)/1e9);world=-5.4+.8*age;v=0. if age==0 else .8
            if state['gate_tick'] is not None and state['ticks']>=state['gate_tick']+6:world=4.8;v=.4
            m=odom(state['ns'],world+5.4,v);m.twist.twist.linear.y=m.twist.twist.angular.z=0.;subs['/odom'](m)
            row={'schema':SCHEMA,'run_id':'fixture','seed':42,'method':method,'state':'ready','ready':True,'status_source_ns':state['ns']+lead,'latest_odom_source_ns':state['ns']+lead}
            if fault and state['gate_tick'] is not None and state['ticks']>=state['gate_tick']+3:row.update(state='fault',ready=False,reason='injected actual adapter fault')
            subs['/astra/baseline_rear/status'](String(data=json.dumps(row)))
            if state['ticks']==(3 if lead else 2):arms.append(services['/astra/navigation/arm'](None))
            if state['gate_tick'] is not None:
                cmd=Twist();cmd.linear.x=.6;subs['/astra/method_cmd'](cmd)
        fake.is_shutdown=lambda:state['ticks']>=200 or (state['gate_tick'] is not None and state['ticks']>=state['gate_tick']+10)
        with patch.dict(sys.modules,{'rospy':fake,'actionlib':action,'rosgraph':graph}),patch.dict(os.environ,{'ROS_MASTER_URI':'http://127.0.0.1:14820','THESIS_WS':str(W)}),patch('common_route.load_bundle',return_value=bundle()),patch.object(node.threading,'Thread',Thread),patch.object(node.time,'sleep',advance),patch.object(node.time,'monotonic',side_effect=lambda:100+state['ticks']*.01),patch.object(node.time,'monotonic_ns',side_effect=lambda:10**11+state['ticks']*10**7):node.main()
        self.assertEqual([r.success for r in arms],[True]);self.assertEqual(len(goals),1)
        private=[json.loads(m.data) for m in pubs['/astra/baseline_rear/private_action_result'].messages]
        self.assertEqual(private[0]['shared_phase'],'active');self.assertFalse(private[0]['is_task_terminal_authority'])
        trace=[json.loads(m.data) for m in pubs['/astra/navigation/command_trace'].messages]
        final=trace[-1];self.assertEqual(final['phase'],'fault' if fault else 'complete');self.assertEqual(final['v'],0.)
        if not fault:self.assertEqual(final['terminal_time_authority'],'first_actual_odom_header_meeting_base_goal')
        else:self.assertNotIn('complete',{r['phase'] for r in trace})
        self.assertTrue(cancels)
    def test_teb_goal_shift_preserves_actual_base_terminal(self):self.exercise(METHODS[0])
    def test_dwa_future_adapter_status_wait_preserves_base_terminal(self):self.exercise(METHODS[1],2_000_000)
    def test_adapter_fault_stops_shared_output(self):self.exercise(METHODS[1],fault=True)

if __name__=='__main__':unittest.main()
