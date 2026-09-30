"""Focused B04 review: actual bag extraction and inherited physical task boundary."""
import ast,copy,importlib.util,json,pathlib,sys,tempfile,types,unittest
S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';P=W/'scripts/astra_gazebo';G=W/'src/simulation/m2_gazebo';D=pathlib.Path(__file__).parent
sys.path.insert(0,str(P));sys.path.insert(0,str(P/'tests'))
import test_baseline_integration_b04 as author
import baseline_rear_evidence_b04 as e
import navigation_evidence_b04 as q
import rosbag
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped,TransformStamped
from tf2_msgs.msg import TFMessage
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String

def fillpose(message,payload):
    for k,v in zip('xyz',payload['position']):setattr(message.position,k,v)
    for k,v in zip('xyzw',payload['quaternion']):setattr(message.orientation,k,v)

def messages(rows):
    """Reconstruct all actual ROS message types from captured fixture records."""
    for topic,key in e.TOPICS.items():
        if topic in ('/tf','/tf_static'):continue
        for row in rows[key]:
            p=row['payload']
            if topic in e.STRING_TOPICS:m=String(data=json.dumps(p,allow_nan=False));ns=p.get('source_ns',author.START)
            elif key in ('base','rear'):
                m=Odometry();ns=p['source_ns'];m.header.stamp=author.stamp(ns);m.header.frame_id=p['parent_frame'];m.child_frame_id=p['child_frame'];fillpose(m.pose.pose,p)
                for k,v in zip('xyz',p['linear']):setattr(m.twist.twist.linear,k,v)
                for k,v in zip('xyz',p['angular']):setattr(m.twist.twist.angular,k,v)
                m.pose.covariance=p['pose_covariance'];m.twist.covariance=p['twist_covariance']
            elif key=='goals':
                m=MoveBaseActionGoal();ns=p['source_ns'];m.goal_id.id=p['goal_id'];m.goal.target_pose.header.stamp=author.stamp(ns);m.goal.target_pose.header.frame_id=p['frame'];fillpose(m.goal.target_pose.pose,p)
            else:
                m=RosPath();ns=p['source_ns'];m.header.stamp=author.stamp(ns);m.header.frame_id=p['frame']
                for pose in p['poses']:
                    v=PoseStamped();v.header.stamp=author.stamp(pose['source_ns']);v.header.frame_id=pose['frame'];fillpose(v.pose,pose);m.poses.append(v)
            yield topic,m,row['caller'],ns
    for p in rows['tf']:
        m=TransformStamped();m.header.stamp=author.stamp(p['source_ns']);m.header.frame_id=p['parent'];m.child_frame_id=p['child']
        for k,v in zip('xyz',p['transform'][:3]):setattr(m.transform.translation,k,v)
        for k,v in zip('xyzw',p['transform'][3:]):setattr(m.transform.rotation,k,v)
        yield p['topic'],TFMessage(transforms=[m]),p['caller'],author.START

def bag_replay(path):
    # Compile the actual with-Bag loop from assess, not an independently imagined transport call.
    tree=ast.parse(pathlib.Path(q.__file__).read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='assess')
    block=next(n for n in fn.body if isinstance(n,ast.With)and isinstance(n.items[0].context_expr,ast.Call)and isinstance(n.items[0].context_expr.func,ast.Attribute)and n.items[0].context_expr.func.attr=='Bag')
    class BagProxy:
        def Bag(self,unused,mode):return rosbag.Bag(str(path),mode)
    ns={'rosbag':BagProxy(),'root':path.parent,'rear':e,'bag_rear':e.new_evidence(),'samples':[]}
    exec(compile(ast.Module(body=[block],type_ignores=[]),'<actual frozen B04 assess bag loop>','exec'),ns)
    return ns['bag_rear']

def write_bag(path,rows,drop=None,change_caller=None):
    with rosbag.Bag(str(path),'w')as bag:
        for topic,msg,caller,ns in messages(rows):
            if drop==(topic,ns):continue
            if change_caller==(topic,ns):caller='/wrong_publisher'
            header={'callerid':caller,'type':msg._type,'md5sum':msg._md5sum,'message_definition':msg._full_text}
            bag.write(topic,msg,author.stamp(ns+19_000_000),connection_header=header)

class B04Independent(unittest.TestCase):
    def test_all_topic_types_actual_assessor_loop_and_clock_separation(self):
        for method in e.METHODS:
            with self.subTest(method=method),tempfile.TemporaryDirectory(prefix='b04_independent_',dir=str(D))as tmp:
                rows,value,params=author.fixture(method);path=pathlib.Path(tmp)/'fixture.bag';write_bag(path,rows);bag=bag_replay(path)
                result=e.audit(bag,value,author.START,author.END,author.WATERMARK,params)
                self.assertTrue(result['pass_'],result);self.assertTrue(e.bag_source_equality(rows,bag,author.START,author.END,author.WATERMARK))
                self.assertEqual({k:len(v)for k,v in rows.items()},{k:len(v)for k,v in bag.items()})
                self.assertNotEqual(rows['base'][0]['collector_receipt_clock_ns'],bag['base'][0]['collector_receipt_clock_ns'])

    def test_bag_missing_final_receipt_with_complete_collector(self):
        rows,value,params=author.fixture()
        with tempfile.TemporaryDirectory(prefix='b04_independent_',dir=str(D))as tmp:
            path=pathlib.Path(tmp)/'fixture.bag';write_bag(path,rows,drop=(e.PREFIX+'input_receipt',author.END));bag=bag_replay(path)
            self.assertTrue(e.audit(rows,value,author.START,author.END,author.WATERMARK,params)['pass_'])
            self.assertFalse(e.audit(bag,value,author.START,author.END,author.WATERMARK,params)['pass_'])
            self.assertFalse(e.bag_source_equality(rows,bag,author.START,author.END,author.WATERMARK))

    def test_bag_source_caller_mismatch_fails(self):
        rows,value,params=author.fixture()
        with tempfile.TemporaryDirectory(prefix='b04_independent_',dir=str(D))as tmp:
            path=pathlib.Path(tmp)/'fixture.bag';write_bag(path,rows,change_caller=('/astra/rear_odom',author.START));bag=bag_replay(path)
            self.assertFalse(e.audit(bag,value,author.START,author.END,author.WATERMARK,params)['pass_']);self.assertFalse(e.bag_source_equality(rows,bag,author.START,author.END,author.WATERMARK))

    def test_absent_path_packet_cannot_be_saved_by_three_stream_equality(self):
        rows,value,params=author.fixture()
        with tempfile.TemporaryDirectory(prefix='b04_independent_',dir=str(D))as tmp:
            path=pathlib.Path(tmp)/'fixture.bag';write_bag(path,rows,drop=('/move_base/FixedRouteGlobalPlanner/provided_path',author.START));bag=bag_replay(path)
            self.assertTrue(e.bag_source_equality(rows,bag,author.START,author.END,author.WATERMARK))
            result=e.audit(bag,value,author.START,author.END,author.WATERMARK,params);self.assertFalse(result['pass_']);self.assertFalse(result['checks']['actual_global_plugin_provided_full_rear_polyline'])

    def test_unrelated_duplicate_bootstrap_is_not_silently_filtered(self):
        rows,_,_=author.fixture();row=copy.deepcopy(rows['rear'][0]);row['payload']['source_ns']=0;rows['rear']=[row,copy.deepcopy(row)]+rows['rear']
        self.assertFalse(e.drained(rows,author.START,author.END,author.WATERMARK))

    def test_tail_source_slot_is_required_beyond_task_end(self):
        rows,_,_=author.fixture();watermark=author.END+22_000_000
        self.assertFalse(e.drained(rows,author.START,author.END,watermark))
        for key in ('base','rear','receipts'):
            row=copy.deepcopy(rows[key][-1]);row['payload']['source_ns']+=20_000_000;rows[key].append(row)
        self.assertTrue(e.drained(rows,author.START,author.END,watermark))

    def test_receiver_clocks_do_not_overwrite_availability_contract(self):
        rows,value,params=author.fixture();bag=copy.deepcopy(rows)
        for key in ('base','rear','receipts'):
            for row in bag[key]:row['collector_receipt_clock_ns']+=100_000_000
        self.assertTrue(e.bag_source_equality(rows,bag,author.START,author.END,author.WATERMARK))
        bag['receipts'][0]['payload']['available_ns']=author.START+200_000_001
        self.assertFalse(e.audit(bag,value,author.START,author.END,author.WATERMARK,params)['pass_'])

    def test_raw_byte_caller_invalid_UTF8_fails_capture(self):
        m=Odometry();m._connection_header={'callerid':b'\xff'}
        with self.assertRaises(UnicodeDecodeError):e.capture(e.new_evidence(),'/odom',m,1)

    def test_shutdown_before_gate_retains_failed_audit(self):
        rows=e.new_evidence();_,value,_=author.fixture()
        result=e.audit(rows,value,None,None,None,{})
        self.assertFalse(result['pass_']);self.assertFalse(result['checks']['exact_joined_original_rear_receipts_through_watermark'])

    def test_existing_B03_partial_batch_failure_not_repaired_by_assessor(self):
        # Reuse the actual frozen function counterexample and require the new integrity decision.
        case=author.ReferenceEvidence('test_actual_partial_batch_publish_is_incomplete');case.setUp();case.test_actual_partial_batch_publish_is_incomplete()

if __name__=='__main__':unittest.main(verbosity=2)
