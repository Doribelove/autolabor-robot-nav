#!/usr/bin/env python3
"""New simulation-only actual odom/reference adapter and fixed rear-path service.

No command publication, simulator service, oracle subscription or online search.
Original /odom and base_link TF remain authoritative for the shared outcome.
"""
import hashlib,json,math,os,sys,threading,time
from pathlib import Path
from urllib.parse import urlparse
HELPERS=Path(__file__).resolve().parents[4]/'scripts/astra_gazebo'
sys.path.insert(0,str(HELPERS))
from baseline_rear_runtime_b03 import RearOdometryStream,RearFixedRoute,FRAME,METHODS,SCHEMA,goal_tolerance_contract


def message_row(message):
    p=message.pose.pose;q=p.orientation;t=message.twist.twist
    return {'source_ns':message.header.stamp.to_nsec(),'parent_frame':message.header.frame_id,
        'child_frame':message.child_frame_id,'position':(p.position.x,p.position.y,p.position.z),
        'quaternion':(q.x,q.y,q.z,q.w),'linear':(t.linear.x,t.linear.y,t.linear.z),
        'angular':(t.angular.x,t.angular.y,t.angular.z),'pose_covariance':tuple(message.pose.covariance),
        'twist_covariance':tuple(message.twist.covariance)}


def output_message(sample,factory,stamp_factory):
    result=factory();result.header.stamp=stamp_factory(sample.source_ns);result.header.frame_id=sample.parent_frame;result.child_frame_id=FRAME
    p=result.pose.pose
    p.position.x,p.position.y,p.position.z=sample.pose.xyz
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=sample.pose.xyzw
    t=result.twist.twist;t.linear.x,t.linear.y,t.linear.z=sample.body_twist[:3];t.angular.x,t.angular.y,t.angular.z=sample.body_twist[3:]
    result.pose.covariance=list(sample.pose_covariance);result.twist.covariance=list(sample.twist_covariance)
    return result


def publish_pending(stream,now_ns,publish,factory,stamp_factory):
    """Actual node transport path, injectable for small fake-message tests."""
    published=[]
    for sample,audit in stream.drain(now_ns()):
        output=output_message(sample,factory,stamp_factory)
        stream.check_publication(sample,now_ns())
        publish(output);published.append((sample,audit))
    return published


def consume_message(stream,message,now_ns,publish,factory,stamp_factory):
    stream.ingest(message_row(message),now_ns())
    return publish_pending(stream,now_ns,publish,factory,stamp_factory)


def route_response_poses(service,request):
    s=request.start;g=request.goal
    def extract(p):
        q=p.pose.orientation;v=p.pose.position
        return p.header.frame_id,(v.x,v.y,v.z),(q.x,q.y,q.z,q.w)
    return service.request(*extract(s),*extract(g))


def main():
    import rospy,rosgraph,tf2_ros
    from geometry_msgs.msg import TransformStamped,PoseStamped
    from nav_msgs.msg import Odometry,Path as RosPath
    from nav_msgs.srv import GetPlan,GetPlanResponse
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from common_route import load_bundle
    rospy.init_node('astra_baseline_rear_adapter_b03')
    uri=urlparse(os.environ.get('ROS_MASTER_URI',''))
    if uri.scheme!='http' or uri.hostname not in ('127.0.0.1','localhost','::1') or uri.username or uri.password:raise RuntimeError('loopback master required')
    if rospy.get_param('/use_sim_time',False)is not True or rospy.get_param('/m2_gazebo/simulation_only',False)is not True:raise RuntimeError('simulation markers required')
    if rospy.get_name()!='/astra_baseline_rear_adapter_b03':raise RuntimeError('fixed adapter node identity required')
    run_id=rospy.get_param('~run_id');seed=rospy.get_param('~seed');method=rospy.get_param('~method')
    if not isinstance(run_id,str) or not run_id or type(seed)is not int or seed<=0 or method not in METHODS:raise ValueError('explicit B03 identity required')
    directory=Path(rospy.get_param('~bundle'));expected=rospy.get_param('~bundle_sha256')
    if hashlib.sha256((directory/'common_route.json').read_bytes()).hexdigest()!=expected:raise ValueError('common bundle hash differs')
    _,route,data=load_bundle(directory);service=RearFixedRoute(route,data)
    stream=RearOdometryStream();lock=threading.RLock();last_audit=[None];graph_checked=[False]
    rear_pub=rospy.Publisher('/astra/rear_odom',Odometry,queue_size=1000)
    status_pub=rospy.Publisher('/astra/baseline_rear/status',String,queue_size=1000,latch=True)
    identity_pub=rospy.Publisher('/astra/baseline_rear/identity',String,queue_size=1,latch=True)
    receipt_pub=rospy.Publisher('/astra/baseline_rear/input_receipt',String,queue_size=1000)
    path_pub=rospy.Publisher('/astra/baseline_rear/served_path',RosPath,queue_size=100)
    ident={'run_id':run_id,'seed':seed,'method':method}
    def emit(pub,row):pub.publish(String(data=json.dumps(row,separators=(',',':'),allow_nan=False)))
    def now():return rospy.Time.now().to_nsec()
    def stamp(ns):return rospy.Time(ns//1_000_000_000,ns%1_000_000_000)
    def status(source_now):
        ready=False
        if stream.fault_reason is None:
            try:ready=stream.health(source_now) and graph_checked[0]
            except Exception as exc:stream.fault(str(exc))
        emit(status_pub,dict(ident,schema=SCHEMA,ready=ready,state='fault' if stream.fault_reason else ('ready' if ready else 'bootstrap'),
            reason=stream.fault_reason,status_source_ns=source_now if type(source_now)is int and source_now>=0 else None,
            latest_odom_source_ns=None if stream.latest is None else stream.latest.source_ns,pending_count=len(stream.pending),accepted=stream.accepted))
    def record(rows):
        for sample,audit in rows:
            last_audit[0]=audit
            emit(receipt_pub,dict(ident,schema=SCHEMA,source_ns=sample.source_ns,received_ns=sample.received_ns,available_ns=sample.available_ns,
                output_frame=FRAME,raw_quaternion=audit['raw_quaternion'],raw_quaternion_norm_squared=audit['raw_quaternion_norm_squared'],
                roll_pitch_yaw=audit['roll_pitch_yaw'],rear_body_twist=list(sample.body_twist),actual_rear_vy_retained=True))
    def on_odom(message):
        with lock:
            try:record(consume_message(stream,message,now,rear_pub.publish,Odometry,stamp))
            except Exception as exc:stream.fault(type(exc).__name__+':'+str(exc))
            status(now())
    def on_tf(message):
        with lock:
            for item in message.transforms:
                if item.child_frame_id!=FRAME:continue
                caller=getattr(message,'_connection_header',{}).get('callerid')
                v=item.transform.translation;q=item.transform.rotation
                if caller!=rospy.get_name() or item.header.frame_id!='base_link' or not all(math.isfinite(x) for x in (v.x,v.y,v.z,q.x,q.y,q.z,q.w)) or any(abs(a-b)>1e-12 for a,b in zip((v.x,v.y,v.z,q.x,q.y,q.z,q.w),(-.325,0.,0.,0.,0.,0.,1.))):
                    stream.fault('duplicate_or_wrong_rear_TF_owner');status(now())
    def get_plan(request):
        with lock:
            try:
                if not stream.health(now()) or not graph_checked[0]:raise ValueError('rear adapter not healthy/ready')
                poses=route_response_poses(service,request)
                path=RosPath();path.header.frame_id='map';path.header.stamp=rospy.Time.now()
                for x,y,yaw in poses:
                    p=PoseStamped();p.header.frame_id='map';p.header.stamp=path.header.stamp
                    p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);path.poses.append(p)
                path_pub.publish(path);return GetPlanResponse(plan=path)
            except Exception as exc:raise rospy.ServiceException(str(exc))
    subscribers=[rospy.Subscriber('/odom',Odometry,on_odom,queue_size=1000),rospy.Subscriber('/tf',TFMessage,on_tf,queue_size=1000),rospy.Subscriber('/tf_static',TFMessage,on_tf,queue_size=1000)]
    server=rospy.Service('/astra/baseline_rear/get_plan',GetPlan,get_plan)
    broadcaster=tf2_ros.StaticTransformBroadcaster();transform=TransformStamped();transform.header.stamp=rospy.Time.now();transform.header.frame_id='base_link';transform.child_frame_id=FRAME
    transform.transform.translation.x=-.325;transform.transform.rotation.w=1.;broadcaster.sendTransform(transform)
    emit(identity_pub,dict(ident,schema=SCHEMA,bundle_sha256=expected,input='/odom',output='/astra/rear_odom',tf_parent='base_link',tf_child=FRAME,
        fixed_translation_xyz=[-.325,0.,0.],base_oriented_path_sha256=service.converted['base_oriented_path_sha256'],rear_oriented_path_sha256=service.converted['rear_oriented_path_sha256'],
        pose_covariance='parent fixed axes infinitesimal-rotation first-order',twist_covariance='full body-axis exact linear point-velocity',
        physical_safety_qualified=False,tilted_costmap_equivalence_qualified=False,goal_contract=goal_tolerance_contract(),oracle_used=False))
    last_status_wall=0.;last_graph_wall=0.
    while not rospy.is_shutdown():
        with lock:
            current=now()
            if stream.fault_reason is None:
                try:record(publish_pending(stream,now,rear_pub.publish,Odometry,stamp))
                except Exception as exc:stream.fault(type(exc).__name__+':'+str(exc))
            wall=time.monotonic()
            if wall-last_graph_wall>=1.:
                try:
                    pubs=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
                    if pubs.get('/astra/rear_odom')!=[rospy.get_name()]:raise ValueError('rear odom publisher is not unique')
                    if '/odom' in pubs and pubs['/odom']!=['/gazebo']:raise ValueError('actual odom authority changed')
                    graph_checked[0]=pubs.get('/odom')==['/gazebo']
                except Exception as exc:stream.fault('publisher_roster:'+str(exc))
                last_graph_wall=wall
            if wall-last_status_wall>=.05:status(now());last_status_wall=wall
        time.sleep(.002)

if __name__=='__main__':main()
