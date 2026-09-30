"""B04 static rear-reference evidence; no ROS import or execution authority.

Shared task outcome always belongs to original actual base odometry. Private
move_base results are retained diagnostics, never replacements for that outcome.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from baseline_rear_runtime_b03 import (METHODS,FRAME,SCHEMA,AGE_NS,RearFixedRoute,
    quaternion_pose,goal_tolerance_contract)
from baseline_rear_axle_adapter_b02 import StampedOdometry,adapt_odometry
from common_route import load_bundle

PERIOD_NS=20_000_000
TRANSFORM_ATOL=1e-9
ADAPTER='/astra_baseline_rear_adapter_b03'
COORDINATOR='/astra_navigation_coordinator'
PREFIX='/astra/baseline_rear/'
TOPICS={'/odom':'base','/astra/rear_odom':'rear',PREFIX+'identity':'identity',
        PREFIX+'status':'status',PREFIX+'input_receipt':'receipts',
        PREFIX+'served_path':'served_paths','/move_base/FixedRouteGlobalPlanner/provided_path':'provided_paths',
        '/move_base/goal':'goals',PREFIX+'private_action_result':'private_results',
        '/tf':'tf','/tf_static':'tf'}
STRING_TOPICS={PREFIX+x for x in ('identity','status','input_receipt','private_action_result')}
REQUIRED_RECORD_TOPICS=tuple(TOPICS)
REQUIRED_NODES=(ADAPTER,COORDINATOR,'/move_base')
REFERENCE_CONTRACT={
    'schema':'astra_baseline_rear_static_integration_b04_v1',
    'methods':list(METHODS),'planner_frame':FRAME,'original_odom':'/odom','rear_odom':'/astra/rear_odom',
    'rear_translation_xyz_m':[-.325,0.,0.],'odom_period_ns':PERIOD_NS,
    'maximum_transport_age_ns':AGE_NS,'transform_numeric_comparison_atol':TRANSFORM_ATOL,
    'shared_terminal_reference':'original_actual_base_link','private_action_result_terminal_authority':False,
    'goal_contract':goal_tolerance_contract(),'actor_world_supported':False,
    'execution_authorized':False,'training_authorized':False,'live_qualified':False,
}


def new_evidence():
    return {key:[] for key in set(TOPICS.values())}


def pose_row(p):
    return {'position':[p.position.x,p.position.y,p.position.z],
            'quaternion':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]}


def odom_row(message):
    t=message.twist.twist
    return dict(source_ns=message.header.stamp.to_nsec(),parent_frame=message.header.frame_id,
        child_frame=message.child_frame_id,**pose_row(message.pose.pose),
        linear=[t.linear.x,t.linear.y,t.linear.z],angular=[t.angular.x,t.angular.y,t.angular.z],
        pose_covariance=list(message.pose.covariance),twist_covariance=list(message.twist.covariance))


def capture(rows,topic,message,receipt_clock_ns):
    """Called only under the collector EvidenceStore lock; no transport invented."""
    if topic not in TOPICS:raise ValueError('unknown B04 reference topic')
    if type(receipt_clock_ns)is not int or receipt_clock_ns<0:raise ValueError('integer collector receipt clock required')
    owner=getattr(message,'_connection_header',{}).get('callerid')
    if topic in STRING_TOPICS:item=json.loads(message.data)
    elif topic in ('/odom','/astra/rear_odom'):item=odom_row(message)
    elif topic in ('/tf','/tf_static'):
        for t in message.transforms:
            if t.child_frame_id!=FRAME:continue
            v=t.transform.translation;q=t.transform.rotation
            rows['tf'].append(dict(topic=topic,caller=owner,receipt_clock_ns=receipt_clock_ns,
                source_ns=t.header.stamp.to_nsec(),parent=t.header.frame_id,child=t.child_frame_id,
                transform=[v.x,v.y,v.z,q.x,q.y,q.z,q.w]))
        return
    elif topic=='/move_base/goal':
        p=message.goal.target_pose
        item=dict(source_ns=p.header.stamp.to_nsec(),frame=p.header.frame_id,goal_id=message.goal_id.id,
                  **pose_row(p.pose))
    else:
        item=dict(source_ns=message.header.stamp.to_nsec(),frame=message.header.frame_id,
                  poses=[dict(frame=p.header.frame_id,source_ns=p.header.stamp.to_nsec(),**pose_row(p.pose)) for p in message.poses])
    if type(item)is not dict:raise ValueError('reference payload must be object')
    rows[TOPICS[topic]].append(dict(payload=item,caller=owner,collector_receipt_clock_ns=receipt_clock_ns))


def static_world_only(world_text):
    root=ET.fromstring(world_text)
    for model in root.findall('world/model'):
        if model.findtext('static','false').strip().lower()!='true':raise ValueError('B04 permits static world models only; actor integration is unqualified')
        if any('actor' in p.get('filename','').lower() for p in model.findall('plugin')):raise ValueError('B04 actor plugin is unsupported')
    return True


def graph_audit(system,require_ready=True):
    pubs=dict(system[0]);subs=dict(system[1]);services=dict(system[2])
    expected={'/odom':['/gazebo'],'/astra/rear_odom':[ADAPTER],
      PREFIX+'identity':[ADAPTER],PREFIX+'status':[ADAPTER],PREFIX+'input_receipt':[ADAPTER],
      PREFIX+'served_path':[ADAPTER],PREFIX+'private_action_result':[COORDINATOR],
      '/move_base/goal':[COORDINATOR],'/move_base/FixedRouteGlobalPlanner/provided_path':['/move_base'],
      '/astra/method_cmd':['/move_base'],'/cmd_vel_navigation':[COORDINATOR],'/cmd_vel':['/navigation_command_relay']}
    # Global planner and costmap transports may wait on positive /clock/TF.
    # Do not impose their readiness before the collector unpauses the world.
    checked={k:v for k,v in expected.items() if require_ready or k not in ('/move_base/goal','/move_base/FixedRouteGlobalPlanner/provided_path','/astra/method_cmd')}
    failures=[topic for topic,owners in checked.items() if sorted(pubs.get(topic,[]))!=owners]
    if require_ready:
        for topic in ('/astra/rear_odom','/scan','/tf','/tf_static'):
            if '/move_base' not in subs.get(topic,[]):failures.append('move_base_missing_'+topic)
    for topic in ('/odom','/tf','/tf_static'):
        if ADAPTER not in subs.get(topic,[]):failures.append('adapter_missing_'+topic)
    forbidden={'/gazebo/model_states','/gazebo/link_states','/astra/p03/end_models','/astra/p03/end_links','/astra/physics_contacts'}
    for topic,nodes in system[1]:
        if topic in forbidden and any(n in nodes for n in (ADAPTER,'/move_base')):failures.append('deployment_oracle:'+topic)
    if sorted(services.get(PREFIX+'get_plan',[]))!=[ADAPTER]:failures.append('rear_get_plan_owner')
    return {'pass_':not failures,'issues':failures,'expected_publishers':expected,
            'readiness_required':require_ready,'scope':'Observed ROS registrations and trusted source binding, not instantaneous transport authority proof.'}


def planner_parameters(params,method):
    if method not in METHODS:raise ValueError('B04 method identity required')
    root=Path(__file__).resolve().parents[2]/'src/simulation/m2_gazebo/config'
    expected=yaml.safe_load((root/'astra_baseline_rear_b03.yaml').read_text())
    planner='TebLocalPlannerROS' if method==METHODS[0] else 'DWAPlannerROS'
    if params.get('base_local_planner')!=('teb_local_planner/'+planner if planner=='TebLocalPlannerROS' else 'dwa_local_planner/'+planner) or params.get('base_global_planner')!='astra_common_route/FixedRouteGlobalPlanner':raise ValueError('actual selected planner classes differ')
    for key in ('global_costmap','local_costmap','FixedRouteGlobalPlanner',planner):
        if key not in params:raise ValueError('missing effective planner namespace '+key)
        for field,value in expected[key].items():
            if params[key].get(field)!=value:raise ValueError('effective rear parameter differs: '+key+'/'+field)
    return {'pass_':True,'planner':planner,'reference':FRAME,'strict_effective_fields_checked':sum(len(expected[k]) for k in ('global_costmap','local_costmap','FixedRouteGlobalPlanner',planner))}


def _index(rows,key,lo,hi):
    source=[r['payload'][key] for r in rows]
    if any(type(ns)is not int or ns<0 for ns in source) or any(b<=a for a,b in zip(source,source[1:])):raise ValueError('noninteger/duplicate/regressed reference source stream')
    return {r['payload'][key]:r for r in rows if lo<=r['payload'][key]<=hi}


def drained(rows,start,end,watermark):
    """Join through final actual50Hz slot, not merely formal end; no fabricated tail."""
    if any(type(v)is not int for v in (start,end,watermark)) or not 0<start<end<=watermark:return False
    last=end+(watermark-end)//PERIOD_NS*PERIOD_NS
    expected=list(range(start,last+1,PERIOD_NS))
    try:
        streams=[_index(rows[k],'source_ns',start,last) for k in ('base','rear','receipts')]
        return all(list(x)==expected for x in streams)
    except (KeyError,ValueError,TypeError):return False


def ready(rows,now):
    if not rows['identity'] or not rows['status'] or not rows['base'] or not rows['rear'] or not rows['receipts']:return False
    status=rows['status'][-1]['payload']
    return status.get('state')=='ready' and status.get('ready')is True and type(status.get('status_source_ns'))is int and 0<=now-status['status_source_ns']<=AGE_NS


def audit(rows,value,start,end,watermark,full_parameters):
    issues=[];checks={};diagnostics={'private_results':[]}
    def require(name,fn):
        try:checks[name]=bool(fn())
        except Exception as exc:checks[name]=False;issues.append(name+': '+type(exc).__name__+': '+str(exc))
        if not checks[name] and not any(x.startswith(name+':') for x in issues):issues.append(name+': false')
    expected_id={k:value[k] for k in ('run_id','seed','method')}
    def identity(row):return all(type(row.get(k))is type(v) and row[k]==v for k,v in expected_id.items())
    def near(a,b):return np.shape(a)==np.shape(b) and np.all(np.isfinite(a)) and np.all(np.isfinite(b)) and np.allclose(a,b,rtol=0.,atol=TRANSFORM_ATOL)
    directory=Path(value['shared_bundle']['path']).parent
    try:
        _,route,bundle=load_bundle(directory);fixed=RearFixedRoute(route,bundle)
    except Exception as exc:
        return {'pass_':False,'checks':{'bound_common_bundle':False},'issues':[repr(exc)],'diagnostics':diagnostics}
    require('exact_joined_original_rear_receipts_through_watermark',lambda:drained(rows,start,end,watermark))
    require('rear_identity_and_path_hashes',lambda:bool(rows['identity']) and all(r['caller']==ADAPTER and identity(r['payload']) and r['payload'].get('schema')==SCHEMA and r['payload'].get('bundle_sha256')==value['shared_bundle']['sha256'] and all(r['payload'].get(k)==v for k,v in {'input':'/odom','output':'/astra/rear_odom','tf_parent':'base_link','tf_child':FRAME,'fixed_translation_xyz':[-.325,0.,0.],'base_oriented_path_sha256':fixed.converted['base_oriented_path_sha256'],'rear_oriented_path_sha256':fixed.converted['rear_oriented_path_sha256'],'oracle_used':False,'physical_safety_qualified':False,'tilted_costmap_equivalence_qualified':False,'goal_contract':goal_tolerance_contract()}.items()) for r in rows['identity']))
    require('rear_status_ready_and_never_faulted',lambda:bool(rows['status']) and any(r['payload'].get('ready')is True for r in rows['status']) and all(r['caller']==ADAPTER and identity(r['payload']) and r['payload'].get('schema')==SCHEMA and r['payload'].get('state') in ('bootstrap','ready') and not r['payload'].get('reason') for r in rows['status']))
    require('unique_correct_static_rear_TF_authority',lambda:bool(rows['tf']) and all(r['topic']=='/tf_static' and r['caller']==ADAPTER and r['parent']=='base_link' and r['child']==FRAME and near(r['transform'],[-.325,0.,0.,0.,0.,0.,1.]) for r in rows['tf']))
    def transforms():
        last=end+(watermark-end)//PERIOD_NS*PERIOD_NS
        base=_index(rows['base'],'source_ns',start,last);rear=_index(rows['rear'],'source_ns',start,last);receipts=_index(rows['receipts'],'source_ns',start,last)
        if set(base)!=set(rear) or set(base)!=set(receipts) or not base:return False
        for ns,b in base.items():
            r=rear[ns];receipt=receipts[ns];p=b['payload'];v=r['payload'];a=receipt['payload']
            if b['caller']!='/gazebo' or r['caller']!=ADAPTER or receipt['caller']!=ADAPTER or p['parent_frame']!='odom' or p['child_frame']!='base_link' or v['parent_frame']!='odom' or v['child_frame']!=FRAME:return False
            if not identity(a) or a.get('schema')!=SCHEMA or a.get('output_frame')!=FRAME or a.get('actual_rear_vy_retained')is not True:return False
            if any(type(a.get(k))is not int for k in ('received_ns','available_ns')) or not 0<=a['received_ns']<=a['available_ns'] or not 0<=a['available_ns']-ns<=AGE_NS or abs(a['received_ns']-ns)>AGE_NS:return False
            pose,rpy,norm=quaternion_pose(p['position'],p['quaternion']);sample=StampedOdometry(ns,a['received_ns'],a['available_ns'],'odom','base_link',pose,tuple(p['linear']+p['angular']),tuple(p['pose_covariance']),tuple(p['twist_covariance']));expected=adapt_odometry(sample,a['available_ns'])
            for actual,wanted in ((v['position'],expected.pose.xyz),(v['quaternion'],expected.pose.xyzw),(v['linear']+v['angular'],expected.body_twist),(v['pose_covariance'],expected.pose_covariance),(v['twist_covariance'],expected.twist_covariance),(a['rear_body_twist'],expected.body_twist),(a['raw_quaternion'],p['quaternion']),(a['roll_pitch_yaw'],rpy)):
                if not near(actual,wanted):return False
            if abs(a['raw_quaternion_norm_squared']-norm)>TRANSFORM_ATOL:return False
        return True
    require('full_3d_odometry_covariance_and_source_availability',transforms)
    def path_records(key,owner):
        if not rows[key]:return False
        expected=fixed.converted['rear_poses']
        for r in rows[key]:
            p=r['payload']
            if r['caller']!=owner or p['frame']!='map' or len(p['poses'])!=len(expected):return False
            for pose,(x,y,yaw) in zip(p['poses'],expected):
                if pose['frame']!='map' or not near(pose['position'],[x,y,0.]) or not near(pose['quaternion'],[0.,0.,math.sin(yaw/2),math.cos(yaw/2)]):return False
        return True
    require('served_exact_full_rear_polyline',lambda:path_records('served_paths',ADAPTER))
    require('actual_global_plugin_provided_full_rear_polyline',lambda:path_records('provided_paths','/move_base'))
    def goal():
        if len(rows['goals'])!=1:return False
        r=rows['goals'][0];p=r['payload'];x,y,yaw=fixed.converted['rear_poses'][-1]
        return r['caller']==COORDINATOR and p['frame']=='map' and type(p['source_ns'])is int and p['source_ns']>=start and bool(p['goal_id']) and near(p['position'],[x,y,0.]) and near(p['quaternion'],[0.,0.,math.sin(yaw/2),math.cos(yaw/2)])
    require('single_gated_rear_action_goal',goal)
    for phase in ('bootstrap','terminal'):
        require('effective_rear_planner_parameters_'+phase,lambda phase=phase:planner_parameters(full_parameters[phase],value['method'])['pass_'])
    def private():
        for r in rows['private_results']:
            p=r['payload']
            if r['caller']!=COORDINATOR or not identity(p) or p.get('is_task_terminal_authority')is not False or type(p.get('action_state'))is not int or not 0<=p['action_state']<=9:return False
            diagnostics['private_results'].append(dict(p))
        return True
    require('private_action_results_are_diagnostic_only',private)
    return {'schema':'astra_baseline_reference_evidence_audit_b04_v1','pass_':all(checks.values()),'checks':checks,'issues':issues,'diagnostics':diagnostics,'scope':'Static pipeline source/reference integrity only; task and physical safety remain independently assessed on original base/world records; no continuous physical equivalence or performance claim.'}
