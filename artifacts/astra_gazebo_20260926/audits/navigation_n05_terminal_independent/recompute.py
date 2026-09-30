"""Independent all-24-bag terminal/trace replay; no production assessor import."""
from pathlib import Path
from collections import Counter,defaultdict
import cmath
import csv
import hashlib
import json
import math
import os
import sys
import rosbag
import yaml

A=Path(__file__).resolve().parent;S=A.parents[1]
M=('astra_analytic_route_v1','astra_analytic_preview_eligible_n05')
STAGE='astra_navigation_n05_preview_selection'
cache={};resources={}
def binding(path):
    path=Path(path).resolve();key=str(path);before=path.stat()
    signature=(before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)
    if key in cache:
        assert cache[key][0]==signature,('changed_after_hash',key)
        return cache[key][1]
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        while True:
            chunk=stream.read(4*1024*1024)
            if not chunk:break
            digest.update(chunk)
    after=path.stat();assert signature==(after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns)
    row={'path':key,'sha256':digest.hexdigest(),'bytes':before.st_size}
    cache[key]=(signature,row);resources[key]=row
    return row
def bound(row):
    actual=binding(row['path']);assert actual['sha256']==row['sha256'],row
    return Path(actual['path'])
def read(path):binding(path);return json.loads(Path(path).read_bytes())
def write(name,data):
    path=A/name
    with path.open('x') as f:json.dump(data,f,sort_keys=True,indent=2,allow_nan=False);f.write('\n')
def require(value,label):
    if not value:raise AssertionError(label)
def yaw(q):
    a,b,c,d=q.x,q.y,q.z,q.w
    require(all(math.isfinite(v) for v in (a,b,c,d)),'finite quaternion')
    require(abs(a*a+b*b+c*c+d*d-1)<1e-5,'unit quaternion')
    return math.atan2(2*(a*b+c*d),d*d+a*a-b*b-c*c)
def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi
def goal_test(x,y,heading,goal):return math.hypot(x-goal[0],y-goal[1])<=.25 and abs(wrap(heading-goal[2]))<=.25
def same_identity(value,u):return all(type(value[k]) is type(u[k]) and value[k]==u[k] for k in ('run_id','seed','method'))

release_path=S/'contracts/navigation_n05_release.json';ledger_path=S/'queue/navigation_n05_ledger.json'
release=read(release_path);release_sha=binding(release_path)['sha256'];ledger=read(ledger_path)
require(release_sha=='e9879ee6ca8ca050b9ee036db87c4728f5f1aa38895b69c2ca2d429bbcf6c628','frozen release')
require(ledger['stage']==release['stage']==STAGE and ledger['status']=='terminal','terminal identity')
require(ledger['release_sha256']==release_sha and ledger['maximum_units']==release['maximum_episodes']==24,'budget')
require(all(release[k] is False for k in ('retry_allowed','resume_allowed','training_authorized','real_vehicle_authorized','formal_result')),'release scope')
require(len({r['path'] for r in release['resources']})==len(release['resources']),'release unique resources')
for r in release['resources']:bound(r)
prepared=read(bound(release['prepared']))
for k in ('source_review','passive_geometry_qualification'):bound(release[k])
require(len(prepared['units'])==len(ledger['units'])==24,'all24 identities')
require([(u['run_id'],u['seed'],u['method']) for u in prepared['units']]==[(u['run_id'],u['seed'],u['method']) for u in ledger['units']],'exact prepared/ledger order')
for index,u in enumerate(ledger['units']):
    b=index//2;order=M if b%2==0 else M[::-1]
    require(u['seed']==263002001+b and u['method']==order[index%2] and u['run_id']=='astra_navigation_n05_b%02d_%s'%(b,u['method']),'counterbalanced schedule')
require(all(u['status']=='consumed' for u in ledger['units']) and ledger['consumed_units']==24,'24 consumed')
parent=read(S/'audits/navigation_n05_terminal_replay/report.json')
reported={r['run_id']:r for r in parent['rows']}
require(len(reported)==24,'parent no dropped/duplicate rows')
rows=[];details=[];artifact_count=0;bag_message_count=Counter()
topics=['/odom','/astra/p03/end_links','/astra/route_runtime/trace','/astra/route_runtime/candidates','/astra/navigation/gate','/astra/navigation/status']
for u in ledger['units']:
    root=S/'runs'/u['run_id'];receipt=read(bound(u['receipt']))
    require(Path(u['receipt']['path']).resolve()==(root/'receipt.json').resolve(),'receipt expected path')
    require(same_identity(receipt,u) and receipt['stage']==STAGE and receipt['release_sha256']==release_sha,'receipt identity')
    require(receipt['attempt']==1 and receipt['retry_allowed'] is False and receipt['error'] is None,'no retry/errors')
    require(receipt['integrity_pass'] is u['integrity_pass'] is True and receipt['task_outcome']==u['task_outcome'],'receipt outcome')
    require(receipt['exit_code']==0,'child roslaunch exit receipt')
    require(len({r['path'] for r in receipt['artifacts']})==len(receipt['artifacts']),'artifact unique paths')
    for r in receipt['artifacts']:bound(r);artifact_count+=1
    artifacts={r['path']:r['sha256'] for r in receipt['artifacts']}
    for name in ('assessment.json','collector_raw.json','contract.json','fixture.bag'):
        p=root/name;require(artifacts.get(str(p))==binding(p)['sha256'],'core artifact present')
    assessment=read(bound(receipt['report']));raw=read(root/'collector_raw.json');contract=read(root/'contract.json')
    require(all(same_identity(d,u) for d in (assessment,raw,contract)),'assessment/raw/contract identity')
    bundle=read(bound(contract['shared_bundle']));scene=yaml.safe_load(bound(contract['scene_manifest']).read_bytes())
    goal=bundle['goal_xy_yaw'];require(goal==scene['route']['goal']==raw['gate']['goal_pose'],'same actual world goal')
    start,end=raw['start_ns'],raw['end_ns'];phase=raw['terminal']['phase']
    require(type(start) is int and type(end) is int and 0<start<end<=start+22_000_000_000,'source window')
    require(raw['gate']['source_ns']==start and raw['gate']['deadline_source_ns']==start+22_000_000_000,'gate window')
    require(raw['terminal']['terminal_source_ns']==end and phase==receipt['task_outcome'],'terminal window')
    require(same_identity(raw['gate'],u) and same_identity(raw['terminal'],u),'event identity')
    require(phase in ('timeout','complete'),'retained task type')
    counts=Counter();reasons=Counter();executed=Counter();prev={};dense_count=0;first_dense=None;first_odom=None
    endpoint=None;dense_endpoint=None;gate_records=[];statuses=[];trace_stamps=[];candidate_stamps=[];invalid=[]
    tx,ty,rotation=bundle['map_from_odom_xy_yaw'];rot=cmath.exp(1j*rotation)
    with rosbag.Bag(str(root/'fixture.bag'),'r') as bag:
        info=bag.get_type_and_topic_info().topics
        require(all(t in info for t in topics),'selected topics present')
        per_topic=Counter()
        for topic,msg,_ in bag.read_messages(topics=topics):
            per_topic[topic]+=1;bag_message_count[topic]+=1
            if topic in ('/odom','/astra/p03/end_links'):
                stamp=msg.header.stamp.to_nsec()
                require(stamp>prev.get(topic,-1),'ordered exact pose source');prev[topic]=stamp
                if not start<=stamp<=end:continue
                if topic=='/odom':
                    require(msg.header.frame_id=='odom' and msg.child_frame_id=='base_link','odom frames')
                    p=msg.pose.pose;z=complex(tx,ty)+complex(p.position.x,p.position.y)*rot
                    actual=(z.real,z.imag,wrap(yaw(p.orientation)+rotation))
                    if goal_test(*actual,goal) and first_odom is None:first_odom=stamp
                    if stamp==end:endpoint=actual
                else:
                    require(msg.header.frame_id=='world' and list(msg.name).count('autolabor_m2::base_link')==1,'actual dense link identity')
                    require(len(msg.pose)==len(msg.name),'dense pose/name array')
                    require(stamp==start+dense_count*2_000_000,'complete2ms pose lattice')
                    p=msg.pose[list(msg.name).index('autolabor_m2::base_link')]
                    actual=(p.position.x,p.position.y,yaw(p.orientation));dense_count+=1
                    if goal_test(*actual,goal) and first_dense is None:first_dense=stamp
                    if stamp==end:dense_endpoint=actual
                continue
            d=json.loads(msg.data)
            if topic=='/astra/navigation/gate':gate_records.append(d);continue
            if topic=='/astra/navigation/status':
                if d['phase'] in ('complete','timeout','fault','stopped'):statuses.append(d)
                continue
            stamp=d['stamp_ns'];require(type(stamp)is int,'integer trace/candidate time')
            if not start<=stamp<end:continue
            if topic.endswith('/trace'):
                require(d['run_id']==u['run_id'],'trace run identity')
                action=d['executed'];require(type(action)is int and 0<=action<8,'trace action')
                executed[action]+=1;reasons[str(d.get('stop_reason'))]+=1;trace_stamps.append(stamp)
                require(type(d.get('transaction_deadline_dropped'))is bool,'deadline flag type')
                counts['transaction_deadline_dropped_flag_ticks']+=d['transaction_deadline_dropped']
                if action==0:require(d['v']==0. and d['w']==0.,'executedstop zero command')
            else:
                counts['candidate_decisions']+=1;candidate_stamps.append(stamp)
                mask=d['mask'];require(len(mask)==8 and all(type(x)is bool for x in mask),'true mask')
                require(mask==[x>=0 for x in d['backup_clearances']],'mask matches saved unrounded backup clearance')
                audit=d.get('preview_selection_audit')
                if u['method']==M[0]:require(audit is None,'control no treatment audit')
                else:
                    require(audit['schema']=='astra_analytic_preview_eligibility_n05_v1' and audit['method']==M[1],'treatment audit')
                    full=d['full_preview_clearances_m'];require(len(full)==8 and all(type(x)in (float,int) and math.isfinite(x) for x in full),'finite fullpreview')
                    eligibility=[bool(mask[i] and (i==0 or x>=0.)) for i,x in enumerate(full)]
                    require(audit['backup_mask']==mask and audit['full_preview_clearances_m']==full and audit['selection_eligibility']==eligibility,'exact eligibility')
                    require(audit['true_backup_mask_changed'] is False and audit['physical_safety_certified'] is False,'claims unchanged')
                    chosen=audit['chosen'];original=audit['original_analytic']
                    require(type(chosen)is int and type(original)is int and 0<=chosen<8 and 0<=original<8,'selected ids')
                    require((eligibility[chosen] if any(eligibility) else chosen==0),'eligible selection or explicit empty fallback')
                    counts['treatment_selection_changed_decisions']+=chosen!=original
                    counts['treatment_moving_backup_count_sum']+=sum(mask[1:])
                    counts['treatment_moving_eligible_count_sum']+=sum(eligibility[1:])
                    counts['treatment_selected_stop_decisions']+=chosen==0
        require(all(per_topic[t]==info[t].message_count for t in topics),'all selected messages consumed')
    require(endpoint is not None and dense_endpoint is not None and dense_count==(end-start)//2_000_000+1,'exact terminal/complete goal source')
    require(gate_records and all(d==raw['gate'] for d in gate_records),'actual bag gate')
    require(statuses and all(same_identity(d,u) and d['phase']==phase and d['terminal_source_ns']==end for d in statuses),'actual bag terminal statuses')
    if phase=='timeout':require(end==start+22_000_000_000 and first_odom is None,'timeout exact deadline, no earlier odom success')
    else:require(first_odom==end and first_dense is not None and goal_test(*dense_endpoint,goal),'first source goal completion')
    require(first_dense==assessment['task']['raw_geometric_arrival_source_ns'],'2ms goal independently matches assessment')
    require(assessment['capture_integrity_pass'] is all(assessment['integrity_checks'].values()),'integrity aggregation')
    safety=assessment['capture_integrity_pass'] and all(assessment['safety_checks'].values())
    require(safety is assessment['safety_within_declared_engineering_scope_pass'],'safety aggregation, inherited numeric result')
    valid=safety and phase=='complete' and first_dense is not None
    restricted=(first_dense-start)/1e9 if valid else 22.
    require(valid is assessment['valid_completed'] and restricted==assessment['restricted_goal_time_s'],'independent restricted time')
    row=dict(run_id=u['run_id'],seed=u['seed'],method=u['method'],scene_id=contract['scene_id'],task_phase=phase,
      valid_completed=valid,restricted_goal_time_s=restricted,safety_within_declared_engineering_scope_pass=safety,
      trace_ticks=sum(executed.values()),executed_stop_ticks=executed[0],backup_revalidation_failed_ticks=reasons['backup_revalidation_failed'],
      backup_stop_requested_ticks=reasons['backup_stop_requested'],transaction_deadline_drop_ticks=reasons['transaction_deadline_drop'],
      transaction_deadline_dropped_flag_ticks=counts['transaction_deadline_dropped_flag_ticks'],candidate_decisions=counts['candidate_decisions'],
      treatment_selection_changed_decisions=counts['treatment_selection_changed_decisions'],
      treatment_moving_backup_count_sum=counts['treatment_moving_backup_count_sum'],treatment_moving_eligible_count_sum=counts['treatment_moving_eligible_count_sum'],
      terminal_map_goal_xy_distance_m=round(math.hypot(endpoint[0]-goal[0],endpoint[1]-goal[1]),9),terminal_source_ns=end,
      receipt_sha256=binding(root/'receipt.json')['sha256'],assessment_sha256=binding(root/'assessment.json')['sha256'],
      bag_sha256=binding(root/'fixture.bag')['sha256'],bundle_sha256=contract['shared_bundle']['sha256'])
    require(row==reported[u['run_id']],('all parent fields differ',u['run_id'],[(k,row[k],reported[u['run_id']][k]) for k in row if row[k]!=reported[u['run_id']][k]]))
    detail=dict(run_id=u['run_id'],selected_bag_message_counts=dict(per_topic),goal=goal,start_ns=start,end_ns=end,
      first_dense_goal_source_ns=first_dense,first_odom_goal_source_ns=first_odom,dense_window_samples=dense_count,
      odom_terminal_map_pose=endpoint,dense_terminal_world_pose=dense_endpoint,
      terminal_yaw_error_rad=abs(wrap(endpoint[2]-goal[2])),dense_odom_xy_difference_m=math.hypot(endpoint[0]-dense_endpoint[0],endpoint[1]-dense_endpoint[1]),
      trace_reasons=dict(reasons),executed_actions=dict(executed),treatment_selected_stop_decisions=counts['treatment_selected_stop_decisions'],
      trace_maximum_stamp_gap_ns=max((b-a for a,b in zip(trace_stamps,trace_stamps[1:])),default=None),
      candidate_maximum_stamp_gap_ns=max((b-a for a,b in zip(candidate_stamps,candidate_stamps[1:])),default=None),
      inherited_safety_checks=assessment['safety_checks'],safety_recomputed_from_raw_contacts_and_geometry=False)
    rows.append(row);details.append(detail)
    with (A/'progress.jsonl').open('a') as f:f.write(json.dumps(dict(row=row,detail=detail),sort_keys=True,allow_nan=False)+'\n');f.flush();os.fsync(f.fileno())
    print(u['run_id'],phase,restricted,'mechanisms/endpoints exact',flush=True)

byseed=defaultdict(dict)
for r in rows:require(r['method'] not in byseed[r['seed']],'no pair duplicate');byseed[r['seed']][r['method']]=r
pairs=[]
for seed,data in sorted(byseed.items()):
    require(set(data)==set(M),'complete pair');c,t=[data[m] for m in M];require(c['scene_id']==t['scene_id'] and c['bundle_sha256']==t['bundle_sha256'],'paired scene/map')
    pairs.append(dict(seed=seed,scene_id=c['scene_id'],control_run_id=c['run_id'],treatment_run_id=t['run_id'],control_task_phase=c['task_phase'],treatment_task_phase=t['task_phase'],
      restricted_time_treatment_minus_control_s=t['restricted_goal_time_s']-c['restricted_goal_time_s'],
      final_goal_distance_treatment_minus_control_m=t['terminal_map_goal_xy_distance_m']-c['terminal_map_goal_xy_distance_m'],
      stop_fraction_treatment_minus_control=t['executed_stop_ticks']/t['trace_ticks']-c['executed_stop_ticks']/c['trace_ticks']))
require(pairs==parent['pairs'],'all12 paired values')
outcomes={m:dict(valid_rows=sum(r['method']==m for r in rows),completed=sum(r['method']==m and r['valid_completed'] for r in rows),timeout=sum(r['method']==m and r['task_phase']=='timeout' for r in rows)) for m in M}
require(outcomes==parent['method_outcomes'],'method totals')
mean=sum(p['restricted_time_treatment_minus_control_s'] for p in pairs)/len(pairs)
require(mean==parent['paired_mean_restricted_time_difference_s'],'failure inclusive paired mean')
require(parent['consumed']==parent['integrity_valid']==24 and parent['integrity_failed']==parent['retired_unrun']==0 and parent['complete_pairs']==12,'report ledger counts')
require(not parent['integrity_failed_rows'] and not parent['retired_unrun_rows'],'no hidden actual failures')
require(parent['all_safety_scoped_pass'] is all(r['safety_within_declared_engineering_scope_pass'] for r in rows),'retained safety negatives')
for k in ('formal_result','performance_improvement_demonstrated','general_safety_or_deployment_claim'):require(parent[k] is False,k)
for name in ('episodes.csv','complete_pairs.csv'):
    path=S/'audits/navigation_n05_terminal_replay'/name;binding(path)
    with path.open(newline='') as f:actual=list(csv.DictReader(f))
    expected=rows if name=='episodes.csv' else pairs
    require(len(actual)==len(expected),'CSV cardinality')
    require(all(a=={k:str(v) for k,v in e.items()} for a,e in zip(actual,expected)),name+' exact independent rows')
manager=read(S/'queue/navigation_n05_manager_return.json')
require(manager['stage']==STAGE and manager['release_sha256']==release_sha and manager['returned_code']==0,'manager durable return')
outerlog=S/'logs/navigation_n05_execute_outer.log';binding(outerlog)
require(json.loads(outerlog.read_text().strip())=={'stage':STAGE,'manager_returned_code':0},'stdout manager return')
binding(S/'queue/execute_navigation_n05_once.sh')
exitfile=S/'queue/navigation_n05_outer_exit_code.txt'
require(not exitfile.exists(),'expected missing outer exit receipt')
binding(S/'workspace/scripts/astra_gazebo/run_navigation_n05.py')
binding(S/'workspace/scripts/astra_gazebo/navigation_evidence_n05.py')
binding(S/'audits/navigation_n05_terminal_replay_r04.py')
binding(S/'audits/navigation_n05_terminal_replay_preflight/r04/review.json')
write('recomputed.json',dict(schema='astra_navigation_n05_independent_recomputation_v1',pass_=True,
    release=binding(release_path),ledger=binding(ledger_path),release_resource_count=len(release['resources']),receipt_artifact_bindings_verified=artifact_count,
    consumed=24,retired=0,integrity_receipts_pass=24,complete_pairs=12,rows=rows,pairs=pairs,method_outcomes=outcomes,
    mean_restricted_treatment_minus_control_s=mean,all_parent_fields_and_CSV_exact=True,details=details,
    all_selected_topics_entire_bag_read=True,bag_topic_message_counts=dict(bag_message_count),topics=topics,
    numeric_collision_clearance_replay_performed=False,contact_replay_performed=False,native_PNG_visual_review_performed=False,
    manager_return=manager,outer_exitfile_exists=False,outer_exitfile_path=str(exitfile),
    parent_reported_tool_observation={'session_id':1158,'operation':'write_stdin','exit_code':143,'source':'parent /root message; no independently available local original tool response','independently_verified_from_file':False},
    host_exit_cause='unknown; no signal sender, termination chain or normal-exit inference',
    raw_trace_stamp_semantics='runtime prepublication end /clock time, not driver source stamp',
    candidate_stamp_semantics='decision-visible /clock time, not driver execution',
    stop_fraction_semantics='fraction of recorded trace ticks, not time spent stopped',
    independent_environment={'executable':sys.executable,'version':sys.version,'ROS_master_started':False,'Gazebo_started':False,'training_started':False}))
for _,(signature,row) in cache.items():binding(row['path'])
write('input_resources.json',dict(resources=sorted(resources.values(),key=lambda r:r['path']),hashed_unique_count=len(resources),stat_unchanged_after_read=True))
print('ALL 24 / 12 pairs complete',outcomes,'mean_delta',mean,'resources',len(resources),flush=True)
