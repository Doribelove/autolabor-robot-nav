"""Offline roslaunch parameter expansion only. No ROS master or allocation."""
import hashlib,json,os,subprocess,sys
from pathlib import Path
import yaml
OUT=Path(__file__).resolve().parent;S=OUT.parents[1];W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import run_navigation_n01 as old
import navigation_evidence_p04 as evidence
from frozen_r2_sensor_lifecycle import METHOD,FROZEN_CHECKPOINT_SHA256
launch=W/'src/simulation/m2_gazebo/launch/astra_common_navigation_n02.launch'
root=OUT/'launch_preflight';root.mkdir(exist_ok=False)
prepared=json.loads((S/'contracts/navigation_n01_prepared_r02/prepared.json').read_text());results=[]
for unit in prepared['units']:
    value=json.loads(evidence.read_bound(unit['files'][0]));method=METHOD if unit['method']=='frozen_r2_v060_shared_activation' else unit['method']
    directory=root/method;directory.mkdir()
    # The historical N01 seed is an offline parameter fixture only, never a
    # fresh identity proposal/reservation. Collector disabled and never run.
    args=dict(world=value['world']['path'],run_id='offline_n02_parameters_only',seed=str(unit['seed']),method=method,
      bundle=str(old.SCENE/'common_bundle'),bundle_sha256=value['shared_bundle']['sha256'],scene_manifest=value['scene_manifest']['path'],
      scene_id=value['scene_id'],legacy_report_path=str(directory/'never_written_runtime_report.json'),steering_kd='6.0',
      legacy_checkpoint_path=str(old.CHECKPOINT),legacy_checkpoint_sha256=FROZEN_CHECKPOINT_SHA256,
      contract_path=unit['files'][0]['path'],contract_sha256=unit['files'][0]['sha256'],output=str(directory/'never_started_attempt'),
      bag_path=str(directory/'never_recorded.bag'),bag_topics=' '.join(value['record_topics']),
      collector_script='astra_navigation_collector_n02_PENDING.py',enable_collector='false')
    command=['roslaunch','--dump-params',str(launch)]+[key+':='+str(val) for key,val in args.items()]
    proc=subprocess.run(command,capture_output=True,timeout=30)
    (directory/'parameters.yaml').write_bytes(proc.stdout);(directory/'stderr.log').write_bytes(proc.stderr)
    assert proc.returncode==0,proc.stderr
    actual=yaml.safe_load(proc.stdout);evidence.validate_robot_description(actual['/robot_description'],value)
    assert hashlib.sha256(actual['/robot_description'].encode()).hexdigest()==value['robot_description']['sha256']
    assert actual['/astra_navigation_coordinator/method']==method
    if method==METHOD:
        prefix='/astra_frozen_r2_method/'
        assert actual[prefix+'execution_method']==METHOD
        assert actual[prefix+'actor_checkpoint_sha256']==FROZEN_CHECKPOINT_SHA256
        assert actual[prefix+'method']=='pct_scope_rl' and actual[prefix+'candidate_variant']=='r2_candidate_v060'
        assert actual[prefix+'bundle_sha256']==value['shared_bundle']['sha256']
        required={'execution_method','run_id','seed','method','candidate_variant','scene_id','scene_manifest','report_path','actor_checkpoint_path','actor_checkpoint_sha256','bundle','bundle_sha256'}
        assert required<={key[len(prefix):] for key in actual if key.startswith(prefix)}
    else:
        previous=yaml.safe_load((S/'audits/navigation_n01_root_launch_preflight_r02'/unit['run_id']/'parameters.yaml').read_bytes())
        # Exact physical/planner/provider configuration; only run_id/output/collector are new offline labels.
        selected=('/move_base/','/astra_route_runtime/','/astra_common_route_provider/')
        for key,val in previous.items():
            if key.startswith(selected) and not key.endswith('/run_id'):assert actual[key]==val,key
    results.append(dict(method=method,exit_code=proc.returncode,parameters=evidence.binding(directory/'parameters.yaml'),
        physical_URDF_sha256=value['robot_description']['sha256'],command=command,
        seed_semantics='reused saved N01 parameter fixture only; no proposed or allocated N02 seed',collector_enabled=False))
report=dict(status='four_actual_parameter_expansions_pass_no_execution',source=evidence.binding(launch),rows=results,
  physical_and_nonR2_parameters_unchanged=True,ROS_started=False,seed_allocated=False,
  future_collector_contract='Requires new root-owned N02 helper/collector admitting sensor lifecycle enum. Frozen P04 contract was used only for physical URDF comparison, collector disabled.')
(root/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'status':report['status'],'report':evidence.binding(root/'report.json')}))
