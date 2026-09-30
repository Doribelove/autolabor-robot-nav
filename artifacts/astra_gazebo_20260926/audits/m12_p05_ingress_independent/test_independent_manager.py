"""Independent manager boundaries. Offline imports and synthetic objects only."""
import copy,importlib.util,json,sys
from pathlib import Path
import pytest,yaml
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
P=W/'scripts/astra_gazebo/run_m12_physics_p05.py';spec=importlib.util.spec_from_file_location('independent_manager_p05',P);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

@pytest.mark.parametrize('name',['CMakeLists.txt','package.xml'])
def test_only_two_explicit_build_metadata_paths_can_replace_parent_hash(name):
 path=str(W/'src/simulation/m2_gazebo'/name);resource={'path':path,'sha256':'old'};design={'updated_build_dependencies':{path:{'old_sha256':'old','new_sha256':'new'}}}
 assert m.active_inherited_digest(design,resource)=='new'
 design['updated_build_dependencies'][path]['old_sha256']='wrong'
 with pytest.raises(ValueError):m.active_inherited_digest(design,resource)

@pytest.mark.parametrize('name',['src/astra_physical_ackermann_p02_plugin.cpp','src/astra_physical_ackermann_p05_plugin.cpp','../../devel/lib/libastra_physical_ackermann_p02_plugin.so','urdf/m2_physical_base.xacro','scripts/astra_physical_actuation_probe_p02.py'])
def test_driver_library_geometry_and_old_assessor_cannot_use_build_whitelist(name):
 path=str((W/'src/simulation/m2_gazebo'/name).resolve());resource={'path':path,'sha256':'old'}
 assert m.active_inherited_digest({},resource)=='old'
 design={'updated_build_dependencies':{path:{'old_sha256':'old','new_sha256':'new'}}}
 with pytest.raises(ValueError):m.active_inherited_digest(design,resource)

@pytest.mark.parametrize('key',['physical_command_ingress_source_schema_and_order','physical_command_ingress_matches_source_commands','command_loss_no_driver_receipt_after_drain','final_paused_source_watermark_drained'])
def test_ingress_or_tail_failure_is_infrastructure_never_promoted(key):
 design,_,_=m.load_design();checks=dict.fromkeys(design['prerequisite']['full_check_roster'],True);checks[key]=False
 report={'pass':False,'assessment_incomplete':False,'checks':checks}
 assert m.classify_report(report,design)==('infrastructure_failure',[key])


def test_runtime_identity_not_allocated_and_strict_exact_six_order():
 design,parent,rows=m.load_design();m.check_seed_freshness(rows)
 assert len(rows)==6 and [(r['seed'],r['steering_kd']) for r in rows]==[(260926017,4.),(260926017,6.),(260926018,6.),(260926018,4.),(260926019,4.),(260926019,6.)]
 assert len(design['prerequisite']['full_check_roster'])==122
 assert set(design['updated_build_dependencies'])=={str(W/'src/simulation/m2_gazebo'/n) for n in ('CMakeLists.txt','package.xml')}
 for row in rows:
  c=yaml.safe_load(m.read_bound(S/row['contract_path'],row['contract_sha256']))
  assert c['gates']==parent['gates'] and c['phase_schedule']==parent['phase_schedule'] and c['world_fixture']==parent['world_fixture']
  assert c['gates']['command_loss_relay_drain_grace_s']==.12 and c['gates']['maximum_publish_gap_s']==.075
  assert c['recorder_protocol']['post_pause_drain_wall_s']==5.
  assert not (S/'runs'/row['run_id']).exists()
 assert not m.RUNTIME_LEDGER.exists() and not (S/'contracts/m12_physics_p05_release.json').exists()
