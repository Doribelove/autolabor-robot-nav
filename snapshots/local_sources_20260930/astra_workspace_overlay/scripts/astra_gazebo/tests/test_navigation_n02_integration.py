"""New N02 method wiring only; real saved parameter expansions, no ROS master."""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import pytest
import yaml

SCRIPTS=Path(__file__).resolve().parents[1];W=SCRIPTS.parents[1];S=W.parent
sys.path.insert(0,str(SCRIPTS))
import navigation_coordinator as old
import navigation_coordinator_n02 as new
from frozen_r2_sensor_lifecycle import METHOD,FROZEN_CHECKPOINT_SHA256
PACKAGE=W/'src/simulation/m2_gazebo'
AUDIT=S/'audits/navigation_n02_sensor_lifecycle_integration'
OLD_METHOD='frozen_r2_v060_shared_activation'

spec=importlib.util.spec_from_file_location('n02_real_lifecycle_support',SCRIPTS/'tests/test_frozen_r2_sensor_lifecycle.py')
support=importlib.util.module_from_spec(spec);spec.loader.exec_module(support)
actual_episode=support.actual_episode


def test_only_method_identity_and_module_import_change_coordinator_code():
    core=(SCRIPTS/'navigation_coordinator.py').read_text().replace(OLD_METHOD,METHOD)
    assert (SCRIPTS/'navigation_coordinator_n02.py').read_text()==core
    node=(PACKAGE/'scripts/astra_navigation_coordinator.py').read_text().replace(OLD_METHOD,METHOD).replace('from navigation_coordinator import ','from navigation_coordinator_n02 import ')
    assert (PACKAGE/'scripts/astra_navigation_coordinator_n02.py').read_text()==node
    assert old.METHODS[2]==OLD_METHOD and new.METHODS[2]==METHOD
    assert new.METHODS[:2]+new.METHODS[3:]==old.METHODS[:2]+old.METHODS[3:]


@pytest.mark.parametrize('method',['astra_analytic_route_v1','actual_teb_common_route','dwa_common_route',METHOD])
@pytest.mark.parametrize('terminal',['goal','timeout'])
def test_common_numerical_gate_command_and_terminal_trace_equal(method,terminal):
    def run(module,identity):
        c=module.NavigationCoordinator(module.CoordinatorConfig('offline_comparison',42,identity));rows=[]
        c.observe_odom(1_000_000_000,(-5.4,0,0),(0,0,0))
        c.method_status(dict(run_id='offline_comparison',seed=42,method=identity,state='ready',ready=True))
        c.arm(1_000_000_000)
        c.observe_odom(5_000_000_000,(-5.4,0,0),(0,0,0));rows.append(c.output(5_000_000_000))
        c.observe_odom(7_000_000_000,(-4.,0,0),(.8,0,0));rows.append(c.output(7_000_000_000))
        c.method_status(dict(run_id='offline_comparison',seed=42,method=identity,state='active',ready=True,gate_source_ns=7_000_000_000))
        c.method_command(.6,.08,7_020_000_000,100);rows.append(c.output(7_020_000_000))
        ns=8_000_000_000 if terminal=='goal' else 29_000_000_000
        c.observe_odom(ns,(4.8,.1,.2) if terminal=='goal' else (0.,0.,0.),(.4,0,0));rows.append(c.output(ns))
        c.fault('late fault');rows.append(c.output(ns))
        return json.dumps(rows,sort_keys=True).replace(identity,'COMPARED_METHOD')
    assert run(new,method)==run(old,OLD_METHOD if method==METHOD else method)


def test_new_identity_and_legacy_frame_are_explicit():
    with pytest.raises(ValueError):new.CoordinatorConfig('offline',42,OLD_METHOD)
    with pytest.raises(ValueError,match='legacy x route'):new.CoordinatorConfig('offline',42,METHOD,goal_pose=(5.,1.,0.))
    with pytest.raises(ValueError):old.CoordinatorConfig('offline',42,METHOD)


def test_launch_keeps_other_nodes_physics_and_explicit_future_collector():
    before=ET.parse(PACKAGE/'launch/astra_common_navigation_n01_p04.launch').getroot()
    after=ET.parse(PACKAGE/'launch/astra_common_navigation_n02.launch').getroot()
    assert after.find("arg[@name='collector_script']").get('default') is None
    assert after.find("node[@name='astra_navigation_collector']").get('required')=='true'
    assert ET.tostring(before.find('include'))==ET.tostring(after.find('include'))
    assert ET.tostring(before.find("param[@name='robot_description']"))==ET.tostring(after.find("param[@name='robot_description']"))
    for group in before.findall('group'):
        if OLD_METHOD in group.get('if',''):continue
        matching=next(g for g in after.findall('group') if g.get('if')==group.get('if'))
        group.tail=matching.tail=None  # surrounding comment whitespace is not node configuration
        assert ET.tostring(group)==ET.tostring(matching)
    node=after.find("group/node[@name='astra_frozen_r2_method']")
    assert node.get('type')=='astra_frozen_r2_sensor_lifecycle.py'
    params={p.get('name'):p.get('value') for p in node.findall('param')}
    assert params['execution_method']==METHOD
    assert params['actor_checkpoint_sha256']=='$(arg legacy_checkpoint_sha256)'
    for topic in ('identity','input_receipt','trace','command_event','report'):
        assert '/astra/frozen_r2_sensor_lifecycle/'+topic in after.find("arg[@name='sensor_lifecycle_evidence_topics']").get('value')


def test_real_sensor_constructor_consumes_actual_n02_expanded_parameters(actual_episode):
    f=actual_episode
    params=yaml.safe_load((AUDIT/'launch_preflight'/METHOD/'parameters.yaml').read_text())
    prefix='/astra_frozen_r2_method/'
    actual={'~'+key[len(prefix):]:value for key,value in params.items() if key.startswith(prefix)}
    f.params.clear();f.params.update(actual)
    episode=type(f.episode)()
    assert episode.actor_checkpoint_sha256==FROZEN_CHECKPOINT_SHA256
    assert episode.candidate_variant=='r2_candidate_v060'
    assert f.subscribers[-2:][0][0]=='/odom' and f.subscribers[-1][0]=='/scan'
    assert not hasattr(episode,'model_states') and 'actors' not in episode.scene
    assert episode.command_pub.resolved_name=='/astra/method_cmd'
    assert type(episode).execute_sensor is f.r2.r1.BenchmarkEpisode.execute_sensor
    assert episode.runtime.scan_history.shape==(6,72)
    assert not f.ingress.history_ready()
    report=json.loads((AUDIT/'launch_preflight/report.json').read_text())
    assert len(report['rows'])==4 and not report['ROS_started'] and not report['seed_allocated']


def test_frozen_n01_and_reviewed_r1_node_bytes_unchanged():
    expected={
      SCRIPTS/'navigation_coordinator.py':'1574d381a0cf4e6fc98f0702e91c0b36173035302448a50e35faf29379211b42',
      PACKAGE/'scripts/astra_navigation_coordinator.py':'1f2bb1da971fc2536dd0243562bb523d14875bff52b40b6193541544b1dc9959',
      PACKAGE/'launch/astra_common_navigation_n01_p04.launch':'ee9d511c615cdd380f1bdf89f9e04a49bd32a467b875e995eb351e93879eac37',
      PACKAGE/'scripts/astra_frozen_r2_sensor_lifecycle.py':'765238653925e232243c1caa01a58d9034e1ee7171f0f39b1d1032d31ddd3712'}
    for path,digest in expected.items():assert hashlib.sha256(path.read_bytes()).hexdigest()==digest
