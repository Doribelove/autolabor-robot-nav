"""New pipeline boundaries. Fake subprocess/session, real files; no ROS startup."""
import ast
import csv
import json
import os
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET
import pytest

P = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(P))
import run_navigation_n05 as manager
import navigation_coordinator_n02 as old
import navigation_coordinator_n05 as new

METHOD = 'astra_analytic_preview_eligible_n05'

def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))
    return manager.evidence.binding(path)

def test_exact_proposed_schedule():
    schedule = manager.design()['schedule']
    assert len(schedule) == 24
    assert len({r['seed'] for r in schedule}) == 12
    for i in range(12):
        pair = schedule[2*i:2*i+2]
        assert {r['seed'] for r in pair} == {263002001+i}
        assert [r['method'] for r in pair] == (['astra_analytic_route_v1', METHOD] if i%2==0 else [METHOD, 'astra_analytic_route_v1'])
        assert {r['attempt'] for r in pair} == {1}
        assert len({r['parent_geometry_id'] for r in pair}) == 1

@pytest.mark.parametrize('terminal', ['goal', 'timeout'])
def test_gate_command_and_terminal_numerics_equal(terminal):
    def run(module, method):
        c = module.NavigationCoordinator(module.CoordinatorConfig('offline', 42, method))
        c.observe_odom(1_000_000_000, (-5.4,0,0), (0,0,0))
        c.method_status(dict(run_id='offline',seed=42,method=method,state='ready',ready=True))
        c.arm(1_000_000_000)
        c.observe_odom(7_000_000_000, (-4,0,0), (.8,0,0))
        rows = [c.output(7_000_000_000)]
        c.method_status(dict(run_id='offline',seed=42,method=method,state='active',ready=True,gate_source_ns=7_000_000_000))
        c.method_command(.6,.08,7_020_000_000,100)
        rows.append(c.output(7_020_000_000))
        ns = 8_000_000_000 if terminal=='goal' else 29_000_000_000
        c.observe_odom(ns, (4.8,.1,.2) if terminal=='goal' else (0,0,0), (.4,0,0))
        rows.append(c.output(ns))
        return json.dumps(rows,sort_keys=True).replace(method,'METHOD')
    assert run(new,METHOD) == run(old,'astra_analytic_route_v1')

def test_sensor_callbacks_and_command_guards_unchanged():
    w=P.parents[1]/'src/simulation/m2_gazebo/scripts'
    def functions(path):
        main=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='main')
        return {n.name:ast.dump(n,include_attributes=False) for n in main.body if isinstance(n,ast.FunctionDef)}
    assert functions(w/'astra_route_coordinated_runtime.py') == functions(w/'astra_route_preview_runtime_n05.py')

def test_exact_two_method_launch_and_shared_physics():
    w=P.parents[1]/'src/simulation/m2_gazebo/launch'
    before=ET.parse(w/'astra_common_navigation_n02.launch').getroot()
    after=ET.parse(w/'astra_common_navigation_n05.launch').getroot()
    groups=after.findall('group');assert len(groups)==2
    assert [g.find('node').get('type') for g in groups] == ['astra_route_coordinated_runtime.py','astra_route_preview_runtime_n05.py']
    for g in groups:
        node=g.find('node')
        assert node.get('launch-prefix')=='$(env THESIS_WS)/.venv/bin/python'
        assert node.find("remap[@from='/cmd_vel_navigation']").get('to')=='/astra/method_cmd'
    for selector in ['include', "param[@name='robot_description']", "node[@name='navigation_command_relay']", "node[@name='astra_common_route_provider']"]:
        a,b=before.find(selector),after.find(selector);a.tail=b.tail=None
        assert ET.tostring(a)==ET.tostring(b)

@pytest.fixture
def fixture(tmp_path,monkeypatch):
    stage=tmp_path/'stage';(stage/'queue').mkdir(parents=True);(stage/'runs').mkdir()
    workspace=tmp_path/'workspace';workspace.mkdir()
    monkeypatch.setattr(manager,'STAGE',stage);monkeypatch.setattr(manager,'WORKSPACE',workspace)
    monkeypatch.setenv('THESIS_WS',str(workspace))
    ids=[('offline_control',42,'astra_analytic_route_v1'),('offline_treatment',42,METHOD)]
    monkeypatch.setattr(manager,'identities',lambda:ids)
    pred=stage/'queue/navigation_n04_ledger.json'
    write(pred,dict(stage='astra_navigation_n04',status='terminal',units=[dict(run_id='previous')]))
    settings=dict(live=False,mode='success',checks=0)
    monkeypatch.setattr(manager.common,'owned_processes',lambda root: [123] if settings['live'] else [])
    prepared={'units':[]}
    for name,seed,method in ids:
        d=tmp_path/'prepared'/name;d.mkdir(parents=True)
        world=write(d/'world.sdf',{});scene=write(d/'scene.yaml',{})
        contract=write(d/'contract.json',dict(stage=manager.NAME,run_id=name,seed=seed,method=method,wall_cap_s=300.,world=world,scene_manifest=scene,shared_bundle={}))
        prepared['units'].append(dict(run_id=name,seed=seed,method=method,attempt=1,
            files=[contract,write(d/'robot_description.xml',{}),write(d/'fixture.launch',{}),world],
            conversion_manifest=write(d/'conversion_manifest.json',{}),converted_sdf=write(d/'converted_robot.sdf',{})))
    release=dict(resources=[],**{k:write(tmp_path/(k+'.json'),{}) for k in ('prepared','source_review','passive_geometry_qualification')})
    calls=dict(popen=[],cleanup=[],assess=[])
    class Session:
        def __init__(self,*args):pass
        def verify_unchanged(self):
            settings['checks']+=1
            if settings['mode']=='cache_failure' and settings['checks']==3:raise ValueError('mutated bound resource')
            return release,prepared,{}
    monkeypatch.setitem(sys.modules,'release_validation_session_n05',types.SimpleNamespace(UnchangedReleaseSession=Session))
    monkeypatch.setitem(sys.modules,'run_passive_sweep_q05',types.SimpleNamespace(archive=lambda *a:None))
    with (stage/'seed_registry.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=['run_id','seed','status','source_binding']);writer.writeheader()
        writer.writerows(dict(run_id=n,seed=s,status='reserved_exact_n05_release',source_binding='a'*64) for n,s,_ in ids)
    class Process:
        pid=os.getpid();returncode=0
        def poll(self):return self.returncode
    def popen(args,**kw):
        calls['popen'].append((args,kw))
        if settings['mode']=='launch_failure':raise OSError('injected launch failure')
        return Process()
    monkeypatch.setattr(manager.subprocess,'Popen',popen)
    monkeypatch.setattr(manager.cleanup,'stop_group',lambda p,r:calls['cleanup'].append(r))
    monkeypatch.setattr(manager.evidence,'closure',lambda *a:None)
    def assess(root,value):
        calls['assess'].append(root)
        return dict(capture_integrity_pass=settings['mode']!='integrity_failure',task=dict(task_phase='timeout'))
    monkeypatch.setattr(manager.evidence,'assess',assess)
    class Socket:
        def __enter__(self):return self
        def __exit__(self,*a):pass
        def setsockopt(self,*a):pass
        def bind(self,address):assert address[1] in (14830,14831)
    monkeypatch.setattr(manager.socket,'socket',Socket)
    return types.SimpleNamespace(settings=settings,calls=calls,stage=stage,pred=pred,
        execute=lambda:manager.execute(tmp_path/'release.json','a'*64),
        ledger=lambda:json.loads((stage/'queue/navigation_n05_ledger.json').read_text()))

def test_failures_retained_and_same_inputs_hashed_each_unit(fixture):
    f=fixture;assert f.execute()==0
    assert f.settings['checks']==5
    assert [r['task_outcome'] for r in f.ledger()['units']]==['timeout','timeout']
    assert len(f.calls['popen'])==len(f.calls['cleanup'])==2

@pytest.mark.parametrize('mode',['cache_failure','launch_failure','integrity_failure'])
def test_infrastructure_failure_retires_remaining_without_retry(fixture,mode):
    f=fixture;f.settings['mode']=mode
    assert f.execute()==3
    assert [r['status'] for r in f.ledger()['units']]==['consumed','retired_unrun']
    assert len(f.calls['popen'])==1
    with pytest.raises(ValueError,match='retry prohibited'):f.execute()

@pytest.mark.parametrize('kind',['active_predecessor','live_predecessor'])
def test_predecessor_liveness_outside_cache_before_allocation(fixture,kind):
    f=fixture
    if kind=='active_predecessor':write(f.pred,dict(stage='astra_navigation_n04',status='active',units=[]))
    else:f.settings['live']=True
    with pytest.raises(ValueError):f.execute()
    assert f.settings['checks']==0 and not f.calls['popen']
    assert not (f.stage/'queue/navigation_n05_ledger.json').exists()
