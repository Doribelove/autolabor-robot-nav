import copy,csv,math,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest,yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import prepare_navigation_n04_scenes as n

ROWS=list(csv.DictReader((n.STAGE/'scenes/development_a02_live_v2/diagnostic_blocks.csv').open()))
@pytest.mark.parametrize('index',range(12))
def test_actual_source_derivation_retains_geometry_and_policy(index):
    row=ROWS[index];parent=yaml.safe_load(Path(row['manifest_path']).read_text());raw=Path(row['world_path']).read_text()
    scene,world=n.derive(parent,raw,263001001+index)
    assert n.collision_signature(world)==n.collision_signature(raw)
    assert scene['parent_geometry_id']==parent['parent_geometry_id']
    assert scene['parent_episode_seed']==parent['seed']
    bound=n.bind_gate_identity(world,'n04_qualified_'+str(index),263001001+index,'actual_teb_common_route')
    assert n.collision_signature(bound)==n.collision_signature(world)
    for old,new in zip(ET.fromstring(raw).findall('world/model/plugin'),ET.fromstring(bound).findall('world/model/plugin')):
        expected={e.tag:e.text for e in old if e.tag not in {'route_origin','route_yaw','gate_route_x'}}
        actual={e.tag:e.text for e in new if e.tag not in {'expected_run_id','expected_seed','expected_method','gate_topic','max_gate_transport_delay_ns'}}
        assert expected==actual
    classified,cs,omitted=n.static_input(world,scene)
    assert len(omitted)==len(scene['actors'])
    assert not cs['actors']
    assert all(a['model_name'] not in {m.get('name') for m in ET.fromstring(classified).findall('world/model')} for a in scene['actors'])

def crossing():
    row=next(r for r in ROWS if r['family']=='single_crossing');return n.derive(yaml.safe_load(Path(row['manifest_path']).read_text()),Path(row['world_path']).read_text(),263001003)
@pytest.mark.parametrize('changes',[{'run_id':'UNBOUND'}, {'seed':True},{'seed':263001004},{'method':'dwa'},{'run_id':''}])
def test_identity_rejection(changes):
    scene,world=crossing();args=dict(run_id='n04_probe',seed=263001003,method='dwa_common_route');args.update(changes)
    with pytest.raises(ValueError):n.bind_gate_identity(world,**args)

def test_second_binding_rejected():
    _,world=crossing();bound=n.bind_gate_identity(world,'first',263001003,'actual_teb_common_route')
    with pytest.raises(ValueError):n.bind_gate_identity(bound,'second',263001003,'dwa_common_route')

def test_arbitrary_heading_not_supported_by_manifest_change():
    row=ROWS[0];p=yaml.safe_load(Path(row['manifest_path']).read_text());p['route']['start'][2]=.1
    with pytest.raises(ValueError):n.derive(p,Path(row['world_path']).read_text(),263001001)

def test_converted_full_model_radius_domain():
    robot=n.ReferenceRobot((n.REFERENCE/'robot_description.xml').read_text(),(n.REFERENCE/'conversion/conversion.sdf').read_text())
    assert len(robot.radius_audit)==9
    assert max(x['arbitrary_wheel_spin_pad_required_m'] for x in robot.radius_audit)<1e-5
    assert len(robot.project(n.Pose3(),.78))==9

def test_smooth_witness_analytic_derivative_upper_bound_independent_finite_difference():
    A=1.6;k=math.pi/9;bound=4*A*k*k;kp_bound=40*A*k**3+3*(4*A*k)*bound**2
    def curvature(x):
        t=k*x;s,c=math.sin(t),math.cos(t);yp=4*A*k*s**3*c;ypp=4*A*k*k*(3*s*s*c*c-s**4)
        return ypp/(1+yp*yp)**1.5
    for i in range(901):
        x=9*i/900;eps=1e-5
        assert abs(curvature(x))<=bound+1e-12
        assert abs((curvature(x+eps)-curvature(x-eps))/(2*eps))<kp_bound
