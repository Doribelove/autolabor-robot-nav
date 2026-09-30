"""Independent analytic/native ODE/SciPy references; no ROS node or Gazebo."""
import copy
import json
import math
from pathlib import Path
import random
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import dense_sweep as dense
from geometry import Pose3

URDF = '''<robot name="test"><link name="base_link"/>
<link name="body"><inertial><origin xyz="0 0 .4"/><mass value="2"/>
<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
<collision><origin xyz="0 0 .4"/><geometry><box size="1 .5 .4"/></geometry></collision></link>
<joint name="fixed" type="fixed"><parent link="base_link"/><child link="body"/></joint></robot>'''


def fixture():
    models, links = [], []
    for i in range(11):
        pose = (i*.002, 0., 0., 0., 0., 0., 1.)
        for rows, name in ((models, 'autolabor_m2'), (links, 'autolabor_m2::base_link')):
            rows.append({'stamp_ns':1000000000+i*2000000,'frame_id':'world','names':[name],
                         'poses':[pose],'twists':[(1.,0.,0.,0.,0.,0.)]})
    meta={'schema':'astra_physics_evidence_dense_v2','all_body_states_every_physics_step':True,
          'physics_type':'ode','max_step_s':.002,'phase':'WorldUpdateEnd','pose_frame':'world'}
    return models, links, meta


def test_exact_translation_and_fixed_massless_root_com():
    bodies=dense.robot_body_envelopes(URDF)
    assert bodies['base_link']['com_local_m']==(0.,0.,.4)
    assert bodies['base_link']['mass_kg']==2.
    models,links,meta=fixture()
    report,rows=dense.assess_dense_trace(models,links,URDF,meta)
    assert report['integrity_pass']
    assert report['qualification']=='pending'
    assert len(report['pending_obligations'])==8
    assert len(rows)==10
    assert all(r['robot_point_phase_budget_m']==pytest.approx(.002) for r in rows)
    endpoints=[{'stamp_ns':1000000000,'clearance_m':.5},{'stamp_ns':1020000000,'clearance_m':.52}]
    lower=dense.aggregate_clearance_bounds(endpoints,rows)[0]
    assert lower['solver_intervals']==10
    assert lower['conditional_clearance_lower_bound_m']==pytest.approx(.49)


def test_midstep_rotation_with_com_offset_is_not_endpoint_chord_or_origin_speed():
    angle=2.5; radius=2.;com=(.3,.4,.2)
    q=Pose3.from_xyz_rpy(rpy=(0,0,angle))
    # Hold COM fixed while the link origin moves around it.
    offset=q.rotate(com);second=Pose3(tuple(a-b for a,b in zip(com,offset)),q.xyzw)
    bound=dense.extension_point_budget(Pose3(),second,com,radius)
    assert bound['translation_m']==pytest.approx(0.)
    assert bound['point_phase_budget_m']==pytest.approx(2*radius*math.tan(angle/2))
    assert bound['point_phase_budget_m']>2*radius*math.sin(angle/2)


def test_tiny_rotation_quaternion_sign_invariance_and_near_pi_rejection():
    end=Pose3.from_xyz_rpy(rpy=(0,0,1e-9))
    normal=dense.extension_point_budget(Pose3(),end,(0,0,0),1)
    flipped=dense.extension_point_budget(Pose3(),Pose3(xyzw=tuple(-x for x in end.xyzw)),(0,0,0),1)
    assert normal['point_phase_budget_m']==pytest.approx(1e-9,rel=1e-9,abs=1e-20)
    assert flipped==normal
    with pytest.raises(ValueError,match='near-pi'):
        dense.extension_point_budget(Pose3(),Pose3.from_xyz_rpy(rpy=(0,0,math.pi)),(0,0,0),1)


def test_random_points_against_scipy_rotation_continuous_extension():
    # SciPy composes rotations independently of Pose3/quaternion implementation.
    import numpy as np
    from scipy.spatial.transform import Rotation
    rng=np.random.RandomState(260926)
    for _ in range(100):
        q0=Rotation.random(random_state=rng);axis=rng.normal(size=3);axis/=np.linalg.norm(axis)
        theta=rng.uniform(1e-5,2.8);q1=Rotation.from_rotvec(axis*theta)*q0
        com=rng.normal(size=3);c0=rng.normal(size=3);dc=rng.normal(size=3)*.2
        p0=c0-q0.apply(com);p1=c0+dc-q1.apply(com)
        bound=dense.extension_point_budget(Pose3(p0,q0.as_quat()),Pose3(p1,q1.as_quat()),com,1.7)
        previous=None;path_length=0.
        point=rng.normal(size=3);point=point/np.linalg.norm(point)*1.7
        for u in np.linspace(0,1,201):
            rotation=Rotation.from_rotvec(axis*(2*math.atan(u*math.tan(theta/2))))*q0
            current=c0+u*dc+rotation.apply(point)
            if previous is not None:path_length+=np.linalg.norm(current-previous)
            previous=current
        assert path_length<=bound['point_phase_budget_m']+1e-12


def test_actual_linked_gazebo_ode_native_steps_and_end_damping_counterexample():
    stage=ROOT.parents[2]
    records=[json.loads(x) for x in (stage/'audits/dense_sweep_native_ode_steps.jsonl').read_text().splitlines()]
    assert records[0]['path'].endswith('libgazebo_ode.so.11')
    assert len(records)==303
    for item in records[1:301]:
        assert item['finite_rotation_mode']==0
        q0=item['q0_wxyz'];q1=item['q1_wxyz']
        first=Pose3(item['p0'],q0[1:]+q0[:1]);second=Pose3(item['p1'],q1[1:]+q1[:1])
        b=dense.extension_point_budget(first,second,(0,0,0),.5)
        assert b['com_displacement_world_m']==pytest.approx(tuple(.002*x for x in item['v1']),abs=1e-14)
        assert b['normal_euler_integrating_omega_times_dt']==pytest.approx(tuple(.002*x for x in item['w1']),abs=1e-14)
    damped=records[301]
    observed_displacement=dense.norm(tuple(b-a for a,b in zip(damped['p0'],damped['p1'])))
    naive_bound=.002*dense.norm(damped['v1'])
    assert observed_displacement==pytest.approx(naive_bound/0.8)
    assert observed_displacement>naive_bound


@pytest.mark.parametrize('mutation',['missing','mismatch','reset','bad_twist','missing_body','metadata'])
def test_missing_source_steps_and_invalid_evidence_never_get_repaired(mutation):
    models,links,meta=fixture()
    if mutation=='missing':del models[5];del links[5]
    elif mutation=='mismatch':links[5]['stamp_ns']+=1
    elif mutation=='reset':links[5]['stamp_ns']=1
    elif mutation=='bad_twist':links[5]['twists'][0]=(float('nan'),0,0,0,0,0)
    elif mutation=='missing_body':links[5]['names']=['wrong_body']
    else:meta['all_body_states_every_physics_step']=False
    report,rows=dense.assess_dense_trace(models,links,URDF,meta)
    assert not report['integrity_pass']
    assert report['qualification']=='pending'
    if mutation in ('missing','mismatch','reset','bad_twist','missing_body'):
        result=dense.aggregate_clearance_bounds([{'stamp_ns':1000000000,'clearance_m':.5},
            {'stamp_ns':1020000000,'clearance_m':.5}],rows)
        assert not result[0]['covered_exactly']
        assert result[0]['conditional_clearance_lower_bound_m'] is None


def test_actor_zero_twist_does_not_erase_jump_variation_budget():
    models,links,meta=fixture()
    for i,row in enumerate(models):
        row['names'].append('actor');row['poses'].append((2.,i*.002,0.,0.,0.,0.,1.));row['twists'].append((0.,)*6)
    report,rows=dense.assess_dense_trace(models,links,URDF,meta,{'actor':1.35})
    assert report['integrity_pass']
    assert all(r['actor_total_variation_budget_m']==pytest.approx(.0027) for r in rows)
    models[5]['poses'][1]=(2.,1.,0.,0.,0.,0.,1.)
    report,_=dense.assess_dense_trace(models,links,URDF,meta,{'actor':1.35})
    assert not report['integrity_pass']
    assert any('actor jump' in issue for issue in report['issues'])


def test_discontinuous_actor_jump_still_satisfies_bounded_variation_clearance_bound():
    # Actor jumps toward robot at Begin, then robot moves away. Low-rate
    # endpoint distances hide the intermediate nearest approach.
    intervals=[{'start_ns':1,'end_ns':3,'relative_clearance_variation_budget_m':.6}]
    endpoints=[{'stamp_ns':1,'clearance_m':1.},{'stamp_ns':3,'clearance_m':1.}]
    lower=dense.aggregate_clearance_bounds(endpoints,intervals)[0]['conditional_clearance_lower_bound_m']
    assert lower==pytest.approx(.7)
    assert 1.-.3>=lower # Intermediate after actor's .3m jump.


def test_library_name_or_true_flags_do_not_qualify_the_component():
    models,links,meta=fixture()
    proof={name:True for name in dense.REQUIRED_PROOFS}
    report,_=dense.assess_dense_trace(models,links,URDF,meta,proof_bindings=proof)
    assert len(report['pending_obligations'])==8
    proof={name:{'path':'claimed_engine.so','sha256':'a'*64,'reviewed':True} for name in dense.REQUIRED_PROOFS}
    report,_=dense.assess_dense_trace(models,links,URDF,meta,proof_bindings=proof)
    assert report['qualification']=='pending'
    assert not report['proof_documents_authenticated_by_this_component']


def test_actual_m12_urdf_lumped_body_com_and_all_shapes():
    xml=(ROOT.parents[2]/'audits/m12_physical_independent_robot.xml').read_text()
    bodies=dense.robot_body_envelopes(xml)
    assert len(bodies)==7
    assert sum(b['mass_kg'] for b in bodies.values())==pytest.approx(79.6)
    assert bodies['base_link']['mass_kg']==pytest.approx(66.6)
    assert bodies['base_link']['com_local_m'][2]==pytest.approx((66*.27+.5*.6+.1*.4)/66.6)
    assert sum(len(b['collisions']) for b in bodies.values())==9


def test_unsupported_fixed_body_mapping_fails_closed():
    with pytest.raises(ValueError,match='lumping'):
        dense.robot_body_envelopes(URDF.replace('</robot>','<gazebo><preserveFixedJoint>true</preserveFixedJoint></gazebo></robot>'))


def test_intersample_out_and_back_is_not_deducible_from_endpoint_pose_or_twist():
    # An unrestricted physical path can leave and return with zero endpoint
    # velocity. Equal samples certify only the explicitly defined extension.
    budget=dense.extension_point_budget(Pose3(),Pose3(),(0,0,0),1.)
    assert budget['point_phase_budget_m']==0.
    excursion=lambda u:math.sin(math.pi*u)**2
    assert excursion(.5)==pytest.approx(1.)
    assert excursion(0.)==pytest.approx(0.) and excursion(1.)==pytest.approx(0.)
    models,links,meta=fixture()
    report,_=dense.assess_dense_trace(models,links,URDF,meta)
    assert report['qualification']=='pending'
    assert 'one_pose_integration_per_dense_step_verified' in report['pending_obligations']


def test_rotating_rectangle_bound_against_independent_shapely_distance_samples():
    from shapely.geometry import Polygon
    from shapely.affinity import rotate,translate
    box=Polygon([(-.5,-.2),(.5,-.2),(.5,.2),(-.5,.2)])
    obstacle=Polygon([(2,-.4),(2.4,-.4),(2.4,.4),(2,.4)])
    final_angle=1.8;dx=.6;radius=math.hypot(.5,.2)
    bound=dense.extension_point_budget(Pose3(),Pose3.from_xyz_rpy((dx,0,0),(0,0,final_angle)),(0,0,0),radius)
    samples=[]
    for i in range(1001):
        u=i/1000.;angle=2*math.atan(u*math.tan(final_angle/2))
        shape=translate(rotate(box,angle,origin=(0,0),use_radians=True),xoff=u*dx)
        samples.append(shape.distance(obstacle)-.33)
    lower=min(samples[0],samples[-1])-bound['point_phase_budget_m']/2
    assert min(samples)>=lower-1e-12
