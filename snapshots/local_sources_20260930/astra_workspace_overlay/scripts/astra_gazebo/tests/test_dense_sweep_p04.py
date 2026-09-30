"""P04 focused offline regression against saved Q04 effective-model evidence.

These tests do not relabel Q04's frozen negative result or launch a simulator.
"""
import copy
import itertools
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import dense_sweep_p03 as old
import dense_sweep_p04 as new

STAGE=Path(__file__).resolve().parents[4]
RUN=STAGE/'runs/passive_sweep_q04_static'
CONVERSION=STAGE/'audits/dense_sweep_p04_conversion_q04_r01'
URDF=RUN/'robot_description_runtime.xml'
SDF=CONVERSION/'conversion.sdf'
MANIFEST=CONVERSION/'manifest.json'


@pytest.fixture(scope='module')
def evidence():
    meta=json.loads((STAGE/'audits/passive_q04_result_independent/mass_diagnosis_inputs/runtime_metadata.jsonl').read_text().splitlines()[0])
    model=new.parse_converted_model(URDF.read_text(),SDF.read_text())
    return meta,model


def test_numeric_profile_and_old_bytes_unchanged():
    assert new.PROFILE==old.PROFILE
    assert new.MODEL_TOLERANCE==1e-7
    assert new.sha(old.__file__)=='7de0531ffae59adc226f8de2531367c0e0575191499c00319530f9f8a430de52'


def test_actual_q04_quantization_matches_all_bodies_and_shapes(evidence):
    meta,model=evidence
    geometry=new.runtime_geometry(meta)
    audit=new.validate_runtime_model(geometry,model)
    assert len(audit)==7
    assert len(model['source_mapping']['collision_map'])==9
    base=next(x for x in audit if x['body']=='base_link')
    ideal=new.core.robot_body_envelopes(URDF.read_text())['base_link']
    actual=geometry['autolabor_m2::base_link']
    assert actual['com'][2]==.272673
    assert abs(actual['com'][2]-ideal['com_local_m'][2])>new.MODEL_TOLERANCE
    assert base['used_outward_union_radius_m']>base['runtime_radius_m']
    assert base['used_outward_union_radius_m']==pytest.approx(.6495887595854349,abs=1e-15)


@pytest.mark.parametrize('field',['mass','com','inertial_pose','tensor','native_tensor','shape_pose','shape_size','shape_identity','body_missing'])
def test_runtime_mismatches_still_fail_at_unchanged_tolerance(evidence,field):
    meta,model=evidence;meta=copy.deepcopy(meta)
    robot=next(x for x in meta['config']['models'] if x['name']==new.ROBOT)
    link=next(x for x in robot['links'] if x['name'].endswith('::base_link'))
    if field=='mass':link['mass_kg']+=2e-7
    elif field=='com':link['com_local_m'][2]+=2e-7
    elif field=='inertial_pose':link['inertia_pose']['xyz'][2]+=2e-7
    elif field=='tensor':link['inertia_ixx_iyy_izz_ixy_ixz_iyz'][0]+=2e-7
    elif field=='native_tensor':link['native_ode_inertia_rowmajor_stride4'][0]+=2e-7
    elif field=='shape_pose':link['collisions'][0]['relative_pose']['xyz'][0]+=2e-7
    elif field=='shape_size':link['collisions'][0]['size_xyz_m'][0]+=2e-7
    elif field=='shape_identity':link['collisions'][0]['name']+='wrong'
    elif field=='body_missing':robot['links'].remove(link)
    with pytest.raises(ValueError):new.runtime_geometry(meta,model)


@pytest.mark.parametrize('field',['fixed_frame','fixed_attachment','moving_parent','joint_axis','mass','com','tensor','shape_pose','shape_size','extra_shape'])
def test_converted_model_cannot_drift_from_full_urdf_mapping(field):
    root=ET.fromstring(SDF.read_text());model=root.find('model');base=model.find("link[@name='base_link']")
    if field=='fixed_frame':model.remove(model.find('frame'))
    elif field=='fixed_attachment':model.find('frame').set('attached_to','wrong')
    elif field=='moving_parent':model.find('joint/parent').text='wrong'
    elif field=='joint_axis':model.find('joint/axis/xyz').text='1 0 0'
    elif field=='mass':base.find('inertial/mass').text='66.600001'
    elif field=='com':base.find('inertial/pose').text='0 0 0.272674 0 0 0'
    elif field=='tensor':base.find('inertial/inertia/ixx').text='3.314461'
    elif field=='shape_pose':base.find('collision/pose').text='0.000001 0 0 0 0 0'
    elif field=='shape_size':base.find('collision/geometry/box/size').text='1.301 0.65 0.4'
    elif field=='extra_shape':base.append(copy.deepcopy(base.find('collision')))
    with pytest.raises(ValueError):new.parse_converted_model(URDF.read_text(),ET.tostring(root,encoding='unicode'))


def test_original_and_converted_shapes_enclosed_about_one_runtime_com(evidence):
    meta,model=evidence;geometry=new.runtime_geometry(meta,model)
    for name,body in model['bodies'].items():
        actual=geometry[new.ROBOT+'::'+name];centre=np.array(actual['com'])
        for shape in body['source_shapes']+body['shapes']:
            # Independent SciPy rotation and explicit surface points, not _radius.
            if shape['kind']=='box':
                local=np.array(list(itertools.product(*[[-x/2,x/2] for x in shape['dimensions']])))
            else:
                radius,length=shape['dimensions']
                local=np.array([(radius*math.cos(a),radius*math.sin(a),z) for a in np.linspace(0,2*math.pi,137) for z in (-length/2,length/2)])
            points=Rotation.from_quat(shape['origin'].xyzw).apply(local)+shape['origin'].xyz
            assert np.max(np.linalg.norm(points-centre,axis=1))<=actual['radius']
    # Distinct-center max is not generally an enclosing construction.
    shapes=[{'origin':new.Pose3((2.,0,0)),'kind':'box','dimensions':(.2,.2,.2)}]
    wrong=new._radius(shapes,(2.,0,0))
    right=new._radius(shapes,(0.,0,0))
    assert wrong<.2 and right>2


def test_content_identical_copies_and_real_converter_maps_authenticated(tmp_path):
    urdf=tmp_path/'copied.xml';sdf=tmp_path/'copied.sdf';manifest=tmp_path/'copied.json'
    urdf.write_bytes(URDF.read_bytes());sdf.write_bytes(SDF.read_bytes());manifest.write_bytes(MANIFEST.read_bytes())
    audit=new.authenticate_conversion(manifest,urdf,sdf,{},diagnostic=True)
    assert audit['pass'],audit['issues']
    assert audit['runtime_converter_correspondence_qualified'] is False
    fresh=new.authenticate_conversion(manifest,urdf,sdf,{},diagnostic=False)
    assert not fresh['pass'] and 'runtime mapped sdformat' in fresh['issues'][0]


@pytest.mark.parametrize('change',['missing_process','repeated_process','same_output','empty_libraries','wrong_tool','wrong_library','changed_sdf'])
def test_conversion_claims_fail_closed(tmp_path,change):
    value=json.loads(MANIFEST.read_text());sdf=SDF
    if change=='missing_process':value['process_evidence']=[]
    elif change=='repeated_process':value['process_evidence'][1]=copy.deepcopy(value['process_evidence'][0])
    elif change=='same_output':value['reproduced_sdf']=copy.deepcopy(value['converted_sdf'])
    elif change=='empty_libraries':value['loaded_libraries']=[]
    elif change=='wrong_tool':value['converter_executable']['sha256']='0'*64
    elif change=='wrong_library':value['runtime_correspondence_libraries'][0]['sha256']='0'*64
    elif change=='changed_sdf':
        sdf=tmp_path/'changed.sdf';sdf.write_bytes(SDF.read_bytes()+b'\n')
    path=tmp_path/'manifest.json';path.write_text(json.dumps(value))
    assert not new.authenticate_conversion(path,URDF,sdf,{},diagnostic=True)['pass']


@pytest.mark.parametrize('change',['input_path','extra_arg','output_path','mapped_inode','repeated_lifetime'])
def test_actual_converter_record_associations_are_required(tmp_path,change):
    manifest=json.loads(MANIFEST.read_text())
    record=json.loads(Path(manifest['process_evidence'][0]['path']).read_text())
    if change=='input_path':record['command'][3]='/wrong/input.xml'
    elif change=='extra_arg':record['command'].append('--unreviewed')
    elif change=='output_path':record['stdout']['path']=manifest['reproduced_sdf']['path']
    elif change=='mapped_inode':record['mapped_elf_files'][0]['inode']+=1
    elif change=='repeated_lifetime':
        other=json.loads(Path(manifest['process_evidence'][1]['path']).read_text())
        record['pid']=other['pid'];record['proc_start_ticks']=other['proc_start_ticks']
    record_path=tmp_path/'record.json';record_path.write_text(json.dumps(record))
    manifest['process_evidence'][0]=new.binding(record_path)
    path=tmp_path/'manifest.json';path.write_text(json.dumps(manifest))
    assert not new.authenticate_conversion(path,URDF,SDF,{},diagnostic=True)['pass']
