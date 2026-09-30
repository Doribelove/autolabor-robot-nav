#!/usr/bin/env python3
"""Actual Q04 model/metadata with independent synthetic fault mutations, no simulation."""
import copy,hashlib,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';R=S/'runs/passive_sweep_q04_static'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import dense_sweep_p04 as p
source=Path(p.__file__);source_sha=hashlib.sha256(source.read_bytes()).hexdigest()
urdf=(R/'robot_description_runtime.xml').read_text();sdf=(D/'conversion/conversion.sdf').read_text()
metadata=[json.loads(x)for x in (S/'audits/passive_q04_result_independent/mass_diagnosis_inputs/runtime_metadata.jsonl').read_text().splitlines()][0]
model=p.parse_converted_model(urdf,sdf);geometry=p.runtime_geometry(metadata);radii=p.validate_runtime_model(geometry,model);rows=[]
def check(name,fn,expected_rejection):
 try:fn();rejected=False;error=None
 except (ValueError,KeyError,TypeError)as e:rejected=True;error=str(e)
 rows.append({'name':name,'expected_rejection':expected_rejection,'rejected':rejected,'pass':rejected==expected_rejection,'error':error})
def base(v):return next(l for m in v['config']['models']if m['name']=='autolabor_m2'for l in m['links']if l['name']=='autolabor_m2::base_link')
def fault_metadata(kind):
 value=copy.deepcopy(metadata);b=base(value)
 if kind=='COM':b['com_local_m'][2]+=1e-6
 elif kind=='inertia':b['inertia_ixx_iyy_izz_ixy_ixz_iyz'][0]+=1e-5
 elif kind=='ODE_inertia':b['native_ode_inertia_rowmajor_stride4'][0]+=1e-5
 elif kind=='mass':b['mass_kg']+=1e-4
 elif kind=='finite_rotation':b['finite_rotation_mode']=1
 elif kind=='shape_dimension':next(x for x in b['collisions']if x['kind']=='box')['size_xyz_m'][0]+=1e-4
 elif kind=='body_missing':
  robot=next(m for m in value['config']['models']if m['name']=='autolabor_m2');robot['links']=robot['links'][:-1]
 p.runtime_geometry(value,model)
def fault_sdf(kind):
 value=ET.fromstring(sdf);body=next(x for x in value.findall('model/link')if x.get('name')=='base_link')
 if kind=='mass':element=body.find('inertial/mass');element.text=str(float(element.text)+1e-4)
 elif kind=='inertia':element=body.find('inertial/inertia/ixx');element.text=str(float(element.text)+1e-5)
 elif kind=='collision_missing':body.remove(body.find('collision'))
 elif kind=='fixed_frame_attachment':value.find('model/frame').set('attached_to','not_the_declared_parent')
 p.parse_converted_model(urdf,ET.tostring(value,encoding='unicode'))
check('actual_saved_model_and_metadata_baseline',lambda:p.runtime_geometry(metadata,model),False)
for kind in ('COM','inertia','ODE_inertia','mass','finite_rotation','shape_dimension','body_missing'):check('runtime_'+kind,lambda kind=kind:fault_metadata(kind),True)
for kind in ('mass','inertia','collision_missing','fixed_frame_attachment'):check('converted_'+kind,lambda kind=kind:fault_sdf(kind),True)
checks={'all12_actual_based_cases':len(rows)==12 and all(r['pass']for r in rows),'frozen_1e7_tolerance':p.MODEL_TOLERANCE==1e-7,'all7_body_union_radii_outward':len(radii)==7 and all(r['used_outward_union_radius_m']>=max(r['runtime_radius_m'],r['converted_radius_about_runtime_com_m'],r['URDF_source_radius_about_runtime_com_m'])for r in radii),'source_unchanged_during_review':source_sha==hashlib.sha256(source.read_bytes()).hexdigest()}
result={'pass':all(checks.values()),'checks':checks,'tests':rows,'reviewed_source_sha256':source_sha,'radius_audits':radii,'mapping':model['source_mapping'],'scope':'Synthetic fault mutations of actual frozen Q04 metadata/URDF and newly recorded offline conversion. Not a new Gazebo result and not a historical result reclassification.'}
with (D/'actual_model_tests.json').open('x')as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps({'pass':result['pass'],'checks':checks,'rows':rows},indent=2));assert result['pass']
