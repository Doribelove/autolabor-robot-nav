#!/usr/bin/env python3
"""Actual offline conversion records plus marked synthetic closure/fault cases."""
import copy,json,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];R=S/'runs/passive_sweep_q04_static';W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'));import dense_sweep_p04 as p
M=D/'conversion/manifest.json';manifest=json.loads(M.read_text());U=R/'robot_description_runtime.xml';F=D/'conversion/conversion.sdf';rows=[]
def save(path,obj):
 with path.open('x')as f:json.dump(obj,f,indent=2);f.write('\n')
 return p.binding(path)
def assess(name,m,audit,diagnostic,expected):
 path=D/(name+'.manifest.json');save(path,m);value=p.authenticate_conversion(path,U,F,audit,diagnostic)
 rows.append({'case':name,'expected_pass':expected,'pass':value['pass']==expected,'assessment':value});return value
original=p.authenticate_conversion(M,U,F,{},True);rows.append({'case':'actual_two_converters_diagnostic_baseline','expected_pass':True,'pass':original['pass'],'assessment':original})
for kind in ('wrong_input_argv','wrong_stdout_path','duplicate_process_record'):
 changed=copy.deepcopy(manifest)
 if kind=='duplicate_process_record':changed['process_evidence'][1]=changed['process_evidence'][0]
 else:
  record=json.loads(Path(changed['process_evidence'][0]['path']).read_text())
  if kind=='wrong_input_argv':record['command'][3]=str(D/'wrong_input.urdf')
  else:record['stdout']=changed['reproduced_sdf']
  record_binding=save(D/(kind+'.process.json'),record);changed['process_evidence'][0]=record_binding;changed['resources'].append(record_binding)
 assess(kind,changed,{},True,False)
# Positive/negative fresh-schema validator fixtures are NOT new runtime evidence.
engine=json.loads((R/'engine_binary_maps.json').read_text());engine['synthetic_fixture_only']=True
library=manifest['runtime_correspondence_libraries'][0]
engine['loaded_modules'].append(dict(library,roles=['urdf_to_sdf_converter']))
engine_binding=save(D/'synthetic_engine_maps.json',engine)
closure=json.loads((R/'run_closure.json').read_text());closure.update(schema='astra_p04_run_closure_v1',synthetic_fixture_only=True,qualification_authority='none')
closure['bindings'].update(engine_binary_maps=[engine_binding],converted_robot_sdf=[p.binding(F)],urdf_sdf_conversion_manifest=[p.binding(M)],conversion_tool=[manifest['converter_executable']],conversion_libraries=[library])
closure_binding=save(D/'synthetic_closure.json',closure);audit=p.authenticate_closure(closure_binding['path'],1160000000,11160000000)
value=p.authenticate_conversion(M,U,F,audit,False);rows.append({'case':'fresh_schema_synthetic_all_bindings_valid','expected_pass':True,'pass':audit['pass']and value['pass'],'closure_audit':audit,'assessment':value,'synthetic_not_simulation_evidence':True})
for name in ('missing_runtime_sdformat','missing_conversion_manifest_role'):
 bad=copy.deepcopy(audit)
 if name=='missing_runtime_sdformat':
  no_role=copy.deepcopy(engine);no_role['loaded_modules']=[x for x in no_role['loaded_modules']if 'urdf_to_sdf_converter'not in x.get('roles',[])];bb=save(D/'synthetic_engine_no_sdformat.json',no_role)
  bad['bindings']=[dict(bb,role=x['role'])if x['role']=='engine_binary_maps'else x for x in bad['bindings']]
 else:bad['bindings']=[x for x in bad['bindings']if x['role']!='urdf_sdf_conversion_manifest']
 value=p.authenticate_conversion(M,U,F,bad,False);rows.append({'case':name,'expected_pass':False,'pass':not value['pass'],'assessment':value,'synthetic_not_simulation_evidence':True})
report={'pass':all(x['pass']for x in rows),'tests':rows,'scope':'Only validator fixtures. Fresh-schema positive uses explicitly augmented old-Q04 closure data and conveys no live qualification or historical reclassification.','reviewed_source_sha256':p.sha(p.__file__),'actual_conversion_manifest':p.binding(M)}
save(D/'actual_conversion_binding_tests.json',report);print(json.dumps({'pass':report['pass'],'cases':len(rows),'failed':[x['case']for x in rows if not x['pass']]},indent=2));assert report['pass']
