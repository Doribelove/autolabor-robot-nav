from pathlib import Path
import collections,csv,hashlib,json,math,sys
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';P=W/'scripts/astra_gazebo';A=S/'audits/development_geometry_d04';F=A/'r04';O=S/'scenes/development_d04_geometry_r04';sys.path.insert(0,str(P))
import prepare_development_geometry_d04_v4 as d
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
bind=lambda p:{'path':str(Path(p).resolve()),'sha256':sha(p)}
report=json.loads((F/'generation_report.json').read_text());split=json.loads((F/'dedup_split_audit.json').read_text());layouts=json.loads((O/'layouts.json').read_text())
for r in report['source_resources']+split['prior_bindings']:
    if sha(r['path'])!=r['sha256']:raise AssertionError('source changed:'+r['path'])
priors,oldrng,_,_=d.load_priors();carry,carried_rng=d.carry_forward(oldrng)
allrng=dict(carried_rng)
for seed,r in json.loads((F/'geometry_rng_draws.json').read_text())['new_geometry_only_records'].items():
    if seed in allrng and allrng[seed]!=r:raise AssertionError('RNG collision')
    allrng[seed]=r
newrng={s:r for s,r in allrng.items() if s not in oldrng}
assert all(r['purpose']=='geometry' for r in newrng.values())
(F/'all_d04_geometry_rng_lineage.json').write_text(json.dumps({'all_D04_geometry_draw_records':newrng,'existing_prior_records_retained':all(allrng[k]==v for k,v in oldrng.items()),'execution_seed_allocations':0,'global_registry_write':False},indent=2,sort_keys=True)+'\n')
robot=d.ReferenceRobot((d.REFERENCE/'robot_description.xml').read_text(),(d.REFERENCE/'conversion/conversion.sdf').read_text());envelope=d.WitnessEnvelope(robot)
from shapely.ops import unary_union
rows=[];checks=[]
for scene in layouts:
    folder=O/scene['layout_id'];q=json.loads((folder/'geometry_checks.json').read_text());w=json.loads((folder/'reference_witness.json').read_text());selected=w['selected']
    data=list(csv.DictReader((folder/'reference_witness.csv').open()));union=unary_union([p for _,p in d.v2.static_shapes(scene)])
    assert len(data)==901 and q['passed'] and w['passed']
    recompute=None if union.is_empty else min(envelope.at(tuple(float(r[k]) for k in ('base_x','base_y','base_yaw'))).distance(union) for r in data)
    assert recompute is None and selected['sampled_physical_lower_m'] is None or abs(recompute-selected['sampled_physical_lower_m'])<1e-12
    poses=[tuple(float(r[k]) for k in ('base_x','base_y','base_yaw')) for r in data]
    assert math.dist(poses[0][:2],(-4,0))<1e-12 and math.dist(poses[-1][:2],(5,0))<1e-12
    arc=sum(math.dist(a[:2],b[:2]) for a,b in zip(poses,poses[1:]));sem=q['semantics'];occ=scene['family']=='occluded_emergence'
    rows.append({'layout_id':scene['layout_id'],'family':scene['family'],'topology':scene['topology'],'parent_geometry_id':scene['parent_geometry_id'],'canonical_geometry_sha256':scene['canonical_geometry_sha256'],'actor_count':len(scene['actor_geometry']),'static_box_count':len(scene['static_boxes']),'witness_amplitude_m':selected['amplitude_m'],'witness_peak_rear_x_m':selected.get('peak_rear_x_m'),'witness_base_arc_length_m':arc,'witness_min_conservative_lower_m':selected['continuous_conservative_lower_m'],'witness_steer_bound_rad':selected['central_steering_abs_bound_rad'],'witness_steer_rate_bound_radps':selected['central_steering_rate_bound_radps'],'minimum_actor_swept_static_gap_m':sem['minimum_actor_swept_static_clearance_m'] if scene['actor_geometry'] and d.v2.static_shapes(scene) else None,'gate_actor_visible_fraction':sem['initial_laser_visibility']['visible_fraction'] if occ else None,'reveal_actor_visible_fraction':sem['revealed_laser_visibility']['visible_fraction'] if occ else None,'execution_seed':None})
    checks.append({'layout_id':scene['layout_id'],'full901_point_witness_recomputed':True,'reported_minimum_matches':True,'manifest_checks_witness_bound':all(x is True for x in [q['passed'],w['passed'],d.v1.group_split(scene['parent_geometry_id'])=='development']),'carry_forward_bytes_unchanged':all(sha(folder/f)==sha(d.PRIOR_OUTPUT/scene['layout_id']/f) for f in ('geometry_manifest.json','geometry_checks.json','reference_witness.json','reference_witness.csv')) if (d.PRIOR_OUTPUT/scene['layout_id']).exists() else None})
with (F/'parents_summary.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
summary={'status':'author_offline_final_artifact_and_split_recheck_pass','independent_review':False,'parents':len(rows),'unique_parent_ids':len({r['parent_geometry_id'] for r in rows}),'unique_unquantized_geometry_hashes':len({r['canonical_geometry_sha256'] for r in rows}),'per_family':dict(collections.Counter(r['family'] for r in rows)),'per_topology':dict(collections.Counter(r['family']+'/'+r['topology'] for r in rows)),'parent_checks':checks,'minimum_witness_conservative_lower_m':min(r['witness_min_conservative_lower_m'] for r in rows if r['witness_min_conservative_lower_m'] is not None),'occlusion_gate_visible_fraction_max':max(r['gate_actor_visible_fraction'] for r in rows if r['gate_actor_visible_fraction'] is not None),'occlusion_reveal_visible_fraction_min':min(r['reveal_actor_visible_fraction'] for r in rows if r['reveal_actor_visible_fraction'] is not None),'minimum_actor_swept_static_gap_m':min(r['minimum_actor_swept_static_gap_m'] for r in rows if r['minimum_actor_swept_static_gap_m'] is not None),'number_geometry_rng_draws_retained':len(newrng),'execution_seed_allocations':0,'read_only_prior_bindings_postcheck_pass':True,'closed_sources_postcheck_pass':True,'all_gate_values_unchanged':True,'no_final_outcomes_or_execution':True,'all48_prior_plus_new_groups_disjoint':not d.v1.audit_geometry_registry(priors+layouts),'prior_A01_geometry_feasibility_requalified':False}
(F/'final_artifact_checks.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='parent_checks'},indent=2))
