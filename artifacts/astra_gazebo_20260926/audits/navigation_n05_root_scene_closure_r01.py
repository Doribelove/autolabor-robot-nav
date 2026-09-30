"""Versioned N05 scene equality audit; no ROS or new seed allocation."""
from pathlib import Path
import csv
import hashlib
import json
import sys
import yaml

S=Path(__file__).resolve().parents[1]
W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
from prepare_navigation_n05_scenes import collision_signature

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

oldroot=S/'scenes/navigation_n04_development_r01'
newroot=S/'scenes/navigation_n05_development_r01'
reportpath=newroot/'preparation_report.json'
report=json.loads(reportpath.read_text())
assert report['status']=='offline_development_inputs_ready_pending_actor_live_qualification'
assert report['new_independent_geometry_count']==0
assert len(report['files'])==183
for item in report['files']:
    assert sha(item['path'])==item['sha256']
with (oldroot/'diagnostic_blocks.csv').open() as stream:before=list(csv.DictReader(stream))
with (newroot/'diagnostic_blocks.csv').open() as stream:after=list(csv.DictReader(stream))
assert len(before)==len(after)==12
results=[]
for a,b in zip(before,after):
    assert {k:a[k] for k in ('family','layout_id','parent_geometry_id','scene_id','replicate')}=={
           k:b[k] for k in ('family','layout_id','parent_geometry_id','scene_id','replicate')}
    assert int(b['seed'])==263002001+len(results)
    wa,wb=Path(a['world_path']),Path(b['world_path'])
    ma,mb=Path(a['manifest_path']),Path(b['manifest_path'])
    assert sha(wa)==a['world_sha256'] and sha(wb)==b['world_sha256']
    assert sha(ma)==a['manifest_sha256'] and sha(mb)==b['manifest_sha256']
    assert collision_signature(wa.read_text())==collision_signature(wb.read_text())
    assert (wa.parent/'map_export/map.pgm').read_bytes()==(wb.parent/'map_export/map.pgm').read_bytes()
    assert (wa.parent/'common_bundle/common_route.json').read_bytes()==(wb.parent/'common_bundle/common_route.json').read_bytes()
    oldscene,newscene=yaml.safe_load(ma.read_text()),yaml.safe_load(mb.read_text())
    assert len(oldscene['actors'])==len(newscene['actors'])
    # Actor arrays retain their original behavior, path and initial geometry.
    for x,y in zip(oldscene['actors'],newscene['actors']):
        for k in ('model_name','policy','start','end','conflict','gate_delay_s'):
            assert x[k]==y[k],(b['seed'],k)
    results.append(dict(seed=int(b['seed']),family=b['family'],parent_geometry_id=b['parent_geometry_id'],
        collision_signature_equal=True,raster_byte_equal=True,common_route_byte_equal=True,
        actor_policy_initial_geometry_equal=True,world_sha256=sha(wb),manifest_sha256=sha(mb)))

oldreviewpath=S/'audits/navigation_n04_scenes_independent/review.json'
oldreview=json.loads(oldreviewpath.read_text())
occluded=oldreview['static_geometry_results']['occluded_emergence']['supplement_quintic']
assert oldreview['independent_review'] is True and occluded['static_pass'] is True
assert occluded['interval_minus_033_lower_m']>0
assert report['all_static_witnesses_pass'] is False
out=S/'audits/navigation_n05_root_scene_closure_r01'
out.mkdir(exist_ok=False)
summary=dict(status='N05_twelve_proposed_blocks_geometry_and_route_byte_equal_to_N04',
    results=results,files_verified=len(report['files']),parent_geometries=6,
    N04_independent_scene_review=dict(path=str(oldreviewpath),sha256=sha(oldreviewpath)),
    inherited_occluded_static_supplement=occluded,
    limitations=['Occluded original sin4 witness remains negative; positive quintic static existence is only inherited geometry evidence.',
                 'No runtime tracking, actor timing, performance, independent-geometry or real vehicle qualification.',
                 'Twelve seeds proposed but unregistered/unexecuted; two repeated inputs per six parent geometries.'],
    report_binding=dict(path=str(reportpath),sha256=sha(reportpath)),ROS_started=False,seed_allocated=False)
(out/'report.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(dict(status=summary['status'],rows=len(results),files_verified=len(report['files']))))
