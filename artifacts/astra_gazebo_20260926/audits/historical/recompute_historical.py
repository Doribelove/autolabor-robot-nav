#!/usr/bin/env python3
"""Read-only independent audit of immutable historical R1/R2 evidence.

No historical assessor imports, ROS, training, or historical writes. Output belongs
only to the Astra successor audit. Geometry is chassis-versus-actor XY geometry,
not a replacement for contact sensing or a complete 3D wheel model.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
import platform
import sys
import time
from pathlib import Path

import numpy as np
import yaml


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump_csv(path, rows):
    with path.open('w', newline='') as stream:
        out = csv.DictWriter(stream, fieldnames=list(rows[0]))
        out.writeheader()
        out.writerows(rows)


def rectangle(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*a-s*b, y+s*a+c*b) for a,b in
            [(-length/2,-width/2),(length/2,-width/2),
             (length/2,width/2),(-length/2,width/2)]]


def point_segment(p, a, b):
    vx, vy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*vx+(p[1]-a[1])*vy)/(vx*vx+vy*vy)))
    return math.hypot(p[0]-a[0]-t*vx, p[1]-a[1]-t*vy)


def signed_rectangle_gap(first, second):
    """Euclidean distance when disjoint; negative SAT translation depth inside."""
    max_gap = -float('inf')
    for poly in (first, second):
        for a,b in zip(poly, poly[1:]+poly[:1]):
            nx,ny = -(b[1]-a[1]), b[0]-a[0]
            norm = math.hypot(nx,ny)
            nx,ny = nx/norm,ny/norm
            p = [x*nx+y*ny for x,y in first]
            q = [x*nx+y*ny for x,y in second]
            max_gap = max(max_gap, min(q)-max(p), min(p)-max(q))
    if max_gap <= 0.:
        return max_gap
    return min(point_segment(p,a,b) for pa,pb in ((first,second),(second,first))
               for p in pa for a,b in zip(pb,pb[1:]+pb[:1]))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=args.workspace.resolve(); out=args.output.resolve(); out.mkdir(parents=True,exist_ok=True)
    out.relative_to(root/'artifacts/astra_gazebo_20260926/audits/historical')
    started=time.time()
    # Independently check geometry primitives before analyzing actual evidence.
    unit=rectangle(0,0,0,2,2)
    assert abs(signed_rectangle_gap(unit,rectangle(3,0,0,2,2))-1)<1e-12
    assert abs(signed_rectangle_gap(unit,rectangle(1.5,0,0,2,2))+.5)<1e-12
    assert abs(signed_rectangle_gap(unit,rectangle(3,3,0,2,2))-math.sqrt(2))<1e-12
    assert signed_rectangle_gap(unit,rectangle(1.5,0,math.pi/4,1,1))<0
    audit={'schema_version':'1.0','status':'in_progress','historical_writes':False,
           'ROS_started':False,'training_started':False,'python':sys.version,
           'numpy':np.__version__,'pyyaml':yaml.__version__,'platform':platform.platform(),
           'command':' '.join(sys.argv),'script_sha256':sha(Path(__file__)),
           'rounds':{},'limitations':[
             'Historical sources are read only; newly diagnosed issues do not overwrite frozen outcomes.',
             'Geometry uses synchronized-as-logged model_states snapshots; actor/ego fields may be asynchronously sampled by the historical logger.',
             'Chassis XY intersection is sufficient for a projected chassis collision given nominal overlapping Z, but no historical Z/roll/pitch/joint trace exists for full 3D collision reconstruction.',
             'A nonintersecting chassis does not exclude wheel/sensor collision.',
             'Historical zero contact messages are not independently validated physical no-contact evidence.',
             'Source current hashes and historical resource bindings are reported separately; no claim of full execution binary closure is added.' ]}
    integrity=[]; failures=[]; geometry=[]; direction=[]; fingerprints=[]; tf=[]
    for round_id in (1,2):
        base=root/f'artifacts/gazebo_pct_teb_dwa_benchmark/formal/round_{round_id}'
        assessment_path=base/f'formal_r{round_id}_assessment.json'
        assess=json.loads(assessment_path.read_text())
        prereg=yaml.load((base/f'formal_r{round_id}_preregistration.yaml').read_text(),Loader=yaml.CSafeLoader)
        source_csv=base/f'formal_r{round_id}_episode_results.csv'
        rows=list(csv.DictReader(source_csv.open()))
        checks=[]; method_summary={}
        assert len(rows)==(640 if round_id==1 else 800)
        assert len({(r['seed'],r['scene_id'],r['method'],r['attempt']) for r in rows})==len(rows)
        assert all(r['attempt']=='1' for r in rows)
        assert sha(source_csv)==assess['episode_results_sha256']
        for method in ('pct_scope_rl','analytic_prior','actual_teb','dwa'):
            rr=[r for r in rows if r['method']==method]
            method_summary[method]={'episodes':len(rr),'valid_goals':sum(r['valid_goal']=='True' for r in rr),
              'mean_failure_inclusive_time_s':float(np.mean([float(r['failure_inclusive_time_s']) for r in rr])),
              'minimum_conservative_clearance_m':min(float(r['minimum_conservative_clearance_m']) for r in rr),
              'failure_counts':dict(collections.Counter(r['failure_reason'] for r in rr))}
            assert method_summary[method]['valid_goals']==assess['method_summaries'][method]['valid_goals']
            assert abs(method_summary[method]['mean_failure_inclusive_time_s']-assess['method_summaries'][method]['mean_failure_inclusive_time_s'])<1e-12
        blocks=sorted({r['block_id'] for r in rows}); lookup={(r['block_id'],r['method']):r for r in rows}
        comparisons={}
        for baseline in ('actual_teb','dwa'):
            pct=np.array([float(lookup[b,'pct_scope_rl']['failure_inclusive_time_s']) for b in blocks])
            ref=np.array([float(lookup[b,baseline]['failure_inclusive_time_s']) for b in blocks])
            spec=prereg['multiplicity']; rng=np.random.default_rng(spec['bootstrap_seeds'][baseline])
            effects=[]
            # Batching preserves the exact row-major RNG sequence while bounding RAM.
            for offset in range(0,spec['bootstrap_samples'],2000):
                indices=rng.integers(0,len(blocks),size=(min(2000,spec['bootstrap_samples']-offset),len(blocks)))
                a=pct[indices].mean(axis=1); b=ref[indices].mean(axis=1)
                effects.extend(((b-a)/b).tolist())
            ci=np.quantile(effects,[.0125,.9875]); point=float((ref.mean()-pct.mean())/ref.mean())
            original=assess['primary_comparisons'][baseline]
            assert abs(point-original['relative_reduction'])<1e-12
            assert np.allclose(ci,[original['bootstrap']['lower_97_5_percent'],original['bootstrap']['upper_97_5_percent']],rtol=0,atol=1e-12)
            comparisons[baseline]={'relative_reduction':point,'ci_97_5':ci.tolist(),
               'bootstrap_samples':len(effects),'seed':spec['bootstrap_seeds'][baseline],'frozen_assessment_match':True}
        for r in rows:
            report_path=root/r['report_path']; receipt_path=root/r['receipt_path']
            raw=report_path.read_bytes(); receipt=json.loads(receipt_path.read_text()); d=yaml.load(raw,Loader=yaml.CSafeLoader); result=d['result']
            assert hashlib.sha256(raw).hexdigest()==r['report_sha256']==receipt['episode_report_sha256']
            assert sha(receipt_path)==r['receipt_sha256']
            reason=('contact' if result.get('contact_state_count',0)>0 else
                    'conservative_clearance_violation' if result['minimum_conservative_clearance_m']<0 else
                    'execution_integrity_failure' if not d['execution_integrity_pass'] else
                    'goal' if result['goal_reached'] else 'timeout_or_nonarrival')
            assert reason==r['failure_reason']
            assert abs((result['restricted_elapsed_s'] if reason=='goal' else 22)-float(r['failure_inclusive_time_s']))<1e-12
            assert abs(result['minimum_conservative_clearance_m']-float(r['minimum_conservative_clearance_m']))<1e-12
            assert d['seed']==int(r['seed']) and d['method']==r['method'] and d['scene_id']==r['scene_id']
            integrity.append({'round':round_id,'seed':r['seed'],'method':r['method'],'report_path':r['report_path'],
                              'report_sha256':r['report_sha256'],'receipt_sha256':r['receipt_sha256'],'hash_and_scientific_replay_pass':True})
            if round_id!=2:continue
            scene_path=report_path.parent/'scene_manifest.yaml'; scene=yaml.load(scene_path.read_text(),Loader=yaml.CSafeLoader)
            normalized={k:v for k,v in scene.items() if k not in ('seed','split')}
            fingerprints.append({'seed':r['seed'],'method':r['method'],'scene_id':r['scene_id'],
             'scene_without_seed_split_sha256':hashlib.sha256(json.dumps(normalized,sort_keys=True).encode()).hexdigest(),
             'actor_speeds_mps':json.dumps([a['base_speed_mps'] for a in scene['actors']])})
            if reason=='execution_integrity_failure':
                tf.append({'seed':r['seed'],'method':r['method'],'scene_id':r['scene_id'],
                 'scan_timestamp_transform_failure_count':result['scan_timestamp_transform_failure_count'],
                 'scan_timestamp_transform_success_rate':result['scan_timestamp_transform_success_rate'],
                 'latest_tf_success_rate':result['tf_success_rate'],'maximum_scan_age_s':result['maximum_scan_age_s'],
                 'maximum_tf_age_s':result['maximum_tf_age_s'],'log_path':str(report_path.parent.relative_to(root)/'roslaunch.log')})
            if r['method']=='pct_scope_rl' and reason!='goal':failures.append(dict(r))
            if r['method']!='pct_scope_rl' or int(r['seed']) not in (198136,198146):continue
            for i,t in enumerate(d['control_trace']):
                ego=t['ego_pose']; polygon=rectangle(ego['x_m'],ego['y_m'],ego['yaw_rad'],1.04,.70)
                gaps=[]; circles=[]
                for actor in t['actor_poses']:
                    # Both historical actor instances are yaw +/-pi/2 square boxes.
                    ap=rectangle(actor['x_m'],actor['y_m'],math.pi/2,.72,.72)
                    gaps.append(signed_rectangle_gap(polygon,ap))
                    circles.append(math.hypot(ego['x_m']-actor['x_m'],ego['y_m']-actor['y_m'])-1.15)
                geometry.append({'seed':r['seed'],'sample':i,'time_s':t['time_s'],
                  'x_m':ego['x_m'],'y_m':ego['y_m'],'yaw_rad':ego['yaw_rad'],'speed_mps':t['speed_mps'],
                  'recorded_conservative_clearance_m':t['conservative_clearance_m'],
                  'independent_circle_clearance_m':min(circles),'chassis_actor_signed_gap_m':min(gaps),
                  'chassis_actor_overlap':min(gaps)<0,'ROS_contact_count_episode':result['contact_state_count']})
            for i,t in enumerate(d['decision_trace']):
                a=t['executed_action']; tracker=t['directional_tracker_audit']; safety=t['candidate_safety_diagnostics']['candidates'][a]
                direction.append({'seed':r['seed'],'sample':i,'time_s':t['time_s'],'executed_action':a,
                  'ego_x_m':t['ego_pose']['x_m'],'ego_y_m':t['ego_pose']['y_m'],
                  'tracker_direction_x':tracker['safety_direction_world'][0],'tracker_direction_y':tracker['safety_direction_world'][1],
                  'tracker_valid':tracker['valid'],'tracker_fit_residual_m':tracker['fit_residual_m'],
                  'selected_backup_clearance_m':safety['backup_clearance_m'],'mask_stop':t['action_mask'][0],
                  'safe_nonstop_count':sum(t['action_mask'][1:]),'recorded_conservative_clearance_m':t['conservative_clearance_m']})
        audit['rounds'][str(round_id)]={'episodes':len(rows),'method_summary':method_summary,'comparisons':comparisons,
            'assessment_sha256':sha(assessment_path),'episode_csv_sha256':sha(source_csv),
            'raw_hash_and_scientific_replay_count':sum(x['round']==round_id for x in integrity)}
        print('round',round_id,'done',flush=True)
    audit['geometry']={}
    for seed in ('198136','198146'):
        rr=[r for r in geometry if r['seed']==seed];worst=min(rr,key=lambda r:r['chassis_actor_signed_gap_m'])
        audit['geometry'][seed]={'samples':len(rr),'chassis_overlap_sample_count':sum(r['chassis_actor_overlap'] for r in rr),
          'minimum_chassis_actor_signed_gap_m':worst['chassis_actor_signed_gap_m'],'worst_time_s':worst['time_s'],
          'max_absolute_circle_replay_error_m':max(abs(r['recorded_conservative_clearance_m']-r['independent_circle_clearance_m']) for r in rr),
          'ROS_contact_count_episode':0,'interpretation':'offline projected geometry diagnosis; not a rewritten historical outcome'}
    audit['randomization']={scene:{'unique_geometry_policy_speed_fingerprints':len({r['scene_without_seed_split_sha256'] for r in fingerprints if r['scene_id']==scene}),
         'actor_speed_vectors':sorted({r['actor_speeds_mps'] for r in fingerprints if r['scene_id']==scene})} for scene in sorted({r['scene_id'] for r in fingerprints})}
    audit['tf_integrity_failure_episodes']=len(tf)
    audit['tf_rejected_scan_total']=sum(r['scan_timestamp_transform_failure_count'] for r in tf)
    for name,rows in [('raw_replay',integrity),('r2_pct_failures',failures),('r2_failure_geometry',geometry),('r2_failure_direction',direction),('r2_scene_fingerprints',fingerprints),('r2_tf_failures',tf)]:
        dump_csv(out/(name+'.csv'),rows)
    audit['status']='offline_historical_statistics_and_raw_replay_pass_with_new_diagnostic_findings'
    audit['elapsed_wall_seconds']=time.time()-started
    (out/'audit.json').write_text(json.dumps(audit,indent=2,sort_keys=True)+'\n')
    print(json.dumps({k:audit[k] for k in ('status','geometry','randomization','tf_integrity_failure_episodes','tf_rejected_scan_total','elapsed_wall_seconds')},indent=2))


if __name__=='__main__':main()
