#!/usr/bin/env python3
"""Inventory generated ROS dynamic-config defaults absent from B04 launch dumps.

This is a read-only source inventory. It neither observes an initialized
move_base namespace nor asserts which defaults the live plugin writes.
"""

import hashlib
import importlib
import json
from pathlib import Path


STAGE=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent/'inventory.json'
MODULES={
    'teb':'teb_local_planner.cfg.TebLocalPlannerReconfigureConfig',
    'dwa':'dwa_local_planner.cfg.DWAPlannerConfig',
    'costmap':'costmap_2d.cfg.Costmap2DConfig',
    'inflation':'costmap_2d.cfg.InflationPluginConfig',
    'obstacle':'costmap_2d.cfg.ObstaclePluginConfig',
}
METHODS={
    'actual_teb_rear_reference_b03':('TebLocalPlannerROS','teb'),
    'dwa_rear_reference_b03':('DWAPlannerROS','dwa'),
}


def binding(path):
    path=Path(path).resolve(strict=True)
    return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


def compare(defaults,prepared):
    return {'generated_default_keys':len(defaults),'prepared_explicit_keys':len(prepared),
            'generated_keys_absent_from_offline_launch_dump':sorted(set(defaults)-set(prepared)),
            'explicit_keys_not_in_generated_cfg':sorted(set(prepared)-set(defaults)),
            'explicit_override_values_differ_from_generated_defaults':sorted(k for k in defaults.keys()&prepared.keys()
                                                                     if type(defaults[k])is not type(prepared[k]) or defaults[k]!=prepared[k]),
            'generated_defaults':defaults}


def main():
    if OUT.exists():raise FileExistsError('preserve previous inventory bytes')
    cfg={};sources=[]
    for key,name in MODULES.items():
        mod=importlib.import_module(name)
        defaults=dict(mod.defaults)
        if not defaults:raise ValueError('empty generated config defaults: '+name)
        cfg[key]=defaults
        sources.append({'role':key,'module':name,'binding':binding(mod.__file__)})
    reports=[]
    for name,(active_key,cfg_key) in METHODS.items():
        p=STAGE/'audits/baseline_integration_b04/launch_preflight_final_r2'/name/'effective_move_base.json'
        tree=json.loads(p.read_text())
        method={'method':name,'active_planner':active_key,'offline_launch_dump':binding(p),
                'active_planner_cfg':compare(cfg[cfg_key],tree[active_key]),
                'costmaps':{}}
        for scope in ('global_costmap','local_costmap'):
            values=tree[scope]
            actual_plugins={(x['name'],x['type']) for x in values['plugins']}
            required={('static_layer','costmap_2d::StaticLayer'),
                      ('obstacle_layer','costmap_2d::ObstacleLayer'),
                      ('inflation_layer','costmap_2d::InflationLayer')}
            if actual_plugins!=required:raise ValueError('unexpected offline layer structure')
            method['costmaps'][scope]={
                'costmap_cfg':compare(cfg['costmap'],values),
                'inflation_cfg':compare(cfg['inflation'],values['inflation_layer']),
                'obstacle_cfg':compare(cfg['obstacle'],values['obstacle_layer'])}
        reports.append(method)
    b05=STAGE/'audits/baseline_static_qualification_b05/component_report.json'
    b06=STAGE/'audits/baseline_static_qualification_b06_independent/review.json'
    report={'schema':'astra_b07_generated_dynamic_defaults_inventory_v1',
            'status':'offline_gap_inventory_no_runtime_qualification',
            'module_sources':sources,'methods':reports,
            'b05_prior':binding(b05),'b06_exact_int_repair_review':binding(b06),
            'generator':binding(Path(__file__)),
            'authority':{'execution_authorized':False,'budget':0,'seeds_registered':0,
                         'ros_gazebo_started':False,'training_steps':0},
            'limits':['Generated cfg values are source candidates, not an observed initialized /move_base tree.',
                      'Actual dynamic-reconfigure namespace, auto-default timing and shape/physical ingress remain future gates.',
                      'No TEB/DWA static or dynamic baseline performance claim is made.']}
    OUT.write_text(json.dumps(report,ensure_ascii=False,indent=2,sort_keys=True)+'\n')
    print(json.dumps({'output':str(OUT),
                      'active_missing_defaults':{x['method']:len(x['active_planner_cfg']['generated_keys_absent_from_offline_launch_dump']) for x in reports},
                      'sha256':binding(OUT)['sha256']}))


if __name__=='__main__':main()
