#!/usr/bin/env python3
"""Additional exact-input and split checks, no process or simulation startup."""
import csv,json,math,xml.etree.ElementTree as ET
from pathlib import Path
import recompute as r

def near(a,b):
    a,b=(r.independent_canonical(v,False) for v in (a,b))
    if a['family']!=b['family'] or len(a['boxes'])!=len(b['boxes']) or len(a['actor_path_points'])!=len(b['actor_path_points']):return False
    x,y=a['wall_half_width'],b['wall_half_width']
    if (x is None)!=(y is None) or (x is not None and abs(x-y)>=.2):return False
    return all(abs(x-y)<s for aa,bb in zip(a['boxes'],b['boxes']) for x,y,s in zip(aa,bb,(.2,.2,.1,.1,.1))) and all(abs(x-y)<.2 for aa,bb in zip(a['actor_path_points'],b['actor_path_points']) for x,y in zip(aa,bb))

def physical_model(xml):
    m=ET.fromstring(xml).find('model')
    return {'bodies':{l.get('name'):{'pose':r.canon(l.find('pose')) if l.find('pose') is not None else None,'inertial':r.canon(l.find('inertial')),'collision':[r.canon(c) for c in l.findall('collision')]}for l in m.findall('link')},'joints':[r.canon(j) for j in m.findall('joint')]}

def main():
    r.read(__file__);r.read(r.OUT/'recompute.py')
    rows=list(csv.DictReader(r.read(r.BASE/'diagnostic_blocks.csv').decode().splitlines()))
    layouts=r.js(r.S/'scenes/development_a01/layouts.json')+r.js(r.S/'scenes/development_a02/layouts.json')
    duplicate_pairs=[(a['layout_id'],b['layout_id']) for i,a in enumerate(layouts) for b in layouts[:i] if near(a,b)]
    assert not duplicate_pairs and len(layouts)==24
    parameters=[];geometry=[];bad_identity_rejections=0
    for row in rows:
        scene=r.ym(row['manifest_path']);text=r.read(row['world_path']).decode();root=ET.fromstring(text);actors=scene['actors'];actor_names=[a['model_name'] for a in actors]
        for a in actors:
            cell={'policy':a['policy'],'speed_cell':int(round(a['base_speed_mps']/.05)),'response_distance_cell':int(round(a['response_distance_m']/.2)),'margin_cell':int(round(a['response_time_margin_s']/.1)),'delay_cell':int(round(a['gate_delay_s']/.2))}
            cid='actor_'+r.digest(cell)[:24];v=int(r.digest(['astra_gazebo_20260926',cid,'allocation_v1'])[:16],16)%100
            assert cid==a['parameter_cell_id'] and 60<=v<80
            parameters.append({'seed':int(row['seed']),'actor':a['model_name'],'parameter_cell_id':cid,'hash_bucket':v,'split':'development'})
        # Initial actor-to-actor disjointness, not asynchronous crossing safety.
        ap=r.world_parts(text,[m.get('name') for m in root.findall('world/model') if m.get('name') not in actor_names])
        assert all(p.distance(q)>0 for i,(_,p,_) in enumerate(ap) for _,q,_ in ap[:i])
        physical=[(n,list(p.exterior.coords),z) for n,p,z in r.world_parts(text,actor_names)]
        paths=[{k:a[k] for k in ('model_name','start','end','conflict','collision_length_m','collision_width_m','collision_height_m','initial_pose_xyz_rpy')}for a in actors]
        geometry.append({'static':physical,'paths':paths})
        for args in [('',int(row['seed']),r.prep.METHODS[0]),('UNBOUND_TEST',int(row['seed']),r.prep.METHODS[0]),('x',int(row['seed']),'wrong_method')]+([('x',int(row['seed'])+999,r.prep.METHODS[0])] if actors else []):
            try:r.prep.bind_gate_identity(text,*args)
            except ValueError:bad_identity_rejections+=1
            else:raise AssertionError('bad identity accepted')
    assert all(geometry[i]==geometry[i+1] for i in range(0,12,2))
    reference=physical_model(r.read(r.BASE/'reference_robot/converted_robot.sdf'))
    physical_copies=[]
    for path in sorted((r.S/'contracts/navigation_n04_prepared_r01').glob('*/conversion/conversion.sdf')):
        assert physical_model(r.read(path))==reference
        physical_copies.append(str(path))
    assert len(physical_copies)==48
    registry=r.read(r.S/'seed_registry.csv');snap=r.OUT/'seed_registry_at_review.csv';snap.write_bytes(registry)
    # Bind immutable snapshot, not the mutable original, in final resources.
    del r.inputs[str((r.S/'seed_registry.csv').resolve())];r.read(snap)
    registered={int(v) for line in csv.DictReader(registry.decode().splitlines()) for v in line.values() if v and v.isdigit()}
    assert not registered.intersection(int(x['seed']) for x in rows)
    design=r.js(r.S/'contracts/navigation_n04_design.json');sched=design['schedule'];assert len(sched)==48 and design['maximum_episodes']==48
    for i,row in enumerate(rows):
        block=[x for x in sched if x['block_index']==i]
        assert len(block)==4 and {x['method'] for x in block}==set(design['methods']) and all(x['seed']==int(row['seed']) and x['parent_geometry_id']==row['parent_geometry_id'] for x in block)
    order={m:{p:sum(x['method']==m and x['method_order_in_block']==p for x in sched)for p in range(1,5)}for m in design['methods']}
    assert all(v==3 for m in order.values() for v in m.values())
    result={'schema':'astra_n04_independent_scene_closure_v1','status':'independent_exact_scene_and_split_closure_pass','parent_layouts_recomputed':24,'within_supplied_layouts_near_duplicate_pairs':duplicate_pairs,'parameter_cells':parameters,'all_actor_initial_shapes_disjoint':True,'all_twelve_replica_static_and_actor_path_geometries_identical_within_parent':True,'runtime_prepared_physical_model_equal_reference_count':len(physical_copies),'runtime_prepared_model_paths':physical_copies,'bad_identity_rejections':bad_identity_rejections,'proposed_seeds_absent_from_registry_snapshot':True,'paired_four_method_blocks':12,'method_order_counts':order,'materialized_train_validation_final_geometry_registry_available':False,'scope':'all24 supplied old development layouts and exact12 inherited N04 inputs only; not a global final-test isolation theorem','source_resources':[{'path':p,'sha256':h} for p,h in sorted(r.inputs.items())]}
    assert all(r.hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in r.inputs.items())
    r.save('closure_result.json',result);print('PASS',len(parameters),'actor parameter cells;',len(physical_copies),'converted physical models;',bad_identity_rejections,'negative identity cases')

if __name__=='__main__':main()
