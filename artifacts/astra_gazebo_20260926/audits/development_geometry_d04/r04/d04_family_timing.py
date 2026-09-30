from pathlib import Path
import hashlib,json,math,random,sys
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');P=S/'workspace/scripts/astra_gazebo';sys.path.insert(0,str(P));import scene_generator as v1
O=S/'scenes/development_d04_geometry_r04';A=S/'audits/development_geometry_d04/r04';rows=[]
for s in json.loads((O/'layouts.json').read_text()):
    for i,a in enumerate(s['actor_geometry']):
        rng=random.Random('D04-family-timing-v1|'+s['parent_geometry_id']+'|'+str(i));picked=None
        for draw in range(10000):
            p=v1.actor_parameters('development',rng)
            if s['family']=='sequential_interaction' and i>0 and not .8<=p['gate_delay_s']<=2.8:continue
            dt=p['gate_delay_s']+math.dist(a['start'],a['conflict'])/p['base_speed_mps']-(a['conflict'][0]+4)/.8
            if abs(dt)<=2.:picked=dict(p,nominal_arrival_difference_s=dt);break
        if picked is None:raise RuntimeError('family timing witness not found')
        rows.append({'layout_id':s['layout_id'],'family':s['family'],'actor_index':i,'selected_parameter_existence_witness':picked,'geometry_only_rng_key':'D04-family-timing-v1|'+s['parent_geometry_id']+'|'+str(i),'execution_seed_allocated':False})
r={'scope':'additional offline existence parameters, NOT released actor configuration; generic parent witness fields remain immutable','actors':rows,'all_cells_development':all(v1.group_split(r['selected_parameter_existence_witness']['parameter_cell_id'])=='development' for r in rows),'sequential_second_delay_uses_intersection_of_generic0_to1_and_family0point8_to2point8':True,'all_unreactive_arrival_abs_difference_at_most2s':True,'live_reactive_or_temporal_visibility_qualification':False,'global_seed_registry_modified':False}
(A/'family_actor_timing_witnesses.json').write_text(json.dumps(r,indent=2,sort_keys=True)+'\n');print(len(rows))
