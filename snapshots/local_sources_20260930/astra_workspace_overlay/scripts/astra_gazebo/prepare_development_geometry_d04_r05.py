#!/usr/bin/env python3
"""R05 joint parent/actual-cell allocation and raster-safe static input audit.

Deterministic enumeration only: no random generator, seed, ROS or episode.
"""
import argparse,copy,csv,hashlib,itertools,json,math,shutil,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon,Point,MultiPoint
from shapely.ops import unary_union
import prepare_development_geometry_d04_v4 as old
from static_map_export import make_artifacts
from common_route import build_bundle,load_bundle
from route_candidate_core import occupied_union

STAGE=old.STAGE;VERSION='astra_development_geometry_d04_r05'
DESIGN=STAGE/'audits/development_geometry_d04_r05/design.json'
DESIGN_SHA='f680b01930e12c2291a56c25b17c1859e1d46e344369ed8319c7b0b4e09b67ac'
PRIOR=STAGE/'scenes/development_d04_geometry_r04'
sha=old.sha;bind=old.bind;write=old.write
POLICIES=('cautious','hesitant','assertive')
CELL_KEYS=('policy','speed_cell','response_distance_cell','margin_cell','delay_cell')

def joint(parent,cell):
    fields=('astra_gazebo_20260926',parent,cell,'allocation_v1')
    if any(not isinstance(v,str) or not v or '|' in v or '\n' in v for v in fields):raise ValueError('canonical nonempty delimiter-free IDs required')
    payload='|'.join(fields);digest=hashlib.sha256(payload.encode('utf-8')).hexdigest();bucket=int(digest[:16],16)%100
    split='train' if bucket<60 else 'development' if bucket<80 else 'validation' if bucket<90 else 'final_test'
    return {'payload_utf8':payload,'payload_utf8_hex':payload.encode().hex(),'sha256':digest,'bucket':bucket,'split':split}

def cell_id(cell):
    if not isinstance(cell,dict) or set(cell)!=set(CELL_KEYS) or cell['policy'] not in POLICIES:raise ValueError('exact actual actor cell required')
    bounds={'speed_cell':(13,22),'response_distance_cell':(15,25),'margin_cell':(9,17),'delay_cell':(0,5)}
    if any(type(cell[k])is not int or not low<=cell[k]<=high for k,(low,high) in bounds.items()):raise ValueError('actor cell outside frozen finite domain')
    return 'actor_'+old.v1.digest(cell)[:24]

def tuple_cell(cells):
    if not cells:return 'actor_none_v1'
    ids=[cell_id(c) for c in cells]
    if len(cells)==1:return ids[0]
    actual=[[f'benchmark_actor_{i+1}',key] for i,key in enumerate(ids)]
    encoded=json.dumps(actual,separators=(',',':'),ensure_ascii=False).encode('utf-8')
    return 'actor_tuple_v1_'+hashlib.sha256(encoded).hexdigest()

def parameters(cell):
    return {'parameter_cell_id':cell_id(cell),'policy':cell['policy'],'base_speed_mps':cell['speed_cell']/20,
        'response_distance_m':cell['response_distance_cell']/5,'response_time_margin_s':cell['margin_cell']/10,'gate_delay_s':cell['delay_cell']/5}

class Ownership:
    def __init__(self,parents=None,cells=None):self.parents=dict(parents or {});self.cells=dict(cells or {})
    def reject(self,parent,cells):
        row=joint(parent,tuple_cell(cells))
        if row['split']!='development':return 'joint_hash_not_development',row
        if self.parents.get(parent,'development')!='development':return 'prior_parent_split_ownership',row
        if any(self.cells.get(cell_id(c),'development')!='development' for c in cells):return 'prior_atomic_actor_cell_split_ownership',row
        return None,row
    def commit(self,parent,cells):
        reason,row=self.reject(parent,cells)
        if reason:raise ValueError(reason)
        self.parents[parent]='development'
        for c in cells:self.cells[cell_id(c)]='development'
        return row

def prior_data():
    parents,_,bindings,derived=old.load_priors()
    current=json.loads((PRIOR/'layouts.json').read_text());parents+=current;bindings.append(bind(PRIOR/'layouts.json'))
    owner=Ownership({p['parent_geometry_id']:p['split'] for p in parents})
    for p in current:
        for row in p['actor_parameter_existence_witnesses']:owner.cells[row['parameter_cell_id']]='development'
    supplement=STAGE/'audits/development_geometry_d04/r04/family_actor_timing_witnesses.json';bindings.append(bind(supplement))
    for row in json.loads(supplement.read_text())['actors']:owner.cells[row['selected_parameter_existence_witness']['parameter_cell_id']]='development'
    # Existing actual parameter cells are read from input manifests, never outcomes.
    for cohort in ('development_a01','development_a02'):
        table=STAGE/'scenes'/cohort/'diagnostic_blocks.csv';bindings.append(bind(table))
        for row in csv.DictReader(table.open()):
            import yaml
            path=Path(row['manifest_path'])
            if sha(path)!=row['manifest_sha256']:raise ValueError('prior actor manifest changed')
            manifest=yaml.safe_load(path.read_text());bindings.append(bind(path))
            for actor in manifest['actors']:
                key=actor['parameter_cell_id'];split=manifest['split']
                if key in owner.cells and owner.cells[key]!=split:raise ValueError('historical actor-cell ownership conflict')
                owner.cells[key]=split
    return parents,current,owner,bindings

def admissible_actor_cells(scene,index,policy_start,used):
    actor=scene['actor_geometry'][index];order=POLICIES[policy_start:]+POLICIES[:policy_start]
    for policy,speed,dist,margin,delay in itertools.product(order,range(13,23),range(15,26),range(9,18),range(6)):
        cell=dict(zip(CELL_KEYS,(policy,speed,dist,margin,delay)));key=cell_id(cell)
        if key in used:continue # new actual cells, not cosmetic relabelling
        p=parameters(cell)
        if scene['family']=='sequential_interaction' and index>0 and not .8<=p['gate_delay_s']<=2.8:continue
        difference=p['gate_delay_s']+math.dist(actor['start'],actor['conflict'])/p['base_speed_mps']-(actor['conflict'][0]+4)/.8
        if abs(difference)<=2.:yield cell

def find_cells(scene,owner,slot,rejections):
    if not scene['actor_geometry']:
        reason,row=owner.reject(scene['parent_geometry_id'],[])
        if reason:rejections.append(dict(row,parent_geometry_id=scene['parent_geometry_id'],reason=reason))
        return None if reason else ([],row)
    lists=[list(admissible_actor_cells(scene,i,(slot+i)%3,set(owner.cells))) for i in range(len(scene['actor_geometry']))]
    for trial,cells in enumerate(itertools.product(*lists)):
        if trial>=50000:raise RuntimeError('bounded actual-cell product exhausted')
        if len({cell_id(c) for c in cells})!=len(cells):continue
        reason,row=owner.reject(scene['parent_geometry_id'],cells)
        if reason:rejections.append(dict(row,parent_geometry_id=scene['parent_geometry_id'],actual_cells=cells,reason=reason));continue
        return list(cells),row
    raise RuntimeError('no family-eligible actor cell tuple')

def new_geometries(family):
    if family=='static_avoidance':
        for x,y,length,width,half in itertools.product((-1.4,-.8,-.2,.4,1.,1.6,2.2),(-.5,0.,.5),(.55,.95,1.35),(.5,.8,1.05),(2.5,2.9,3.2)):
            yield {'family':family,'topology':'one_block_offset','corridor_half_width_m':half,'static_boxes':[old.v1.box('block_1',x,y,length,width)],'static_circles':[],'actor_geometry':[],
                'route':{'frame':'world','start':[-4.,0.,0.],'goal':[5.,0.,0.],'spawn':[-5.4,0.,0.],'length_m':9.,'transform_implemented':'yaw_zero_only'}}
    elif family=='width_change_corridor':
        for topology,wide,narrow,shift,length,center in itertools.product(('symmetric_throat','offset_throat','two_stage_throat'),(1.8,2.2,2.6),(.9,1.2,1.5),(-.5,0.,.5),(1.4,2.4,3.4),(-1.6,-.4,.8,2.)):
            if topology=='symmetric_throat' and shift!=0.:continue
            boxes=[]
            for sign in (-1,1):
                boxes.extend([old.v1.box('entry_'+str(sign),-3.5,sign*(wide+.1),3.,.2,1.),old.v1.box('throat_'+str(sign),center,shift+sign*(narrow+.1),length,.2,1.),old.v1.box('exit_'+str(sign),4.5,sign*(wide+.1),2.5,.2,1.)])
                if topology=='two_stage_throat':boxes.append(old.v1.box('second_throat_'+str(sign),-1.9,-shift+sign*(narrow+.3),.8,.2,1.))
            yield {'family':family,'topology':topology,'corridor_half_width_m':None,'static_boxes':boxes,'static_circles':[],'actor_geometry':[],
                'route':{'frame':'world','start':[-4.,0.,0.],'goal':[5.,0.,0.],'spawn':[-5.4,0.,0.],'length_m':9.,'transform_implemented':'yaw_zero_only'}}
    else:raise ValueError('only no-actor replacements are preregistered')

def static_sdf(scene):
    root=ET.Element('sdf',version='1.6');world=ET.SubElement(root,'world',name='D04_R05_STATIC_GEOMETRY_ONLY_NO_EPISODE')
    for b in scene['static_boxes']:old.v1.add_box(world,b['name'],b['x'],b['y'],b['length'],b['width'],b['height'],b.get('yaw_rad',0.))
    half=scene['corridor_half_width_m']
    if half is not None:
        for sign in (-1,1):old.v1.add_box(world,'wall_'+str(sign),0,sign*(half+.1),18,.2,1.)
    return ET.tostring(root,encoding='unicode')+'\n'

def guide_check(route,blocked,envelope):
    segments=[];corners=[];points=route.points;headings=[]
    for a,b in zip(points,points[1:]):
        heading=math.atan2(b[1]-a[1],b[0]-a[0]);headings.append(heading)
        swept=unary_union([envelope.at((*a,heading)),envelope.at((*b,heading))]).convex_hull
        gap=swept.distance(blocked);segments.append({'physical_gap_m':gap,'conservative_gap_m':gap-.33,'intersects':swept.intersects(blocked)})
    for i,p in enumerate(points):
        before=0. if i==0 else headings[i-1];after=route.goal_yaw if i==len(points)-1 else headings[i]
        if abs(math.atan2(math.sin(after-before),math.cos(after-before)))<1e-10:continue
        disk=Point(p).buffer(envelope.radius/math.cos(math.pi/128),resolution=32)
        gap=disk.distance(blocked);corners.append({'point_index':i,'before_rad':before,'after_rad':after,'physical_gap_m':gap,'conservative_gap_m':gap-.33,'all_yaw_disk_covers_transition':True})
    minimum=min([r['conservative_gap_m'] for r in segments+corners])
    return {'passed':minimum>0,'minimum_full_footprint_conservative_gap_m':minimum,'segments':segments,'heading_transitions':corners,
        'proof':'convex hull of constant-heading endpoint footprints is exact translation sweep; outward circumdisk bounds every corner rotation',
        'Ackermann_feasible_corner_transition':False,'scope':'static occupancy of exported geometric guide; neither physical tracking nor in-place Ackermann-turn claim'}

def raster_check(scene,folder,envelope,native_witness):
    folder.mkdir(parents=True);text=static_sdf(scene);(folder/'static_geometry.sdf').write_text(text)
    source=copy.deepcopy(scene);source['actors']=[];source['robot_collision_reference']={'spawn_xyz_rpy':[-5.4,0.,0.,0.,0.,0.]}
    pgm,meta,request,audit=make_artifacts(text,source,resolution=.05,bounds_xyxy=(-20,-15,20,15))
    exported=folder/'map_export';exported.mkdir();(exported/'map.pgm').write_bytes(pgm);(exported/'map.yaml').write_bytes(meta);write(exported/'route_request.json',request);write(exported/'audit.json',audit)
    data=build_bundle(exported,folder/'common_bundle',radius=1.);grid,route,_=load_bundle(folder/'common_bundle');blocked=occupied_union(grid)
    native=unary_union([p for _,p in old.v2.static_shapes(scene)])
    if native.difference(blocked.buffer(1e-12)).area>1e-10:raise ValueError('raster failed to cover native static geometry')
    original=native_witness['rows'];selected=native_witness['selected'];gap=min(envelope.at(r[2:5]).distance(blocked) for r in original)
    lower=gap-selected['point_lipschitz_per_rear_x']*.01/2-.33
    raster_witness=old.witness(envelope,[('occupied_unknown_union',blocked)])
    guide=guide_check(route,blocked,envelope)
    result={'passed':bool(raster_witness['passed'] and guide['passed']),'original_native_witness_conservative_lower_after_raster_m':lower,
        'original_native_witness_survives_raster':lower>0,'raster_witness':{k:v for k,v in raster_witness.items() if k!='rows'},'guide':guide,
        'occupied_unknown_pixel_union_exact':True,'native_collision_union_covered':True,'resolution_m':.05,'guide_radius_m':1.,'source_map_and_guide_bindings':[bind(folder/'common_bundle'/n) for n in ('map.pgm','map.yaml','common_route.json')],
        'no_carving_no_margin_reduction':True,'execution_authorized':False}
    write(folder/'raster_guide_checks.json',result)
    if raster_witness['passed']:
        with (folder/'raster_witness.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(('rear_x','rear_y','base_x','base_y','base_yaw','curvature'));w.writerows(raster_witness['rows'])
    return result

def build(output,audit):
    output,audit=Path(output),Path(audit)
    if sha(DESIGN)!=DESIGN_SHA:raise ValueError('preregistered explicit edge convention changed')
    if output.exists():raise FileExistsError('retain all previous R05 inputs')
    output.mkdir(parents=True);audit.mkdir(parents=True,exist_ok=True)
    priors,current,owner,bindings=prior_data();robot=old.ReferenceRobot((old.REFERENCE/'robot_description.xml').read_text(),(old.REFERENCE/'conversion/conversion.sdf').read_text());envelope=old.WitnessEnvelope(robot)
    rejections=[];geometry_rejections=[];selected=[];selected_geometries=[];combos=[];attempt=0
    # Materialized before first candidate filtering; design and parent ownership never rewritten.
    write(audit/'initial_ownership.json',{'parent_groups':owner.parents,'actual_actor_cells':owner.cells,'prior_bindings':bindings,'design':bind(DESIGN),'source':bind(__file__)})
    for family in old.v1.FAMILIES:
        originals=[p for p in current if p['family']==family]
        # Retain original order/geometry where joint predicate permits; no-actor replacements follow fixed Cartesian enumeration.
        candidates=itertools.chain(((p,True,i) for i,p in enumerate(originals)),((p,False,i) for i,p in enumerate(new_geometries(family))) if family in ('static_avoidance','width_change_corridor') else ())
        accepted=0
        for raw,inherited,ordinal in candidates:
            if accepted==4:break
            scene=copy.deepcopy(raw);pid=old.v1.parent_id(scene);scene['parent_geometry_id']=pid;attempt+=1
            if not inherited and any(old.v1.near_duplicate(scene,p) or ((distance:=old.separation(scene,p))is not None and distance<2.) for p in priors+selected_geometries):
                geometry_rejections.append({'attempt':attempt,'family':family,'ordinal':ordinal,'parent_geometry_id':pid,'reason':'registered_or_selected_structural_near_duplicate'});continue
            try:found=find_cells(scene,owner,accepted,rejections)
            except Exception as exc:
                write(audit/'rejected_joint_combinations.json',rejections);write(audit/'rejected_geometry_and_raster.json',geometry_rejections)
                write(audit/'retained_failure.json',{'family':family,'parent':pid,'exception':type(exc).__name__+':'+str(exc),'N06_release':False});raise
            if found is None:continue
            cells,jointrow=found;qc=old.full_geometry_checks(scene,robot,envelope)
            route=old.witness(envelope,old.v2.static_shapes(scene)) if qc['passed'] else None
            if not qc['passed'] or not route['passed']:
                record={'attempt':attempt,'family':family,'ordinal':ordinal,'parent_geometry_id':pid,'reason':'full_geometry_or_native_witness_failed','geometry':scene,'checks':qc,'witness':None if route is None else {k:v for k,v in route.items() if k!='rows'}}
                geometry_rejections.append(record);continue
            rr=None
            if family=='static_avoidance':
                attempt_folder=audit/'raster_attempts'/f'attempt_{attempt:05d}'
                try:rr=raster_check(scene,attempt_folder,envelope,route)
                except Exception as exc:
                    write(attempt_folder/'exception.json',{'exception':type(exc).__name__+':'+str(exc),'parent_geometry_id':pid,'geometry':scene});rr={'passed':False,'exception':str(exc)}
                if not rr['passed']:
                    geometry_rejections.append({'attempt':attempt,'family':family,'ordinal':ordinal,'parent_geometry_id':pid,'reason':'raster_or_exported_guide_failed','geometry':scene,'artifact_directory':str(attempt_folder)});continue
            jointrow=owner.commit(pid,cells);layout=f'd04r05_development_{family}_{accepted:03d}';folder=output/layout;folder.mkdir()
            actors=[dict(copy.deepcopy(g),**parameters(c),model_name=f'benchmark_actor_{i+1}',actual_cell=c) for i,(g,c) in enumerate(zip(scene['actor_geometry'],cells))]
            scene.update(schema=VERSION,layout_id=layout,split='development',actor_parameter_cell_id=tuple_cell(cells),actors=actors,
                joint_allocation=jointrow,execution_authorized=False,execution_seed=None,new_seed_allocated=False,
                parent_geometry_retained_from_r04=inherited,source_r04_layout_id=raw.get('layout_id') if inherited else None,
                geometry_enumeration=None if inherited else {'family_grid_ordinal':ordinal,'design_sha256':DESIGN_SHA},canonical_geometry_sha256=old.v1.digest(old.v1.canonical_geometry(scene,False)))
            if not inherited:scene['geometry_rng_seed']=None
            scene.pop('actor_parameter_existence_witnesses',None)
            write(folder/'geometry_manifest.json',scene);write(folder/'geometry_checks.json',qc);write(folder/'reference_witness.json',{k:v for k,v in route.items() if k!='rows'})
            with (folder/'reference_witness.csv').open('w',newline='') as f:
                w=csv.writer(f);w.writerow(('rear_x','rear_y','base_x','base_y','base_yaw','curvature'));w.writerows(route['rows'])
            if rr:shutil.copytree(attempt_folder,folder/'raster_and_guide')
            selected.append(scene);selected_geometries.append(scene);combos.append({'layout_id':layout,'parent_geometry_id':pid,'actor_parameter_cell_id':tuple_cell(cells),'actual_atomic_cells':cells,'joint':jointrow,'parent_prior_owner':initial_parent_owner(priors,pid),'inherited_geometry':inherited,'raster_guide_pass':None if rr is None else rr['passed']})
            accepted+=1;print(json.dumps({'accepted':layout,'parent':pid,'bucket':jointrow['bucket'],'inherited':inherited,'attempt':attempt}),flush=True)
            write(audit/'rejected_joint_combinations.json',rejections);write(audit/'rejected_geometry_and_raster.json',geometry_rejections)
        if accepted!=4:
            write(audit/'rejected_joint_combinations.json',rejections);write(audit/'rejected_geometry_and_raster.json',geometry_rejections)
            write(audit/'retained_failure.json',{'family':family,'accepted':accepted,'selected_total':len(selected),'reason':'finite input support exhausted','N06_release':False});raise RuntimeError('R05 finite geometry/cell support exhausted '+family)
    write(output/'layouts.json',selected);write(audit/'selected_joint_combinations.json',combos)
    write(audit/'final_ownership.json',{'parent_groups':owner.parents,'actual_actor_cells':owner.cells,'global_registry_modified':False,'future_allocation_must_consume_this_ownership':True})
    write(audit/'generation_report.json',{'status':'24_joint_development_combinations_and4_static_raster_guides_offline_pass_review_pending','design':bind(DESIGN),'source':bind(__file__),'selected_combinations':len(selected),'new_parent_geometries':sum(not p['parent_geometry_retained_from_r04'] for p in selected),'retained_r04_parent_geometries':sum(p['parent_geometry_retained_from_r04'] for p in selected),'new_execution_or_geometry_seeds_allocated':0,'rejected_joint_combinations':len(rejections),'rejected_geometry_or_raster':len(geometry_rejections),'N06_static_raster_guide_count':4,'ROS_started':False,'final_outcomes_read':False,'execution_release':False,'independent_review':False,'source_resources':[bind(p) for p in sorted(output.rglob('*')) if p.is_file()],'retained_prior_bindings':bindings})

def initial_parent_owner(priors,parent):
    values={p['split'] for p in priors if p['parent_geometry_id']==parent}
    if len(values)>1:raise ValueError('prior split conflict')
    return next(iter(values)) if values else None

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--audit',required=True,type=Path);a=p.parse_args();build(a.output,a.audit)
