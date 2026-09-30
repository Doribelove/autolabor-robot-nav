#!/usr/bin/env python3
"""D04 input-only parent geometries. No episodes, execution seeds or simulator.

Reuse frozen v2 draws; preserve input rejections and independent-unit identity.
The finite witness is static planar geometry, never a planner success label.
"""
import argparse,copy,csv,hashlib,json,math,random,sys
from collections import Counter
from pathlib import Path
import numpy as np
import yaml
from shapely.geometry import Polygon,Point,MultiPoint
from shapely.ops import unary_union
import scene_generator_v2 as v2
from prepare_navigation_n04_scenes import ReferenceRobot,REFERENCE
from geometry import Pose3

v1=v2.v1;STAGE=v1.STAGE;VERSION='astra_development_geometry_d04_v1'
TOPOLOGIES={
 'static_avoidance':['one_block_offset'],
 'single_crossing':list(v1.TOPOLOGIES['single_crossing']),
 'head_on_yield':list(v1.TOPOLOGIES['head_on_yield']),
 'sequential_interaction':['alternating_crossings','same_side_crossings'],
 'occluded_emergence':['single_screen'],
 'width_change_corridor':list(v1.TOPOLOGIES['width_change_corridor'])}
AMPLITUDES=(0.,.8,-.8,1.2,-1.2,1.6,-1.6)

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def bind(p):return {'path':str(Path(p).resolve()),'sha256':sha(p)}
def write(p,data):Path(p).write_text(json.dumps(data,indent=2,sort_keys=True,allow_nan=False)+'\n')

def separation(a,b):
    """Maximum structural cell difference, with reflection equivalence retained."""
    x,y=v1.canonical_geometry(a,False),v1.canonical_geometry(b,False)
    if x['family']!=y['family']:return None
    if len(x['boxes'])!=len(y['boxes']) or len(x['actor_path_points'])!=len(y['actor_path_points']) or (x['wall_half_width'] is None)!=(y['wall_half_width'] is None):return None
    differences=[]
    if x['wall_half_width'] is not None:differences.append(abs(x['wall_half_width']-y['wall_half_width'])/.2)
    for aa,bb in zip(x['boxes'],y['boxes']):differences.extend(abs(u-v)/step for u,v,step in zip(aa,bb,(.2,.2,.1,.1,.1)))
    for aa,bb in zip(x['actor_path_points'],y['actor_path_points']):differences.extend(abs(u-v)/.2 for u,v in zip(aa,bb))
    return max(differences,default=0.)

def load_priors():
    priors=[];records={};bindings=[];derivations=[]
    for cohort in ('development_a01','development_a02'):
        path=STAGE/'scenes'/cohort/'layouts.json';report=path.parent/'generation_report.json'
        priors.extend(json.loads(path.read_text()));bindings.extend([bind(path),bind(report)])
        for seed,record in json.loads(report.read_text())['rng_records'].items():
            if seed in records and records[seed]!=record:raise ValueError('prior RNG identity conflict')
            records[seed]=record
    if v1.audit_geometry_registry(priors):raise ValueError('prior independent geometry registry invalid')
    byid={p['parent_geometry_id']:p for p in priors}
    for cohort in ('navigation_n04_development_r01','navigation_n05_development_r01'):
        table=STAGE/'scenes'/cohort/'diagnostic_blocks.csv';bindings.append(bind(table))
        rows=list(csv.DictReader(table.open()))
        for row in rows:
            p=Path(row['manifest_path'])
            if sha(p)!=row['manifest_sha256']:raise ValueError('derived prior manifest hash changed')
            s=yaml.safe_load(p.read_text());bindings.append(bind(p));parent=byid[s['parent_geometry_id']]
            raw=copy.deepcopy(s);raw['static_boxes']=s['parent_static_boxes'];raw['static_circles']=s['parent_static_circles'];raw['corridor_half_width_m']=s['parent_corridor_half_width_m']
            if v1.canonical_geometry(raw,False)!=v1.canonical_geometry(parent,False):raise ValueError('N04/N05 derived identity actually changed parent geometry')
            derivations.append({'cohort':cohort,'parent_geometry_id':s['parent_geometry_id'],'canonical_parent_matches':True,'count_as_new_parent':False})
    return priors,records,bindings,derivations

class WitnessEnvelope:
    """Outward full-9 converted geometry support, all wheel steering/spin + tilt."""
    def __init__(self,robot):
        neutral=robot.project(Pose3());front={'front_left_steer_link','front_right_steer_link','front_left_wheel_link','front_right_wheel_link'}
        parts=[Polygon(p.polygon_xy) for p in neutral if p.link_name not in front];self.front_radii={}
        for side,sign in [('left',1),('right',-1)]:
            pivot=(.325,sign*.3)
            selected=[p for p in neutral if p.link_name in ('front_'+side+'_steer_link','front_'+side+'_wheel_link')]
            radius=max(math.dist(v,pivot) for p in selected for v in p.polygon_xy)+1e-5
            self.front_radii[side]=radius;parts.append(Point(pivot).buffer(radius/math.cos(math.pi/48),resolution=12))
        # ReferenceRobot independently bounds every converted 3D component by1m
        # about base, and checks wheel-spin skew <=1e-5m. Tilt sum bound .05rad.
        self.tilt_pad=2*math.sin(.05/2);self.spin_pad=1e-5
        hull=unary_union(parts).convex_hull.buffer((self.tilt_pad+self.spin_pad)/math.cos(math.pi/64),resolution=16)
        raw=np.asarray(hull.exterior.coords)[:-1];angles=np.arange(16)*2*math.pi/16
        directions=np.column_stack((np.cos(angles),np.sin(angles)));support=np.max(raw@directions.T,axis=0)
        self.vertices=np.asarray([np.linalg.solve(np.stack((directions[i],directions[(i+1)%16])),[support[i],support[(i+1)%16]]) for i in range(16)])
        if not Polygon(self.vertices).buffer(1e-12).covers(hull):raise ValueError('outward envelope check')
        self.radius=float(np.linalg.norm(self.vertices,axis=1).max());self.components=len(neutral)
        if self.components!=9:raise ValueError('actual nine-component reference required')
    def at(self,pose):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        return Polygon(self.vertices@np.array([[c,s],[-s,c]])+np.array([x,y]))

def curve(amplitude,count=901):
    rows=[];k=math.pi/9
    for x in np.linspace(-4.325,4.675,count):
        t=k*(x+4.325);sn,cs=math.sin(t),math.cos(t)
        y=amplitude*sn**4;yp=4*amplitude*k*sn**3*cs;ypp=4*amplitude*k*k*(3*sn*sn*cs*cs-sn**4)
        yaw=math.atan(yp);curvature=ypp/(1+yp*yp)**1.5
        rows.append([float(x),y,float(x+.325*math.cos(yaw)),y+.325*math.sin(yaw),yaw,curvature])
    return rows

def witness(envelope,static):
    union=unary_union([p for _,p in static]);attempts=[]
    for amplitude in AMPLITUDES:
        a=abs(amplitude);k=math.pi/9;yp=4*a*k;angle=4*a*k*k;kp=40*a*k**3+3*yp*angle**2
        point_bound=math.sqrt(1+yp*yp)+(envelope.radius+.325)*angle
        step=.01;pad=point_bound*step/2
        rows=curve(amplitude);minimum=None;intersection=False
        for row in rows:
            shape=envelope.at(row[2:5])
            if not union.is_empty:
                gap=shape.distance(union)
                if shape.intersects(union):intersection=True;gap=0.
                minimum=gap if minimum is None else min(minimum,gap)
                if gap<=.33+pad:break # finite profile rejected, not a proof no path exists
        lower=None if minimum is None else minimum-pad
        delta_bound=math.atan(.65*angle)
        # d(delta)/ds <= .65*|kappa'|; choose slow witness independently of outcomes.
        speed=.4 if kp==0 else min(.4,.70/(.65*kp))
        record={'amplitude_m':amplitude,'sampled_physical_lower_m':minimum,'continuous_physical_lower_m':lower,
            'continuous_conservative_lower_m':None if lower is None else lower-.33,'point_lipschitz_per_rear_x':point_bound,
            'curvature_abs_bound_m_inv':angle,'central_steering_abs_bound_rad':delta_bound,
            'central_steering_limit_rad':.4964,'witness_rear_speed_mps':speed,'central_steering_rate_bound_radps':.65*kp*speed,
            'steering_rate_limit_radps':.75,'sample_count_checked':len(rows) if minimum is None or lower>.33 else rows.index(row)+1,
            'positive_static_margin':not intersection and (lower is None or lower>.33),'passed':delta_bound<=.4964 and not intersection and (lower is None or lower>.33)}
        attempts.append(record)
        if record['passed']:return {'passed':True,'selected':record,'profile_attempts':attempts,'rows':rows,
            'scope':'ideal planar static rear-bicycle existence witness; no dynamics, tracking, elapsed-cap or actor avoidance qualification'}
    return {'passed':False,'profile_attempts':attempts,'scope':'finite witness family unsuccessful, not geometry infeasibility proof'}

def full_geometry_checks(scene,robot,envelope):
    static=v2.static_shapes(scene);union=unary_union([p for _,p in static]);initial={};actors=[]
    for i,a in enumerate(scene['actor_geometry']):
        yaw=math.atan2(a['end'][1]-a['start'][1],a['end'][0]-a['start'][0]);actors.append((i,Polygon(v1.vertices(*a['start'],yaw,.72,.72))))
    for name,x in [('spawn',-5.4),('gate',-4.),('goal',5.)]:
        parts=robot.project(Pose3.from_xyz_rpy((x,0,0),(0,0,0)));polys=[Polygon(p.polygon_xy) for p in parts]
        gaps=[p.distance(union) for p in polys] if not union.is_empty else []
        overlaps=any(p.intersects(union) for p in polys) if not union.is_empty else False
        initial[name]={'actual_component_count':len(parts),'static_intersection':overlaps,'minimum_physical_static_m':min(gaps,default=None),
            'minimum_conservative_static_m':min(gaps)-.33 if gaps else None,'initial_actor_intersection':any(p.intersects(a) for p in polys for _,a in actors)}
    actor_pairs=[{'left':i,'right':j,'intersects':a.intersects(b),'physical_distance_m':a.distance(b)} for n,(i,a) in enumerate(actors) for j,b in actors[n+1:]]
    sem=v2.geometry_semantics(scene)
    visibility=[{'actor_index':i,'at_gate':v2.laser_visibility((-4,0,0),a,static)} for i,a in enumerate(scene['actor_geometry'])]
    if scene['family']=='occluded_emergence':
        a=scene['actor_geometry'][0];pose=(a['conflict'][0]+.6,0,0);gap=envelope.at(pose).distance(union)
        sem['reveal_full_envelope_physical_gap_m']=gap;sem['reveal_full_envelope_pass']=gap>.33
    passed=all(not r['static_intersection'] and not r['initial_actor_intersection'] and (r['minimum_physical_static_m'] is None or r['minimum_physical_static_m']>.33) for r in initial.values()) and not any(p['intersects'] for p in actor_pairs) and sem['actor_swept_static_pass']
    if scene['family']=='occluded_emergence':passed=passed and sem['initial_low_visibility_pass'] and sem['geometric_reveal_pass'] and sem['reveal_full_envelope_pass']
    return {'passed':passed,'initial_full9_geometry':initial,'initial_actor_pairs':actor_pairs,'semantics':sem,'laser_at_gate':visibility}

def parameter_witnesses(scene):
    result=[]
    for i,a in enumerate(scene['actor_geometry']):
        rng=random.Random(VERSION+'|nominal-parameter-witness|'+v1.parent_id(scene)+'|'+str(i));selected=None
        for _ in range(2000):
            p=v1.actor_parameters('development',rng);arrival=p['gate_delay_s']+math.dist(a['start'],a['conflict'])/p['base_speed_mps'];ego=(a['conflict'][0]+4)/.8
            if abs(arrival-ego)<=2.:selected=dict(p,actor_index=i,nominal_arrival_difference_s=arrival-ego);break
        if selected is None:return None
        result.append(selected)
    return result

def build(output,audit,per_family=4):
    output,audit=Path(output),Path(audit)
    if output.exists():raise FileExistsError('D04 output is immutable')
    if per_family!=4:raise ValueError('this design fixes four new parents per family')
    audit.mkdir(parents=True,exist_ok=True);output.mkdir(parents=True)
    priors,records,bindings,derivations=load_priors();guard=v2.SeedGuard(records)
    urdf=REFERENCE/'robot_description.xml';sdf=REFERENCE/'conversion/conversion.sdf';robot=ReferenceRobot(urdf.read_text(),sdf.read_text());envelope=WitnessEnvelope(robot)
    write(audit/'input_design.json',{'schema':VERSION,'families':v1.FAMILIES,'parents_per_family':4,'topologies':TOPOLOGIES,'witness_amplitudes':AMPLITUDES,'maximum_draws_per_slot':2000,'new_structural_separation_cells_minimum':2.,'margin_m':.33,'execution_seed_allocation':False,'source_bindings':[bind(__file__),bind(Path(v2.__file__)),bind(Path(v1.__file__)),bind(STAGE/'contracts/splits.yaml'),bind(STAGE/'contracts/acceptance.yaml'),bind(urdf),bind(sdf)],'source_outcomes_used':False})
    layouts=[];rejects=[];counterexamples=[]
    for family in v1.FAMILIES:
        for index in range(4):
            topology=TOPOLOGIES[family][index%len(TOPOLOGIES[family])];accepted=False
            for draw in range(2000):
                seed=guard.named('geometry',VERSION,family,index,draw);scene=v2.sample_geometry(family,topology,random.Random(seed));pid=v1.parent_id(scene)
                reason=None;details=None
                if v1.group_split(pid)!='development':reason='nondevelopment_parent_hash'
                elif any(v1.near_duplicate(scene,old) or ((d:=separation(scene,old)) is not None and d<2.) for old in priors+layouts):reason='prior_or_new_structural_near_duplicate'
                elif not v1.initial_geometry_qc(scene)['horizontal_initial_geometry_pass']:reason='legacy_initial_collision'
                elif family=='occluded_emergence' and not .5<=scene['sampled_screen_conflict_distance_m']<=1.4:reason='screen_conflict_distance_outside_frozen_range'
                else:
                    qc=full_geometry_checks(scene,robot,envelope)
                    if not qc['passed']:reason='full_geometry_or_ray_semantics';details=qc
                    else:
                        params=parameter_witnesses(scene)
                        if params is None:reason='no_development_parameter_cell_nominal_arrival_witness'
                        else:
                            route=witness(envelope,v2.static_shapes(scene))
                            if not route['passed']:reason='finite_static_reference_witness_failed';details=route
                if reason:
                    rejects.append({'family':family,'slot':index,'draw':draw,'geometry_rng_seed':seed,'parent_geometry_id':pid,'reason':reason})
                    if reason not in ('nondevelopment_parent_hash','prior_or_new_structural_near_duplicate') and sum(c['reason']==reason for c in counterexamples)<4:
                        counterexamples.append({'reason':reason,'geometry':scene,'details':details,'draw':draw,'geometry_rng_seed':seed})
                    continue
                layout_id=f'd04_development_{family}_{index:03d}';folder=output/layout_id;folder.mkdir()
                scene.update(schema=VERSION,layout_id=layout_id,parent_geometry_id=pid,canonical_geometry_sha256=v1.digest(v1.canonical_geometry(scene,False)),split='development',geometry_rng_seed=seed,execution_seed=None,execution_authorized=False,actor_parameter_existence_witnesses=params)
                write(folder/'geometry_manifest.json',scene)
                witness_rows=route.pop('rows')
                with (folder/'reference_witness.csv').open('w',newline='') as f:
                    w=csv.writer(f);w.writerow(['rear_x','rear_y','base_x','base_y','base_yaw','curvature']);w.writerows(witness_rows)
                write(folder/'geometry_checks.json',qc);write(folder/'reference_witness.json',route);layouts.append(scene);accepted=True
                print(json.dumps({'accepted':layout_id,'draw':draw,'parent':pid,'amplitude':route['selected']['amplitude_m']}),flush=True);break
            if not accepted:
                write(audit/'retained_generation_failure.json',{'family':family,'slot':index,'draws':2000,'accepted_parents':len(layouts),'no_threshold_relaxation':True});write(audit/'rejected_draws.json',rejects);write(audit/'counterexamples.json',counterexamples)
                raise RuntimeError('bounded D04 sampler exhausted '+family)
    issues=v1.audit_geometry_registry(priors+layouts)
    if issues:raise ValueError('final combined geometry/split audit failed')
    write(output/'layouts.json',layouts);write(audit/'rejected_draws.json',rejects);write(audit/'counterexamples.json',counterexamples)
    newrecords={k:v for k,v in guard.records.items() if k not in records}
    if any(v['purpose']!='geometry' for v in newrecords.values()):raise AssertionError('execution stream allocated')
    write(audit/'geometry_rng_draws.json',{'new_geometry_only_records':newrecords,'collision_log':guard.collision_log,'global_registry_written':False,'execution_seeds_allocated':0})
    # Save topology/contract limitations as actual deterministic counterexamples.
    oc=v2.sample_geometry('occluded_emergence','L_corner_screen',random.Random(17));seq=v2.sample_geometry('sequential_interaction','crossing_then_oncoming',random.Random(17))
    write(audit/'inherited_generator_limitations.json',{'occlusion_requested_L_corner_actual':oc['topology'],'occlusion_actual_geometry':oc,'oncoming_secondary_lateral_m':seq['actor_geometry'][1]['conflict'][1],'oncoming_secondary_geometry':seq,'D04_coverage_exclusions':['occlusion L_corner/staggered silently collapse to single_screen in v2','crossing_then_oncoming fixed1.25m lateral path lacks demonstrated second conflict; not included here','static staggered offset1.15m differs generic central-obstacle .55m range; only one_block_offset included'], 'these_are_input_generator_limitations_not_simulator_failures':True})
    separations=[{'left':a['parent_geometry_id'],'right':b['parent_geometry_id'],'max_structural_cell_difference':separation(a,b)} for i,a in enumerate(layouts) for b in priors+layouts[:i] if a['family']==b['family']]
    write(audit/'dedup_split_audit.json',{'prior_bindings':bindings,'prior_unique_parents':len(priors),'N04_N05_derivation_checks':derivations,'new_parents':len(layouts),'new_per_family':dict(Counter(s['family'] for s in layouts)),'combined_registry_issues':issues,'structural_separations':separations,'all_new_geometry_groups_development':all(v1.group_split(s['parent_geometry_id'])=='development' for s in layouts),'actor_parameter_witness_cells_development':all(v1.group_split(p['parameter_cell_id'])=='development' for s in layouts for p in s['actor_parameter_existence_witnesses']),'all_rigid_transforms_reflections_cosmetics_must_inherit_parent':True,'train_validation_final_geometry_or_outcomes_loaded':False,'future_registry_obligation':'check all proposed train/validation/final parents against combined A01/A02/D04 canonical and near-duplicate groups before allocation; no episode-row random split','hash_implementation_scope':'inherits allocation_v1 parent hash and independently restricted actor-cell hash; frozen contract literal joint(parent,actorcell) hash is not implemented; grouping is conservatively enforced by both belonging to development, no final qualification claim'})
    write(audit/'envelope_reference.json',{'source_urdf':bind(urdf),'converted_sdf':bind(sdf),'component_count':envelope.components,'vertices':envelope.vertices.tolist(),'front_rotating_envelope_radii':envelope.front_radii,'tilt_sum_limit_rad':.05,'tilt_pad_m':envelope.tilt_pad,'wheel_spin_pad_m':envelope.spin_pad,'base_radius_m':envelope.radius,'shape_radius_audit':robot.radius_audit,'map_export_or_rasterization_included':False})
    resources=[bind(p) for p in sorted(output.rglob('*')) if p.is_file()]
    write(audit/'generation_report.json',{'status':'24_new_development_geometry_parents_offline_checks_pass_independent_review_pending','generator':bind(__file__),'new_parents':len(layouts),'per_family':dict(Counter(s['family'] for s in layouts)),'rejection_reasons':dict(Counter(r['reason'] for r in rejects)),'all_actual9_initial_static_checks_pass':True,'all_actor_swept_static_checks_pass':True,'all_occlusion_gate_reveal_checks_pass':True,'all_static_reference_witnesses_pass':True,'safety_margin_m':.33,'execution_seeds_allocated':0,'global_seed_registry_modified':False,'ROS_started':False,'new_final_outcomes_read':False,'method_outcomes_used_for_selection':False,'source_resources':resources,'remaining':['independent review','full actor temporal and reactive semantics','shared raster map/guide derivation','physical rear/reference tracking','new episode RNG/identity release','complete24parentsperfamily development and heldout qualification remain unmet'],'independent_review':False,'runtime_ready':False,'live_evidence':False})

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--audit',type=Path,required=True);args=p.parse_args();build(args.output,args.audit)
