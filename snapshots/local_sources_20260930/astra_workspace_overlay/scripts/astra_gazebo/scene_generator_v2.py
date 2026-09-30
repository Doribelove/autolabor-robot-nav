#!/usr/bin/env python3
"""New diagnostic geometry after the retained A01 semantic audit.

No simulator launch and no final set generation.  Visibility is checked with
the actual 720-beam planar scan angles, not just a centre sightline.
"""
import argparse
import copy
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET
import yaml
from shapely.geometry import Polygon,LineString,Point,MultiPoint

HERE=Path(__file__).parent
spec=importlib.util.spec_from_file_location('astra_scene_v1_readonly',HERE/'scene_generator.py')
v1=importlib.util.module_from_spec(spec);spec.loader.exec_module(v1)
STAGE=v1.STAGE
VERSION='astra_scene_generator_v2'


class SeedGuard(v1.SeedGuard):
    """Versioned first-free linear allocation; occupied identities never change.

    Every rejected hash slot is retained in collision_log. Call order is the
    frozen family/index/draw order; named reaccess is independent of call order.
    This allocates fresh inputs, never retries a simulator episode.
    """
    allocation_version='sha256_first16_mod_range_then_first_free_linear_v1'
    def __init__(self, records=None):
        super().__init__(records)
        self.logical_to_seed={}
        self.collision_log=[]
        for seed,record in self.records.items():
            identity=(record['purpose'],record['key'])
            if identity in self.logical_to_seed and self.logical_to_seed[identity]!=int(seed):
                raise ValueError('prior logical identity has multiple seeds')
            self.logical_to_seed[identity]=int(seed)

    def named(self,purpose,*key):
        logical=v1.canonical_json([VERSION,purpose,*key]);lo,hi=v1.SEED_RANGES[purpose]
        identity=(purpose,logical)
        if identity in self.logical_to_seed:return self.logical_to_seed[identity]
        initial=lo+int(hashlib.sha256(logical.encode()).hexdigest()[:16],16)%(hi-lo+1)
        for displacement in range(hi-lo+1):
            seed=lo+(initial-lo+displacement)%(hi-lo+1)
            occupied=self.records.get(str(seed))
            if occupied is not None:
                self.collision_log.append({'logical_key':logical,'purpose':purpose,
                    'initial_candidate':initial,'occupied_seed':seed,
                    'displacement':displacement,'occupied_record':copy.deepcopy(occupied)})
                continue
            self.register(seed,logical,purpose);self.logical_to_seed[identity]=seed
            return seed
        raise ValueError('fresh seed namespace exhausted: '+purpose)


def polygon(box):
    return Polygon(v1.vertices(box['x'],box['y'],box.get('yaw_rad',0.),box['length'],box['width']))


def static_shapes(scene):
    items=[(b['name'],polygon(b)) for b in scene['static_boxes']]
    h=scene['corridor_half_width_m']
    if h is not None:
        items += [('wall_'+str(sign),Polygon(v1.vertices(0,sign*(h+.1),0,18,.2))) for sign in (-1,1)]
    return items


def swept_actor(actor):
    yaw=math.atan2(actor['end'][1]-actor['start'][1],actor['end'][0]-actor['start'][0])
    return MultiPoint(v1.vertices(*actor['start'],yaw,.72,.72)+v1.vertices(*actor['end'],yaw,.72,.72)).convex_hull


def laser_visibility(observer,actor,static,beams=720):
    """Full scan ray intersection at height .60; static boxes exceed this height."""
    x,y,yaw=observer;origin=Point(x,y)
    ayaw=math.atan2(actor['end'][1]-actor['start'][1],actor['end'][0]-actor['start'][0])
    target=Polygon(v1.vertices(*actor['start'],ayaw,.72,.72))
    actor_beams=visible=0;blocked_by={};visible_indices=[]
    for i in range(beams):
        angle=yaw-math.pi+i*2*math.pi/(beams-1)
        ray=LineString([(x,y),(x+30*math.cos(angle),y+30*math.sin(angle))])
        hit=ray.intersection(target)
        if hit.is_empty:continue
        actor_distance=origin.distance(hit)
        if actor_distance<.1:continue
        actor_beams+=1;blocked=[]
        for name,poly in static:
            sh=ray.intersection(poly)
            if not sh.is_empty and origin.distance(sh)<actor_distance-1e-8:blocked.append(name)
        if not blocked:visible+=1;visible_indices.append(i)
        for name in blocked:blocked_by[name]=blocked_by.get(name,0)+1
    return {'beams':beams,'beam_min_angle_rad':-math.pi,'beam_max_angle_rad':math.pi,
            'potential_actor_beams':actor_beams,'visible_actor_beams':visible,
            'visible_fraction':visible/actor_beams if actor_beams else None,
            'blocked_by':blocked_by,'visible_indices':visible_indices,
            'observer_pose':list(observer),'actor_position':list(actor['start'])}


def geometry_semantics(scene):
    static=static_shapes(scene);clearances=[]
    for i,a in enumerate(scene['actor_geometry']):
        sw=swept_actor(a)
        for name,shape in static:
            clearances.append({'actor_index':i,'static_name':name,'distance_m':sw.distance(shape),'intersection':sw.intersects(shape)})
    report={'minimum_actor_swept_static_clearance_m':min([r['distance_m'] for r in clearances] or [100.]),
            'actor_swept_static_pairs':clearances,'actor_swept_static_pass':all(not r['intersection'] and r['distance_m']>=.05 for r in clearances)}
    if scene['family']=='occluded_emergence':
        a=scene['actor_geometry'][0]
        initial=laser_visibility((-4.,0.,0.),a,static)
        reveal_pose=(a['conflict'][0]+.60,0.,0.)
        reveal=laser_visibility(reveal_pose,a,static)
        reveal_body=Polygon(v1.vertices(*reveal_pose,1.04,.7))
        reveal_clearance=min([reveal_body.distance(s) for _,s in static] or [100.])
        report.update(initial_laser_visibility=initial,revealed_laser_visibility=reveal,
                      reveal_static_physical_clearance_m=reveal_clearance,
                      initial_low_visibility_pass=initial['potential_actor_beams']>=5 and initial['visible_fraction']<=.2,
                      geometric_reveal_pass=reveal['potential_actor_beams']>=5 and reveal['visible_fraction']>=.8 and reveal_clearance>=.33,
                      reveal_scope='two static sensor poses; actual reactive actor/ego temporal emergence remains a live gate')
    return report


def sample_geometry(family,topology,rng):
    scene=v1._sample_geometry(family,topology,rng)
    if family=='occluded_emergence':
        # Screen is on the sensor sightline, spatially separated from the
        # entire actor path. Samples remain within the frozen shape ranges.
        cx=rng.uniform(-1.15,.3);side=rng.choice([-1,1]);dist=rng.uniform(1.6,2.25)
        actor=v1.crossing(cx,90.,side,dist)
        sx=rng.uniform(.85,1.25);sy=rng.uniform(.55,.95)
        length=rng.uniform(.8,min(1.4,2*(sx-.46)));width=rng.uniform(.55,.70)
        scene['actor_geometry']=[actor]
        scene['static_boxes']=[v1.box('occlusion_screen',cx-sx,side*sy,length,width)]
        # A distance range in the design is an input gate, never relaxed using
        # measured performance or simply because acceptance is difficult.
        scene['sampled_screen_conflict_distance_m']=math.hypot(sx,sy)
        scene['topology']='single_screen'
    elif family=='head_on_yield':
        for a in scene['actor_geometry']:
            side=1 if a['start'][1]>=0 else -1;y=side*rng.uniform(.75,1.0)
            for key in ('start','end','conflict'):a[key][1]=round(y,4)
    return scene


def sample_layout(family,index,guard,existing,cohort):
    topology=v1.TOPOLOGIES[family][index%3]
    for draw in range(10000):
        seed=guard.named('geometry',cohort,family,index,draw)
        candidate=sample_geometry(family,topology,random.Random(seed));pid=v1.parent_id(candidate)
        if v1.group_split(pid)!='development':continue
        if any(v1.near_duplicate(candidate,old) for old in existing):continue
        if not v1.initial_geometry_qc(candidate)['horizontal_initial_geometry_pass']:continue
        shapes=static_shapes(candidate)
        if any(polygon({'x':x,'y':0.,'length':1.04,'width':.70}).distance(shape)<.36 for x in (-5.4,-4.,5.) for _,shape in shapes):continue
        if family=='occluded_emergence' and not .5<=candidate['sampled_screen_conflict_distance_m']<=1.4:continue
        semantics=geometry_semantics(candidate)
        if not semantics['actor_swept_static_pass']:continue
        if family=='occluded_emergence' and not (semantics['initial_low_visibility_pass'] and semantics['geometric_reveal_pass']):continue
        candidate.update(layout_id=f'{cohort}_development_{family}_{index:03d}',parent_geometry_id=pid,
           canonical_geometry_sha256=v1.digest(v1.canonical_geometry(candidate,False)),split='development',geometry_seed=seed,
           initial_geometry_qc=v1.initial_geometry_qc(candidate),offline_semantics=semantics)
        return candidate
    raise RuntimeError('No admissible input geometry before finite sampling guard: '+family)


def episode(layout,replicate,guard):
    scene=v1.episode_scene(layout,replicate,guard)
    scene['generator_version']=VERSION;scene['generator_sha256']=v1.file_sha(__file__)
    scene['generator_v1_dependency_sha256']=v1.file_sha(HERE/'scene_generator.py')
    scene['robot_collision_reference']['spawn_xyz_rpy'][2]=0.
    scene['robot_collision_reference']['vertical_spawn_qualification']='z0 new common environment; live support/contact qualification pending'
    # Geometry-only nominal arrival constraint is independent of method outcomes.
    rng=random.Random(scene['rng_substreams']['actor_parameters'])
    for a in scene['actors']:
        d=math.dist(a['start'],a['conflict']);t_ego=max(0.,a['conflict'][0]+4.)/.8
        for _ in range(10000):
            params=v1.actor_parameters('development',rng)
            if abs(d/params['base_speed_mps']-t_ego)<=2.:
                a.update(params);break
        else:raise RuntimeError('No split-eligible nominal actor timing parameter for '+scene['layout_id'])
        a['nominal_arrival_difference_s']=d/a['base_speed_mps']-t_ego
    scene['environment_version']='astra_common_environment_v2_z0_source_stamped_physics_evidence'
    scene['readiness'].update(actor_swept_static_geometry=True,
        initial_lidar_occlusion=scene['family']!='occluded_emergence' or layout['offline_semantics']['initial_low_visibility_pass'],
        offline_geometric_reveal=scene['family']!='occluded_emergence' or layout['offline_semantics']['geometric_reveal_pass'])
    return scene


def render(scene):
    root=ET.fromstring(v1.render(scene));world=root.find('world')
    camera=world.find("model[@name='astra_evidence_camera']")
    camera.find('pose').text='0 0 19 0 1.5707963267948966 1.5707963267948966'
    scene['evidence_camera']['pose_xyz_rpy']=[0,0,19,0,math.pi/2,math.pi/2]
    plugin=ET.SubElement(world,'plugin',name='astra_evidence',filename='libastra_evidence_plugin.so')
    ET.SubElement(plugin,'state_hz').text='50'
    scene['physics_evidence']={'plugin':'libastra_evidence_plugin.so','world_state_hz':50,
      'model_topic':'/astra/world_models','link_topic':'/astra/world_links',
      'world_state_message_type':'gazebo_msgs/WorldState','source_stamp':'WorldUpdateEnd',
      'contact_topic':'/astra/physics_contacts','contact_rate':'every_physics_step_including_empty',
      'metadata_topic':'/astra/physics_metadata','twist_semantics':'Gazebo internal velocity; derive pose velocity separately for SetWorldPose models',
      'force_wrench_fields':'not_populated_do_not_interpret_as_measured_force',
      'live_topic_qualification':'pending'}
    return '<?xml version="1.0"?>\n'+ET.tostring(root,encoding='unicode')+'\n'


def build(output,prior_paths,layouts_per_family=1,replicates=2):
    output=Path(output)
    if output.exists():raise FileExistsError('No overwrite of generated evidence')
    priors=[];records={}
    for p in prior_paths:
        p=Path(p);priors.extend(json.loads(p.read_text()))
        old_report=json.loads((p.parent/'generation_report.json').read_text())
        for seed,value in old_report['rng_records'].items():
            if seed in records and records[seed]!=value:raise ValueError('prior seed collision')
            records[seed]=value
    if v1.audit_geometry_registry(priors):raise ValueError('prior geometry registry invalid')
    guard=SeedGuard(records);layouts=[];scenes=[]
    for family in v1.FAMILIES:
        for index in range(layouts_per_family):
            layout=sample_layout(family,index,guard,priors+layouts,output.name);layouts.append(layout)
            for rep in range(replicates):scenes.append(episode(layout,rep,guard))
    output.mkdir(parents=True);rows=[]
    for scene in scenes:
        folder=output/scene['layout_id']/f'replicate_{scene["replicate"]:02d}';folder.mkdir(parents=True)
        world=folder/'world.sdf';manifest=folder/'scene_manifest.yaml';world.write_text(render(scene));manifest.write_text(yaml.safe_dump(scene,sort_keys=False))
        rows.append({'layout_id':scene['layout_id'],'family':scene['family'],'topology':scene['topology'],
          'parent_geometry_id':scene['parent_geometry_id'],'geometry_sha256':scene['canonical_geometry_sha256'],
          'replicate':scene['replicate'],'seed':scene['seed'],'scene_id':scene['scene_id'],
          'world_path':str(world),'world_sha256':v1.file_sha(world),'manifest_path':str(manifest),
          'manifest_sha256':v1.file_sha(manifest),'eligibility':'new_geometry_qualification_pending_live_and_static_witness'})
    with (output/'diagnostic_blocks.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    report={'status':'v2_new_qualification_inputs_materialized_not_execution_release','execution_started':False,
       'generator_version':VERSION,'generator_sha256':v1.file_sha(__file__),'v1_dependency_sha256':v1.file_sha(HERE/'scene_generator.py'),
       'layouts':len(layouts),'blocks':len(scenes),'prior_layout_count':len(priors),
       'prior_bindings':[{'path':str(p),'sha256':v1.file_sha(p)} for p in prior_paths],
       'geometry_registry_issues':v1.audit_geometry_registry(priors+layouts),
       'actor_swept_static_all_pass':all(l['offline_semantics']['actor_swept_static_pass'] for l in layouts),
       'occlusion_details':[l['offline_semantics'] for l in layouts if l['family']=='occluded_emergence'],
       'rng_records':guard.records,'new_rng_seed_count':len(guard.records)-len(records),
       'seed_allocation_version':guard.allocation_version,'seed_allocation_collision_log':guard.collision_log,
       'episode_seeds':[s['seed'] for s in scenes],'actor_base_speeds':sorted({round(a['base_speed_mps'],4) for s in scenes for a in s['actors']}),
       'environment_version':'astra_common_environment_v2_z0_source_stamped_physics_evidence',
       'pending':['static Ackermann witness on each new layout','live world-state/contact/source-stamp qualification','all four wheels/body support at z0','gate-relative actor delay plugin','named sensor/actuator RNG wiring','live occlusion/emergence along measured ego and actor trajectories','route/world bridge live integration','complete contracted development population'],
       'final_set_generated':False}
    (output/'generation_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (output/'layouts.json').write_text(json.dumps(layouts,indent=2,allow_nan=False)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path)
    p.add_argument('--prior-layouts',required=True,nargs='+',type=Path)
    p.add_argument('--layouts-per-family',type=int,default=1);p.add_argument('--replicates',type=int,default=2)
    a=p.parse_args();r=build(a.output,a.prior_layouts,a.layouts_per_family,a.replicates)
    print(json.dumps({k:r[k] for k in ['status','layouts','blocks','prior_layout_count','geometry_registry_issues','actor_swept_static_all_pass','actor_base_speeds']},indent=2))
