#!/usr/bin/env python3
"""Input-only, versioned Astra scene generation; never launches an experiment.

The initial implementation supports 9 m, yaw-zero route-frame scenes only.
Rigid world transformations, formal release, navigation feasibility and physics
qualification are deliberately explicit pending gates, not inferred successes.
"""
from __future__ import annotations
import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET
import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
STAGE = WORKSPACE.parent
VERSION = 'astra_scene_generator_v1'
FAMILIES = ('static_avoidance','single_crossing','head_on_yield',
            'sequential_interaction','occluded_emergence','width_change_corridor')
LEGACY_SCENE_IDS = dict(zip(FAMILIES, ('core_static_avoidance','core_single_crossing',
    'core_head_on_yield','core_sequential_interaction','core_partially_occluded','core_width_change')))
TOPOLOGIES = {
 'static_avoidance': ('one_block_offset','two_staggered_blocks','asymmetric_passage_with_side_bay'),
 'single_crossing': ('near_mid_crossing','oblique_crossing','crossing_with_offset_approach'),
 'head_on_yield': ('symmetric_corridor','left_passing_bay','right_passing_bay'),
 'sequential_interaction': ('alternating_crossings','same_side_crossings','crossing_then_oncoming'),
 'occluded_emergence': ('single_screen','L_corner_screen','staggered_screens'),
 'width_change_corridor': ('symmetric_throat','offset_throat','two_stage_throat')}
SEED_RANGES = {'geometry': (260930000,260999999),
               'train': (261000000,262999999),'development': (263000000,263999999),
               'validation': (264000000,264999999),'final_test': (265000000,265999999),
               'stress_ood': (266000000,266999999),'visualization_only': (267000000,267999999)}


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical_json(value).encode()).hexdigest()


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class SeedGuard:
    def __init__(self, records=None):
        self.records = dict(records or {})

    def register(self, seed, key, purpose):
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise ValueError('seed must be an integer')
        lo,hi = SEED_RANGES[purpose]
        if not lo <= seed <= hi:
            raise ValueError('seed outside fresh purpose range: '+purpose)
        previous = self.records.get(str(seed))
        if previous is not None and previous != {'key':key,'purpose':purpose}:
            raise ValueError('seed collision: '+str(seed))
        self.records[str(seed)] = {'key':key,'purpose':purpose}
        return seed

    def named(self, purpose, *key):
        logical = canonical_json([VERSION,purpose,*key])
        lo,hi = SEED_RANGES[purpose]
        # A hash collision fails closed; it is never hidden by silent reseeding.
        seed = lo + int(hashlib.sha256(logical.encode()).hexdigest()[:16],16) % (hi-lo+1)
        return self.register(seed,logical,purpose)


def group_split(parent_id):
    # Parent geometry and all its parameter-cell variants stay in one split.
    value = int(digest(['astra_gazebo_20260926',parent_id,'allocation_v1'])[:16],16) % 100
    return ('train' if value < 60 else 'development' if value < 80 else
            'validation' if value < 90 else 'final_test')


def q(value, step=.2):
    return int(math.floor(float(value)/step+.5))


def box(name,x,y,length,width,height=1.2):
    return {'name':name,'x':round(x,4),'y':round(y,4),'length':round(length,4),
            'width':round(width,4),'height':height,'yaw_rad':0.0}


def crossing(x,angle_deg=90.0,side=-1,distance=3.6):
    angle=math.radians(angle_deg);dx,dy=math.cos(angle),math.sin(angle)
    if side>0:dx,dy=-dx,-dy
    return {'start':[round(x-distance*dx,4),round(-distance*dy,4)],
            'end':[round(x+4.5*dx,4),round(4.5*dy,4)],
            'conflict':[round(x,4),0.0]}


def _sample_geometry(family, topology, rng):
    half=None;boxes=[];actors=[]
    if family=='static_avoidance':
        half=rng.uniform(2.55,3.10)
        x=rng.uniform(-1.5,.4);w=rng.uniform(.5,1.05)
        boxes=[box('block_1',x,rng.uniform(-.45,.45),rng.uniform(.55,1.25),w)]
        if topology=='two_staggered_blocks':
            boxes.append(box('block_2',x+2.3,rng.choice([-1,1])*1.15,rng.uniform(.5,.9),.55))
        elif topology=='asymmetric_passage_with_side_bay':
            boxes.append(box('side_indentation',1.9,half-.4,1.3,.8))
    elif family=='single_crossing':
        x=rng.uniform(-1.5,2.3)
        angle=90.0 if topology=='near_mid_crossing' else rng.choice([65.,75.,105.,115.])
        actors=[crossing(x,angle,rng.choice([-1,1]),rng.uniform(2.8,4.3))]
        if topology=='crossing_with_offset_approach':
            boxes=[box('approach_edge',x-1.6,rng.choice([-1,1])*2.4,1.0,.45)]
    elif family=='head_on_yield':
        half=rng.uniform(1.85,2.7);side=1 if rng.random()>.5 else -1
        actors=[{'start':[round(rng.uniform(3.6,4.4),4),round(side*rng.uniform(.65,1.4),4)],
                 'end':[-5.5,round(side*rng.uniform(.65,1.4),4)],'conflict':[0.,0.]}]
        actors[0]['end'][1]=actors[0]['start'][1]
        actors[0]['conflict'][1]=actors[0]['start'][1]
        if topology!='symmetric_corridor':
            bay_side=1 if topology=='left_passing_bay' else -1
            # Piecewise wall leaves one 2.4 m passing bay; the other wall is full.
            boxes=[box('opposite_wall',0.,-bay_side*(half+.1),16.,.2,1.),
                   box('bay_entry_wall',-3.5,bay_side*(half+.1),5.,.2,1.),
                   box('bay_exit_wall',3.7,bay_side*(half+.1),4.6,.2,1.),
                   box('bay_back_wall',.25,bay_side*(half+.85),2.5,.2,1.)]
            half=None
    elif family=='sequential_interaction':
        first=rng.uniform(-1.6,-.9);second=first+rng.uniform(1.9,3.3)
        actors=[crossing(first,rng.choice([75.,90.,105.]),-1,rng.uniform(3.2,4.3))]
        if topology=='crossing_then_oncoming':
            actors.append({'start':[4.3,1.25],'end':[-4.8,1.25],'conflict':[second,1.25]})
        else:
            actors.append(crossing(second,rng.choice([75.,90.,105.]),
                                   1 if topology=='alternating_crossings' else -1,rng.uniform(3.2,4.3)))
    elif family=='occluded_emergence':
        x=rng.uniform(-1.35,1.75);side=rng.choice([-1,1]);width=rng.uniform(.35,.65)
        actors=[crossing(x,rng.choice([75.,90.,105.]),side,rng.uniform(3.5,4.4))]
        boxes=[box('screen_1',x-.1,side*rng.uniform(1.45,1.8),rng.uniform(.7,1.45),width)]
        if topology=='L_corner_screen':
            boxes.append(box('screen_2',x+.6,side*2.25,.35,.8))
        elif topology=='staggered_screens':
            boxes.append(box('screen_2',x+1.65,-side*1.9,.85,.4))
    elif family=='width_change_corridor':
        wide=rng.uniform(2.05,2.8);narrow=rng.uniform(1.0,1.55)
        shift=0.0 if topology=='symmetric_throat' else rng.uniform(-.45,.45)
        length=rng.uniform(1.4,3.2);center=rng.uniform(-.1,1.4)
        for sign in (-1,1):
            boxes.extend([box('entry_'+str(sign),-3.5,sign*(wide+.1),3.,.2,1.),
                          box('throat_'+str(sign),center,shift+sign*(narrow+.1),length,.2,1.),
                          box('exit_'+str(sign),4.5,sign*(wide+.1),2.5,.2,1.)])
            if topology=='two_stage_throat':
                boxes.append(box('second_throat_'+str(sign),-1.9,-shift+sign*(narrow+.3),.8,.2,1.))
    else:raise ValueError('unknown family')
    return {'family':family,'topology':topology,'corridor_half_width_m':half,
            'static_boxes':boxes,'static_circles':[],'actor_geometry':actors,
            'route':{'start':[-4.,0.,0.],'goal':[5.,0.,0.], 'spawn':[-5.4,0.,0.],
                     'length_m':9.,'frame':'world','transform_implemented':'yaw_zero_only'}}


def canonical_geometry(scene, quantized=True):
    """Ignore labels, seeds, policy and cosmetic transforms; group reflections."""
    def number(v,step):return q(v,step) if quantized else round(float(v),4)
    options=[]
    for sign in (1,-1):
        boxes=sorted([[number(b['x'],.2),number(sign*b['y'],.2),number(b['length'],.1),
                       number(b['width'],.1),number(b['height'],.1)] for b in scene['static_boxes']])
        actors=sorted([[number(a[k][0],.2),number(sign*a[k][1],.2)]
                       for a in scene['actor_geometry'] for k in ('start','end','conflict')])
        options.append({'family':scene['family'],'wall_half_width':None if scene['corridor_half_width_m'] is None else number(scene['corridor_half_width_m'],.2),
                        'boxes':boxes,'actor_path_points':actors,'route_length_m':9.0})
    return min(options,key=canonical_json)


def parent_id(scene):
    return 'geom_'+digest(canonical_geometry(scene))[:24]


def _flatten_numbers(value):
    if isinstance(value,dict):return sum((_flatten_numbers(value[k]) for k in sorted(value)),[])
    if isinstance(value,list):return sum((_flatten_numbers(v) for v in value),[])
    if isinstance(value,(int,float)):return [value]
    return []


def near_duplicate(left,right):
    a,b=canonical_geometry(left,False),canonical_geometry(right,False)
    if a['family']!=b['family'] or len(a['boxes'])!=len(b['boxes']) or len(a['actor_path_points'])!=len(b['actor_path_points']):return False
    if (a['wall_half_width'] is None)!=(b['wall_half_width'] is None):return False
    # Exact declared cells: positions .20 m, dimensions .10 m.
    if a['wall_half_width'] is not None and abs(a['wall_half_width']-b['wall_half_width'])>=.2:return False
    for aa,bb in zip(a['boxes'],b['boxes']):
        if any(abs(x-y)>=s for x,y,s in zip(aa,bb,[.2,.2,.1,.1,.1])):return False
    for aa,bb in zip(a['actor_path_points'],b['actor_path_points']):
        if any(abs(x-y)>=.2 for x,y in zip(aa,bb)):return False
    return True


def vertices(x,y,yaw,length,width):
    c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*a-s*b,y+s*a+c*b) for a,b in
            [(-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2)]]


def intersects(a,b):
    for polygon in (a,b):
        for p,n in zip(polygon,polygon[1:]+polygon[:1]):
            axis=(-(n[1]-p[1]),n[0]-p[0]);ap=[x*axis[0]+y*axis[1] for x,y in a];bp=[x*axis[0]+y*axis[1] for x,y in b]
            if max(ap)<min(bp)-1e-9 or max(bp)<min(ap)-1e-9:return False
    return True


def initial_geometry_qc(scene):
    static=[(b['name'],vertices(b['x'],b['y'],b.get('yaw_rad',0.),b['length'],b['width'])) for b in scene['static_boxes']]
    half=scene['corridor_half_width_m']
    if half is not None:
        static.extend([(f'wall_{sign}',vertices(0,sign*(half+.1),0,18,.2)) for sign in (-1,1)])
    moving=[('autolabor_m2',vertices(-5.4,0,0,1.04,.70))]
    for i,a in enumerate(scene['actor_geometry']):
        yaw=math.atan2(a['end'][1]-a['start'][1],a['end'][0]-a['start'][0])
        moving.append((f'benchmark_actor_{i+1}',vertices(*a['start'],yaw,.72,.72)))
    bad=[]
    for i,(name,p) in enumerate(moving):
        for other,other_p in static+moving[i+1:]:
            if intersects(p,other_p):bad.append([name,other])
    return {'illegal_initial_intersections':bad,'horizontal_initial_geometry_pass':not bad,
            'robot_instances':1,'actor_count':len(scene['actor_geometry']),
            'robot_initial_centered_wheel_envelope':'within_1.04_by_0.70_chassis_projection',
            'wheel_ground_and_vertical_collision_qc':'pending_live_physics',
            'navigation_feasibility':'pending_independent_Ackermann_path_qualification',
            'occlusion_and_conflict_semantic_qualification':'pending_geometric_ray_and_timing_audit'}


def actor_parameters(split,rng):
    for _ in range(10000):
        cell={'policy':rng.choice(['cautious','hesitant','assertive']),
              'speed_cell':rng.randrange(13,23),'response_distance_cell':rng.randrange(15,26),
              'margin_cell':rng.randrange(9,18),'delay_cell':rng.randrange(0,6)}
        cell_id='actor_'+digest(cell)[:24]
        if group_split(cell_id)==split:
            return dict(policy=cell['policy'],base_speed_mps=.05*cell['speed_cell'],
                        response_distance_m=.2*cell['response_distance_cell'],
                        response_time_margin_s=.1*cell['margin_cell'],
                        gate_delay_s=.2*cell['delay_cell'],parameter_cell_id=cell_id)
    raise RuntimeError('actor parameter split rejection budget exhausted')


def audit_geometry_registry(layouts):
    """Audit all supplied previous splits, not just the new episode table."""
    problems=[]
    for i,scene in enumerate(layouts):
        if scene.get('parent_geometry_id')!=parent_id(scene):
            problems.append({'type':'parent_id_mismatch','layout_id':scene.get('layout_id')})
        if scene.get('split')!=group_split(parent_id(scene)):
            problems.append({'type':'split_assignment_mismatch','layout_id':scene.get('layout_id')})
        for old in layouts[:i]:
            if near_duplicate(scene,old):
                problems.append({'type':'near_duplicate_cross_split' if old['split']!=scene['split'] else 'near_duplicate',
                                 'left':old.get('layout_id'),'right':scene.get('layout_id')})
    return problems


def sample_layout(family,index,split,guard,existing,cohort='sampling_v1'):
    topology=TOPOLOGIES[family][index%len(TOPOLOGIES[family])]
    for draw in range(10000):
        # Candidate identities are input-derived; rejection never sees outcomes.
        seed=guard.named('geometry',cohort,family,index,draw)
        candidate=_sample_geometry(family,topology,random.Random(seed))
        pid=parent_id(candidate)
        if group_split(pid)!=split:continue
        if any(near_duplicate(candidate,s) for s in existing):continue
        qc=initial_geometry_qc(candidate)
        if not qc['horizontal_initial_geometry_pass']:continue
        candidate.update(layout_id=f'{cohort}_{split}_{family}_{index:03d}',parent_geometry_id=pid,
                         canonical_geometry_sha256=digest(canonical_geometry(candidate,False)),
                         split=split,geometry_seed=seed,initial_geometry_qc=qc)
        return candidate
    raise RuntimeError('geometry generation exhausted admissible input draws')


def episode_scene(layout,replicate,guard):
    split=layout['split'];lid=layout['layout_id']
    seed=guard.named(split,lid,replicate,'gazebo')
    streams={name:guard.named(split,lid,replicate,name) for name in
             ['actor_parameters','actor_response','sensor_noise','actuator']}
    rng=random.Random(streams['actor_parameters']);actors=[]
    for i,geometry in enumerate(layout['actor_geometry']):
        params=actor_parameters(split,rng);yaw=math.atan2(geometry['end'][1]-geometry['start'][1],geometry['end'][0]-geometry['start'][0])
        actors.append(dict(copy.deepcopy(geometry),**params,model_name=f'benchmark_actor_{i+1}',
           radius_m=.36,collision_shape='box',collision_length_m=.72,collision_width_m=.72,
           collision_height_m=1.0,initial_pose_xyz_rpy=[*geometry['start'],.5,0.,0.,yaw],
           response_seed=guard.named(split,lid,replicate,'actor_response',i),
           initial_gate_delay_supported_by_legacy_plugin=False))
    return dict(copy.deepcopy(layout),schema_version='astra_scene_1.0',scene_id=LEGACY_SCENE_IDS[layout['family']],
       seed=seed,replicate=replicate,actors=actors,rng_substreams=streams,
       actor_policy_input_to_planners=False,gazebo_model_states_evaluator_only=True,
       episode_cap_s=22.,robot_collision_reference={'link':'chassis_link','center_xyz_m':[0.,0.,.27],
          'length_m':1.04,'width_m':.70,'height_m':.32,'spawn_xyz_rpy':[-5.4,0.,.02,0.,0.,0.],
          'vertical_spawn_qualification':'pending; legacy .02 m spawn must not be assumed settled'},
       generator_version=VERSION,generator_sha256=file_sha(__file__),
       readiness={'static_initial_horizontal_qc':True,'live_physics_qc':False,
           'navigation_feasibility':False,'occlusion_semantics':False,'gate_relative_actor_delay':False,
           'named_rng_wired_to_all_live_plugins':False,'final_execution':False})


def add_box(world,name,x,y,length,width,height,yaw=0.,color='.40 .46 .52 1'):
    m=ET.SubElement(world,'model',name=name);ET.SubElement(m,'static').text='true'
    ET.SubElement(m,'pose').text=f'{x:.8f} {y:.8f} {height/2:.8f} 0 0 {yaw:.8f}'
    link=ET.SubElement(m,'link',name='body')
    for tag in ('collision','visual'):
        item=ET.SubElement(link,tag,name=tag);g=ET.SubElement(item,'geometry');shape=ET.SubElement(g,'box')
        ET.SubElement(shape,'size').text=f'{length} {width} {height}'
        if tag=='visual':
            mat=ET.SubElement(item,'material');ET.SubElement(mat,'ambient').text=color;ET.SubElement(mat,'diffuse').text=color
    return m


def add_camera(world):
    m=ET.SubElement(world,'model',name='astra_evidence_camera');ET.SubElement(m,'static').text='true'
    ET.SubElement(m,'pose').text='0 0 18 0 1.5707963267948966 0'
    link=ET.SubElement(m,'link',name='camera_link');s=ET.SubElement(link,'sensor',name='overview',type='camera')
    ET.SubElement(s,'always_on').text='true';ET.SubElement(s,'update_rate').text='2'
    c=ET.SubElement(s,'camera');ET.SubElement(c,'horizontal_fov').text='1.05';im=ET.SubElement(c,'image')
    for k,v in [('width','2560'),('height','1440'),('format','R8G8B8')]:ET.SubElement(im,k).text=v
    clip=ET.SubElement(c,'clip');ET.SubElement(clip,'near').text='.1';ET.SubElement(clip,'far').text='100'
    p=ET.SubElement(s,'plugin',name='astra_camera_ros',filename='libgazebo_ros_camera.so')
    for k,v in [('robotNamespace','/astra'),('alwaysOn','true'),('updateRate','2'),('cameraName','camera'),('imageTopicName','image_raw'),('cameraInfoTopicName','camera_info'),('frameName','astra_camera_optical')]:ET.SubElement(p,k).text=v


def render(scene):
    sdf=ET.Element('sdf',version='1.6');world=ET.SubElement(sdf,'world',name='astra_multilayout_v1')
    ET.SubElement(world,'gravity').text='0 0 -9.81'
    physics=ET.SubElement(world,'physics',name='astra_ode',type='ode');ET.SubElement(physics,'max_step_size').text='.002';ET.SubElement(physics,'real_time_update_rate').text='500'
    light=ET.SubElement(world,'light',name='sun',type='directional');ET.SubElement(light,'pose').text='0 0 10 0 0 0';ET.SubElement(light,'diffuse').text='.8 .8 .8 1';ET.SubElement(light,'direction').text='-.5 .1 -.9'
    m=ET.SubElement(world,'model',name='ground_plane');ET.SubElement(m,'static').text='true';link=ET.SubElement(m,'link',name='link')
    for tag in ('collision','visual'):
        item=ET.SubElement(link,tag,name=tag);g=ET.SubElement(item,'geometry');plane=ET.SubElement(g,'plane');ET.SubElement(plane,'normal').text='0 0 1';ET.SubElement(plane,'size').text='40 30'
        if tag=='visual':
            mat=ET.SubElement(item,'material');ET.SubElement(mat,'ambient').text='.72 .76 .78 1';ET.SubElement(mat,'diffuse').text='.72 .76 .78 1'
    half=scene['corridor_half_width_m']
    if half is not None:
        for sign in (-1,1):add_box(world,f'benchmark_wall_{sign}',0,sign*(half+.1),18,.2,1.)
    for b in scene['static_boxes']:add_box(world,'benchmark_'+b['name'],b['x'],b['y'],b['length'],b['width'],b['height'],b['yaw_rad'])
    for actor in scene['actors']:
        yaw=actor['initial_pose_xyz_rpy'][-1]
        m=add_box(world,actor['model_name'],*actor['start'],.72,.72,1.,yaw,color='.83 .30 .12 1')
        p=ET.SubElement(m,'plugin',name=actor['model_name']+'_controller',filename='libv3_reactive_actor_plugin.so')
        values={'ego_model':'autolabor_m2','policy':actor['policy'],'seed':actor['response_seed'],
          'start':' '.join(map(str,actor['start'])),'end':' '.join(map(str,actor['end'])),
          'conflict':' '.join(map(str,actor['conflict'])),'base_speed':actor['base_speed_mps'],
          'initial_speed':actor['base_speed_mps'],'time_offset':0.,'start_delay':5.,
          'activation_ego_x':-4.02,'ego_observation_period':.20,'ego_observation_hold':.30,
          'response_distance':actor['response_distance_m'],'response_time_margin':actor['response_time_margin_s'],'z':.5}
        for k,v in values.items():ET.SubElement(p,k).text=str(v)
    add_camera(world)
    scene['expected_models']=[m.attrib['name'] for m in world.findall('model')]+['autolabor_m2']
    scene['evidence_camera']={'pose_xyz_rpy':[0,0,18,0,math.pi/2,0],'width':2560,'height':1440,
         'horizontal_fov_rad':1.05,'update_hz':2,'projection':'perspective','topic':'/astra/camera/image_raw'}
    ET.indent(sdf) if hasattr(ET,'indent') else None
    return '<?xml version="1.0"?>\n'+ET.tostring(sdf,encoding='unicode')+'\n'


def build_diagnostic(output,layouts_per_family=3,replicates=2,prior_layout_paths=()):
    output=Path(output)
    if output.exists():raise FileExistsError('Existing scene evidence will not be overwritten: '+str(output))
    prior=[]
    for path in prior_layout_paths:prior.extend(json.loads(Path(path).read_text()))
    errors=audit_geometry_registry(prior)
    if errors:raise ValueError('prior geometry registry failed: '+canonical_json(errors))
    guard=SeedGuard();layouts=[];scenes=[]
    for family in FAMILIES:
        for index in range(layouts_per_family):
            layout=sample_layout(family,index,'development',guard,prior+layouts,cohort=output.name);layouts.append(layout)
            for replicate in range(replicates):scenes.append(episode_scene(layout,replicate,guard))
    output.mkdir(parents=True)
    plan=[]
    for scene in scenes:
        directory=output/scene['layout_id']/f'replicate_{scene["replicate"]:02d}';directory.mkdir(parents=True)
        world_path=directory/'world.sdf';manifest_path=directory/'scene_manifest.yaml'
        world_path.write_text(render(scene));manifest_path.write_text(yaml.safe_dump(scene,sort_keys=False))
        plan.append({'layout_id':scene['layout_id'],'family':scene['family'],'topology':scene['topology'],
          'parent_geometry_id':scene['parent_geometry_id'],'geometry_sha256':scene['canonical_geometry_sha256'],
          'replicate':scene['replicate'],'seed':scene['seed'],'scene_id':scene['scene_id'],
          'world_path':str(world_path),'world_sha256':file_sha(world_path),
          'manifest_path':str(manifest_path),'manifest_sha256':file_sha(manifest_path),
          'eligibility':'diagnostic_geometry_only_pending_runtime_and_semantic_qc'})
    with (output/'diagnostic_blocks.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(plan[0]));w.writeheader();w.writerows(plan)
    report={'status':'offline_scene_diagnostic_materialized_not_execution_release',
      'layouts':len(layouts),'blocks':len(scenes),'families':list(FAMILIES),'execution_started':False,
      'geometry_sha_count':len({s['canonical_geometry_sha256'] for s in layouts}),
      'prior_geometry_registry_files':[{'path':str(p),'sha256':file_sha(p)} for p in prior_layout_paths],
      'prior_layout_count':len(prior),
      'full_supplied_geometry_registry_issues':audit_geometry_registry(prior+layouts),
      'parent_ids_unique':len({s['parent_geometry_id'] for s in layouts})==len(layouts),
      'all_geometry_groups_assigned_development':all(group_split(s['parent_geometry_id'])=='development' for s in layouts),
      'near_duplicate_pair_count':sum(near_duplicate(a,b) for i,a in enumerate(layouts) for b in layouts[i+1:]),
      'all_initial_horizontal_geometry_pass':all(s['initial_geometry_qc']['horizontal_initial_geometry_pass'] for s in layouts),
      'actor_speed_values':sorted({round(a['base_speed_mps'],6) for s in scenes for a in s['actors']}),
      'actor_parameter_cells':len({a['parameter_cell_id'] for s in scenes for a in s['actors']}),
      'all_actor_parameter_cells_development':all(group_split(a['parameter_cell_id'])=='development' for s in scenes for a in s['actors']),
      'geometry_seed_values':sorted(s['geometry_seed'] for s in layouts),
      'sample_parameters_by_layout':[{'layout_id':s['layout_id'],'topology':s['topology'],
                                    'canonical_geometry':canonical_geometry(s,False)} for s in layouts],
      'rng_guard_unique_seed_count':len(guard.records),'rng_records':guard.records,
      'limitations':['This 3-layout x 2-seed diagnostic is not the contracted complete development population.',
        'No final geometries or outcomes were materialized.',
        'World yaw/translation bridge, independent Ackermann feasibility, occlusion/conflict semantic qualification and live z/contact/TF checks are pending.',
        'Actor base speed and response parameters really change in SDF; legacy response seed has zero jitter and is not stochastic behavior.',
        'gate_delay_s is reserved sampled metadata; existing actor plugin cannot apply a gate-relative delay, so it is marked unsupported.',
        'Separate named RNG seeds are recorded, but current live scanner/actuator still require wiring and random-tape qualification.',
        'Geometry splits and actor-cell splits are implemented; overall train/validation/final set materialization and whole-roster cross-split audit are pending.'],
      'bound_contract_sha256':file_sha(STAGE/'contracts/splits.yaml'),'generator_sha256':file_sha(__file__)}
    (output/'generation_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (output/'layouts.json').write_text(json.dumps(layouts,indent=2,allow_nan=False)+'\n')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--layouts-per-family',type=int,default=3);p.add_argument('--replicates',type=int,default=2)
    p.add_argument('--prior-layouts',type=Path,nargs='*',default=[],help='All earlier generated layout registries, including other splits; missing external registries remain an explicit audit limitation.')
    args=p.parse_args()
    if args.layouts_per_family<1 or args.replicates<1:p.error('counts must be positive')
    report=build_diagnostic(args.output,args.layouts_per_family,args.replicates,args.prior_layouts)
    print(json.dumps({k:report[k] for k in ['status','layouts','blocks','geometry_sha_count','near_duplicate_pair_count','actor_speed_values']},indent=2))


if __name__=='__main__':main()
