#!/usr/bin/env python3
"""Derive twelve existing development inputs; no ROS or episode execution.

One parent geometry per family, two parameter repeats, not twelve independent
geometries. Runtime worlds need explicit per-method gate identity binding.
"""
import argparse,copy,csv,hashlib,json,math,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from shapely.geometry import Polygon,MultiPoint,box as geos_box
import numpy as np
from shapely.ops import unary_union
from geometry import Pose3,RobotCollisionModel,project_box,project_cylinder,project_actor_box,compound_clearance
from static_map_export import make_artifacts,static_projections
from common_route import build_bundle,SharedStaticMap
from prepare_navigation_n01_scene import legacy_static_boxes
from dense_sweep_p04 import parse_converted_model
from scene_generator_v2 import laser_visibility
HERE=Path(__file__).resolve().parent;STAGE=HERE.parents[2]
PLUGIN='libastra_event_actor_n04_plugin.so'
METHODS=('astra_analytic_route_v1','astra_pct_route_v1','actual_teb_common_route','dwa_common_route','frozen_r2_v060_sensor_lifecycle_v1')
LIMIT_NS=50000000
REFERENCE=STAGE/'contracts/navigation_n02_prepared_r01/astra_navigation_n02_frozen_r2_v060_sensor_lifecycle_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def write_json(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def put(parent,key,value):
    old=parent.findall(key)
    if len(old)>1:raise ValueError('duplicate SDF field '+key)
    item=old[0] if old else ET.SubElement(parent,key);item.text=str(value)

def collision_signature(text):
    root=ET.fromstring(text);out=[]
    for m in root.findall('world/model'):
        for l in m.findall('link'):
            for c in l.findall('collision'):
                out.append((m.get('name'),m.findtext('pose','0 0 0 0 0 0'),l.get('name'),l.findtext('pose','0 0 0 0 0 0'),ET.tostring(c,encoding='unicode')))
    return out

def bind_gate_identity(world_text,run_id,seed,method):
    """Return per-run XML, changing ONLY expected_* actor metadata in template."""
    if not isinstance(run_id,str) or not run_id or 'UNBOUND' in run_id or len(run_id)>160 or type(seed)is not int or not 0<seed<=2147483647 or method not in METHODS:
        raise ValueError('explicit runtime identity required')
    root=ET.fromstring(world_text)
    for p in root.findall('world/model/plugin'):
        if p.get('filename')!=PLUGIN:raise ValueError('unexpected model plugin')
        if p.findtext('expected_run_id')!='UNBOUND_N04_RUN' or p.findtext('expected_method')!='UNBOUND_N04_METHOD' or p.findtext('expected_seed')!=str(seed):
            raise ValueError('identity template mismatch or second binding')
        for k,v in [('expected_run_id',run_id),('expected_seed',seed),('expected_method',method)]:put(p,k,v)
    result=ET.tostring(root,encoding='unicode')+'\n'
    if collision_signature(result)!=collision_signature(world_text):raise AssertionError('identity changed geometry')
    return result

def derive(parent,world_text,seed):
    s=copy.deepcopy(parent);root=ET.fromstring(world_text);world=root.find('world')
    if parent['route']['spawn']!=[-5.4,0.,0.] or parent['route']['start']!=[-4.,0.,0.] or parent['route']['goal']!=[5.,0.,0.]:
        raise ValueError('nonidentity route requires a separately named R2/control adapter')
    for actor in s['actors']:
        p=world.find("model[@name='%s']/plugin"%actor['model_name'])
        if p is None or p.get('filename')!='libastra_reactive_actor_plugin.so':raise ValueError('source actor identity')
        if p.findtext('policy')!=actor['policy'] or float(p.findtext('gate_delay'))!=actor['gate_delay_s']:raise ValueError('actor parameter mismatch')
        for key in ('start','end','conflict'):
            if list(map(float,p.findtext(key).split()))!=actor[key]:raise ValueError('actor path mismatch')
        for key in ('route_origin','route_yaw','gate_route_x'):
            old=p.find(key)
            if old is None:raise ValueError('missing original geometric gate field')
            p.remove(old)
        p.set('filename',PLUGIN)
        for key,val in [('expected_run_id','UNBOUND_N04_RUN'),('expected_seed',seed),('expected_method','UNBOUND_N04_METHOD'),('gate_topic','/astra/navigation/gate'),('max_gate_transport_delay_ns',LIMIT_NS)]:put(p,key,val)
        actor.update(live_plugin=PLUGIN,gate_status_topic='/astra/actors/'+actor['model_name']+'/gate_status',gate_authority='coordinator_actual_odom_header_event',live_gate_qualification=False)
    for plugin in list(world.findall('plugin')):world.remove(plugin)
    dense=ET.SubElement(world,'plugin',name='whole_contact_evidence',filename='libastra_dense_evidence_plugin.so');put(dense,'state_hz',50)
    ET.SubElement(world,'plugin',name='passive_sweep_evidence',filename='libastra_physics_metadata_p03_plugin.so')
    text=ET.tostring(root,encoding='unicode')+'\n'
    if collision_signature(text)!=collision_signature(world_text):raise AssertionError('parent geometry changed')
    s.update(schema='astra_navigation_n04_development_scene_v1',seed=seed,parent_episode_seed=parent['seed'],split='development',
        environment_version='astra_n04_physical_p05_event_actor_50ms',runtime_identity_bound=False,execution_authorized=False,
        parent_static_boxes=copy.deepcopy(parent.get('static_boxes',[])),parent_static_circles=copy.deepcopy(parent.get('static_circles',[])),
        parent_corridor_half_width_m=parent.get('corridor_half_width_m'),parent_robot_collision_reference=copy.deepcopy(parent['robot_collision_reference']),
        robot_collision_reference={'reference':'base_link','spawn_xyz_rpy':[-5.4,0.,0.,0.,0.,0.],'full_collision_count':9},
        camera={'topic':parent['evidence_camera']['topic'],'width':2560,'height':1440,'rate_hz':2},
        input_variant_id=parent['layout_id']+'_n04_event_actor',new_independent_geometry=False,
        frozen_R2_common_guide_consumption=False,nonidentity_heading_supported=False,
        actor_activation={'source':'/astra/navigation/gate actual_odom_header_first_crossing','max_gate_transport_delay_ns':LIMIT_NS,
          'first_consuming_Begin_active_dt_ns':0,'no_unreceived_interval_backfill':True,'live_qualification':'pending',
          'expected_identity':'per-run materialized SDF expected_run_id/expected_seed/expected_method'},
        source_seed_change_scope='new simulator noise seed only; parent actor parameter/response seeds unchanged')
    return s,text

def static_input(world_text,scene):
    """Explicit removal, not monkeypatching the frozen exporter classifier."""
    root=ET.fromstring(world_text);world=root.find('world');omitted=[]
    for actor in scene['actors']:
        m=world.find("model[@name='%s']"%actor['model_name'])
        if m is None or len(m.findall('plugin'))!=1 or m.find('plugin').get('filename')!=PLUGIN:raise ValueError('new actor classification mismatch')
        omitted.append({'model':actor['model_name'],'reason':'declared event-controlled actor omitted from static prior','model_xml_sha256':hashlib.sha256(ET.tostring(m)).hexdigest()});world.remove(m)
    s=copy.deepcopy(scene);s['actors']=[]
    return ET.tostring(root,encoding='unicode')+'\n',s,omitted

class ReferenceRobot:
    def __init__(self,urdf,sdf):
        self.tree=RobotCollisionModel(urdf);self.converted=parse_converted_model(urdf,sdf)
        self.zero={n:0. for n,j in self.tree.joints.items() if j.kind!='fixed'}
        self.radius_audit=[]
        for body,b in self.converted['bodies'].items():
            distance=0.;link=body
            while link in self.tree.by_child:
                joint=self.tree.by_child[link];distance+=math.sqrt(sum(x*x for x in joint.origin.xyz));link=joint.parent
            for shape in b['shapes']:
                d=shape['dimensions'];extent=math.sqrt(sum((x/2)**2 for x in d)) if shape['kind']=='box' else math.hypot(d[0],d[1]/2)
                local=math.sqrt(sum(x*x for x in shape['origin'].xyz))+extent
                if distance+local>1. or ('front_' in body and local>.3):raise ValueError('witness analytic shape radius assumption fails')
                spin_error=0.
                if 'wheel_link' in body:
                    axis=shape['origin'].rotate((0.,0.,1.));joint_axis=self.tree.by_child[body].axis
                    skew=math.sqrt(max(0.,1-sum(a*b for a,b in zip(axis,joint_axis))**2))
                    spin_error=2*extent*skew
                    if spin_error>1e-5:raise ValueError('quantized wheel arbitrary-spin engineering pad insufficient')
                self.radius_audit.append({'shape':shape['name'],'root_radius_triangle_bound_m':distance+local,'steering_local_radius_bound_m':local if 'front_' in body else None,'arbitrary_wheel_spin_pad_required_m':spin_error})
    def project(self,pose,curvature=0.):
        joints=dict(self.zero)
        for side,sign in [('left',1),('right',-1)]:joints['front_'+side+'_steer_joint']=math.atan(.65*curvature/(1-sign*.3*curvature))
        poses={self.tree.root_link:pose}
        def fk(name):
            if name not in poses:
                j=self.tree.by_child[name];angle=joints.get(j.name,0.)
                motion=Pose3(xyzw=tuple(a*math.sin(angle/2) for a in j.axis)+(math.cos(angle/2),))
                poses[name]=fk(j.parent).compose(j.origin).compose(motion)
            return poses[name]
        out=[]
        for body,b in self.converted['bodies'].items():
            for c in b['shapes']:
                p=fk(body).compose(c['origin'])
                out.append(project_box(c['name'],c['dimensions'],p,body) if c['kind']=='box' else project_cylinder(c['name'],*c['dimensions'],p,link_name=body))
        return out

def reduced_clearance(robot,obstacles):
    r=compound_clearance(robot,obstacles)
    return {k:v for k,v in r.items() if k!='pairs'}

def witness(robot,obstacles,family):
    """Ideal static rear-axle bicycle witness; not a measured physical trajectory."""
    amplitude=1.6 if family in ('static_avoidance','occluded_emergence') else 0.
    k=math.pi/9.;yp_bound=4*amplitude*k;angle_bound=4*amplitude*k*k
    curvature_bound=angle_bound;kp_bound=40*amplitude*k**3+3*yp_bound*angle_bound**2
    steer_derivative_bound=.65*kp_bound/(1-.3*curvature_bound)**2
    # Static body extents are checked below; joint-axis wheel skew from SDF's
    # six-digit roll is padded separately. Only ideal planar/no-slip geometry.
    body_radius_bound=1.;steer_radius_bound=.3
    point_lipschitz=math.sqrt(1+yp_bound**2)+(body_radius_bound+.325)*angle_bound+steer_radius_bound*steer_derivative_bound
    static_union=unary_union([Polygon(p.polygon_xy) for p in obstacles])
    minimum=None;maxcurv=0.;sample_count=901;step=9/(sample_count-1)
    for i in range(sample_count):
        x=-4.325+step*i;t=k*(x+4.325);sin,cos=math.sin(t),math.cos(t)
        y=amplitude*sin**4;yp=4*amplitude*k*sin**3*cos;ypp=4*amplitude*k*k*(3*sin*sin*cos*cos-sin**4)
        yaw=math.atan(yp);curvature=ypp/(1+yp*yp)**1.5;maxcurv=max(maxcurv,abs(curvature))
        pose=Pose3.from_xyz_rpy((x+.325*math.cos(yaw),y+.325*math.sin(yaw),0),(0,0,yaw))
        shapes=robot.project(pose,curvature)
        local=robot.project(Pose3(),curvature)
        if any(math.hypot(*v)>body_radius_bound for p in local for v in p.polygon_xy):raise AssertionError('witness radius bound')
        if not static_union.is_empty:
            polygons=[Polygon(p.polygon_xy) for p in shapes]
            if any(p.intersects(static_union) for p in polygons):gap=0.
            else:gap=min(p.distance(static_union) for p in polygons)
            minimum=gap if minimum is None else min(minimum,gap)
    lower=None if minimum is None else minimum-point_lipschitz*step/2-1e-5
    return {'scope':'ideal planar static Ackermann geometry, not actual physical/control or dynamic-actor feasibility',
      'shape_model':'9 converted-SDF primitives; wheel spin zero; <=1e-5m axis-quantization engineering pad',
      'rear_curve':'x from -4.325 to 4.675, y=A*sin(pi*(x+4.325)/9)^4','amplitude_m':amplitude,
      'samples':sample_count,'parameter_step_m':step,'max_sampled_curvature_m_inv':maxcurv,
      'analytic_curvature_abs_upper_bound':curvature_bound,'max_central_steer_bound_rad':math.atan(.65*curvature_bound),
      'central_steer_limit_rad':.4964,'point_displacement_lipschitz_per_rear_x':point_lipschitz,
      'sampled_min_physical_xy_m':minimum,'static_interval_physical_xy_lower_m':lower,
      'static_interval_minus_033_lower_m':None if lower is None else lower-.33,
      'static_geometric_witness_pass':math.atan(.65*curvature_bound)<=.4964 and (lower is None or lower>.33),
      'shape_radius_audit':robot.radius_audit,'live_tracking_qualified':False,'dynamic_actor_avoidance_claim':False}

def scene_qc(scene,world_text,robot,do_witness=True):
    projection_xml,s,omitted=static_input(world_text,scene);obstacles,_=static_projections(projection_xml,s)
    actors=[project_actor_box(a['model_name'],Pose3.from_xyz_rpy(a['initial_pose_xyz_rpy'][:3],a['initial_pose_xyz_rpy'][3:])) for a in scene['actors']]
    initial={}
    for name,x in [('spawn',-5.4),('gate',-4.),('goal',5.)]:
        parts=robot.project(Pose3.from_xyz_rpy((x,0,0),(0,0,0)))
        initial[name]={'static':reduced_clearance(parts,obstacles),'initial_actors':reduced_clearance(parts,actors),
          'wheel_z_intervals_m':{p.link_name:[p.z_min_m,p.z_max_m] for p in parts if 'wheel_link' in p.link_name},'primitive_count':len(parts)}
    swept=[]
    for a in scene['actors']:
        yaw=math.atan2(a['end'][1]-a['start'][1],a['end'][0]-a['start'][0]);points=[]
        for xy in (a['start'],a['end']):points.extend(project_actor_box(a['model_name'],Pose3.from_xyz_rpy((*xy,.5),(0,0,yaw))).polygon_xy)
        hull=MultiPoint(points).convex_hull
        distances=[(p.name,hull.distance(Polygon(p.polygon_xy)),hull.intersects(Polygon(p.polygon_xy))) for p in obstacles]
        swept.append({'model':a['model_name'],'full_path_physical_xy_gap_m':min([x[1] for x in distances],default=None),
          'static_intersections':[x[0] for x in distances if x[2]],'shape':'0.72m square complete constant-heading translation hull',
          'dynamic_ego_clearance_claim':False})
    visibility=None
    if scene['family']=='occluded_emergence':
        a=scene['actors'][0];static=[(p.name,Polygon(p.polygon_xy)) for p in obstacles if p.z_min_m<=.6<=p.z_max_m]
        reveal=(a['conflict'][0]+.6,0.,0.)
        visibility={'at_gate':laser_visibility((-4,0,0),a,static),'at_static_reveal_pose':laser_visibility(reveal,a,static),
          'reveal_pose_compound_static':reduced_clearance(robot.project(Pose3.from_xyz_rpy((*reveal[:2],0),(0,0,reveal[2]))),obstacles),
          'scope':'ray diagnostic with actor at initial pose, height .60, 720 beams; actual moving visibility pending'}
    return {'initial_full_compound':initial,'actor_full_path_static_sweep':swept,'occlusion':visibility,
      'static_witness':witness(robot,obstacles,scene['family']) if do_witness else None,
      'Z_interpretation':'XY projection with Z interval exclusion; not a measured 3D contact or live support qualification'}

def build(source,output,registry,reference=REFERENCE):
    source,output,registry,reference=map(Path,(source,output,registry,reference))
    if output.exists():raise FileExistsError('preserve existing N04 derivation')
    rows=list(csv.DictReader((source/'diagnostic_blocks.csv').open()));seeds=list(range(263001001,263001013))
    if len(rows)!=12 or len({r['parent_geometry_id'] for r in rows})!=6:raise ValueError('six parent geometries x two input repeats required')
    registry_bytes=registry.read_bytes()
    registered={int(v) for row in csv.DictReader(registry_bytes.decode().splitlines()) for v in row.values() if v and v.isdigit()}
    if set(seeds)&registered:raise ValueError('proposed development seed already in registry')
    urdf=reference/'robot_description.xml';sdf=reference/'conversion/conversion.sdf';robot=ReferenceRobot(urdf.read_text(),sdf.read_text())
    output.mkdir(parents=True);(output/'reference_robot').mkdir();shutil.copyfile(urdf,output/'reference_robot/robot_description.xml');shutil.copyfile(sdf,output/'reference_robot/converted_robot.sdf')
    before={str(source/'diagnostic_blocks.csv'):sha(source/'diagnostic_blocks.csv')};derived=[];cache={}
    for index,(old,seed) in enumerate(zip(rows,seeds)):
        mp,wp=Path(old['manifest_path']),Path(old['world_path'])
        if sha(mp)!=old['manifest_sha256'] or sha(wp)!=old['world_sha256']:raise ValueError('parent source hash differs')
        before.update({str(mp):sha(mp),str(wp):sha(wp)})
        parent=yaml.safe_load(mp.read_text());scene,world=derive(parent,wp.read_text(),seed)
        folder=output/parent['layout_id']/('replicate_%02d'%parent['replicate']);folder.mkdir(parents=True)
        (folder/'parent').mkdir();shutil.copyfile(mp,folder/'parent/scene_manifest.yaml');shutil.copyfile(wp,folder/'parent/world.sdf')
        (folder/'world.sdf').write_text(world)
        projection_xml,projection_scene,omitted=static_input(world,scene)
        exported=folder/'map_export';exported.mkdir();(exported/'static_projection_input.sdf').write_text(projection_xml)
        pgm,meta,request,audit=make_artifacts(projection_xml,projection_scene)
        audit.update(actual_world_template_sha256=sha(folder/'world.sdf'),explicit_dynamic_model_omissions=omitted,
          static_collision_geometry_equals_parent=True,classification_adapter='new event actors explicitly removed before frozen static exporter')
        boxes,legacy=legacy_static_boxes(pgm,meta);scene.update(static_boxes=boxes,static_circles=[],corridor_half_width_m=None,r2_static_input_adapter=legacy)
        scene['input_derivation']={'parent_manifest':{'path':str(mp),'sha256':sha(mp)},'parent_world':{'path':str(wp),'sha256':sha(wp)},
          'generator_sha256':sha(__file__),'parent_geometry_retained':True,'new_independent_geometry':False,'parent_episode_seed':parent['seed']}
        (exported/'map.pgm').write_bytes(pgm);(exported/'map.yaml').write_bytes(meta);write_json(exported/'route_request.json',request);write_json(exported/'audit.json',audit);write_json(exported/'legacy_raster_adapter.json',legacy)
        route=build_bundle(exported,folder/'common_bundle',radius=1.)
        grid=SharedStaticMap(pgm,meta);rr,cc=np.nonzero(grid.pixels!=254);r=grid.resolution;ox,oy=grid.origin[:2]
        raster_union=unary_union([geos_box(ox+c*r,oy+row*r,ox+(c+1)*r,oy+(row+1)*r) for row,c in zip(rr,cc)])
        box_union=unary_union([geos_box(b['x']-b['length']/2,b['y']-b['width']/2,b['x']+b['length']/2,b['y']+b['width']/2) for b in boxes])
        difference=float(raster_union.symmetric_difference(box_union).area)
        if difference>1e-10:raise ValueError('independent GEOS raster union differs')
        write_json(folder/'legacy_map_union_independent.json',{'status':'shared_raster_rectangle_input_adapter_offline_pass',
          'box_count':len(boxes),'source_raster_blocked_count':len(rr),'raster_cell_union_mismatch_count':legacy['cell_union_symmetric_difference_count'],
          'GEOS_union_symmetric_difference_area_m2':difference,'floating_area_tolerance_m2':1e-10,'route_length_m':route['route_length_m'],
          'route_endpoint_matches':True,'outside_map_and_fixed_route_consumer_equivalence':False,'no_Gazebo_started':True,
          'reference_method':'independent GEOS union of every blocked raster cell versus declared rectangles; author-generated, not independent reviewer approval'})
        # Geometry repeats share the same numeric QC; actor policy does not
        # alter complete start/end sweep or ideal static witness.
        key=old['parent_geometry_id']
        if key not in cache:cache[key]=scene_qc(scene,world,robot)
        qc=copy.deepcopy(cache[key]);qc['parent_geometry_id']=key;qc['seed']=seed
        write_json(folder/'geometry_qc.json',qc)
        (folder/'scene_manifest.yaml').write_text(yaml.safe_dump(scene,sort_keys=False))
        record={'input_index':index,'seed':seed,'family':scene['family'],'replicate':scene['replicate'],'parent_geometry_id':key,
          'scene_directory':str(folder.resolve()),'scene_manifest':str((folder/'scene_manifest.yaml').resolve()),'world_template':str((folder/'world.sdf').resolve()),
          'common_bundle':str((folder/'common_bundle').resolve()),'route_length_m':route['route_length_m'],
          'static_witness_pass':qc['static_witness']['static_geometric_witness_pass'],'actor_count':len(scene['actors']),
          'runtime_identity_bound':False,'live_qualification':'pending'}
        derived.append(record)
        print(json.dumps({'prepared':index+1,'family':scene['family'],'seed':seed,'static_witness_pass':record['static_witness_pass']}),flush=True)
    if registry.read_bytes()!=registry_bytes or not all(sha(p)==h for p,h in before.items()):raise RuntimeError('source or registry changed during derivation')
    csv_rows=[]
    for old,new in zip(rows,derived):
        row=dict(old);folder=Path(new['scene_directory'])
        row.update(seed=new['seed'],world_path=str(folder/'world.sdf'),world_sha256=sha(folder/'world.sdf'),
          manifest_path=str(folder/'scene_manifest.yaml'),manifest_sha256=sha(folder/'scene_manifest.yaml'),
          input_index=new['input_index'],common_bundle=new['common_bundle'],eligibility='offline_prepared_live_qualification_pending',
          input_variant_id=old['layout_id']+'_n04_event_actor',environment_version='astra_n04_physical_p05_event_actor_50ms')
        csv_rows.append(row)
    with (output/'diagnostic_blocks.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(csv_rows[0]));writer.writeheader();writer.writerows(csv_rows)
    files=[{'path':str(p.resolve()),'sha256':sha(p)} for p in sorted(output.rglob('*')) if p.is_file()]
    report={'schema':'astra_navigation_n04_scene_preparation_v1','status':'offline_development_inputs_ready_pending_actor_live_qualification',
      'execution_authorized':False,'seeds_registered':False,'proposed_seeds':seeds,'registry_binding':{'path':str(registry.resolve()),'sha256':sha(registry)},
      'input_rows':derived,'parent_geometry_count':6,'new_independent_geometry_count':0,'parent_bindings':before,'parent_bytes_preserved':True,
      'reference_geometry_only':{'urdf_sha256':sha(urdf),'converted_sdf_sha256':sha(sdf),'reference_seed':260926033,'runtime_per_run_URDF_and_conversion_binding_required':True},
      'files':files,'generator_sha256':sha(__file__),'all_static_witnesses_pass':all(x['static_witness_pass'] for x in derived),
      'limitations':['six reused parent geometries; repeats are parameter/noise repeats','fixed R2 audits common route but does not consume it',
        'all current routes are identity heading; arbitrary SE2 R2 bridge unimplemented','static witness is ideal geometric and excludes moving actors/physical tracking',
        'source folders contain no raster figures; parent world visual XML is retained byte-exact','event actor transport/fault/roster qualification pending fresh live stage',
        'camera native 2560x1440 configuration retained; no rendered image created offline']}
    write_json(output/'preparation_report.json',report);return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=STAGE/'scenes/development_a02_live_v2');p.add_argument('--output',type=Path,required=True);p.add_argument('--registry',type=Path,default=STAGE/'seed_registry.csv');a=p.parse_args();build(a.source,a.output,a.registry)
