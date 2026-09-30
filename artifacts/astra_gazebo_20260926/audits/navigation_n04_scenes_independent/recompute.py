#!/usr/bin/env python3
"""Independent offline N04 static input audit. Never starts ROS or allocates IDs.

Geometry is parsed directly from actual converted SDF and projected using local
4x4 transforms, not the preparation author's ReferenceRobot or geometry API.
The author's pure identity/export functions are invoked only for interface tests.
"""
import copy,csv,hashlib,io,itertools,json,math,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from PIL import Image
from shapely.geometry import MultiPoint,Polygon,Point,LineString,box
from shapely.ops import unary_union

OUT=Path(__file__).resolve().parent
S=OUT.parents[1]; W=S/'workspace'; SRC=W/'scripts/astra_gazebo'
sys.path.insert(0,str(SRC))
import prepare_navigation_n04_scenes as prep
from static_map_export import make_artifacts

BASE=S/'scenes/navigation_n04_development_r01'
OLD=S/'scenes/development_a02_live_v2'
inputs={}
def read(p):
    p=Path(p).resolve(); data=p.read_bytes();inputs[str(p)]=hashlib.sha256(data).hexdigest();return data
def js(p):return json.loads(read(p))
def ym(p):return yaml.safe_load(read(p))
def save(name,value): (OUT/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def canon(e):return (e.tag,tuple(sorted(e.attrib.items())),(e.text or '').strip(),tuple(canon(c) for c in e))
def digest(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def vals(text,default):return np.array(list(map(float,(text or default).split())))
def transform(text=None):
    x,y,z,r,p,w=vals(text,'0 0 0 0 0 0');c,s=math.cos,math.sin
    Rz=np.array([[c(w),-s(w),0],[s(w),c(w),0],[0,0,1.]])
    Ry=np.array([[c(p),0,s(p)],[0,1,0],[-s(p),0,c(p)]])
    Rx=np.array([[1,0,0],[0,c(r),-s(r)],[0,s(r),c(r)]])
    T=np.eye(4);T[:3,:3]=Rz@Ry@Rx;T[:3,3]=[x,y,z];return T
def rot(axis,angle):
    a=np.asarray(axis);K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    T=np.eye(4);T[:3,:3]=np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K);return T
def primitive(c):
    b=c.find('geometry/box'); cy=c.find('geometry/cylinder')
    if b is not None:
        dims=vals(b.findtext('size'),'');v=np.array(list(itertools.product(*[(-d/2,d/2) for d in dims])))
        return {'kind':'box','dims':dims,'vertices':v,'extent':float(np.linalg.norm(dims/2))}
    if cy is not None:
        r=float(cy.findtext('radius'));l=float(cy.findtext('length'));a=np.arange(48)*2*math.pi/48
        v=np.array([(r/math.cos(math.pi/48)*math.cos(t),r/math.cos(math.pi/48)*math.sin(t),z) for z in (-l/2,l/2) for t in a])
        return {'kind':'cylinder','dims':np.array([r,l]),'vertices':v,'extent':math.hypot(r,l/2)}
    if c.find('geometry/plane') is not None:return None
    raise AssertionError('unsupported actual collision geometry')
def projected(shape,T):
    v=shape['vertices']@T[:3,:3].T+T[:3,3]
    return MultiPoint(v[:,:2]).convex_hull
def z_interval(shape,T):
    if shape['kind']=='box':h=float(np.abs(T[2,:3])@(shape['dims']/2))
    else:
        r,l=shape['dims'];h=r*math.hypot(T[2,0],T[2,1])+l/2*abs(T[2,2])
    return [float(T[2,3]-h),float(T[2,3]+h)]

class Robot:
    def __init__(self,path):
        m=ET.fromstring(read(path)).find('model');self.links={x.get('name'):x for x in m.findall('link')};self.joints={};self.shapes=[]
        for j in m.findall('joint'):
            assert j.find('pose').get('relative_to')==j.findtext('parent')
            child=j.findtext('child');lp=self.links[child].find('pose')
            assert lp.get('relative_to')==j.get('name') and np.allclose(transform(lp.text),np.eye(4),atol=0,rtol=0)
            self.joints[child]={'name':j.get('name'),'parent':j.findtext('parent'),'T':transform(j.findtext('pose')),'axis':vals(j.findtext('axis/xyz'),'')}
        for name,l in self.links.items():
            for c in l.findall('collision'):
                p=primitive(c);p.update(name=c.get('name'),body=name,T=transform(c.findtext('pose')));self.shapes.append(p)
        assert len(self.links)==7 and len(self.shapes)==9
        self.radius=[]
        for p in self.shapes:
            link=p['body'];d=0.
            while link in self.joints:
                j=self.joints[link];d+=float(np.linalg.norm(j['T'][:3,3]));link=j['parent']
            local=float(np.linalg.norm(p['T'][:3,3]))+p['extent'];pad=0.
            if 'wheel_link' in p['body']:
                axis=p['T'][:3,2];ja=self.joints[p['body']]['axis'];pad=2*p['extent']*math.sqrt(max(0.,1-float(axis@ja)**2))
            assert d+local<=1 and ('front_' not in p['body'] or local<=.3) and pad<=1e-5
            self.radius.append({'name':p['name'],'radius_bound_m':d+local,'front_local_bound_m':local if 'front_' in p['body'] else None,'arbitrary_wheel_spin_skew_pad_m':pad})
    def parts(self,x,y,yaw,k=0.):
        poses={'base_link':transform('%r %r 0 0 0 %r'%(x,y,yaw))}
        def fk(name):
            if name not in poses:
                j=self.joints[name];a=0.
                if 'steer_joint' in j['name']:
                    sign=1 if 'left' in j['name'] else -1;a=math.atan(.65*k/(1-sign*.3*k))
                poses[name]=fk(j['parent'])@j['T']@rot(j['axis'],a)
            return poses[name]
        return [(p['name'],projected(p,fk(p['body'])@p['T']),z_interval(p,fk(p['body'])@p['T'])) for p in self.shapes]

def world_parts(text,omit=()):
    out=[]
    for m in ET.fromstring(text).findall('world/model'):
        if m.get('name') in omit:continue
        for l in m.findall('link'):
            for c in l.findall('collision'):
                p=primitive(c)
                if p is None:continue
                T=transform(m.findtext('pose'))@transform(l.findtext('pose'))@transform(c.findtext('pose'));z=z_interval(p,T)
                if z[1]<.01 or z[0]>.70:continue
                out.append((m.get('name')+'::'+l.get('name')+'::'+c.get('name'),projected(p,T),z))
    return out

def independent_canonical(scene,quantized):
    def n(v,s):return int(math.floor(float(v)/s+.5)) if quantized else round(float(v),4)
    options=[]
    for sign in (1,-1):
        options.append({'family':scene['family'],'wall_half_width':None if scene['corridor_half_width_m'] is None else n(scene['corridor_half_width_m'],.2),
          'boxes':sorted([[n(b['x'],.2),n(sign*b['y'],.2),n(b['length'],.1),n(b['width'],.1),n(b['height'],.1)] for b in scene['static_boxes']]),
          'actor_path_points':sorted([[n(a[k][0],.2),n(sign*a[k][1],.2)] for a in scene['actor_geometry'] for k in ('start','end','conflict')]),'route_length_m':9.})
    return min(options,key=lambda x:json.dumps(x,sort_keys=True,separators=(',',':')))

def curve_samples(robot,static,family,quintic=False):
    A=1.6 if family in ('static_avoidance','occluded_emergence') else 0.;a=math.pi/9
    if quintic:
        p=1.875*A/4.5;q=10*math.sqrt(3)/3*A/4.5**2;r=60*A/4.5**3
    else:p=4*A*a;q=4*A*a*a;r=40*A*a**3
    kp=r+3*p*q*q;dp=.65*kp/(1-.3*q)**2;L=math.sqrt(1+p*p)+1.325*q+.3*dp
    rows=[];u=unary_union([v[1] for v in static]);maximum=0.
    for i in range(901):
        x=-4.325+i*.01
        if quintic:
            t=(x+4.325)/4.5;sgn=1 if t<=1 else -1;t=t if t<=1 else 2-t
            y=A*(10*t**3-15*t**4+6*t**5);dy=sgn*A/4.5*30*t*t*(1-t)**2;ddy=A/4.5**2*60*t*(1-t)*(1-2*t)
        else:
            t=a*(x+4.325);si,co=math.sin(t),math.cos(t);y=A*si**4;dy=4*A*a*si**3*co;ddy=4*A*a*a*(3*si*si*co*co-si**4)
        yaw=math.atan(dy);k=ddy/(1+dy*dy)**1.5;bx=x+.325*math.cos(yaw);by=y+.325*math.sin(yaw);parts=robot.parts(bx,by,yaw,k)
        distances=[(v[1].distance(u),v[0]) for v in parts] if not u.is_empty else []
        gap,worst=min(distances) if distances else (None,None);maximum=max(maximum,abs(k))
        rows.append({'sample':i,'rear_x':x,'base_x':bx,'base_y':by,'yaw':yaw,'curvature':k,'physical_xy_gap':gap,'nearest_robot_shape':worst})
    minimum=min([x['physical_xy_gap'] for x in rows if x['physical_xy_gap'] is not None],default=None)
    lower=None if minimum is None else minimum-L*.005-1e-5
    result={'curve':'quintic' if quintic else 'sin4','samples':901,'sampled_min_physical_xy_m':minimum,'interval_physical_xy_lower_m':lower,'interval_minus_033_lower_m':None if lower is None else lower-.33,'point_L_per_rear_x':L,'curvature_bound':q,'steer_bound_rad':math.atan(.65*q),'max_sampled_curvature':maximum,'static_pass':math.atan(.65*q)<=.4964 and (lower is None or lower>.33),'derivative_bounds':[p,q,r,kp,dp]}
    with (OUT/('%s_%s_samples.csv'%(family,result['curve']))).open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    return result,rows

def visibility(observer,target,static):
    origin=Point(observer);potential=[];visible=[]
    for i in range(720):
        a=-math.pi+2*math.pi*i/719;ray=LineString([observer,(observer[0]+30*math.cos(a),observer[1]+30*math.sin(a))]);hit=ray.intersection(target)
        if hit.is_empty or origin.distance(hit)<.1:continue
        potential.append(i);d=origin.distance(hit)
        if all(ray.intersection(p).is_empty or origin.distance(ray.intersection(p))>=d-1e-8 for _,p,z in static if z[0]<=.60<=z[1]):visible.append(i)
    return {'potential_count':len(potential),'visible_count':len(visible),'visible_indices':visible}

def main():
    read(__file__)
    for filename in ('prepare_navigation_n04_scenes.py','static_map_export.py','common_route.py','geometry.py','dense_sweep_p04.py','scene_generator.py','scene_generator_v2.py','prepare_navigation_n01_scene.py'):read(SRC/filename)
    for filename in ('occlusion_static_witness_r1.py','occlusion_static_witness_r1.json','occlusion_static_witness_r1.samples.json'):read(S/'audits/navigation_n04_scene_preparation'/filename)
    rows=list(csv.DictReader(read(BASE/'diagnostic_blocks.csv').decode().splitlines()));old=list(csv.DictReader(read(OLD/'diagnostic_blocks.csv').decode().splitlines()))
    assert len(rows)==len(old)==12
    urdf=BASE/'reference_robot/robot_description.xml';sdf=BASE/'reference_robot/converted_robot.sdf';read(urdf);robot=Robot(sdf)
    assert read(urdf)==read(prep.REFERENCE/'robot_description.xml') and read(sdf)==read(prep.REFERENCE/'conversion/conversion.sdf')
    layouts=js(S/'scenes/development_a02/layouts.json');prior=js(S/'scenes/development_a01/layouts.json');read(S/'contracts/splits.yaml')
    split=[]
    for lay in prior+layouts:
        pid='geom_'+digest(independent_canonical(lay,True))[:24];v=int(digest(['astra_gazebo_20260926',pid,'allocation_v1'])[:16],16)%100
        assert pid==lay['parent_geometry_id'] and ('train' if v<60 else 'development' if v<80 else 'validation' if v<90 else 'final_test')==lay['split']=='development'
        assert digest(independent_canonical(lay,False))==lay['canonical_geometry_sha256']
        split.append({'parent':pid,'split':lay['split'],'hash_bucket':v})
    checks=[];geometry={};binding_cases=0;negative_cases=0
    for index,(row,parentrow) in enumerate(zip(rows,old)):
        assert int(row['input_index'])==index and int(row['seed'])==263001001+index
        for k in ('family','layout_id','parent_geometry_id','geometry_sha256','replicate'):assert row[k]==parentrow[k]
        folder=Path(row['manifest_path']).parent;scene=ym(folder/'scene_manifest.yaml');parent=ym(parentrow['manifest_path']);text=read(folder/'world.sdf').decode();oldtext=read(parentrow['world_path']).decode()
        assert read(folder/'parent/world.sdf').decode()==oldtext and ym(folder/'parent/scene_manifest.yaml')==parent
        assert hashlib.sha256(text.encode()).hexdigest()==row['world_sha256'] and inputs[str((folder/'scene_manifest.yaml').resolve())]==row['manifest_sha256']
        assert scene['split']==parent['split']=='development' and scene['new_independent_geometry'] is False and scene['route']==parent['route']
        assert scene['parent_static_boxes']==parent['static_boxes'] and scene['parent_static_circles']==parent['static_circles'] and scene['parent_corridor_half_width_m']==parent['corridor_half_width_m']
        assert len(scene['actors'])==len(parent['actors'])
        for a,b in zip(scene['actors'],parent['actors']):
            for k,v in b.items():
                if k not in ('live_plugin','gate_status_topic','gate_authority','live_gate_qualification'):assert a[k]==v,(k,a[k],v)
        # Exact whole XML preservation except preregistered actor gate and world evidence plugins.
        x=ET.fromstring(text);oldx=ET.fromstring(oldtext)
        for world in (x.find('world'),oldx.find('world')):
            for p in list(world.findall('plugin')):world.remove(p)
        for p,q in zip(x.findall('world/model/plugin'),oldx.findall('world/model/plugin')):
            for e in ('expected_run_id','expected_seed','expected_method','gate_topic','max_gate_transport_delay_ns'):p.remove(p.find(e))
            for e in ('route_origin','route_yaw','gate_route_x'):q.remove(q.find(e))
            p.set('filename',q.get('filename'))
        assert canon(x)==canon(oldx)
        for method in prep.METHODS:
            bound=prep.bind_gate_identity(text,'offline_audit_'+method,int(row['seed']),method);a=ET.fromstring(text);b=ET.fromstring(bound)
            for z in (a,b):
                for p in z.findall('world/model/plugin'):
                    for k in ('expected_run_id','expected_seed','expected_method'):p.remove(p.find(k))
            assert canon(a)==canon(b);binding_cases+=1
            if scene['actors']:
                try:prep.bind_gate_identity(bound,'second',int(row['seed']),method)
                except ValueError:negative_cases+=1
                else:raise AssertionError('actor second binding accepted')
        for bad in (True,-1,0,2**31):
            try:prep.bind_gate_identity(text,'offline',bad,prep.METHODS[0])
            except ValueError:negative_cases+=1
            else:raise AssertionError('bad identity accepted')
        actor_names=[a['model_name'] for a in scene['actors']];static=world_parts(text,actor_names);savedstatic=read(folder/'map_export/static_projection_input.sdf').decode();stripped=ET.fromstring(text)
        for m in list(stripped.findall('world/model')):
            if m.get('name') in actor_names:stripped.find('world').remove(m)
        assert canon(stripped)==canon(ET.fromstring(savedstatic))
        ss=copy.deepcopy(scene);ss['actors']=[];pgm,meta,request,audit=make_artifacts(savedstatic,ss)
        assert pgm==read(folder/'map_export/map.pgm')==read(folder/'common_bundle/map.pgm')
        assert meta==read(folder/'map_export/map.yaml')==read(folder/'common_bundle/map.yaml')
        assert not any(name in json.dumps(audit['static_collision_components']) for name in actor_names)
        # Independent rectangle-to-cell coverage, no sharing author's union implementation.
        pixels=np.flipud(np.asarray(Image.open(io.BytesIO(pgm))));m=yaml.safe_load(meta);r=m['resolution'];ox,oy=m['origin'][:2];cover=np.zeros(pixels.shape,dtype=bool)
        for rect in scene['static_boxes']:
            edges=[(rect['x']-rect['length']/2-ox)/r,(rect['x']+rect['length']/2-ox)/r,(rect['y']-rect['width']/2-oy)/r,(rect['y']+rect['width']/2-oy)/r]
            assert max(abs(v-round(v)) for v in edges)<1e-9
            c0,c1,r0,r1=map(lambda v:int(round(v)),edges);cover[r0:r1,c0:c1]=True
        assert np.array_equal(cover,pixels!=254)
        route=js(folder/'common_bundle/common_route.json');points=route['points_xy'];length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:]));assert abs(length-route['route_length_m'])<1e-12 and points[0]==[-4,0] and points[-1]==[5,0]
        qc=js(folder/'geometry_qc.json')
        if row['family'] not in geometry:
            initial={}
            for name,px in [('spawn',-5.4),('gate',-4.),('goal',5.)]:
                parts=robot.parts(px,0,0);actor_initial=world_parts(text,[m.get('name') for m in ET.fromstring(text).findall('world/model') if m.get('name') not in actor_names]);allpairs=[]
                for kind,obs in [('static',static),('actors',actor_initial)]:
                    ds=[p.distance(q) for _,p,z in parts for _,q,zz in obs if not(z[1]<zz[0] or zz[1]<z[0])]
                    assert all(d>0 for d in ds);initial[name+'_'+kind+'_gap_m']=min(ds,default=None)
                initial[name+'_wheel_z']={n:z for n,p,z in parts if 'wheel_link' in n}
            sweeps=[]
            for actor in scene['actors']:
                model=ET.fromstring(text).find("world/model[@name='%s']"%actor['model_name']);collision=model.find('link/collision');shape=primitive(collision);assert shape['kind']=='box' and np.array_equal(shape['dims'],[.72,.72,1.])
                yaw=math.atan2(actor['end'][1]-actor['start'][1],actor['end'][0]-actor['start'][0]);polys=[]
                for xy in (actor['start'],actor['end']):polys.append(projected(shape,transform('%r %r .5 0 0 %r'%(*xy,yaw))))
                hull=unary_union(polys).convex_hull;dist=[hull.distance(p) for _,p,_ in static];assert all(d>0 for d in dist)
                sweeps.append({'actor':actor['model_name'],'complete_path_static_gap_m':min(dist,default=None),'intersection':False})
            witness,samples=curve_samples(robot,static,row['family']);original=qc['static_witness'];assert witness['static_pass']==original['static_geometric_witness_pass']
            if witness['sampled_min_physical_xy_m'] is not None:assert abs(witness['sampled_min_physical_xy_m']-original['sampled_min_physical_xy_m'])<1e-10
            detail={'family':row['family'],'initial':initial,'actor_sweeps':sweeps,'original_sin4':witness,'nominal_longitudinal_distance_m':9.,'actual_common_guide_length_m':length}
            if row['family']=='occluded_emergence':
                detail['supplement_quintic'],samples=curve_samples(robot,static,row['family'],True)
                actual=js(S/'audits/navigation_n04_scene_preparation/occlusion_static_witness_r1.json');assert abs(detail['supplement_quintic']['interval_physical_xy_lower_m']-actual['interval_physical_xy_lower_m'])<1e-10 and detail['supplement_quintic']['static_pass']
                actor=scene['actors'][0];shape=primitive(ET.fromstring(text).find("world/model[@name='%s']/link/collision"%actor['model_name']));target=projected(shape,transform('%r %r .5 0 0 %r'%(*actor['start'],math.atan2(actor['end'][1]-actor['start'][1],actor['end'][0]-actor['start'][0]))))
                detail['ray_visibility']={'gate':visibility((-4,0),target,static),'reveal':visibility((actor['conflict'][0]+.6,0),target,static)}
                assert detail['ray_visibility']['gate']['visible_count']==1 and detail['ray_visibility']['gate']['potential_count']==30 and detail['ray_visibility']['reveal']['visible_count']==60
            geometry[row['family']]=detail
        checks.append({'input_index':index,'seed':int(row['seed']),'parent_geometry_id':row['parent_geometry_id'],'family':row['family'],'actual_guide_length_m':length,'inheritance':True,'actors_absent_from_static_prior':True,'blocked_raster_equals_rectangles':True,'parent_policy_and_response_seeds_unchanged':True})
        print(json.dumps({'row_complete':index,'family':row['family']}),flush=True)
    # Same parent, same physical map and guide; policy/parameter replicas not new geometry.
    for i in range(0,12,2):
        for rel in ('map.pgm','map.yaml','common_route.json'):assert read(Path(rows[i]['common_bundle'])/rel)==read(Path(rows[i+1]['common_bundle'])/rel)
    assert len({x['parent_geometry_id'] for x in checks})==6
    summary={'schema':'astra_n04_independent_static_scene_recompute_v1','status':'independent_static_inputs_and_witness_recompute_pass','execution_started':False,'input_count':12,'old_unique_parent_geometries':6,'split_audit':split,'cross_split_population_materialized':False,'rows':checks,'geometry':geometry,'robot_radius_audit':robot.radius,'identity_positive_cases':binding_cases,'identity_negative_cases':negative_cases,'all_six_families_have_ideal_static_witness':all(g['original_sin4']['static_pass'] or g.get('supplement_quintic',{}).get('static_pass',False) for g in geometry.values()),'old_occlusion_sin4_negative_retained':not geometry['occluded_emergence']['original_sin4']['static_pass'],'source_resources':[{'path':p,'sha256':h}for p,h in sorted(inputs.items())]}
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in inputs.items())
    save('recompute_result.json',summary);print('PASS',len(inputs),'source bindings')

if __name__=='__main__':main()
