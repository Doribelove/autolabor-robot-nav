#!/usr/bin/env python3
"""Read-only independent checks; no production generator/parser/geometry imports."""
import collections, copy, csv, hashlib, itertools, json, math, pathlib, sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import shapely
from shapely.geometry import Polygon, MultiPoint, box
from shapely.ops import unary_union

S = pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926')
A = S/'audits/development_geometry_d04_r05'
O = S/'scenes/development_d04_geometry_r05'
D = pathlib.Path(__file__).parent
READ = {}
CHECKS = []

def read(p, binary=False):
    p = pathlib.Path(p).resolve(); data = p.read_bytes()
    READ[str(p)] = hashlib.sha256(data).hexdigest()
    return data if binary else data.decode()

def js(p): return json.loads(read(p))
def encoded(x): return json.dumps(x,sort_keys=True,separators=(',',':'),ensure_ascii=False).encode()
def digest(x): return hashlib.sha256(encoded(x)).hexdigest()
def check(name, value, details=None):
    CHECKS.append(dict(name=name,passed=bool(value),details=details))
    if not value: raise AssertionError((name,details))
def near(a,b,tol=2e-11): return abs(a-b)<=tol

def bindings(obj):
    if isinstance(obj,dict):
        if isinstance(obj.get('path'),str) and isinstance(obj.get('sha256'),str): yield obj
        for value in obj.values(): yield from bindings(value)
    elif isinstance(obj,list):
        for value in obj: yield from bindings(value)

def canonical(scene, quantized=False):
    def n(v,step): return int(math.floor(float(v)/step+.5)) if quantized else round(float(v),4)
    versions=[]
    for sign in (1,-1):
        versions.append(dict(family=scene['family'],route_length_m=9.0,
            wall_half_width=None if scene['corridor_half_width_m'] is None else n(scene['corridor_half_width_m'],.2),
            boxes=sorted([[n(b['x'],.2),n(sign*b['y'],.2),n(b['length'],.1),n(b['width'],.1),n(b['height'],.1)] for b in scene['static_boxes']]),
            actor_path_points=sorted([[n(a[k][0],.2),n(sign*a[k][1],.2)] for a in scene['actor_geometry'] for k in ('start','end','conflict')])))
    return min(versions,key=encoded)

def separation(a,b):
    a,b=canonical(a),canonical(b)
    if a['family']!=b['family'] or len(a['boxes'])!=len(b['boxes']) or len(a['actor_path_points'])!=len(b['actor_path_points']) or (a['wall_half_width'] is None)!=(b['wall_half_width'] is None): return None
    diffs=[]
    if a['wall_half_width'] is not None: diffs.append(abs(a['wall_half_width']-b['wall_half_width'])/.2)
    diffs += [abs(u-v)/s for p,q in zip(a['boxes'],b['boxes']) for u,v,s in zip(p,q,(.2,.2,.1,.1,.1))]
    diffs += [abs(u-v)/.2 for p,q in zip(a['actor_path_points'],b['actor_path_points']) for u,v in zip(p,q)]
    return max(diffs,default=0.)

def cell_id(c): return 'actor_'+digest(c)[:24]
def cell_tuple(cells):
    if not cells: return 'actor_none_v1'
    if len(cells)==1: return cell_id(cells[0])
    return 'actor_tuple_v1_'+digest([[f'benchmark_actor_{i+1}',cell_id(c)] for i,c in enumerate(cells)])
def joint(p,c):
    payload=f'astra_gazebo_20260926|{p}|{c}|allocation_v1'
    sha=hashlib.sha256(payload.encode()).hexdigest(); bucket=int(sha[:16],16)%100
    return dict(payload_utf8=payload,payload_utf8_hex=payload.encode().hex(),sha256=sha,bucket=bucket,
        split=next(s for lo,hi,s in [(0,60,'train'),(60,80,'development'),(80,90,'validation'),(90,100,'final_test')] if lo<=bucket<hi))

def static_rectangles(scene):
    rectangles=[(b['x']-b['length']/2,b['y']-b['width']/2,b['x']+b['length']/2,b['y']+b['width']/2) for b in scene['static_boxes']]
    h=scene['corridor_half_width_m']
    if h is not None:
        for s in (-1,1): rectangles.append((-9,s*(h+.1)-.1,9,s*(h+.1)+.1))
    return rectangles

def pgm(p):
    data=read(p,True); magic,shape,maxv,pixels=data.split(b'\n',3)
    assert magic==b'P5' and maxv==b'255'; width,height=map(int,shape.split())
    return np.frombuffer(pixels,np.uint8).reshape(height,width)[::-1].copy()

def pixel_union(data,origin,res):
    # Independently extract each contiguous blocked horizontal run.
    rectangles=[]
    for iy,row in enumerate(data!=254):
        edges=np.flatnonzero(np.diff(np.r_[False,row,False].astype(np.int8)))
        for left,right in zip(edges[::2],edges[1::2]):
            rectangles.append(box(origin[0]+left*res,origin[1]+iy*res,origin[0]+right*res,origin[1]+(iy+1)*res))
    return unary_union(rectangles)

def placed(vertices,pose):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    return Polygon(vertices @ np.array([[c,s],[-s,c]])+np.array([x,y]))

def profile_rows(selected):
    x=np.linspace(-4.325,4.675,901);a=selected['amplitude_m'];peak=selected['peak_rear_x_m']
    if peak is None:
        k=math.pi/9;t=k*(x+4.325);sn=np.sin(t);cs=np.cos(t)
        y=a*sn**4;dy=4*a*k*sn**3*cs;ddy=4*a*k*k*(3*sn*sn*cs*cs-sn**4)
        yp=4*abs(a)*k;angle=4*abs(a)*k*k;third=40*abs(a)*k**3
    else:
        before=x<=peak;length=np.where(before,peak+4.325,4.675-peak);u=np.where(before,(x+4.325)/length,(4.675-x)/length)
        y=a*(10*u**3-15*u**4+6*u**5)
        dy=a*(30*u**2-60*u**3+30*u**4)/length*np.where(before,1.,-1.)
        ddy=a*(60*u-180*u**2+120*u**3)/(length*length)
        length=min(peak+4.325,4.675-peak);yp=1.875*abs(a)/length;angle=6*abs(a)/length**2;third=60*abs(a)/length**3
    yaw=np.arctan(dy);rows=np.column_stack((x,y,x+.325*np.cos(yaw),y+.325*np.sin(yaw),yaw,ddy/(1+dy**2)**1.5))
    return rows,yp,angle,third

def witness(scene_dir, csv_path, selected, blocked, vertices, radius):
    actual=np.loadtxt(read(csv_path).splitlines(),delimiter=',',skiprows=1)
    expected,yp,angle,third=profile_rows(selected)
    assert actual.shape==(901,6);rowerr=float(np.max(abs(actual-expected)))
    if rowerr>1e-11: raise AssertionError(('witness poses',csv_path,rowerr))
    lipschitz=math.hypot(1,yp)+(radius+.325)*angle
    assert near(lipschitz,selected['point_lipschitz_per_rear_x'])
    sampled=min(placed(vertices,row[2:5]).distance(blocked) for row in actual)
    lower=sampled-.005*lipschitz-.33
    kp=third+3*yp*angle**2
    assert near(math.atan(.65*angle),selected['central_steering_abs_bound_rad'])
    assert near(.65*kp*selected['witness_rear_speed_mps'],selected['central_steering_rate_bound_radps'])
    assert selected['central_steering_abs_bound_rad']<=.4964 and selected['central_steering_rate_bound_radps']<=.75
    return dict(sample_count=901,max_pose_error=rowerr,sampled_physical_gap_m=sampled,point_lipschitz=lipschitz,interval_deduction_m=.005*lipschitz,conservative_lower_m=lower)

def run():
    author=js(A/'implementation_report.json');manifest=js(A/'source_evidence_manifest.json')
    resources=list(bindings(manifest));errors=[]
    for r in resources:
        read(r['path'],True)
        if READ[str(pathlib.Path(r['path']).resolve())]!=r['sha256']:errors.append(r)
    check('all_303_author_resource_bindings',len(resources)==303 and not errors,dict(count=len(resources),errors=errors))
    oldmanifest=js(S/'audits/development_geometry_d04/r04/source_evidence_manifest.json');oldresources=list(bindings(oldmanifest))
    for r in oldresources:
        read(r['path'],True)
        assert READ[str(pathlib.Path(r['path']).resolve())]==r['sha256'],r
    check('r04_all_151_resources_unchanged',len(oldresources)==151,len(oldresources))
    oldprior=js(S/'scenes/development_a01/layouts.json')+js(S/'scenes/development_a02/layouts.json')
    r04=js(S/'scenes/development_d04_geometry_r04/layouts.json');priors=oldprior+r04
    initial=js(A/'initial_ownership.json');final=js(A/'final_ownership.json');layouts=js(O/'layouts.json')
    initialparent={s['parent_geometry_id']:s['split'] for s in priors}
    check('initial_48_parent_owners_reconstructed',len(initialparent)==48 and initialparent==initial['parent_groups'])
    cells={p['parameter_cell_id']:'development' for s in r04 for p in s['actor_parameter_existence_witnesses']}
    for p in js(S/'audits/development_geometry_d04/r04/family_actor_timing_witnesses.json')['actors']: cells[p['selected_parameter_existence_witness']['parameter_cell_id']]='development'
    derived=[]
    byid={s['parent_geometry_id']:s for s in oldprior}
    for cohort in ('development_a01','development_a02','navigation_n04_development_r01','navigation_n05_development_r01'):
        rows=list(csv.DictReader(read(S/'scenes'/cohort/'diagnostic_blocks.csv').splitlines()))
        for row in rows:
            data=read(row['manifest_path']);assert hashlib.sha256(data.encode()).hexdigest()==row['manifest_sha256']
            scene=yaml.safe_load(data)
            for actor in scene['actors']:
                key=actor['parameter_cell_id']
                if cohort.startswith('navigation_'): assert initial['actual_actor_cells'][key]==scene['split']
                else:
                    assert key not in cells or cells[key]==scene['split'];cells[key]=scene['split']
            if cohort.startswith('navigation_'):
                raw=dict(scene,static_boxes=scene['parent_static_boxes'],static_circles=scene['parent_static_circles'],corridor_half_width_m=scene['parent_corridor_half_width_m'])
                assert canonical(raw)==canonical(byid[scene['parent_geometry_id']]);derived.append((cohort,scene['parent_geometry_id']))
    check('initial_78_actual_cell_owners_and_all_N04_N05_derived_inputs',cells==initial['actual_actor_cells'] and len(cells)==78,dict(cells=len(cells),derived_rows=len(derived)))
    selected=js(A/'selected_joint_combinations.json');selected_byid={r['layout_id']:r for r in selected}
    parent_owner=dict(initialparent);cell_owner=dict(cells);retained=[];new=[];pairs=[];allcells=[]
    for scene in layouts:
        assert js(O/scene['layout_id']/'geometry_manifest.json')==scene
        assert not scene['static_circles'] and all(b.get('yaw_rad',0.)==0. for b in scene['static_boxes'])
        assert scene['route']['start']==[-4.,0.,0.] and scene['route']['goal']==[5.,0.,0.] and scene['route']['spawn']==[-5.4,0.,0.] and scene['route']['length_m']==9.
        parent='geom_'+digest(canonical(scene,True))[:24]
        assert parent==scene['parent_geometry_id'] and digest(canonical(scene))==scene['canonical_geometry_sha256']
        assert scene['execution_seed'] is None and scene['execution_authorized'] is False and scene['new_seed_allocated'] is False
        actual=[a['actual_cell'] for a in scene['actors']];group=cell_tuple(actual);j=joint(parent,group)
        assert j==scene['joint_allocation']==selected_byid[scene['layout_id']]['joint'] and j['split']=='development'
        assert group==scene['actor_parameter_cell_id'] and parent_owner.get(parent,'development')=='development'
        for i,(actor,geometry,cell) in enumerate(zip(scene['actors'],scene['actor_geometry'],actual)):
            assert actor['model_name']==f'benchmark_actor_{i+1}' and all(actor[k]==geometry[k] for k in ('start','end','conflict'))
            key=cell_id(cell);assert key==actor['parameter_cell_id'] and key not in cell_owner
            assert actor['policy']==cell['policy'] and actor['base_speed_mps']==cell['speed_cell']/20 and actor['response_distance_m']==cell['response_distance_cell']/5 and actor['response_time_margin_s']==cell['margin_cell']/10 and actor['gate_delay_s']==cell['delay_cell']/5
            assert set(cell)=={'policy','speed_cell','response_distance_cell','margin_cell','delay_cell'}
            assert cell['policy'] in ('cautious','hesitant','assertive')
            for k,lo,hi in [('speed_cell',13,22),('response_distance_cell',15,25),('margin_cell',9,17),('delay_cell',0,5)]: assert type(cell[k]) is int and lo<=cell[k]<=hi
            delta=actor['gate_delay_s']+math.dist(actor['start'],actor['conflict'])/actor['base_speed_mps']-(actor['conflict'][0]+4)/.8
            assert abs(delta)<=2.
            if scene['family']=='sequential_interaction' and i:assert actor['gate_delay_s']>=.8
            cell_owner[key]='development';allcells.append(key)
        assert len(actual)==len(scene['actor_geometry'])
        if scene['parent_geometry_retained_from_r04']:
            old=next(s for s in r04 if s['layout_id']==scene['source_r04_layout_id'])
            assert old['parent_geometry_id']==parent
            # Exact geometry fields, including actor role order, not merely parent hash.
            for k in ('static_boxes','static_circles','corridor_half_width_m','actor_geometry','route','topology','geometry_rng_seed'): assert old[k]==scene[k],(scene['layout_id'],k)
            retained.append(parent)
        else:
            assert parent not in initialparent and scene['geometry_rng_seed'] is None;new.append(parent)
        parent_owner[parent]='development'
        for prior in priors+layouts:
            if prior['parent_geometry_id']==parent:continue
            gap=separation(scene,prior)
            if gap is not None:pairs.append(dict(left=parent,right=prior['parent_geometry_id'],separation_cells=gap));assert gap>=2.,pairs[-1]
    check('24_actual_joint_dev_rows_17_exact_retained_7_new',len(layouts)==24 and len(set(retained))==17 and len(new)==7,dict(retained=retained,new=new,family_counts=dict(collections.Counter(s['family'] for s in layouts))))
    check('actual_20_new_atomic_cells_final_ownership_no_relabel',len(allcells)==20 and parent_owner==final['parent_groups'] and cell_owner==final['actual_actor_cells'],dict(initial_cells=len(cells),new_cells=len(allcells),final_parents=len(parent_owner)))
    check('structural_two_cell_distinctness_all_comparable_pairs',all(p['separation_cells']>=2 for p in pairs),dict(comparable_pairs=len(pairs),minimum=min(p['separation_cells'] for p in pairs)))
    rejected=js(A/'rejected_joint_combinations.json')
    for r in rejected:
        group=cell_tuple(r.get('actual_cells',[]));j=joint(r['parent_geometry_id'],group)
        assert all(r[k]==v for k,v in j.items())
        assert r['reason']=='joint_hash_not_development' and j['split']!='development'
    failures=js(A/'rejected_geometry_and_raster.json')
    for r in failures:
        if r['reason']=='full_geometry_or_native_witness_failed':assert not r['witness']['passed'] and all(not p['passed'] for p in r['witness']['profile_attempts'])
    check('150_rejected_joint_combinations_recomputed_12_geometry_failures_retained',len(rejected)==150 and len(failures)==12,dict(joint_splits=dict(collections.Counter(r['split'] for r in rejected)),geometry_reasons=dict(collections.Counter(r['reason'] for r in failures))))
    r04static=[p for p in r04 if p['family']=='static_avoidance']
    check('prior_static_owners_unchanged_despite_nondev_sentinel',all(joint(p['parent_geometry_id'],'actor_none_v1')['split']!='development' and initialparent[p['parent_geometry_id']]=='development' and final['parent_groups'][p['parent_geometry_id']]=='development' for p in r04static),[joint(p['parent_geometry_id'],'actor_none_v1')['bucket'] for p in r04static])
    env=js(S/'audits/development_geometry_d04/r04/envelope_reference.json');vertices=np.asarray(env['vertices']);radius=max(np.linalg.norm(vertices,axis=1))
    assert near(radius,env['base_radius_m']);statics=[]
    for scene in layouts:
        if scene['family']!='static_avoidance':continue
        folder=O/scene['layout_id'];raster=folder/'raster_and_guide';saved=js(raster/'raster_guide_checks.json')
        data=pgm(raster/'map_export/map.pgm');meta=yaml.safe_load(read(raster/'map_export/map.yaml'));h,w=data.shape;origin=meta['origin'];res=meta['resolution']
        assert (h,w)==(600,800) and res==.05 and origin==[-20.,-15.,0.]
        assert read(raster/'map_export/map.pgm',True)==read(raster/'common_bundle/map.pgm',True)
        assert read(raster/'map_export/map.yaml',True)==read(raster/'common_bundle/map.yaml',True)
        # Closed cell/axis-aligned physical rectangle intersection, independent of exporter and GEOS.
        left=origin[0]+np.arange(w)*res;bottom=origin[1]+np.arange(h)*res
        expected=np.full((h,w),254,np.uint8)
        rects=static_rectangles(scene)
        for x0,y0,x1,y1 in rects:
            hitx=(left<=x1)&(left+res>=x0);hity=(bottom<=y1)&(bottom+res>=y0)
            expected[np.ix_(hity,hitx)]=0
        expected[[0,-1],:]=205;expected[:,[0,-1]]=205
        mismatches=np.argwhere(data!=expected)
        # Track roundoff-sensitive exact-touch cells separately, do not silently relax interior coverage.
        touch_only=[];unexplained=[]
        for iy,ix in mismatches:
            px0=left[ix];py0=bottom[iy];px1=px0+res;py1=py0+res
            area_hit=any(min(px1,x1)-max(px0,x0)>1e-12 and min(py1,y1)-max(py0,y0)>1e-12 for x0,y0,x1,y1 in rects)
            close_touch=any(min(px1,x1)-max(px0,x0)>=-1e-12 and min(py1,y1)-max(py0,y0)>=-1e-12 for x0,y0,x1,y1 in rects)
            row=[int(iy),int(ix),int(data[iy,ix]),int(expected[iy,ix])]
            if not area_hit and close_touch:touch_only.append(row)
            else:unexplained.append(row)
        assert not unexplained,unexplained[:5]
        blocked=pixel_union(data,origin,res);native=unary_union([box(*r) for r in rects]);uncovered=native.difference(blocked.buffer(1e-12)).area
        assert uncovered<1e-10
        xml=ET.fromstring(read(raster/'static_geometry.sdf'));models=xml.findall('world/model');assert len(models)==len(rects) and not xml.findall('.//plugin')
        for model,rect in zip(models,rects):
            pos=list(map(float,model.findtext('pose').split()));size=list(map(float,model.findtext('link/collision/geometry/box/size').split()))
            assert all(near(x,y) for x,y in zip((pos[0]-size[0]/2,pos[1]-size[1]/2,pos[0]+size[0]/2,pos[1]+size[1]/2),rect))
            assert pos[3:]==[0.,0.,0.]
        native_selection=js(folder/'reference_witness.json')['selected'];native_result=witness(folder,folder/'reference_witness.csv',native_selection,blocked,vertices,radius)
        assert near(native_result['conservative_lower_m'],saved['original_native_witness_conservative_lower_after_raster_m'])
        selected=saved['raster_witness']['selected'];result=witness(folder,raster/'raster_witness.csv',selected,blocked,vertices,radius)
        assert near(result['conservative_lower_m'],selected['continuous_conservative_lower_m']) and result['conservative_lower_m']>0
        route=js(raster/'common_bundle/common_route.json');assert route['start_xy_yaw']==scene['route']['start'] and route['goal_xy_yaw']==scene['route']['goal'] and route['map_from_odom_xy_yaw']==scene['route']['spawn']
        for key,p in [('map_image_sha256',raster/'common_bundle/map.pgm'),('map_yaml_sha256',raster/'common_bundle/map.yaml'),('route_request_sha256',raster/'map_export/route_request.json')]:read(p,True);assert route[key]==READ[str(p.resolve())]
        pts=np.asarray(route['points_xy']);headings=np.arctan2(np.diff(pts[:,1]),np.diff(pts[:,0]));segments=[];corners=[]
        assert np.array_equal(pts[0],[-4.,0.]) and np.array_equal(pts[-1],[5.,0.])
        for p,q,yaw in zip(pts[:-1],pts[1:],headings):
            coords=list(placed(vertices,(*p,yaw)).exterior.coords)+list(placed(vertices,(*q,yaw)).exterior.coords)
            segments.append(MultiPoint(coords).convex_hull.distance(blocked)-.33)
        for i,p in enumerate(pts):
            before=0. if i==0 else headings[i-1];after=0. if i==len(pts)-1 else headings[i]
            if abs(math.atan2(math.sin(after-before),math.cos(after-before)))<1e-10:continue
            # Explicit outward 128-gon, no buffer call shared with production guide check.
            angles=np.arange(128)*(-2*math.pi/128);r=radius/math.cos(math.pi/128)
            circle=Polygon(np.column_stack((p[0]+r*np.cos(angles),p[1]+r*np.sin(angles))))
            corners.append(dict(point_index=i,conservative_gap_m=circle.distance(blocked)-.33))
        assert len(segments)==len(saved['guide']['segments']) and len(corners)==len(saved['guide']['heading_transitions'])
        assert all(near(v,r['conservative_gap_m']) for v,r in zip(segments,saved['guide']['segments']))
        assert all(v['point_index']==r['point_index'] and near(v['conservative_gap_m'],r['conservative_gap_m']) for v,r in zip(corners,saved['guide']['heading_transitions']))
        guide_min=min(segments+[r['conservative_gap_m'] for r in corners]);assert guide_min>0 and near(guide_min,saved['guide']['minimum_full_footprint_conservative_gap_m'])
        statics.append(dict(layout_id=scene['layout_id'],parent=scene['parent_geometry_id'],raster_cells=int(data.size),raster_unexplained_mismatches=unexplained,raster_fp_exact_touch_discrepancies=touch_only,native_uncovered_area_m2=uncovered,native_witness_after_raster=native_result,selected_raster_witness=result,guide_segments=len(segments),guide_corners=len(corners),guide_conservative_lower_m=guide_min,guide_ackermann_corner_feasibility=False))
    check('four_static_raster_pixels_witnesses_and_all_guide_segments_corners',len(statics)==4,statics)
    check('negative_native_raster_slot001_retained',statics[1]['native_witness_after_raster']['conservative_lower_m']<0 and statics[1]['selected_raster_witness']['conservative_lower_m']>0)
    return dict(schema='astra_d04_r05_independent_recompute_v1',passed=True,checks=CHECKS,statics=statics,counts=dict(author_bindings=len(resources),old_bindings=len(oldresources),parent_groups=24,retained=17,new=7,new_atomic_cells=20,joint_rejections=150,geometry_rejections=12),environment=dict(executable=sys.executable,python=sys.version,numpy=np.__version__,numpy_file=np.__file__,shapely=shapely.__version__,shapely_file=shapely.__file__,yaml=yaml.__version__),resources=[dict(path=p,sha256=s) for p,s in sorted(READ.items())])

if __name__=='__main__':
    try: result=run()
    except Exception as exc:
        import traceback
        result=dict(passed=False,exception=type(exc).__name__+':'+str(exc),traceback=traceback.format_exc(),checks=CHECKS,resources=[dict(path=p,sha256=s) for p,s in sorted(READ.items())])
    output=D/sys.argv[1]
    with output.open('x') as f: json.dump(result,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
    print(json.dumps(dict(passed=result['passed'],output=str(output),checks=len(CHECKS),exception=result.get('exception'))))
    sys.exit(0 if result['passed'] else 1)
