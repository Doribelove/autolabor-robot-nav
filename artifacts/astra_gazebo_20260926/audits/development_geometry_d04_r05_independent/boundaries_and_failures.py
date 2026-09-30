#!/usr/bin/env python3
"""Small API counterexamples and independent retained-negative reconstruction."""
import copy, hashlib, itertools, json, math, pathlib, sys, unittest
import numpy as np
from shapely.geometry import box
from shapely.ops import unary_union
import recompute as ref
sys.path.insert(0,str(ref.S/'workspace/scripts/astra_gazebo'))
import prepare_development_geometry_d04_r05 as subject

layouts=ref.js(ref.O/'layouts.json')
dynamic=next(s for s in layouts if len(s['actors'])==2)
cells=[a['actual_cell'] for a in dynamic['actors']]

class InterfaceBoundaries(unittest.TestCase):
    def test_exact_half_open_boundaries_first16_not_full_hash(self):
        wanted={0,59,60,79,80,89,90,99};seen={}
        for i in range(2000):
            parent='fixture_parent_'+str(i);row=ref.joint(parent,'actor_none_v1')
            if row['bucket'] in wanted:seen.setdefault(row['bucket'],(parent,row))
        self.assertEqual(set(seen),wanted)
        for parent,row in seen.values():self.assertEqual(subject.joint(parent,'actor_none_v1'),row)
        self.assertTrue(any(int(row['sha256'],16)%100!=row['bucket'] for _,row in seen.values()))

    def test_actual_role_order_is_bound(self):
        self.assertEqual(subject.tuple_cell(cells),ref.cell_tuple(cells))
        self.assertNotEqual(subject.tuple_cell(cells),subject.tuple_cell(cells[::-1]))

    def test_new_tuple_cannot_bypass_prior_atomic_owner(self):
        p=dynamic['parent_geometry_id'];key=ref.cell_id(cells[1])
        owner=subject.Ownership({p:'development'},{key:'validation'})
        self.assertEqual(owner.reject(p,cells)[0],'prior_atomic_actor_cell_split_ownership')
        with self.assertRaisesRegex(ValueError,'prior_atomic'):owner.commit(p,cells)
        self.assertEqual(owner.cells,{key:'validation'})

    def test_development_joint_cannot_relabel_parent(self):
        p=dynamic['parent_geometry_id'];owner=subject.Ownership({p:'train'})
        self.assertEqual(owner.reject(p,cells)[0],'prior_parent_split_ownership')
        with self.assertRaisesRegex(ValueError,'prior_parent'):owner.commit(p,cells)
        self.assertEqual(owner.parents,{p:'train'})

    def test_zero_actor_sentinel_fixed_and_not_atomic_owner(self):
        self.assertEqual(subject.tuple_cell([]),'actor_none_v1')
        scene=next(s for s in layouts if not s['actors']);owner=subject.Ownership()
        owner.commit(scene['parent_geometry_id'],[]);self.assertEqual(owner.cells,{})

    def test_exact_cell_schema_bool_and_bounds(self):
        original=cells[0]
        for key,value in [('speed_cell',True),('speed_cell',12),('speed_cell',23),('delay_cell',-1),('delay_cell',6),('margin_cell',9.),('policy','other')]:
            c=dict(original);c[key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):subject.cell_id(c)
        c=dict(original,unused=1)
        with self.assertRaises(ValueError):subject.cell_id(c)

    def test_payload_collision_delimiters_rejected(self):
        for value in ('','bad|field','bad\nfield',None,False):
            with self.subTest(value=value),self.assertRaises(ValueError):subject.joint(value,'actor_none_v1')

    def test_reflection_cosmetic_variants_not_new_parents(self):
        scene=copy.deepcopy(layouts[0]);parent=ref.digest(ref.canonical(scene,True))
        scene['layout_id']='cosmetic_only';scene['geometry_rng_seed']=999
        for b in scene['static_boxes']:b['y']=-b['y'];b['name']='renamed'
        self.assertEqual(ref.digest(ref.canonical(scene,True)),parent)

def bx(name,x,y,l,w,h=1.2):
    return dict(name=name,x=round(x,4),y=round(y,4),length=round(l,4),width=round(w,4),height=h,yaw_rad=0.)

def candidate(family,ordinal):
    if family=='static_avoidance':
        row=list(itertools.product((-1.4,-.8,-.2,.4,1.,1.6,2.2),(-.5,0.,.5),(.55,.95,1.35),(.5,.8,1.05),(2.5,2.9,3.2)))[ordinal]
        x,y,l,w,half=row;boxes=[bx('block_1',x,y,l,w)]
    else:
        rows=[r for r in itertools.product(('symmetric_throat','offset_throat','two_stage_throat'),(1.8,2.2,2.6),(.9,1.2,1.5),(-.5,0.,.5),(1.4,2.4,3.4),(-1.6,-.4,.8,2.)) if r[0]!='symmetric_throat' or r[3]==0.]
        topology,wide,narrow,shift,length,center=rows[ordinal];half=None;boxes=[]
        for sign in (-1,1):
            boxes += [bx('entry_'+str(sign),-3.5,sign*(wide+.1),3.,.2,1.),bx('throat_'+str(sign),center,shift+sign*(narrow+.1),length,.2,1.),bx('exit_'+str(sign),4.5,sign*(wide+.1),2.5,.2,1.)]
            if topology=='two_stage_throat':boxes += [bx('second_throat_'+str(sign),-1.9,-shift+sign*(narrow+.3),.8,.2,1.)]
    return dict(family=family,corridor_half_width_m=half,static_boxes=boxes,static_circles=[],actor_geometry=[])

def failures():
    priors=sum([ref.js(ref.S/'scenes'/c/'layouts.json') for c in ('development_a01','development_a02','development_d04_geometry_r04')],[])
    events=[json.loads(line) for line in ref.read(ref.A/'generation_attempt01.log').splitlines() if line.startswith('{"accepted":')]
    byid={s['layout_id']:s for s in layouts};env=ref.js(ref.S/'audits/development_geometry_d04/r04/envelope_reference.json');vertices=np.asarray(env['vertices']);rows=[]
    for record in ref.js(ref.A/'rejected_geometry_and_raster.json'):
        scene=candidate(record['family'],record['ordinal']);parent='geom_'+ref.digest(ref.canonical(scene,True))[:24]
        assert parent==record['parent_geometry_id']
        if record['reason']=='registered_or_selected_structural_near_duplicate':
            earlier=[byid[e['accepted']] for e in events if e['attempt']<record['attempt']]
            matches=[dict(parent=s['parent_geometry_id'],difference_cells=d) for s in priors+earlier if (d:=ref.separation(scene,s)) is not None and d<2.]
            assert matches
            rows.append(dict(attempt=record['attempt'],reason=record['reason'],matches=matches))
        else:
            assert ref.canonical(scene)==ref.canonical(record['geometry'])
            blocked=unary_union([box(*r) for r in ref.static_rectangles(scene)]);result=[]
            for profile in record['witness']['profile_attempts']:
                pts,yp,angle,third=ref.profile_rows(profile);count=profile['sample_count_checked']
                sampled=min(ref.placed(vertices,row[2:5]).distance(blocked) for row in pts[:count])
                lipschitz=math.hypot(1,yp)+(env['base_radius_m']+.325)*angle
                lower=sampled-lipschitz*.005-.33
                assert ref.near(lower,profile['continuous_conservative_lower_m']) and lower<=0.
                result.append(dict(curve=profile['curve'],amplitude_m=profile['amplitude_m'],peak_rear_x_m=profile['peak_rear_x_m'],sample_count=count,negative_lower_m=lower))
            rows.append(dict(attempt=record['attempt'],reason=record['reason'],finite_profiles=result))
    for scene in layouts:
        if not scene['parent_geometry_retained_from_r04']:
            reconstructed=candidate(scene['family'],scene['geometry_enumeration']['family_grid_ordinal'])
            assert ref.canonical(scene)==ref.canonical(reconstructed)
    return rows

if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(InterfaceBoundaries)
    tests=unittest.TextTestRunner(verbosity=2).run(suite)
    result=dict(schema='astra_d04_r05_independent_boundaries_v1',tests_run=tests.testsRun,api_tests_pass=tests.wasSuccessful())
    try:result.update(retained_failures=failures(),negative_recompute_pass=True)
    except Exception as e:
        import traceback
        result.update(negative_recompute_pass=False,exception=type(e).__name__+':'+str(e),traceback=traceback.format_exc())
    result['resources']=[dict(path=p,sha256=s) for p,s in sorted(ref.READ.items())]
    p=ref.D/sys.argv[1]
    with p.open('x') as f:json.dump(result,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
    print(json.dumps(dict(tests_run=result['tests_run'],api_tests_pass=result['api_tests_pass'],negative_recompute_pass=result['negative_recompute_pass'],exception=result.get('exception'))))
    sys.exit(0 if result['api_tests_pass'] and result['negative_recompute_pass'] else 1)
