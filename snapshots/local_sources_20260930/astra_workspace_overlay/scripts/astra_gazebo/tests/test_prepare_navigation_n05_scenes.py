"""Light source/metadata tests only; no geometry witness/map/matrix generation."""
from pathlib import Path
import ast,copy,csv,json,sys,tempfile,unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import prepare_navigation_n04_scenes as old
import prepare_navigation_n05_scenes as n
import event_actor_contract_n05 as event

ROWS=list(csv.DictReader((n.STAGE/'scenes/development_a02_live_v2/diagnostic_blocks.csv').open()))
def parent(index):
    row=ROWS[index]
    return yaml.safe_load(Path(row['manifest_path']).read_text()),Path(row['world_path']).read_text()

def normalize_new_world(world,newseed,oldseed):
    return world.replace('libastra_event_actor_n05_plugin.so',old.PLUGIN).replace('UNBOUND_N05','UNBOUND_N04').replace(str(newseed),str(oldseed))


class DerivationTests(unittest.TestCase):
    def test_twelve_actual_parent_xml_derivations_geometry_policy_equal(self):
        for i in range(12):
            with self.subTest(index=i):
                p,raw=parent(i);saved=copy.deepcopy(p)
                control,cw=old.derive(p,raw,263001001+i);new,nw=n.derive(p,raw,263002001+i)
                self.assertEqual(p,saved);self.assertEqual(n.collision_signature(raw),n.collision_signature(nw))
                self.assertEqual(normalize_new_world(nw,263002001+i,263001001+i),cw)
                norm=json.loads(json.dumps(new).replace('n05','n04').replace('N05','N04'))
                norm['seed']=control['seed'];norm.pop('paired_method_roster');norm.pop('derivation_design_sha256')
                self.assertEqual(norm,control)
    def test_each_declared_pair_metadata_binds_reviewed_actor_schema(self):
        for block in range(12):
            p,raw=parent(block);scene,world=n.derive(p,raw,263002001+block)
            for row in n.load_design()['schedule'][2*block:2*block+2]:
                bound=n.bind_gate_identity(world,row['run_id'],row['seed'],row['method']);actors=event.world_actors(bound)
                self.assertEqual(set(actors),{a['model_name'] for a in scene['actors']})
                self.assertTrue(all(a['method']==row['method'] and a['seed']==row['seed'] and a['max_gate_transport_delay_ns']==50_000_000 for a in actors.values()))
                self.assertEqual(n.collision_signature(bound),n.collision_signature(world))
    def test_actor_static_omission_matches_old_by_geometry(self):
        for i in [2,6,8]:
            p,raw=parent(i);s,w=n.derive(p,raw,263002001+i);xml,ss,omitted=n.static_input(w,s)
            self.assertFalse(ss['actors']);self.assertEqual(len(omitted),len(s['actors']))
            self.assertFalse({a['model_name'] for a in s['actors']} & {m.get('name') for m in ET.fromstring(xml).findall('world/model')})
    def test_parent_identity_mismatch_rejected(self):
        p,w=parent(0)
        for key,value in [('replicate',True),('family','wrong'),('layout_id','wrong'),('parent_geometry_id','wrong'),('scene_id','wrong')]:
            q=copy.deepcopy(p);q[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):n.derive(q,w,263002001)
    def test_arbitrary_route_transform_still_rejected(self):
        p,w=parent(0);p['route']['start'][2]=.1
        with self.assertRaises(ValueError):n.derive(p,w,263002001)
    def test_unknown_or_old_seed_rejected(self):
        p,w=parent(0)
        for seed in [263001001,263002013,True,263002001.]:
            with self.subTest(seed=seed),self.assertRaises(ValueError):n.derive(p,w,seed)
    def test_wrong_policy_or_actor_path_not_silently_changed(self):
        p,w=parent(2)
        for key,value in [('policy','not_existing'),('start',[123,456])]:
            q=copy.deepcopy(p);q['actors'][0][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):n.derive(q,w,263002003)
    def test_exact_schedule_binds_actorless_world_too(self):
        p,w=parent(0);s,world=n.derive(p,w,263002001);self.assertFalse(s['actors'])
        for rid,seed,method in [('arbitrary',263002001,n.METHODS[0]),('astra_navigation_n05_b00_'+n.METHODS[0],263002002,n.METHODS[0]),('astra_navigation_n05_b00_actual_teb_common_route',263002001,'actual_teb_common_route')]:
            with self.subTest(rid=rid),self.assertRaises(ValueError):n.bind_gate_identity(world,rid,seed,method)
    def test_actor_world_second_binding_rejected(self):
        p,w=parent(2);_,world=n.derive(p,w,263002003);rows=n.load_design()['schedule'][4:6]
        bound=n.bind_gate_identity(world,rows[0]['run_id'],rows[0]['seed'],rows[0]['method'])
        with self.assertRaises(ValueError):n.bind_gate_identity(bound,rows[1]['run_id'],rows[1]['seed'],rows[1]['method'])
    def test_geometry_qc_functions_ast_identical_to_frozen_n04(self):
        def nodes(path):return {x.name:ast.dump(x,include_attributes=False) for x in ast.parse(path.read_text()).body if isinstance(x,(ast.ClassDef,ast.FunctionDef))}
        a=nodes(Path(old.__file__));b=nodes(Path(n.__file__))
        for name in ['put','collision_signature','ReferenceRobot','reduced_clearance','witness','scene_qc','static_input']:
            with self.subTest(name=name):self.assertEqual(a[name],b[name])
        self.assertEqual(n.sha(old.__file__),n.PARENT_PREPARER_SHA256)


class ContractGuards(unittest.TestCase):
    def test_actual_frozen_design_exact(self):
        x=n.load_design();self.assertEqual(len(x['schedule']),24);self.assertEqual(n.METHODS,tuple(x['methods']))
    def test_schedule_arm_seed_order_types_and_capacity_rejected(self):
        for key,value in [('seed',True),('seed',263002002),('method',n.METHODS[1]),('run_id','wrong'),('replicate',1),('method_order_in_block',2),('attempt',2)]:
            x=copy.deepcopy(n.load_design());x['schedule'][0][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):n.validate_design_data(x)
        for key,value in [('execution_authorized',True),('allocated_budget',False),('proposed_maximum_episodes',25),('methods',['actual_teb_common_route'])]:
            x=copy.deepcopy(n.load_design());x[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):n.validate_design_data(x)
    def test_design_bytes_changed_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'design.json';p.write_bytes(n.DESIGN.read_bytes()+b' ')
            with self.assertRaisesRegex(ValueError,'SHA'):n.load_design(p)
    def test_seed_collision_stops_before_geometry_and_no_output(self):
        with tempfile.TemporaryDirectory() as td:
            d=Path(td);reg=d/'registry.csv';reg.write_text('seed\n263002001\n');before=reg.read_bytes();out=d/'output'
            with patch.object(n,'ReferenceRobot',side_effect=AssertionError('geometry must not run')):
                with self.assertRaisesRegex(ValueError,'already in registry'):n.build(n.STAGE/'scenes/development_a02_live_v2',out,reg)
            self.assertEqual(reg.read_bytes(),before);self.assertFalse(out.exists())
    def test_existing_output_not_overwritten(self):
        with tempfile.TemporaryDirectory() as td:
            d=Path(td);out=d/'output';out.mkdir();f=out/'keep';f.write_text('original')
            with self.assertRaises(FileExistsError):n.build(d,out,d/'absent_registry')
            self.assertEqual(f.read_text(),'original')

if __name__=='__main__':unittest.main()
