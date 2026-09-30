import ast,copy,hashlib,json,math,sys,unittest
from pathlib import Path
from unittest.mock import patch
from shapely.geometry import Polygon,box
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
import prepare_development_geometry_d04_r05 as d
from common_route import CommonRoute

def cell(speed=13):return {'policy':'cautious','speed_cell':speed,'response_distance_cell':15,'margin_cell':9,'delay_cell':0}
def dev_parent(cells):
    for i in range(1000):
        p='geom_fixture_%03d'%i
        if d.joint(p,d.tuple_cell(cells))['split']=='development':return p
    raise AssertionError('fixture support')

class JointAllocationTests(unittest.TestCase):
    def test_closed_percentage_bins_from_first16_only(self):
        for bucket,split in [(0,'train'),(59,'train'),(60,'development'),(79,'development'),(80,'validation'),(89,'validation'),(90,'final_test'),(99,'final_test')]:
            digest=('%016x'%bucket)+'f'*48
            class FixedHash:
                def hexdigest(self):return digest
            with patch.object(d.hashlib,'sha256',return_value=FixedHash()):
                r=d.joint('geom_fixture','actor_none_v1')
            self.assertEqual(r['bucket'],bucket);self.assertEqual(r['split'],split)
    def test_fixed_pipe_utf8_vector_and_existing_r04_negative(self):
        parent='geom_ec93836d6b248dad518f908b';r=d.joint(parent,'actor_none_v1')
        raw=b'astra_gazebo_20260926|geom_ec93836d6b248dad518f908b|actor_none_v1|allocation_v1'
        self.assertEqual(r['payload_utf8_hex'],raw.hex());self.assertEqual(r['sha256'],hashlib.sha256(raw).hexdigest());self.assertEqual(r['bucket'],35);self.assertEqual(r['split'],'train')
    def test_delimiter_and_newline_rejected(self):
        for value in ['x|y','x\ny','',None]:
            with self.assertRaises(ValueError):d.joint(value,'actor_none_v1')
    def test_zero_actor_sentinel_and_no_unused_parameters(self):
        self.assertEqual(d.tuple_cell([]),'actor_none_v1');self.assertNotEqual(d.tuple_cell([cell()]),'actor_none_v1')
    def test_actor_single_cell_matches_frozen_canonical_id(self):
        self.assertEqual(d.cell_id(cell()),'actor_'+d.old.v1.digest(cell())[:24]);self.assertEqual(d.tuple_cell([cell()]),d.cell_id(cell()))
    def test_actual_tuple_role_order_changes_identity(self):
        a,b=cell(),cell(14);self.assertNotEqual(d.tuple_cell([a,b]),d.tuple_cell([b,a]))
        raw=json.dumps([['benchmark_actor_1',d.cell_id(a)],['benchmark_actor_2',d.cell_id(b)]],separators=(',',':')).encode()
        self.assertEqual(d.tuple_cell([a,b]),'actor_tuple_v1_'+hashlib.sha256(raw).hexdigest())
    def test_bool_out_of_range_and_extra_cell_keys_rejected(self):
        for change in ({'speed_cell':True},{'speed_cell':12},{'unused_seed':42}):
            c=cell();c.update(change)
            with self.assertRaises(ValueError):d.cell_id(c)
    def test_parent_ownership_cannot_be_overridden_by_joint(self):
        p=dev_parent([cell()]);o=d.Ownership({p:'train'});self.assertEqual(o.reject(p,[cell()])[0],'prior_parent_split_ownership')
        with self.assertRaises(ValueError):o.commit(p,[cell()])
        self.assertEqual(o.parents[p],'train')
    def test_atomic_cell_ownership_not_bypassed_by_tuple(self):
        cells=[cell(),cell(14)];p=dev_parent(cells);o=d.Ownership(cells={d.cell_id(cells[1]):'validation'})
        self.assertEqual(o.reject(p,cells)[0],'prior_atomic_actor_cell_split_ownership')
    def test_commit_development_and_reject_later_nondev_combination(self):
        cells=[cell()];p=dev_parent(cells);o=d.Ownership();o.commit(p,cells)
        self.assertEqual(o.parents[p],'development');self.assertEqual(o.cells[d.cell_id(cells[0])],'development')
        for speed in range(14,23):
            c=cell(speed)
            if d.joint(p,d.tuple_cell([c]))['split']!='development':
                with self.assertRaises(ValueError):o.commit(p,[c])
                self.assertEqual(o.parents[p],'development');break
        else:self.fail('negative fixture absent')
    def test_seedless_source_and_registered_design(self):
        self.assertEqual(d.sha(d.DESIGN),d.DESIGN_SHA)
        tree=ast.parse(Path(d.__file__).read_text())
        calls=[ast.dump(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)]
        self.assertFalse(any('Random' in c or 'SeedGuard' in c for c in calls))
    def test_finite_static_enumeration_native_contract_ranges(self):
        rows=list(d.new_geometries('static_avoidance'));self.assertEqual(len(rows),567)
        for s in rows:
            self.assertFalse(s['actor_geometry']);b=s['static_boxes'][0]
            self.assertTrue(2<=b['x']+4<=6.7);self.assertTrue(4.8<=2*s['corridor_half_width_m']<=6.5)
    def test_same_actual_geometry_not_relabelled_as_new(self):
        s=next(d.new_geometries('static_avoidance'));q=copy.deepcopy(s);q['layout_id']='cosmetic';q['split']='development'
        self.assertEqual(d.old.v1.parent_id(s),d.old.v1.parent_id(q))

class GuideSweepTests(unittest.TestCase):
    class Envelope:
        radius=math.sqrt(.02)
        def at(self,p):
            x,y,theta=p;c,s=math.cos(theta),math.sin(theta)
            return Polygon([(x+c*a-s*b,y+s*a+c*b) for a,b in [(-.1,-.1),(.1,-.1),(.1,.1),(-.1,.1)]])
    def test_translation_sweep_detects_obstacle_between_clear_endpoints(self):
        r=d.guide_check(CommonRoute([(-1.,0),(1.,0)]),box(-.01,-.01,.01,.01),self.Envelope())
        self.assertFalse(r['passed']);self.assertTrue(r['segments'][0]['intersects'])
    def test_corner_yaw_enclosure_and_no_Ackermann_claim(self):
        r=d.guide_check(CommonRoute([(-1.,0),(0.,0),(0.,1)]),box(5.,5.,6.,6.),self.Envelope())
        self.assertTrue(r['passed']);self.assertEqual(len(r['heading_transitions']),2);self.assertFalse(r['Ackermann_feasible_corner_transition'])

if __name__=='__main__':unittest.main()
