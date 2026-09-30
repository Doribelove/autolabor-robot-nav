import copy,json,math,sys,unittest
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
import prepare_development_geometry_d04 as d

class D04Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot=d.ReferenceRobot((d.REFERENCE/'robot_description.xml').read_text(),(d.REFERENCE/'conversion/conversion.sdf').read_text())
        cls.envelope=d.WitnessEnvelope(cls.robot)
        cls.old=json.loads((d.STAGE/'scenes/development_a02/layouts.json').read_text())
    def test_existing_derived_parent_identity(self):
        priors,records,bindings,derivations=d.load_priors()
        self.assertEqual(len(priors),24);self.assertEqual(len(derivations),24)
        self.assertEqual(len({x['parent_geometry_id'] for x in derivations}),6)
    def test_duplicate_cosmetic_and_reflection_rejected(self):
        a=self.old[0];b=copy.deepcopy(a);b['layout_id']='cosmetic'
        self.assertTrue(d.v1.near_duplicate(a,b));self.assertEqual(d.separation(a,b),0.)
        for box in b['static_boxes']:box['y']=-box['y']
        for actor in b['actor_geometry']:
            for field in ('start','end','conflict'):actor[field][1]=-actor[field][1]
        self.assertEqual(d.v1.parent_id(a),d.v1.parent_id(b));self.assertEqual(d.separation(a,b),0.)
    def test_meaningful_structural_difference(self):
        a=self.old[0];b=copy.deepcopy(a);b['static_boxes'][0]['x']+=.6
        self.assertGreaterEqual(d.separation(a,b),2.);self.assertFalse(d.v1.near_duplicate(a,b))
    def test_no_episode_seed_allocation(self):
        g=d.v2.SeedGuard();g.named('geometry',d.VERSION,'fixture',0,0)
        self.assertEqual({r['purpose'] for r in g.records.values()},{'geometry'})
    def test_converted_full9_envelope_covers_steer_and_tilt(self):
        self.assertEqual(self.envelope.components,9)
        for curvature,roll,pitch in [(0,0,0),(.8,0,0),(-.8,0,0),(.6,.03,.02),(-.6,-.02,.03)]:
            shapes=self.robot.project(d.Pose3.from_xyz_rpy((0,0,0),(roll,pitch,0)),curvature)
            for shape in shapes:self.assertTrue(self.envelope.at((0,0,0)).buffer(1e-12).covers(Polygon(shape.polygon_xy)))
    def test_reference_curve_base_rear_endpoint_and_curvature(self):
        for a in d.AMPLITUDES:
            rows=d.curve(a,91);np.testing.assert_allclose(rows[0][2:5],[-4,0,0],atol=1e-12);np.testing.assert_allclose(rows[-1][2:5],[5,0,0],atol=1e-12)
            self.assertLessEqual(max(abs(r[-1]) for r in rows),4*abs(a)*(math.pi/9)**2+1e-12)
    def test_open_static_witness_does_not_claim_dynamic_or_time_cap(self):
        r=d.witness(self.envelope,[]);self.assertTrue(r['passed']);self.assertEqual(r['selected']['amplitude_m'],0)
        self.assertIn('no dynamics',r['scope']);self.assertLessEqual(r['selected']['central_steering_rate_bound_radps'],.75)
    def test_inherited_occlusion_topology_counterexample(self):
        import random
        self.assertEqual(d.v2.sample_geometry('occluded_emergence','L_corner_screen',random.Random(17))['topology'],'single_screen')
    def test_wall_blocked_witness_fails_without_margin_relaxation(self):
        wall=Polygon([(-1,-20),(1,-20),(1,20),(-1,20)])
        result=d.witness(self.envelope,[('wall',wall)])
        self.assertFalse(result['passed']);self.assertEqual(len(result['profile_attempts']),len(d.AMPLITUDES))
    def test_actor_parameter_existence_is_development_only(self):
        for scene in self.old:
            rows=d.parameter_witnesses(scene);self.assertIsNotNone(rows)
            for row in rows:
                self.assertEqual(d.v1.group_split(row['parameter_cell_id']),'development')
                self.assertLessEqual(abs(row['nominal_arrival_difference_s']),2.)

if __name__=='__main__':unittest.main()
