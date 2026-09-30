import random,sys,unittest
from pathlib import Path
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
import prepare_development_geometry_d04_v2 as d

class RevisionTests(unittest.TestCase):
    def test_actual_prior_rng_and11_parent_reconstruction(self):
        _,records,_,_=d.load_priors();carry,replayed=d.carry_forward(records)
        self.assertEqual(len(carry),11);self.assertGreater(len(replayed),len(records))
        self.assertTrue(all(p['execution_seed'] is None for p in carry))
    def test_new_bay_geometry_within_frozen_dimensions(self):
        for side in ('left_passing_bay','right_passing_bay'):
            scene=d.sample_geometry('head_on_yield',side,random.Random(123))
            p=scene['d04_bay_parameters'];self.assertTrue(1.8<=p['opening_length_m']<=3.)
            self.assertTrue(.6<=p['extra_clear_depth_m']<=1.2)
            self.assertEqual(len(scene['static_boxes']),6)
            self.assertTrue(d.v1.initial_geometry_qc(scene)['horizontal_initial_geometry_pass'])
            self.assertTrue(d.v2.geometry_semantics(scene)['actor_swept_static_pass'])
    def test_only_missing_slots_sampled_and_no_gate_relaxation(self):
        self.assertEqual(d.TOPOLOGIES['head_on_yield'][3],'left_passing_bay')
        self.assertEqual(d.AMPLITUDES,(0.,.8,-.8,1.2,-1.2,1.6,-1.6))

if __name__=='__main__':unittest.main()
