import random,sys,unittest
from pathlib import Path
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
import prepare_development_geometry_d04_v4 as d

class LScreenTests(unittest.TestCase):
    def test_exact19_prior_parent_rng_reconstruction(self):
        _,records,_,_=d.load_priors();parents,replayed=d.carry_forward(records)
        self.assertEqual(len(parents),19);self.assertGreater(len(replayed),len(records))
    def test_real_L_return_not_just_label(self):
        scene=d.sample_geometry('occluded_emergence','L_corner_screen',random.Random(17))
        self.assertEqual(scene['topology'],'L_corner_screen');self.assertEqual(len(scene['static_boxes']),2)
        a,b=map(d.v2.polygon,scene['static_boxes'])
        self.assertLess(a.distance(b),1e-4);self.assertGreater(b.difference(a).area,.4)
        self.assertEqual(scene['static_boxes'][1]['length'],.6);self.assertEqual(scene['static_boxes'][1]['width'],.7)
    def test_old_v2_stays_single_screen(self):
        scene=d.v2.sample_geometry('occluded_emergence','L_corner_screen',random.Random(17))
        self.assertEqual(scene['topology'],'single_screen');self.assertEqual(len(scene['static_boxes']),1)

if __name__=='__main__':unittest.main()
