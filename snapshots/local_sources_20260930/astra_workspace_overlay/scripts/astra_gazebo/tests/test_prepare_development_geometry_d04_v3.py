import math,sys,unittest
from pathlib import Path
import numpy as np
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
import prepare_development_geometry_d04_v3 as d

class SmoothWitnessTests(unittest.TestCase):
    def test_carryforward_preserves17_exact_parents_and_all_rng(self):
        _,records,_,_=d.load_priors();parents,replayed=d.carry_forward(records)
        self.assertEqual(len(parents),17);self.assertGreater(len(replayed),len(records))
    def test_asymmetric_endpoint_tangent_curvature_and_bounds(self):
        for peak in (-1.,1.5):
            for a in (1.,-1.,1.2,-1.2):
                rows=d.asymmetric_curve(a,peak);length=min(peak+4.325,4.675-peak)
                np.testing.assert_allclose(rows[0][2:],[ -4,0,0,0],atol=1e-12)
                np.testing.assert_allclose(rows[-1][2:],[5,0,0,0],atol=1e-12)
                self.assertLessEqual(max(abs(r[-1]) for r in rows),6*abs(a)/length**2)
                self.assertLess(math.atan(.65*6*abs(a)/length**2),.4964)
                for r in rows:
                    self.assertAlmostEqual(r[2]-r[0],.325*math.cos(r[4]));self.assertAlmostEqual(r[3]-r[1],.325*math.sin(r[4]))

if __name__=='__main__':unittest.main()
