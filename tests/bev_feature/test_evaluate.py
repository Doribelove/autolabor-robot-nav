import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/bev_feature'))
from evaluate import evaluate

class TestEvaluation(unittest.TestCase):
    def test_error_duplicate_and_static_spread_are_distinct(self):
        frames=[dict(stamp_ns=1,truth=[dict(id='a',x=2.,y=1.,seam=True)],
                     predictions=[dict(x=2.05,y=1.,track_id=7),dict(x=2.1,y=1.,track_id=8)]),
                dict(stamp_ns=2,truth=[dict(id='a',x=2.,y=1.,seam=False)],
                     predictions=[dict(x=1.95,y=1.,track_id=7)])]
        r=evaluate(frames)
        self.assertAlmostEqual(r['error_m']['mean'],.05)
        self.assertEqual(r['seam_extra_duplicates'],1)
        self.assertEqual(r['stationary_map_stability']['a']['track_ids'],[7])
        self.assertAlmostEqual(r['stationary_map_stability']['a']['radial_spread_m']['max'],.05)
