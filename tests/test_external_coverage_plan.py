import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.append(str(Path(__file__).resolve().parents[1]/'src/application/autolabor_coverage/src'))
from autolabor_coverage.coverage_geometry import GridMap,CoveragePlanner,Point,CoverageTimeParameters
from autolabor_coverage.external_coverage_plan import load_plan


class ExternalContourPlanTest(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.path=Path(self.directory.name)/'plan.json'
        self.grid=GridMap(60,60,.1,0,0,[0]*3600)
        self.planner=CoveragePlanner(self.grid,footprint_front=1.32,footprint_rear=.62,footprint_half_width=.5)
        self.region=[Point(0,0),Point(6,0),Point(6,6),Point(0,6)]
        self.data=dict(grid_digest=self.grid.digest(),operation_width_m=1.2,overlap_ratio=.15,
            region=[[p.x,p.y] for p in self.region],swaths=[[[1,1],[3,1]],[[4,2],[4,4]],[[3,4],[1.5,4]]],
            preview_swept_known_area_m2=5.)

    def tearDown(self):self.directory.cleanup()

    def load(self,width=1.2):
        self.path.write_text(json.dumps(self.data))
        return load_plan(str(self.path),self.planner,self.region,width,.15,Point(2,2),0.,CoverageTimeParameters())

    def test_preserves_contour_order_and_direction(self):
        p=self.load()
        self.assertEqual([[[s.start.x,s.start.y],[s.end.x,s.end.y]] for s in p.swaths],self.data['swaths'])
        self.assertGreater(p.estimated_total_time_sec,0.)

    def test_rejects_other_map(self):
        self.data['grid_digest']='wrong'
        with self.assertRaisesRegex(ValueError,'different occupancy'):self.load()

    def test_rejects_width_change(self):
        with self.assertRaisesRegex(ValueError,'width/overlap'):self.load(1.0)

    def test_checks_interior_obstacle(self):
        cells=[0]*3600;cells[10*60+25]=100
        self.grid=GridMap(60,60,.1,0,0,cells);self.planner.grid=self.grid
        self.data['grid_digest']=self.grid.digest()
        with self.assertRaisesRegex(ValueError,'footprint'):self.load()

    def test_asymmetric_front_is_checked_for_reversed_sweep(self):
        self.data['swaths']=[[[2.7,2],[.7,2]]]
        with self.assertRaisesRegex(ValueError,'footprint'):self.load()


if __name__=='__main__':unittest.main()
