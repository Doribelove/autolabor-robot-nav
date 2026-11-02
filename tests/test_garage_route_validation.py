import math
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from garage_route_validation import GarageMap


class GarageRouteValidationTest(unittest.TestCase):
    def setUp(self):
        self.grid=GarageMap()
        self.grid.cells=np.zeros((100,100),np.int8)
        self.grid.width=self.grid.height=100
        self.grid.resolution=.1;self.grid.origin=np.array([0.,0.])

    def test_full_vehicle_interior_is_checked(self):
        self.grid.cells[32,37]=100
        self.assertFalse(self.grid.valid([[3,3,0]])[0])

    def test_unknown_space_rejected(self):
        self.grid.cells[30,40]=-1
        self.assertFalse(self.grid.valid([[3,3,0]])[0])

    def test_forward_and_reverse_arc_at_radius_limit(self):
        for sign in [1.,-1.]:
            t=np.linspace(0,sign*.6,30);radius=1.35
            path=np.column_stack([4+radius*np.sin(t),4+radius*(1-np.cos(t)),t])
            ok,stats=self.grid.verify_path(path)
            self.assertTrue(ok,stats)
            self.assertLessEqual(stats['max_curvature'],1/1.35+1e-8)

    def test_tight_turn_is_rejected(self):
        t=np.linspace(0,.6,20);radius=.8
        ok,stats=self.grid.verify_path(np.column_stack([4+radius*np.sin(t),4+radius*(1-np.cos(t)),t]))
        self.assertFalse(ok);self.assertIn('radius',stats['reason'])

    def test_sideways_and_in_place_motion_rejected(self):
        for path in [[[3,3,0],[3,4,0]],[[3,3,0],[3,3,math.pi/2]]]:
            self.assertFalse(self.grid.verify_path(path)[0])


if __name__=='__main__':unittest.main()
