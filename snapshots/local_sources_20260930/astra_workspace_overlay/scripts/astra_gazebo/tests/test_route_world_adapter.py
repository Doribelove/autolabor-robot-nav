import importlib.util
from pathlib import Path
import math
import sys
import unittest

spec=importlib.util.spec_from_file_location('astra_route_world_adapter',Path(__file__).parents[1]/'route_world_adapter.py')
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)


class RouteAdapterTests(unittest.TestCase):
    def test_roundtrip_and_command_frame_semantics(self):
        frame=m.RouteFrame(12.,-3.,1.2);pose=(-4.,.6,-.7)
        back=frame.pose_to_route(frame.pose_to_world(pose))
        for a,b in zip(back,pose):self.assertAlmostEqual(a,b,places=12)
        self.assertEqual(frame.body_twist_to_world_command(.7,.2),(.7,.2))

    def test_box_actor_distances_and_orientation_preserved(self):
        frame=m.RouteFrame(-3.,8.,math.pi/2)
        b=frame.box_to_world({'x':2.,'y':1.,'length':.7,'width':.4,'yaw_rad':.3})
        self.assertAlmostEqual(b['yaw_rad'],math.pi/2+.3)
        actor={'start':[0.,-3.],'end':[0.,4.],'conflict':[0.,0.]}
        a=frame.actor_path_to_world(actor)
        self.assertAlmostEqual(math.dist(a['start'],a['end']),7.)
        self.assertEqual((b['length'],b['width']),(.7,.4))

    def test_delay_uses_gate_crossing_not_absolute_start_time(self):
        gate=m.GateRelativeActivation(.8)
        self.assertFalse(gate.update(100.,-5.))
        self.assertFalse(gate.update(105.,-4.01))
        self.assertFalse(gate.update(105.79,-3.8))
        self.assertTrue(gate.update(105.8,-3.7))
        with self.assertRaises(RuntimeError):gate.update(1.,-5.)


if __name__=='__main__':unittest.main()
