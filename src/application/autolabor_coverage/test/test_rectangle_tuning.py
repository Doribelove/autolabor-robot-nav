#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import threading
import unittest
from types import SimpleNamespace

from autolabor_coverage.coverage_geometry import Point, minimum_area_rectangle, polygon_area, point_in_polygon


class RectangleTest(unittest.TestCase):
    def test_rotated_rectangle_is_minimum_not_axis_aligned_box(self):
        a = .39
        points = [Point(x*math.cos(a)-y*math.sin(a), x*math.sin(a)+y*math.cos(a))
                  for x,y in ((0,0),(6,0),(6,2),(0,2),(2,1))]
        rect = minimum_area_rectangle(points)
        self.assertEqual(len(rect), 4)
        self.assertAlmostEqual(polygon_area(rect),12)
        for p in points:
            # Include boundary using edge halfplanes.
            self.assertTrue(all((b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x)>=-1e-8
                                for a,b in zip(rect,rect[1:]+rect[:1])))

    def test_unordered_duplicate_concave_selection(self):
        points=[Point(4,1),Point(0,0),Point(0,3),Point(4,3),Point(4,0),Point(0,0)]
        self.assertAlmostEqual(polygon_area(minimum_area_rectangle(points)),12)

    def test_triangle_and_large_map_offset(self):
        points=[Point(1000000,1000000),Point(1000004,1000000),Point(1000000,1000002)]
        self.assertAlmostEqual(polygon_area(minimum_area_rectangle(points)),8)

    def test_degenerate_nonfinite_and_tiny_rejected(self):
        for points in ([],[Point(0,0)]*4,[Point(0,0),Point(1,1),Point(2,2)],
                       [Point(0,0),Point(1,0),Point(float('nan'),1)],
                       [Point(0,0),Point(.1,0),Point(0,.1)]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                minimum_area_rectangle(points)

    def test_fitting_is_idempotent(self):
        rect=minimum_area_rectangle([Point(0,0),Point(3,1),Point(4,3),Point(0,4)])
        self.assertAlmostEqual(polygon_area(rect),polygon_area(minimum_area_rectangle(rect)))


class TuningTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('tuning_manager',str(Path(__file__).resolve().parents[1]/'scripts/coverage_manager.py'))
        cls.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.module)

    def setUp(self):
        from autolabor_coverage.cfg import TransitTuningConfig
        self.config=dict(TransitTuningConfig.defaults)
        self.manager=self.module.CoverageManager.__new__(self.module.CoverageManager)
        m=self.manager
        m.lock=threading.RLock();m._transit_tuning_last=None
        m.max_speed_limit=1.6;m.max_reverse_speed_limit=.8;m.max_angular_speed_limit=1.
        m.linear_accel_limit=2.;m.angular_accel_limit=1.
        m.active=m.batch_active=m.plan_pending=m.start_pending=m.navigation_profile_update_pending=False
        m.plan=m.original_teb=None
        import ast, inspect, textwrap
        tree=ast.parse(textwrap.dedent(inspect.getsource(m._transit_tuning_callback)))
        assignment=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='fields' for t in n.targets))
        for key,attribute in ast.literal_eval(assignment.value).items():setattr(m,attribute,self.config[key])
        m._transit_tuning_callback(dict(self.config),0)

    def test_idle_apply_changes_real_profile(self):
        self.config['hybrid_transit_weight_viapoint']=23.
        self.config['hybrid_transit_max_forward_speed_mps']=.7
        self.manager._transit_tuning_callback(self.config,0)
        self.assertEqual(self.manager.hybrid_transit_weight_viapoint,23.)
        from autolabor_coverage.coverage_geometry import CoverageTimeParameters
        self.assertEqual(self.manager._transit_time_parameters(CoverageTimeParameters()).max_forward_speed_mps,.7)

    def test_active_or_preview_preserves_actual_values(self):
        # Avoid a ROS master for the diagnostic log.
        from unittest.mock import patch
        with patch.object(self.module.rospy,'logwarn'):
            for key in ('active','batch_active','plan_pending','start_pending','navigation_profile_update_pending','plan','original_teb'):
                with self.subTest(key=key):
                    setattr(self.manager,key,True)
                    changed=dict(self.config,hybrid_transit_weight_viapoint=99.)
                    result=self.manager._transit_tuning_callback(changed,0)
                    self.assertEqual(result['hybrid_transit_weight_viapoint'],8.)
                    setattr(self.manager,key,None if key in ('plan','original_teb') else False)

    def test_persisted_values_win_over_launch_defaults_after_restart(self):
        import tempfile
        from autolabor_coverage.cfg import TransitTuningConfig
        from autolabor_coverage.tuning_store import load_values
        with tempfile.TemporaryDirectory() as directory:
            m=self.manager
            m._transit_tuning_state_file=directory+'/transit.json'
            changed=dict(self.config,hybrid_transit_weight_viapoint=23.,hybrid_transit_max_forward_speed_mps=.7)
            m._transit_tuning_callback(changed,0)
            defaults={k:v for k,v in TransitTuningConfig.defaults.items() if k!='groups'}
            saved=load_values(m._transit_tuning_state_file,defaults,TransitTuningConfig.min,TransitTuningConfig.max)
            # Recreate startup state: old YAML defaults must not overwrite disk.
            self.setUp()
            self.manager._transit_tuning_state_file=directory+'/transit.json'
            self.manager._transit_tuning_last=None
            self.manager._transit_tuning_saved=saved
            result=self.manager._transit_tuning_callback(dict(self.config),0)
            self.assertEqual(result['hybrid_transit_weight_viapoint'],23.)
            self.assertEqual(self.manager.hybrid_transit_max_forward_speed,.7)

    def test_failed_persistence_does_not_change_runtime_or_receipt(self):
        from unittest.mock import patch
        m=self.manager;m._transit_tuning_state_file='/not-used/transit.json'
        with patch('autolabor_coverage.tuning_store.save_values',side_effect=OSError('disk full')), patch.object(self.module.rospy,'logerr'):
            actual=m._transit_tuning_callback(dict(self.config,hybrid_transit_weight_viapoint=99.),0)
        self.assertEqual(actual['hybrid_transit_weight_viapoint'],8.)
        self.assertEqual(m.hybrid_transit_weight_viapoint,8.)

    def test_nan_and_configured_cap_rejected(self):
        for value in (float('nan'),2.):
            result=self.manager._transit_tuning_callback(dict(self.config,hybrid_transit_max_forward_speed_mps=value),0)
            self.assertEqual(result['hybrid_transit_max_forward_speed_mps'],0.)

if __name__=='__main__': unittest.main()
