import copy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from autolabor_coverage.coverage_geometry import GridMap, Point, CoveragePlanner, CoverageTimeParameters
from autolabor_coverage.external_coverage_plan import load_plan
from autolabor_coverage.validated_path import check_path
from autolabor_coverage.msg import HybridTransitionResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath

spec=importlib.util.spec_from_file_location('manager', 'src/application/autolabor_coverage/scripts/coverage_manager.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def line(a,b):
    return [[float(x),0.,0.] for x in np.linspace(a,b,int(round((b-a)/.05))+1)]


class ExternalPathsTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.filename=Path(self.tmp.name)/'plan.json'
        self.grid=GridMap(200,100,.1,-5.,-5.,[0]*20000)
        self.planner=CoveragePlanner(self.grid,footprint_front=1.32,footprint_rear=.62,footprint_half_width=.5)
        self.polygon=[Point(-5,-5),Point(15,-5),Point(15,5),Point(-5,5)]
        self.data=dict(format_version=2,grid_digest=self.grid.digest(),operation_width_m=1.2,overlap_ratio=.15,
            region=[[p.x,p.y] for p in self.polygon],swaths=[[[0,0],[2,0]],[[3,0],[5,0]]],
            transitions=[line(-1,0),line(2,3)],preview_swept_known_area_m2=4.8)

    def load(self,origin=None):
        self.filename.write_text(json.dumps(self.data))
        return load_plan(str(self.filename),self.planner,self.polygon,1.2,.15,origin or Point(-1,0),0.,CoverageTimeParameters())

    def manager(self):
        m=module.CoverageManager.__new__(module.CoverageManager)
        m.path_sample_spacing=.05;m.external_plan_file=str(self.filename)
        m.hierarchical_hybrid_on_demand=True;m.online_hybrid_without_precompute=False
        return m

    @patch.object(rospy.Time,'now',return_value=rospy.Time(1))
    def test_loaded_curves_are_drawn_and_cusp_executor_receives_them(self,_):
        p=self.load();m=self.manager()
        path=m._planned_path(p.swaths,Point(-1,0),p.transition_paths)
        self.assertAlmostEqual(path.poses[0].pose.position.x,-1.)
        self.assertAlmostEqual(path.poses[-1].pose.position.x,5.)
        actions=m._segments(p.swaths,Point(-1,0),p.transition_paths)
        self.assertEqual([a['type'] for a in actions],['transit','sweep','transit','sweep'])
        self.assertTrue(all(a['path'].poses for a in actions))
        self.assertEqual([a['gear'] for a in actions if a['type']=='transit'],[1,1])

    @patch.object(rospy.Time,'now',return_value=rospy.Time(1))
    def test_missing_curves_never_draw_direct_cross_obstacle_edges(self,_):
        p=self.load();m=self.manager()
        self.assertFalse(m._planned_path(p.swaths,Point(-1,0)).poses)
        self.assertFalse(m._planned_path(p.swaths,Point(-1,0),[p.transition_paths[0],None]).poses)
        moved=self.load(Point(-2,0));self.assertIsNone(moved.transition_paths[0])
        self.assertAlmostEqual(m._planned_path(moved.swaths,Point(-2,0),moved.transition_paths).poses[0].pose.position.x,0.)

    def test_map_or_disconnected_path_rejected(self):
        self.data['grid_digest']='other'
        with self.assertRaisesRegex(ValueError,'different occupancy'):self.load()
        self.data['grid_digest']=self.grid.digest();self.data['transitions'][1]=line(1.9,3)
        with self.assertRaisesRegex(ValueError,'previous sweep'):self.load()

    def test_interior_pillar_unknown_and_small_turn_are_rejected(self):
        for occupancy in [100,-1]:
            data=list(self.grid.data);data[50*200+50]=occupancy
            grid=GridMap(200,100,.1,-5,-5,data)
            with self.assertRaisesRegex(ValueError,'footprint'):
                check_path(line(-1,0),grid,1.32,.62,.5,1.35)
        angles=np.linspace(0,.3,30)
        arc=np.column_stack([.8*np.sin(angles),.8*(1-np.cos(angles)),angles])
        with self.assertRaisesRegex(ValueError,'turning radius'):
            check_path(arc,self.grid,1.32,.62,.5,1.35)

    @patch.object(rospy.Time,'now',return_value=rospy.Time(1))
    def test_start_recomputes_only_entry_and_preserves_map_verified_curves(self,_):
        p=self.load();m=self.manager();m.hybrid_precompute_timeout_sec=5
        m._wait_for_hybrid_planner=lambda **kwargs:None;m._transit_profile=lambda:None
        calls=[]
        fresh=copy.deepcopy(p.transition_paths[0]);fresh.poses[0].pose.position.x=-2
        result=HybridTransitionResult(candidate_index=0,transition_index=0,
            outcome=HybridTransitionResult.OUTCOME_SUCCESS,path=fresh)
        def compute(**kwargs):
            calls.append(kwargs);return SimpleNamespace(success=True,results=[result],message='ok')
        m._call_hybrid_planner=compute
        revised=m._precompute_hybrid_candidates(p,Point(-2,0),0,CoverageTimeParameters(),self.planner,'id')
        self.assertEqual(len(calls[0]['transitions']),1)
        self.assertAlmostEqual(calls[0]['transitions'][0].start.pose.position.x,-2.)
        self.assertEqual(revised.transition_paths[1],p.transition_paths[1])
        self.assertAlmostEqual(revised.transition_paths[0].poses[0].pose.position.x,-2.)
        result.outcome=HybridTransitionResult.OUTCOME_NO_PATH
        with self.assertRaisesRegex(RuntimeError,'entry'):
            m._precompute_hybrid_candidates(p,Point(-2,0),0,CoverageTimeParameters(),self.planner,'id')


if __name__=='__main__':unittest.main()
