#!/usr/bin/env python3
"""Exercise the real plan loader, visual path and fixed-gear actions offline."""
import importlib.util,json,time
from pathlib import Path
from unittest.mock import patch
import rospy
from autolabor_coverage.coverage_geometry import GridMap,CoveragePlanner,Point,CoverageTimeParameters
from autolabor_coverage.external_coverage_plan import load_plan
from garage_route_validation import GarageMap
from garage_drivable_route import ROOT

began=time.monotonic();g=GarageMap();grid=GridMap(g.width,g.height,g.resolution,*g.origin,g.cells.ravel())
planner=CoveragePlanner(grid,footprint_front=1.32,footprint_rear=.62,footprint_half_width=.5,minimum_turning_radius=1.35)
d=json.loads((ROOT/'verified_plan.json').read_text());pose=d['transitions'][0][0]
plan=load_plan(str(ROOT/'verified_plan.json'),planner,[Point(*p) for p in d['region']],1.2,.15,Point(*pose[:2]),pose[2],CoverageTimeParameters())
spec=importlib.util.spec_from_file_location('manager','src/application/autolabor_coverage/scripts/coverage_manager.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
manager=m.CoverageManager.__new__(m.CoverageManager);manager.path_sample_spacing=.1
manager.external_plan_file=str(ROOT/'verified_plan.json');manager.hierarchical_hybrid_on_demand=True
manager.online_hybrid_without_precompute=False
with patch.object(rospy.Time,'now',return_value=rospy.Time(1)):
    path=manager._planned_path(plan.swaths,Point(*pose[:2]),plan.transition_paths)
    segments=manager._segments(plan.swaths,Point(*pose[:2]),plan.transition_paths)
assert len(plan.swaths)==len(plan.transition_paths)
assert path.poses and len(segments)>len(plan.swaths)
assert all(s['type'] in ['transit','sweep'] and s['path'].poses for s in segments)
report=dict(passed=True,swaths=len(plan.swaths),connectors=len(plan.transition_paths),display_poses=len(path.poses),
    controller_segments=len(segments),forward_actions=sum(s.get('gear')==1 for s in segments),reverse_actions=sum(s.get('gear')==-1 for s in segments),
    seconds=time.monotonic()-began,real_navigation_started=False,ros_publishers_created=False)
(ROOT/'import_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
