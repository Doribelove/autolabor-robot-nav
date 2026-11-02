import importlib.util
import math
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock, patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'src/scripts/robot_bringup/scripts'))
from coverage_region_preset import rectangle_in_map
from online_coverage_mapper import occupancy_snapshot
from fused_scan_mapper import SparseOccupancyMapper
from nav_msgs.msg import OccupancyGrid,Odometry
import rospy

spec=importlib.util.spec_from_file_location('online_coverage_manager',ROOT/'src/application/autolabor_coverage/scripts/coverage_manager.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class OnlineCoverageTest(unittest.TestCase):
    def manager(self, online=True):
        m=module.CoverageManager.__new__(module.CoverageManager)
        m.lock=threading.RLock();m.online_mapping=online;m.online_map_identity='session-identity'
        m.map_digest='';m.state='IDLE';m.active=False;m.batch_active=False
        m._publish_status=Mock();m._clear_visualizations=Mock();m._request_cancel=Mock()
        return m

    def grid(self, obstacle=False):
        msg=OccupancyGrid();msg.header.frame_id='map';msg.info.width=3;msg.info.height=3
        msg.info.resolution=.1;msg.info.origin.orientation.w=1.
        msg.data=[0]*8+[100 if obstacle else -1]
        return msg

    def test_online_map_growth_keeps_task_and_updates_obstacles(self):
        m=self.manager();m._map_callback(self.grid())
        before=m.grid;m.plan=object();plan=m.plan;m.plan_id='current';m.state='READY'
        m.active=True;m._map_callback(self.grid(True))
        self.assertEqual(m.plan_id,'current');self.assertIs(m.plan,plan)
        self.assertIsNot(m.grid,before);self.assertEqual(m.map_digest,'session-identity')
        m._request_cancel.assert_not_called()

    def test_static_map_change_still_invalidates_and_cancels(self):
        m=self.manager(False);m._map_callback(self.grid());m.plan=object();m.plan_id='old'
        m.active=True;m._map_callback(self.grid(True))
        self.assertIsNone(m.plan);self.assertEqual(m.plan_id,'')
        m._request_cancel.assert_called_once()

    def test_online_pose_requires_current_finite_fastlio_pose(self):
        m=self.manager();m.localization_fresh_sec=.75
        msg=Odometry();msg.header.frame_id='camera_init';msg.child_frame_id='body'
        msg.header.stamp=rospy.Time(100);msg.pose.pose.orientation.w=1.
        with patch.object(rospy.Time,'now',return_value=rospy.Time(100.1)):
            m._odom_callback(msg);self.assertTrue(m.localized)
            msg.header.stamp=rospy.Time(90);m._odom_callback(msg);self.assertFalse(m.localized)
            msg.header.stamp=rospy.Time(100);msg.pose.pose.position.x=float('nan')
            m._odom_callback(msg);self.assertFalse(m.localized)

    def test_rectangle_uses_vehicle_heading_and_requested_offset(self):
        self.assertEqual(rectangle_in_map(0,0,0),[(3.,1.5),(13.,1.5),(13.,4.5),(3.,4.5)])
        points=rectangle_in_map(20,10,math.pi/2)
        self.assertAlmostEqual(sum(x for x,y in points)/4,17.)
        self.assertAlmostEqual(sum(y for x,y in points)/4,18.)
        area=abs(sum(points[i][0]*points[(i+1)%4][1]-points[(i+1)%4][0]*points[i][1] for i in range(4)))/2
        self.assertAlmostEqual(area,30.)

    def test_grid_export_preserves_unknown_and_persistent_obstacles(self):
        m=SparseOccupancyMapper();m.cells={(0,0):-2.,(1,0):2.,(2,0):2.,(3,0):0.}
        m.occupied_observations[(1,0)]=5;m.occupied_observations[(2,0)]=1
        width,height,x,y,data=occupancy_snapshot(m,padding_m=0.)
        self.assertEqual(data.tolist(),[[0,100,-1,-1]])

    def test_flat_prior_cannot_erase_obstacles_or_clear_outside_region(self):
        m=SparseOccupancyMapper(resolution=1.)
        m.cells={(0,0):2.,(1,0):2.,(2,0):0.,(4,0):0.}
        m.occupied_observations[(0,0)]=5;m.occupied_observations[(1,0)]=1
        rect=[(0,0),(3,0),(3,1),(0,1)]
        _,_,_,_,data=occupancy_snapshot(m,padding_m=0.,flat_region=rect)
        self.assertEqual(data[0].tolist(),[100,-1,0,-1,-1])


if __name__=='__main__':unittest.main()
