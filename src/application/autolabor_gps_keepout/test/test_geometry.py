#!/usr/bin/env python3
"""Non-motion route simulations and user input / clearance checks."""
import math
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / 'src'))
sys.path.insert(0, str(HERE.parents[2] / 'gps_module' / 'src'))
from shapely.geometry import Point, Polygon
from autolabor_gps_keepout.geometry import (
    BODY_RADIUS, KeepoutGeometry, gps_to_xy, xy_to_gps, rasterize, validate_config)
from autolabor_gps_keepout.route import KeepoutGoalRoute
from autolabor_gps_keepout.targets import parse_targets, decode_file

SQUARE = [(10,-5),(20,-5),(20,5),(10,5)]
POINTS = 'pt0\t104.3892865\t30.29387435\n\npt1\t104.3898634\t30.29726472\npt2\t104.3904909\t30.30145079'


class GeometryTests(unittest.TestCase):
    def test_clearance_includes_body_at_every_yaw(self):
        g = KeepoutGeometry([SQUARE])
        self.assertFalse(g.clear_point((10-1-BODY_RADIUS+.01,0)))
        self.assertTrue(g.clear_point((10-1-BODY_RADIUS-.2,0)))
        for a,b in zip(g.path((0,0),(40,0)),g.path((0,0),(40,0))[1:]):
            self.assertTrue(g.clear_line(a,b))
            for k in range(21):
                p=Point(a[0]+(b[0]-a[0])*k/20,a[1]+(b[1]-a[1])*k/20)
                self.assertGreater(p.distance(g.raw)-BODY_RADIUS,1.)

    def test_start_or_goal_in_margin_rejected(self):
        g=KeepoutGeometry([SQUARE])
        for a,b in [((0,0),(15,0)),((15,0),(40,0)),((0,0),(9,0))]:
            with self.assertRaises(ValueError): g.path(a,b)

    def test_rolling_missions_reach_goal_without_crossing_keepouts(self):
        cases = [
            [SQUARE],
            [SQUARE,[(40,-12),(55,-12),(55,2),(40,2)]],
            [[(10,-10),(30,-10),(30,10),(25,10),(25,-5),(15,-5),(15,10),(10,10)]],
            [SQUARE,[(15,-3),(25,-3),(25,8),(15,8)]],
            [[(50,-30),(100,-30),(100,30),(50,30)]],
        ]
        for polygons in cases:
            with self.subTest(polygons=polygons):
                g=KeepoutGeometry(polygons)
                route=KeepoutGoalRoute(160.,0.,lookahead_distance=15.,advance_distance=5.,geometry=g)
                x,y=0.,0.
                segment=route.next_segment(x,y)
                for step in range(3000):
                    if route.should_advance(x,y): segment=route.next_segment(x,y)
                    dx,dy=segment.x-x,segment.y-y
                    d=math.hypot(dx,dy)
                    if d<.001:
                        if segment.is_final: break
                        segment=route.next_segment(x,y)
                        continue
                    scale=min(.25/d,1.)
                    nx,ny=x+scale*dx,y+scale*dy
                    self.assertTrue(g.clear_line((x,y),(nx,ny)))
                    x,y=nx,ny
                else: self.fail('rolling route did not converge')
                self.assertLess(math.hypot(x-160,y),.01)

    def test_close_final_across_obstacle_still_routes_around(self):
        g=KeepoutGeometry([[(4,-2),(6,-2),(6,2),(4,2)]])
        r=KeepoutGoalRoute(10,0,geometry=g)
        self.assertFalse(r.final_is_within_horizon(0,0))
        s=r.next_segment(0,0)
        self.assertFalse(s.is_final)
        self.assertTrue(g.clear_line((0,0),(s.x,s.y)))

    def test_disabled_and_deleted_restore_direct_route(self):
        for g in (KeepoutGeometry(),KeepoutGeometry.from_state({'enabled':False,'local_polygons':[SQUARE]})):
            self.assertEqual(g.path((0,0),(40,0)),[(0,0),(40,0)])
            self.assertTrue(g.motion_clear(15,0,0,.6,0))

    def test_visual_forward_reverse_and_turn(self):
        g=KeepoutGeometry([SQUARE])
        self.assertFalse(g.motion_clear(7.5,0,0,.6,0))
        self.assertTrue(g.motion_clear(7.5,0,0,-.6,0))
        self.assertFalse(g.motion_clear(22.5,0,0,-.6,0))
        self.assertTrue(g.motion_clear(0,0,0,.6,.5))

    def test_disconnected_free_region_has_no_route(self):
        walls=[[(5,-10),(7,-10),(7,10),(5,10)], [(-7,-10),(-5,-10),(-5,10),(-7,10)],
               [(-7,8),(7,8),(7,10),(-7,10)],[(-7,-10),(7,-10),(7,-8),(-7,-8)]]
        with self.assertRaises(ValueError): KeepoutGeometry(walls).path((0,0),(20,0))

    def test_gps_roundtrip(self):
        for origin in [(30.3,104.39),(30.301,104.391)]:
            target=(30.29726472,104.3898634)
            got=xy_to_gps(*gps_to_xy(*target,origin),origin)
            for a,b in zip(got,target): self.assertAlmostEqual(a,b,10)

    def test_raster_coverage_and_north_orientation(self):
        p=Polygon([(1,4),(3,4),(3,8),(1,8)])
        grid,(x0,y0),res=rasterize(p,.1)
        def sample(x,y): return grid[grid.shape[0]-1-int((y-y0)/res),int((x-x0)/res)]
        self.assertEqual(sample(2,6),0)
        self.assertEqual(sample(2,-6),254)
        self.assertEqual(sample(3.02,6),0)
        with self.assertRaises(ValueError): rasterize(p,.1,anchor=(10000,10000))

    def test_invalid_polygon_does_not_pass_validation(self):
        origin=(30.,104.)
        for points in [SQUARE[:2],[(0,0),(1,1),(0,1),(1,0)],[(0,0),(1,0),(2,0)],[(0,0),(1,0),(1,0),(0,1)]]:
            data={'enabled':True,'polygons':[{'points':[xy_to_gps(*p,origin) for p in points]}]}
            with self.assertRaises(ValueError): validate_config(data)
        with self.assertRaises(ValueError): validate_config({'polygons':[None]})


class TargetTests(unittest.TestCase):
    def test_user_points_are_longitude_then_latitude(self):
        points=parse_targets(POINTS)
        self.assertEqual([p['name'] for p in points],['pt0','pt1','pt2'])
        self.assertAlmostEqual(points[2]['latitude'],30.30145079,8)
        self.assertAlmostEqual(points[2]['longitude'],104.3904909,8)
        origin=(30.300616228666666,104.38986213283333)
        nearest=min(points,key=lambda p:math.hypot(*gps_to_xy(p['latitude'],p['longitude'],origin)))
        self.assertEqual(nearest['name'],'pt2')

    def test_survey_csv_gb18030_dms(self):
        text='点名,B,L,N,E\npt0,30:17:37.94766,104:23:21.43144,0,0\n'
        pts=parse_targets(decode_file(text.encode('gb18030')))
        self.assertAlmostEqual(pts[0]['latitude'],30.29387435,8)
        self.assertAlmostEqual(pts[0]['longitude'],104.38928651111112,8)

    def test_bad_or_swapped_points_rejected(self):
        for text in ['pt0 30.3 104.39','pt0 nan 30.3','pt0 104.39 30.3\npt0 104.4 30.4','pt0 104.39','']:
            with self.subTest(text=text),self.assertRaises(ValueError):parse_targets(text)


if __name__=='__main__': unittest.main()
