#!/usr/bin/env python3
"""Airport import, keep-in holes, vehicle clearance and rolling routes."""
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / 'src'))
sys.path.insert(0, str(HERE.parents[2] / 'gps_module' / 'src'))
from shapely.geometry import Polygon, Point, LineString, mapping
from shapely.ops import unary_union
from autolabor_gps_keepout.airport import import_airport_zip, validate_airport, project
from autolabor_gps_keepout.geometry import KeepoutGeometry, BODY_RADIUS, rasterize, validate_config
from autolabor_gps_keepout.route import KeepoutGoalRoute

ORIGIN = (30.3, 104.39)
AREA = Polygon([(0,0),(100,0),(100,60),(0,60)],
               holes=[[(40,20),(60,20),(60,40),(40,40)]])


def fixture_zip(path, prj='GEOGCS["GCS_WGS_1984"]', omit=None):
    import shapefile
    shapes = {'GHWJ': AREA,
              'LH': Polygon([(40,20),(60,20),(60,40),(40,40)]),
              'building': Polygon([(2,2),(8,2),(8,8),(2,8)]),
              'Sbuilding': Polygon([(20,2),(22,2),(22,4),(20,4)]),
              'construction_': Polygon([(80,40),(90,40),(90,55),(80,55)])}
    with zipfile.ZipFile(path, 'w') as archive:
        for name, polygon in shapes.items():
            if name == omit:
                continue
            shp, shx, dbf = io.BytesIO(), io.BytesIO(), io.BytesIO()
            writer = shapefile.Writer(shp=shp, shx=shx, dbf=dbf, shapeType=shapefile.POLYGON)
            writer.field('NAME', 'C')
            from shapely.geometry.polygon import orient
            polygon = orient(project(polygon, ORIGIN, inverse=True), sign=-1)
            writer.poly([list(polygon.exterior.coords)]+[list(r.coords) for r in polygon.interiors])
            writer.record(name); writer.close()
            for suffix, data in [('shp', shp.getvalue()), ('shx', shx.getvalue()),
                                 ('dbf', dbf.getvalue()), ('prj', prj.encode())]:
                archive.writestr('airport/'+name+'.'+suffix, data)


class AirportTests(unittest.TestCase):
    def test_zip_import_subtracts_all_hazards_and_preserves_holes(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'airport.zip';fixture_zip(path)
            airport=import_airport_zip(path)
        geometry=KeepoutGeometry.from_config(validate_config({'enabled':True,'airport':airport}), ORIGIN)
        for point in [(50,30),(5,5),(21,3),(85,45),(-5,30)]:
            self.assertFalse(geometry.clear_point(point),point)
        self.assertTrue(geometry.clear_point((25,30)))
        self.assertEqual(len(airport['source_sha256']),64)

    def test_reject_projected_or_incomplete_map(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'airport.zip'
            fixture_zip(path,prj='PROJCS["not geographic"]')
            with self.assertRaises(ValueError):import_airport_zip(path)
            fixture_zip(path,omit='construction_')
            with self.assertRaises(ValueError):import_airport_zip(path)

    def test_outside_hole_and_manual_regions_block_both_motion_directions(self):
        g=KeepoutGeometry([[(20,20),(25,20),(25,40),(20,40)]],allowed=AREA)
        self.assertFalse(g.clear_point((-10,30)))
        self.assertFalse(g.clear_point((50,30)))
        self.assertFalse(g.clear_point((22,30)))
        self.assertFalse(g.motion_clear(2.2,30,math.pi,.6,0))
        self.assertFalse(g.motion_clear(2.2,30,0,-.6,0))
        self.assertTrue(g.motion_clear(10,30,0,.6,0))
        self.assertFalse(g.clear_line((20,10),(50,30)))

    def test_complete_rolling_route_stays_inside_and_around_hole(self):
        g=KeepoutGeometry(allowed=AREA)
        for start,goal in [((10.,30.),(90.,30.)),((90.,30.),(10.,30.))]:
            route=KeepoutGoalRoute(*goal,geometry=g,lookahead_distance=15.,advance_distance=5.)
            x,y=start; segment=route.next_segment(x,y)
            for _ in range(1500):
                if route.should_advance(x,y): segment=route.next_segment(x,y)
                distance=math.hypot(segment.x-x,segment.y-y)
                if distance<1e-6:
                    if segment.is_final:break
                    segment=route.next_segment(x,y);continue
                fraction=min(1.,.25/distance)
                next_point=(x+(segment.x-x)*fraction,y+(segment.y-y)*fraction)
                self.assertTrue(AREA.covers(LineString([(x,y),next_point])))
                self.assertGreater(AREA.boundary.distance(Point(next_point)),1+BODY_RADIUS)
                x,y=next_point
            else:self.fail('airport rolling mission failed to converge')
            self.assertLess(math.dist((x,y),goal),.01)

    def test_narrow_connection_does_not_allow_shortcut_outside_boundary(self):
        area=unary_union([Polygon([(0,0),(10,0),(10,10),(0,10)]),
                          Polygon([(20,0),(30,0),(30,10),(20,10)]),
                          Polygon([(9,4),(21,4),(21,6),(9,6)])])
        with self.assertRaises(ValueError):KeepoutGeometry(allowed=area).path((5,5),(25,5))

    def test_raster_blocks_outside_and_holes_including_cell_edges(self):
        g=KeepoutGeometry(allowed=AREA)
        grid,(x0,y0),res=rasterize(g.obstacles,.5,allowed=g.allowed_costmap)
        def sample(x,y):return grid[grid.shape[0]-1-int((y-y0)/res),int((x-x0)/res)]
        for p in [(-2,30),(50,30),(.9,30)]:self.assertEqual(sample(*p),0)
        self.assertEqual(sample(20,30),254)

    def test_config_persists_geojson_holes_across_origin_change_and_disable(self):
        airport={'geometry':mapping(project(AREA,ORIGIN,inverse=True)),'name':'test'}
        config=validate_config({'enabled':True,'airport':airport})
        self.assertEqual(len(config['airport']['geometry']['coordinates']),2)
        for origin in [ORIGIN,(30.301,104.391)]:
            g=KeepoutGeometry.from_config(config,origin)
            world=project(g.allowed,origin,inverse=True)
            self.assertLess(world.symmetric_difference(project(AREA,ORIGIN,inverse=True)).area,1e-15)
            restored=KeepoutGeometry.from_state({'enabled':True,'local_allowed':mapping(g.allowed)})
            self.assertTrue(restored.active)
        config['enabled']=False
        self.assertFalse(KeepoutGeometry.from_config(config,ORIGIN).active)
        self.assertTrue(KeepoutGeometry.from_config(config,ORIGIN,for_export=True).active)
        bad=json.loads(json.dumps(airport));bad['geometry']['coordinates'][0][-1][0]+=.001
        with self.assertRaises(ValueError):validate_airport(bad)


if __name__=='__main__':unittest.main()
