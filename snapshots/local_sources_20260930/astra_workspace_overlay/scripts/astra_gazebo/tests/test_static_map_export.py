"""Independent SAT/analytic checks for offline static-map information parity."""
import copy
import json
import math
from pathlib import Path
import random
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import static_map_export as exporter
from geometry import Pose3, convex_polygon_clearance, project_box, project_cylinder


def world(box_pose='0 1 0.5 0 0 0', actor_pose='0 0 .5 0 0 0', actor_plugin='libastra_reactive_actor_plugin.so'):
    return '''<sdf version="1.6"><world name="fixture">
    <model name="ground"><static>true</static><link name="body"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>40 30</size></plane></geometry></collision></link></model>
    <model name="screen"><static>true</static><pose>%s</pose><link name="body"><collision name="shape"><geometry><box><size>1 .4 1</size></box></geometry></collision></link></model>
    <model name="actor"><static>true</static><pose>%s</pose><link name="body"><collision name="shape"><geometry><box><size>.72 .72 1</size></box></geometry></collision></link><plugin name="actor" filename="%s"><base_speed>.8</base_speed></plugin></model>
    <model name="camera"><static>true</static><pose>0 0 19 0 1.57 1.57</pose><link name="camera"/></model>
    </world></sdf>''' % (box_pose, actor_pose, actor_plugin)


def scene():
    return {'scene_id': 'test', 'actors': [{'model_name': 'actor'}],
            'route': {'frame': 'world', 'start': [-2., 0., 0.], 'goal': [2., 0., 0.]},
            'robot_collision_reference': {'spawn_xyz_rpy': [-2.5, 0., 0., 0., 0., 0.]}}


def test_actor_static_true_never_leaks_into_map_and_map_is_pose_policy_invariant():
    first = exporter.make_artifacts(world(), scene())
    second = exporter.make_artifacts(world(actor_pose='100 -30 .5 0 0 1.3').replace('<base_speed>.8', '<base_speed>1.2'), scene())
    assert first[:2] == second[:2]
    assert first[2] == second[2]
    assert first[3]['source_world_sha256'] != second[3]['source_world_sha256']
    assert first[3]['static_collision_components'] == ['screen::body::shape']
    assert any('declared actor' in r['reason'] for r in first[3]['omitted_models_or_collisions'])
    assert first[2]['global_path_polyline'] is None
    assert first[2]['planner_input_authorized'] is False


@pytest.mark.parametrize('plugin', ['libv3_reactive_actor_plugin.so', 'libastra_reactive_actor_plugin.so'])
def test_old_and_new_declared_actor_plugins_are_both_excluded(plugin):
    shapes, _ = exporter.static_projections(world(actor_plugin=plugin), scene())
    assert len(shapes) == 1


def test_unclassified_plugin_or_missing_actor_identity_fails_closed():
    with pytest.raises(ValueError, match='unqualified movement'):
        exporter.make_artifacts(world(actor_plugin='unknown_actor.so'), scene())
    absent = scene();absent['actors'] = []
    with pytest.raises(ValueError, match='unclassified'):
        exporter.make_artifacts(world(), absent)


def test_random_rotated_boxes_raster_match_independent_convex_sat_reference():
    rng = random.Random(260926071)
    comparisons = 0
    for index in range(15):
        projection = project_box('b', (.1+rng.random()*.9, .05+rng.random()*.5, .8),
                                 Pose3.from_xyz_rpy((rng.uniform(-.8,.8), rng.uniform(-.8,.8), .4), (0,0,rng.uniform(-math.pi, math.pi))))
        grid = exporter.rasterize([projection], (-2., -2., 2., 2.), .2)
        for row in range(1, grid['height']-1):
            for column in range(1, grid['width']-1):
                x, y = grid['origin'][0]+column*.2, grid['origin'][1]+row*.2
                pixel = ((x,y),(x+.2,y),(x+.2,y+.2),(x,y+.2))
                reference = convex_polygon_clearance(projection.polygon_xy, pixel, tolerance_m=1e-12)
                assert (grid['cells_bottom_up'][row][column] == 0) is reference.intersects_or_touches
                comparisons += 1
    assert comparisons == 4860


def test_cylinder_union_covers_true_disc_and_has_declared_outward_error():
    cylinder = project_cylinder('c', .37, .7, Pose3((.13,.07,.35)))
    grid = exporter.rasterize([cylinder], (-2., -2., 2., 2.), .05)
    for index in range(200):
        angle = 2*math.pi*index/200
        assert exporter.point_cell(grid, (.13+.369*math.cos(angle), .07+.369*math.sin(angle))) == 0
    for row in range(1, grid['height']-1):
        for column in range(1, grid['width']-1):
            if grid['cells_bottom_up'][row][column] != 0:
                continue
            x, y = grid['origin'][0]+(column+.5)*.05, grid['origin'][1]+(row+.5)*.05
            assert math.hypot(x-.13,y-.07)-.37 <= math.sqrt(2)*.05+cylinder.approximation_error_m+1e-12


def test_tiny_geometry_occupies_intersecting_cells_without_centre_sampling_loss():
    box = project_box('tiny', (.002,.002,.3), Pose3((.026,.026,.15)))
    grid = exporter.rasterize([box], (-1.,-1.,1.,1.), .05)
    assert exporter.point_cell(grid, (.026,.026)) == 0
    assert sum(row.count(0) for row in grid['cells_bottom_up']) >= 1


def test_pgm_vertical_flip_and_unknown_boundary_are_explicit():
    box = project_box('upper', (.2,.2,.3), Pose3((0.,.5,.15)))
    grid = exporter.rasterize([box], (-1.,-1.,1.,1.), .1)
    header = ('P5\n%d %d\n255\n' % (grid['width'],grid['height'])).encode()
    data = grid['pgm'][len(header):]
    row, col = 15,10
    assert grid['cells_bottom_up'][row][col] == 0
    assert data[(grid['height']-1-row)*grid['width']+col] == 0
    assert exporter.point_cell(grid, (0.,-.5)) == 254
    assert exporter.point_cell(grid, (-1.,-.5)) == 205
    assert exporter.point_cell(grid, (-100.,0.)) == 205


def test_obstacle_clipping_or_occupied_goal_is_not_repaired_by_carving():
    box = project_box('b', (1.,1.,1.), Pose3((0.,0.,.5)))
    with pytest.raises(ValueError, match='clip'):
        exporter.rasterize([box], (-.5,-.5,.5,.5), .05)
    blocked = scene();blocked['route']['goal'] = [0.,1.,0.]
    with pytest.raises(ValueError, match='never clears'):
        exporter.make_artifacts(world(), blocked)


def test_map_odom_bridge_uses_spawn_translation_and_rotation_not_identity():
    s = scene();s['robot_collision_reference']['spawn_xyz_rpy'] = [4.,-3.,0.,0.,0.,.8]
    assert exporter.map_from_odom_pose(s) == [4.,-3.,.8]
    s['robot_collision_reference']['spawn_xyz_rpy'][3] = .2
    with pytest.raises(ValueError, match='zero spawn roll'):
        exporter.map_from_odom_pose(s)


def test_open_scene_extent_is_explicit_prior_not_an_artificial_route_corridor():
    first = exporter.make_artifacts(world(), scene())
    assert first[3]['bounds_xyxy'] == [-20.,-15.,20.,15.]
    assert (first[3]['width'],first[3]['height']) == (800,600)
    smaller = exporter.make_artifacts(world(), scene(), bounds_xyxy=(-4.,-3.,4.,3.))
    assert smaller[3]['bounds_xyxy'] == [-4.,-3.,4.,3.]


def test_rendered_sdf_origins_yaw_and_height_band_are_respected():
    shapes, _ = exporter.static_projections(world(box_pose='0 1 .5 0 0 .7853981633974483'), scene())
    xs = [p[0] for p in shapes[0].polygon_xy]
    assert max(xs)-min(xs) == pytest.approx(1.4/math.sqrt(2))
    shapes, omitted = exporter.static_projections(world(box_pose='0 1 5 0 0 0'), scene())
    assert shapes == []
    assert any('Z interval disjoint' in item['reason'] for item in omitted)


def test_map_export_is_immutable_and_hashes_match_written_bytes(tmp_path, monkeypatch):
    import yaml
    source_world, source_scene, output = tmp_path/'world.sdf', tmp_path/'scene.yaml', tmp_path/'export'
    source_world.write_text(world());source_scene.write_text(yaml.safe_dump(scene()))
    monkeypatch.setattr(sys, 'argv', ['static_map_export.py','--world',str(source_world),'--scene',str(source_scene),'--output',str(output)])
    assert exporter.main() == 0
    audit = json.loads((output/'export_audit.json').read_text())
    assert audit['map_image_sha256'] == exporter.sha_bytes((output/'map.pgm').read_bytes())
    assert audit['map_yaml_sha256'] == exporter.sha_bytes((output/'map.yaml').read_bytes())
    assert audit['global_route_integrated'] is False
    with pytest.raises(FileExistsError):
        exporter.main()
