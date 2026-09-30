#!/usr/bin/env python3
"""Offline common static-map exporter; no ROS, actor truth or planner launch.

This builds a shared prior from declared *static* SDF collision geometry. Actor
models remain excluded even when their pose-setting plugin requires static=true.
The output contains a map and a route REQUEST, not an invented collision-free
global path. Runtime adapters and an actual common global path remain separate
integration work. Historical runtime/launch/configuration files are not edited.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from shapely.geometry import Polygon, box as cell_box
from shapely.prepared import prep

from geometry import Pose3, project_box, project_cylinder

ACTOR_PLUGINS = ('libv3_reactive_actor_plugin.so', 'libastra_reactive_actor_plugin.so')


def sha_bytes(value):
    return hashlib.sha256(value).hexdigest()


def finite_vector(values, length):
    values = tuple(float(v) for v in values)
    if len(values) != length or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid finite vector')
    return values


def sdf_pose(element):
    if element is None:
        return Pose3()
    if element.get('relative_to') or element.get('frame'):
        raise ValueError('expand named SDF pose frames before exporting')
    value = finite_vector((element.text or '').split(), 6)
    angles = tuple(math.radians(v) for v in value[3:]) if element.get('degrees') == 'true' else value[3:]
    return Pose3.from_xyz_rpy(value[:3], angles)


def static_projections(world_xml, scene, z_band=(.01, .70), robot_model='autolabor_m2'):
    """Full XY projections whose Z interval intersects the declared body band.

    The interval excludes the floor. Tilted solids use their entire projection
    once intersecting the band, a conservative approximation to a clipped slice.
    Robot, dynamic actors, ground planes and collision-free cameras are omitted.
    Unclassified moving/plugin models, meshes, nested models and includes fail.
    """
    z_band = finite_vector(z_band, 2)
    if z_band[0] >= z_band[1]:
        raise ValueError('invalid robot-height band')
    root = ET.fromstring(world_xml)
    world = root.find('world') if root.tag == 'sdf' else root
    if world is None or world.tag != 'world' or world.findall('.//include'):
        raise ValueError('expanded SDF world without includes required')
    actor_names = {actor['model_name'] for actor in scene.get('actors', [])}
    if len(actor_names) != len(scene.get('actors', [])):
        raise ValueError('duplicate actor identity')
    all_models = world.findall('model')
    names = [model.get('name') for model in all_models]
    if None in names or len(names) != len(set(names)) or not actor_names.issubset(set(names)):
        raise ValueError('world/actor model identity mismatch')
    shapes, omitted = [], []
    for model in all_models:
        name = model.get('name')
        plugins = [p.get('filename') for p in model.findall('plugin')]
        if name == robot_model:
            omitted.append({'model': name, 'reason': 'robot is not a static map obstacle'})
            continue
        if name in actor_names:
            if len(plugins) != 1 or plugins[0] not in ACTOR_PLUGINS:
                raise ValueError('declared actor has unqualified movement classification: '+name)
            omitted.append({'model': name, 'reason': 'declared actor; excluded independently of static flag and current pose'})
            continue
        if plugins or model.findtext('static', 'false').lower() != 'true':
            raise ValueError('unclassified moving or plugin-bearing model: '+name)
        if model.findall('model') or model.findall('joint'):
            raise ValueError('static model must have expanded fixed collision links: '+name)
        model_pose = sdf_pose(model.find('pose'))
        collision_count = 0
        for link in model.findall('link'):
            link_pose = model_pose.compose(sdf_pose(link.find('pose')))
            for index, collision in enumerate(link.findall('collision')):
                collision_count += 1
                identity = name+'::'+str(link.get('name'))+'::'+collision.get('name', 'collision_'+str(index))
                pose = link_pose.compose(sdf_pose(collision.find('pose')))
                geometry = collision.find('geometry')
                if geometry is None or len(geometry) != 1:
                    raise ValueError('single primitive collision required')
                primitive = geometry[0]
                if primitive.tag == 'plane':
                    normal = pose.rotate(finite_vector(primitive.findtext('normal', '0 0 1').split(), 3))
                    if abs(normal[0])+abs(normal[1]) > 1e-9 or normal[2] <= 0 or pose.xyz[2] >= z_band[0]:
                        raise ValueError('only flat ground below robot band may be omitted')
                    omitted.append({'model': name, 'collision': identity, 'reason': 'ground support plane below body band'})
                    continue
                if primitive.tag == 'box':
                    value = project_box(identity, finite_vector(primitive.findtext('size').split(), 3), pose)
                elif primitive.tag == 'cylinder':
                    value = project_cylinder(identity, float(primitive.findtext('radius')), float(primitive.findtext('length')), pose)
                else:
                    raise ValueError('unsupported static collision: '+primitive.tag)
                if value.z_max_m < z_band[0] or value.z_min_m > z_band[1]:
                    omitted.append({'model': name, 'collision': identity, 'reason': 'collision Z interval disjoint from fixed robot band'})
                    continue
                shapes.append(value)
        if not collision_count:
            omitted.append({'model': name, 'reason': 'collision-free model'})
    if len({p.name for p in shapes}) != len(shapes):
        raise ValueError('duplicate static collision identity')
    return shapes, omitted


def rasterize(projections, bounds_xyxy, resolution=.05):
    """Map rows run from minimum Y upwards, as in nav_msgs/OccupancyGrid.

    Any closed pixel square intersecting a static projection is occupied. This
    prevents small/rotated objects disappearing between pixel centres. There is
    no robot-radius or safety-margin inflation here: downstream common policies
    must explicitly bind any such inflation. One unknown outer ring makes the
    finite mapped-domain boundary visible to every consumer.
    """
    resolution = float(resolution)
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('positive finite map resolution required')
    left, bottom, right, top = finite_vector(bounds_xyxy, 4)
    if left >= right or bottom >= top:
        raise ValueError('invalid map bounds')
    left, bottom = math.floor(left/resolution)*resolution, math.floor(bottom/resolution)*resolution
    width, height = math.ceil((right-left)/resolution), math.ceil((top-bottom)/resolution)
    if width < 3 or height < 3 or width*height > 4_000_000:
        raise ValueError('map size outside bounded exporter contract')
    right, top = left+width*resolution, bottom+height*resolution
    cells = [bytearray([254])*width for _ in range(height)]
    for index in range(width):
        cells[0][index] = cells[-1][index] = 205
    for row in cells:
        row[0] = row[-1] = 205
    for projection in projections:
        shape = Polygon(projection.polygon_xy)
        x0, y0, x1, y1 = shape.bounds
        if x0 < left+resolution or y0 < bottom+resolution or x1 > right-resolution or y1 > top-resolution:
            raise ValueError('map bounds would clip a static collision or overwrite unknown boundary')
        prepared = prep(shape)
        c0 = max(1, int(math.floor((x0-left)/resolution))-1)
        c1 = min(width-2, int(math.floor((x1-left)/resolution))+1)
        r0 = max(1, int(math.floor((y0-bottom)/resolution))-1)
        r1 = min(height-2, int(math.floor((y1-bottom)/resolution))+1)
        for row in range(r0, r1+1):
            for column in range(c0, c1+1):
                x, y = left+column*resolution, bottom+row*resolution
                if prepared.intersects(cell_box(x, y, x+resolution, y+resolution)):
                    cells[row][column] = 0
    pgm = ('P5\n%d %d\n255\n' % (width, height)).encode()+b''.join(bytes(row) for row in reversed(cells))
    return {'resolution': resolution, 'origin': [left, bottom, 0.0], 'width': width, 'height': height,
            'bounds': [left, bottom, right, top], 'cells_bottom_up': cells, 'pgm': pgm}


def point_cell(grid, xy):
    x, y = finite_vector(xy, 2)
    column = math.floor((x-grid['origin'][0])/grid['resolution'])
    row = math.floor((y-grid['origin'][1])/grid['resolution'])
    if not (0 <= row < grid['height'] and 0 <= column < grid['width']):
        return 205
    return grid['cells_bottom_up'][row][column]


def map_from_odom_pose(scene):
    """The map/world frame coincides; odom origin is the spawn world pose."""
    spawn = finite_vector(scene['robot_collision_reference']['spawn_xyz_rpy'], 6)
    if abs(spawn[3])+abs(spawn[4]) > 1e-9:
        raise ValueError('planar map/odom bridge requires zero spawn roll and pitch')
    return [spawn[0], spawn[1], spawn[5]]


def make_artifacts(world_xml, scene, resolution=.05, z_band=(.01, .70), bounds_xyxy=None):
    if scene.get('route', {}).get('frame') != 'world':
        raise ValueError('explicit world-frame route request required')
    start = finite_vector(scene['route']['start'], 3)
    goal = finite_vector(scene['route']['goal'], 3)
    spawn_tf = map_from_odom_pose(scene)
    shapes, omitted = static_projections(world_xml, scene, z_band)
    # Known-map extent is an explicit common prior, independent of actor poses
    # and the route's bounding box. Tight route padding would invent an unknown
    # corridor boundary in genuinely open scenes. The default matches the
    # 40-by-30 m declared ground domain of the current six-family worlds.
    bounds = finite_vector(bounds_xyxy if bounds_xyxy is not None else scene.get('static_map_bounds_xyxy', (-20., -15., 20., 15.)), 4)
    grid = rasterize(shapes, bounds, resolution)
    if any(point_cell(grid, p[:2]) != 254 for p in (start, goal, spawn_tf)):
        raise ValueError('spawn/start/goal centre is occupied or unknown; exporter never clears a cell to force success')
    metadata = {'image': 'map.pgm', 'resolution': grid['resolution'], 'origin': grid['origin'],
                'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196, 'mode': 'trinary'}
    yaml_bytes = yaml.safe_dump(metadata, sort_keys=False).encode()
    map_sha, yaml_sha = sha_bytes(grid['pgm']), sha_bytes(yaml_bytes)
    route = {'schema': 'astra_common_route_request_v1', 'frame_id': 'map',
             'map_frame_equals_world': True, 'map_image_sha256': map_sha, 'map_yaml_sha256': yaml_sha,
             'policy_start_xy_yaw': list(start), 'goal_xy_yaw': list(goal),
             'map_from_odom_xy_yaw': spawn_tf,
             'global_path_polyline': None, 'global_path_status': 'pending_shared_planner_and_curvature_clearance_validation',
             'same_endpoints_are_not_a_shared_global_path': True,
             'planner_input_authorized': False,
             'required_next_artifact': 'one static-map-derived ordered global polyline with source bytes/hash, frame, clearance/curvature audit and common pruning rule'}
    audit = {'schema': 'astra_common_static_map_export_v1',
             'source_world_sha256': sha_bytes(world_xml.encode()), 'scene_id': scene.get('scene_id'),
             'parent_geometry_id': scene.get('parent_geometry_id'),
             'map_frame_id': 'map', 'map_frame_equals_world': True,
             'resolution_m': resolution, 'bounds_xyxy': grid['bounds'], 'width': grid['width'], 'height': grid['height'],
             'bounds_policy': 'explicit known static-prior domain; default world-aligned40m x30m, never inferred from actor position or narrow route box',
             'pixel_values': {'occupied': 0, 'free': 254, 'unknown': 205},
             'occupied_cell_count': sum(row.count(0) for row in grid['cells_bottom_up']),
             'unknown_cell_count': sum(row.count(205) for row in grid['cells_bottom_up']),
             'robot_height_band_m': list(z_band), 'static_collision_components': [p.name for p in shapes],
             'omitted_models_or_collisions': omitted,
             'maximum_primitive_outward_error_m': max((p.approximation_error_m for p in shapes), default=0.),
             'raster_outward_distance_bound_m': math.sqrt(2)*resolution,
             'collision_to_total_occupied_union_outward_bound_m': math.sqrt(2)*resolution+max((p.approximation_error_m for p in shapes), default=0.),
             'inflation_or_point_carving_applied': False,
             'map_image_sha256': map_sha, 'map_yaml_sha256': yaml_sha,
             'status': 'offline_common_static_prior_only_not_live_fairness_qualification',
             'global_route_integrated': False, 'old_R2_runtime_unchanged': True,
             'limits': ['A synthetic known static prior shared identically is not online actor truth.',
                        'A tilted primitive intersecting the height band uses its full XY projection; the raster bound applies to that projection, not an exact height-clipped cross-section.',
                        'Static map PGM bytes alone do not make runtime consumption or global paths equal.',
                        'Unknown and outside-map policy must be occupied/fail-closed for every consumer.',
                        'No Ackermann feasibility, route availability, navigation performance or safety claim follows from export.']}
    return grid['pgm'], yaml_bytes, route, audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', required=True, type=Path)
    parser.add_argument('--scene', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--resolution', type=float, default=.05)
    parser.add_argument('--bounds', type=float, nargs=4, metavar=('XMIN','YMIN','XMAX','YMAX'))
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('immutable map export destination already exists')
    world_bytes, scene_bytes = args.world.read_bytes(), args.scene.read_bytes()
    pgm, metadata, route, audit = make_artifacts(world_bytes.decode(), yaml.safe_load(scene_bytes), args.resolution,
                                               bounds_xyxy=args.bounds)
    if args.world.read_bytes() != world_bytes or args.scene.read_bytes() != scene_bytes:
        raise RuntimeError('input changed during offline map export')
    audit['source_bindings'] = [{'path': str(args.world.resolve()), 'sha256': sha_bytes(world_bytes)},
                              {'path': str(args.scene.resolve()), 'sha256': sha_bytes(scene_bytes)},
                              {'path': str(Path(__file__).resolve()), 'sha256': sha_bytes(Path(__file__).read_bytes())},
                              {'path': str(Path(__file__).with_name('geometry.py').resolve()),
                               'sha256': sha_bytes(Path(__file__).with_name('geometry.py').read_bytes())}]
    args.output.mkdir(parents=True)
    (args.output/'map.pgm').write_bytes(pgm)
    (args.output/'map.yaml').write_bytes(metadata)
    (args.output/'route_request.json').write_text(json.dumps(route, indent=2, allow_nan=False)+'\n')
    (args.output/'export_audit.json').write_text(json.dumps(audit, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'output': str(args.output), 'occupied_cells': audit['occupied_cell_count'],
                      'map_image_sha256': audit['map_image_sha256'], 'live_fairness_qualified': False}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
