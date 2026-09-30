"""Independent GEOS/SciPy references and real expanded-M2 collision checks."""
import importlib.util
import itertools
import json
import math
from pathlib import Path
import random
import subprocess
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from shapely.geometry import MultiPoint, Point, Polygon

MODULE = Path(__file__).resolve().parents[1] / 'geometry.py'
SPEC = importlib.util.spec_from_file_location('astra_geometry_under_test', str(MODULE))
g = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = g
SPEC.loader.exec_module(g)
WORKSPACE = Path(__file__).resolve().parents[3]


def scipy_transform(points, xyz, rpy):
    return Rotation.from_euler('xyz', rpy).apply(np.asarray(points)) + np.asarray(xyz)


def geos_clearance(first, second):
    a, b = Polygon(first), Polygon(second)
    if not a.intersects(b):
        return a.distance(b)
    # Independent GEOS Minkowski configuration obstacle; no SAT implementation.
    differences = [(x-u, y-v) for x, y in first for u, v in second]
    configuration_obstacle = MultiPoint(differences).convex_hull
    return -Point(0, 0).distance(configuration_obstacle.boundary)


def test_sat_depth_has_explicit_containment_and_touch_semantics():
    large = ((-2,-2), (2,-2), (2,2), (-2,2))
    small = ((-.5,-.5), (.5,-.5), (.5,.5), (-.5,.5))
    assert g.convex_polygon_clearance(large, small).signed_clearance_m == pytest.approx(-2.5)
    shifted = tuple((x+4,y) for x,y in large)
    result = g.convex_polygon_clearance(large, shifted)
    assert result.intersects_or_touches and result.signed_clearance_m == 0
    diagonal = tuple((x+5,y+5) for x,y in large)
    assert g.convex_polygon_clearance(large, diagonal).signed_clearance_m == pytest.approx(math.sqrt(2))


def test_random_polygons_against_independent_geos_minkowski_reference():
    rng = random.Random(20260926)
    positive = negative = 0
    errors = []
    for index in range(600):
        shapes = []
        for j in range(2):
            points = [(rng.uniform(-1,1), rng.uniform(-1,1)) for _ in range(14)]
            shape = MultiPoint(points).convex_hull
            shift = (rng.uniform(-2,2), rng.uniform(-2,2))
            shapes.append(tuple((x+shift[0],y+shift[1]) for x,y in list(shape.exterior.coords)[:-1]))
        result = g.convex_polygon_clearance(*shapes)
        expected = geos_clearance(*shapes)
        errors.append(abs(result.signed_clearance_m-expected))
        assert result.signed_clearance_m == pytest.approx(expected, abs=2e-10)
        reverse = g.convex_polygon_clearance(*reversed(shapes))
        assert reverse.signed_clearance_m == pytest.approx(expected, abs=2e-10)
        positive += expected > 0
        negative += expected < 0
    assert positive > 100 and negative > 100
    assert max(errors) < 2e-10
    print('REFERENCE_POLYGONS '+json.dumps({'pairs':600,'separated':positive,'overlapping':negative,
                                           'maximum_absolute_error_m':max(errors)}))


def test_box_projection_uses_all_eight_corners_and_full_3d_pose():
    rng = random.Random(701)
    for _ in range(80):
        size = tuple(rng.uniform(.05,2) for _ in range(3))
        xyz = tuple(rng.uniform(-2,2) for _ in range(3))
        rpy = tuple(rng.uniform(-math.pi,math.pi) for _ in range(3))
        projection = g.project_box('box', size, g.Pose3.from_xyz_rpy(xyz,rpy))
        corners = [[sign[i]*size[i]/2 for i in range(3)] for sign in itertools.product((-1,1),repeat=3)]
        transformed = scipy_transform(corners,xyz,rpy)
        reference = MultiPoint(transformed[:,:2]).convex_hull
        actual = Polygon(projection.polygon_xy)
        assert actual.hausdorff_distance(reference) < 2e-12
        assert projection.z_min_m == pytest.approx(transformed[:,2].min(),abs=2e-12)
        assert projection.z_max_m == pytest.approx(transformed[:,2].max(),abs=2e-12)
        assert projection.approximation_error_m == 0


def test_cylinder_is_an_outer_approximation_with_certified_geometric_error_bound():
    radius,length=.15,.10
    for rpy in [(0,0,0), (math.pi/2,0,0), (.4,-.7,1.3)]:
        xyz=(.2,-.7,.8)
        rotation=Rotation.from_euler('xyz',rpy)
        axes=rotation.apply(np.eye(3)).T
        for sides in (16,48,96):
            projection=g.project_cylinder('wheel',radius,length,g.Pose3.from_xyz_rpy(xyz,rpy),sides)
            error=radius*(1/math.cos(math.pi/sides)-1)
            assert projection.approximation_error_m == pytest.approx(error)
            for angle in np.linspace(0,2*math.pi,361):
                normal=np.asarray([math.cos(angle),math.sin(angle),0])
                exact_support=np.dot(normal,xyz)+radius*math.hypot(np.dot(normal,axes[:,0]),np.dot(normal,axes[:,1]))+length/2*abs(np.dot(normal,axes[:,2]))
                polygon_support=max(x*normal[0]+y*normal[1] for x,y in projection.polygon_xy)
                assert polygon_support >= exact_support-2e-12
                assert polygon_support-exact_support <= error+2e-12
            z_radius=radius*math.hypot(axes[2,0],axes[2,1])+length/2*abs(axes[2,2])
            assert projection.z_max_m >= xyz[2]+z_radius-2e-12
            assert projection.z_max_m-(xyz[2]+z_radius) <= error+2e-12
            assert projection.z_min_m <= xyz[2]-z_radius+2e-12
            assert projection.z_min_m == pytest.approx(xyz[2]-z_radius,abs=2e-12)
            assert projection.z_max_m == pytest.approx(xyz[2]+z_radius,abs=2e-12)


def test_margin_and_z_screen_do_not_turn_projection_into_physical_contact():
    robot=g.project_box('robot',(1,1,.5),g.Pose3((0,0,.25)))
    obstacle=g.project_box('obstacle',(1,1,.5),g.Pose3((1.2,0,.25)))
    gap=g.pair_clearance(robot,obstacle)
    assert gap.physical_projected_clearance_m == pytest.approx(.2)
    assert gap.conservative_projected_clearance_m == pytest.approx(-.13)
    assert not gap.potential_3d_intersection
    overhead=g.project_box('overhead',(1,1,.5),g.Pose3((0,0,2)))
    overlap=g.pair_clearance(robot,overhead)
    assert overlap.projected_intersection_or_touch
    assert not overlap.potential_3d_intersection
    assert overlap.z_interval_gap_m == pytest.approx(1.25)
    report=g.compound_clearance([robot],[overhead])
    assert report['minimum_z_conditioned_projected_clearance_m'] is None
    assert report['potential_3d_intersection_pairs'] == []
    json.dumps(report,allow_nan=False)


def test_compound_preserves_empty_space_between_components():
    left=g.project_box('left',(.2,.2,.5),g.Pose3((-1,0,.25)))
    right=g.project_box('right',(.2,.2,.5),g.Pose3((1,0,.25)))
    middle=g.project_box('middle',(.2,.2,.5),g.Pose3((0,0,.25)))
    report=g.compound_clearance([left,right],[middle])
    assert report['minimum_physical_projected_clearance_m'] == pytest.approx(.8)
    assert report['potential_3d_intersection_pairs'] == []
    assert g.compound_clearance([left],[])['minimum_physical_projected_clearance_m'] is None


def test_xy_and_height_screen_can_have_false_positive_for_tilted_boxes():
    first=g.project_box('a',(4,.2,.2),g.Pose3.from_xyz_rpy((0,0,0),(0,.6,0)))
    second=g.project_box('b',(4,.2,.2),g.Pose3.from_xyz_rpy((0,0,1),(0,.6,0)))
    assert g.pair_clearance(first,second).potential_3d_intersection
    # Independent body-frame check: equal orientations, local Z gap > thickness.
    relative=Rotation.from_euler('xyz',(0,.6,0)).inv().apply([0,0,1])
    assert abs(relative[2])>.2


SIMPLE_URDF='''<robot name="fixture">
<link name="root"/>
<link name="wheel"><collision name="wheel_box"><origin xyz="0.2 0 0" rpy="0 0 0.3"/>
<geometry><box size="0.7 0.2 0.3"/></geometry></collision></link>
<joint name="steer" type="revolute"><parent link="root"/><child link="wheel"/>
<origin xyz="0.5 0.4 0.2" rpy="0.1 0.2 0.3"/><axis xyz="0 0 1"/></joint>
</robot>'''


def test_urdf_collision_origin_joint_state_and_world_link_override():
    model=g.RobotCollisionModel(SIMPLE_URDF)
    with pytest.raises(ValueError,match='missing actual joint'):
        model.project(g.Pose3())
    actual=model.project(g.Pose3(),{'steer':.6})[0]
    origin_rotation=Rotation.from_euler('xyz',(.1,.2,.3))
    combined_rotation=origin_rotation*Rotation.from_rotvec([0,0,.6])
    collision_rotation=combined_rotation*Rotation.from_rotvec([0,0,.3])
    collision_center=np.asarray([.5,.4,.2])+combined_rotation.apply([.2,0,0])
    corners=[[sx*.35,sy*.1,sz*.15] for sx,sy,sz in itertools.product((-1,1),repeat=3)]
    expected=MultiPoint((collision_rotation.apply(corners)+collision_center)[:,:2]).convex_hull
    assert Polygon(actual.polygon_xy).hausdorff_distance(expected)<1e-12
    # Actual link state contains the articulated pose and needs no root/joint state.
    link_pose=g.Pose3((.5,.4,.2),tuple(combined_rotation.as_quat()))
    override=model.project(link_world_poses={'wheel':link_pose})[0]
    assert override.pose_source=='actual_link_world_pose'
    assert Polygon(override.polygon_xy).hausdorff_distance(expected)<1e-12


def test_prismatic_mimic_joint_uses_actual_parent_value():
    xml='''<robot name="slide"><link name="root"/><link name="a"/><link name="b">
    <collision><geometry><box size="1 1 1"/></geometry></collision></link>
    <joint name="drive" type="prismatic"><parent link="root"/><child link="a"/><axis xyz="1 0 0"/></joint>
    <joint name="mimic" type="prismatic"><parent link="a"/><child link="b"/><axis xyz="0 1 0"/>
    <mimic joint="drive" multiplier="2" offset="0.1"/></joint></robot>'''
    projection=g.RobotCollisionModel(xml).project(g.Pose3(),{'drive':.4})[0]
    centroid=Polygon(projection.polygon_xy).centroid
    assert (centroid.x,centroid.y)==pytest.approx((.4,.9))


@pytest.fixture(scope='module')
def expanded_m2():
    done=subprocess.run(['/opt/ros/noetic/bin/xacro',str(WORKSPACE/'src/simulation/m2_gazebo/urdf/m2.urdf.xacro')],
                        check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    return g.RobotCollisionModel(done.stdout)


def test_real_m2_has_nine_compound_collisions_including_wheels_and_sensors(expanded_m2):
    positions={name:0.0 for name,j in expanded_m2.joints.items() if j.kind!='fixed'}
    projected=expanded_m2.project(g.Pose3((0,0,.02)),positions)
    assert len(projected)==9
    assert sum(p.primitive_type=='cylinder' for p in projected)==5
    assert sum('wheel' in p.name for p in projected)==4
    assert any('laser' in p.name for p in projected)
    assert any('imu' in p.name for p in projected)
    assert min(p.z_min_m for p in projected)==pytest.approx(.02,abs=.0004)
    steered=dict(positions,front_left_steer_joint=.6,front_right_steer_joint=.6)
    changed=expanded_m2.project(g.Pose3((0,0,.02)),steered)
    original={p.name:p for p in projected}
    wheel_delta=[Polygon(p.polygon_xy).hausdorff_distance(Polygon(original[p.name].polygon_xy))
                 for p in changed if 'front_left_wheel' in p.name]
    assert max(wheel_delta)>.03


def test_real_wheel_extension_can_intersect_when_chassis_does_not(expanded_m2):
    positions={name:0.0 for name,j in expanded_m2.joints.items() if j.kind!='fixed'}
    positions.update(front_left_steer_joint=.65,front_right_steer_joint=.65)
    projections=expanded_m2.project(g.Pose3((0,0,.02)),positions)
    chassis=next(p for p in projections if 'chassis_collision' in p.name)
    wheel=next(p for p in projections if 'front_left_wheel' in p.name)
    tip=max(wheel.polygon_xy,key=lambda p:p[1])
    hazard=g.project_box('low_hazard',(.025,.025,.10),g.Pose3((tip[0],tip[1]-.005,.14)))
    assert g.pair_clearance(chassis,hazard).physical_projected_clearance_m > 0
    assert g.pair_clearance(wheel,hazard).potential_3d_intersection
    assert g.compound_clearance(projections,[hazard])['potential_3d_intersection_pairs']


@pytest.mark.parametrize('operation',[
    lambda:g.Pose3(xyzw=(0,0,0,0)),
    lambda:g.Pose3((float('nan'),0,0)),
    lambda:g.project_box('bad',(0,1,1)),
    lambda:g.project_cylinder('bad',1,1,sides=7),
    lambda:g.convex_polygon_clearance([(0,0),(2,0),(1,.5),(2,2),(0,2)],[(3,0),(4,0),(3,1)]),
    lambda:g.RobotCollisionModel('<robot><link name="root"><collision><geometry><mesh filename="x"/></geometry></collision></link></robot>'),
])
def test_invalid_or_unsupported_geometry_is_explicitly_rejected(operation):
    with pytest.raises(ValueError):
        operation()
