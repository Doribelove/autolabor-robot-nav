import math
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import axle_reference_adapter as a
from route_world_adapter import RouteFrame


def assert_close(x, y, tolerance=1e-10):
    assert len(x) == len(y)
    assert max(abs(u-v) for u, v in zip(x, y)) <= tolerance


@pytest.mark.parametrize('yaw', [-math.pi, -1.1, 0., .9, math.pi])
@pytest.mark.parametrize('vx,vy,w', [(-1.2,.23,-.6), (0.,0.,0.), (1.2,.195,.6), (.4,-.3,.8)])
def test_inverse_shift_preserves_measured_lateral_slip_and_se2(yaw, vx, vy, w):
    pose=(2.7,-3.4,yaw);twist=(vx,vy,w);L=.65
    rear=a.base_pose_to_rear(pose,L)
    assert_close(a.rear_pose_to_base(rear,L),pose)
    rear_twist=a.base_twist_to_rear(twist,L)
    assert_close(a.rear_twist_to_base(rear_twist,L),twist)
    assert rear_twist[1] == pytest.approx(vy-.325*w)
    # Global SE2 is independent of point-reference conversion; body velocity
    # components are invariant to a change of world coordinates.
    f=RouteFrame(-7.,4.,.63)
    p=f.pose_to_world(rear)
    q=a.base_pose_to_rear(f.pose_to_world(pose),L)
    assert_close(p,q)
    assert rear_twist[0] == vx and rear_twist[2] == w


@pytest.mark.parametrize('velocity,omega', [(1.1,.5),(-.7,-.8),(1.2,-.6)])
def test_analytic_circular_trajectory_derivative_at_base(velocity,omega):
    # Independent exact rear-axle circle. The finite derivative of the mapped
    # base path must include midpoint lateral velocity rather than erase it.
    def rear(t):
        return velocity/omega*math.sin(omega*t), velocity/omega*(1-math.cos(omega*t)), omega*t
    t=.73;h=1e-5
    plus=a.rear_pose_to_base(rear(t+h),.65)
    minus=a.rear_pose_to_base(rear(t-h),.65)
    dx=(plus[0]-minus[0])/(2*h);dy=(plus[1]-minus[1])/(2*h)
    yaw=omega*t;c,s=math.cos(yaw),math.sin(yaw)
    measured=(c*dx+s*dy,-s*dx+c*dy,omega)
    assert_close(measured,a.rear_twist_to_base((velocity,0.,omega),.65),1e-9)
    # The old zero-vy midpoint model is detectably different when turning.
    assert abs(measured[1]) > .1


@pytest.mark.parametrize('yaw', [-2.4,0.,1.7])
def test_footprint_world_geometry_is_identical_at_either_reference(yaw):
    base=(4.,-2.,yaw);rear=a.base_pose_to_rear(base,.65)
    for point in [(-.52,-.35),(-.52,.35),(.52,-.35),(.52,.35),(-.325,.3),(.325,-.3)]:
        rear_point=a.base_local_point_to_rear(point,.65)
        assert_close(a.rear_local_point_to_base(rear_point,.65),point)
        assert_close(a.shift_pose(base,point),a.shift_pose(rear,rear_point))


def test_initial_goal_and_general_reference_shift():
    start=(-4.,0.,0.);goal=(5.,0.,0.)
    assert a.base_pose_to_rear(start,.65)==(-4.325,0.,0.)
    assert a.base_pose_to_rear(goal,.65)==(4.675,0.,0.)
    assert_close(a.shift_body_twist((1.,.4,.8),(.2,-.3)),(1.24,.56,.8))
    assert a.LIVE_INTEGRATION_STATUS.startswith('component_only')


@pytest.mark.parametrize('invalid', [math.nan,math.inf,-math.inf])
def test_nonfinite_rejected(invalid):
    for function,args in [(a.shift_pose,((invalid,0.,0.),(0.,0.))),
                          (a.shift_body_twist,((0.,invalid,0.),(0.,0.))),
                          (a.shift_body_twist,((0.,0.,0.),(invalid,0.))),
                          (a.base_pose_to_rear,((0.,0.,0.),invalid))]:
        with pytest.raises(ValueError):function(*args)
    with pytest.raises(ValueError):a.shift_pose((0.,0.),(0.,0.))
    with pytest.raises(ValueError):a.base_pose_to_rear((0.,0.,0.),0.)
