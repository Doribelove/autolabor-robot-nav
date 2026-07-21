import math

from nav_world_model.teb_bridge import local_track_to_fixed


def test_local_track_to_fixed_recovers_absolute_position_and_velocity():
    obstacle = local_track_to_fixed(
        track_id=7, local_x=2.0, local_y=1.0,
        relative_vx=-0.5, relative_vy=0.4, radius=0.3,
        robot_x=3.0, robot_y=4.0, robot_yaw=math.pi / 2.0,
        robot_linear_velocity=0.5,
    )
    assert obstacle.track_id == 7
    assert obstacle.x == 2.0
    assert obstacle.y == 6.0
    assert abs(obstacle.vx + 0.4) < 1.0e-12
    assert abs(obstacle.vy) < 1.0e-12
    assert obstacle.radius == 0.3


def test_local_track_to_fixed_enforces_positive_radius():
    obstacle = local_track_to_fixed(
        track_id=1, local_x=0.0, local_y=0.0,
        relative_vx=0.0, relative_vy=0.0, radius=0.0,
        robot_x=0.0, robot_y=0.0, robot_yaw=0.0,
        robot_linear_velocity=0.0,
    )
    assert obstacle.radius == 0.05
