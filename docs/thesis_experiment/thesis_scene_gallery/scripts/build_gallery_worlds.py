#!/usr/bin/env python3
"""Build paused, presentation-only Gazebo worlds for the thesis scene gallery.

The geometry mirrors the frozen PCT/TEB/DWA benchmark scene manifests, while
materials, actor positions, route markers, and camera poses are adjusted only
for readability.  These worlds are illustrations, not experiment identities.
They contain no planner, policy, evaluator, reactive-actor plugin, or result
writer.
"""

from pathlib import Path


GALLERY_ROOT = Path(__file__).resolve().parents[1]
WORLD_ROOT = GALLERY_ROOT / "source_worlds"

COLORS = {
    "ground": (0.80, 0.82, 0.84, 1.0),
    "road": (0.28, 0.31, 0.35, 1.0),
    "centerline": (0.95, 0.78, 0.20, 1.0),
    "wall": (0.52, 0.56, 0.61, 1.0),
    "obstacle": (0.96, 0.49, 0.12, 1.0),
    "occluder": (0.35, 0.38, 0.42, 1.0),
    "actor": (0.85, 0.12, 0.18, 1.0),
    "actor_path": (0.95, 0.32, 0.35, 0.65),
    "start": (0.12, 0.48, 0.90, 1.0),
    "goal": (0.10, 0.72, 0.35, 1.0),
}


def rgba(name):
    return " ".join(f"{value:.3f}" for value in COLORS[name])


def material(name):
    color = rgba(name)
    return f"""
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
            <specular>0.08 0.08 0.08 1</specular>
          </material>"""


def box(name, x, y, z, sx, sy, sz, color, yaw=0.0):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>
      <link name="body">
        <collision name="collision"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>{material(color)}
        </visual>
      </link>
    </model>"""


def cylinder(name, x, y, z, radius, length, color):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="body">
        <collision name="collision"><geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>{material(color)}
        </visual>
      </link>
    </model>"""


def ground_and_route():
    models = [
        f"""
    <model name="gallery_ground">
      <static>true</static>
      <link name="body">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 30</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>40 30</size></plane></geometry>{material('ground')}
        </visual>
      </link>
    </model>""",
        box("navigable_route", -0.2, 0.0, 0.012, 12.4, 4.8, 0.024, "road"),
        cylinder("start_marker", -5.4, 0.0, 0.035, 0.26, 0.07, "start"),
        cylinder("goal_marker", 5.0, 0.0, 0.035, 0.30, 0.07, "goal"),
    ]
    for index, x in enumerate((-4.5, -3.3, -2.1, -0.9, 0.3, 1.5, 2.7, 3.9)):
        models.append(box(f"centerline_{index}", x, 0.0, 0.035, 0.62, 0.055, 0.025, "centerline"))
    return models


def corridor_walls(half_width=2.55):
    wall_y = half_width + 0.10
    return [
        box("upper_wall", 0.0, wall_y, 0.50, 16.0, 0.20, 1.0, "wall"),
        box("lower_wall", 0.0, -wall_y, 0.50, 16.0, 0.20, 1.0, "wall"),
    ]


def actor(name, x, y):
    return cylinder(name, x, y, 0.52, 0.36, 1.04, "actor")


def vertical_actor_path(name, x, y_center=0.0, length=7.6):
    return box(name, x, y_center, 0.027, 0.07, length, 0.018, "actor_path")


def horizontal_actor_path(name, y, x_center=-0.45, length=8.1):
    return box(name, x_center, y, 0.027, length, 0.07, 0.018, "actor_path")


def world_document(name, models, camera_pose):
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="{name}">
    <gravity>0 0 -9.81</gravity>
    <physics name="gallery_physics" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <light name="sun" type="directional">
      <pose>-2 -3 12 0 0 0</pose>
      <diffuse>0.95 0.95 0.95 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>0.25 0.35 -0.90</direction>
      <cast_shadows>true</cast_shadows>
    </light>
    <scene>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
      <ambient>0.55 0.55 0.58 1</ambient>
      <background>0.93 0.95 0.98 1</background>
      <shadows>true</shadows>
    </scene>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{camera_pose}</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
{''.join(models)}
  </world>
</sdf>
"""


def build_worlds():
    # Rotate the top-down view so the robot's +x travel direction reads left
    # to right on a landscape thesis page.  The height keeps the complete
    # -5.4 m -> +5.0 m route, corridor walls, and crossing paths in frame.
    top_camera = "-0.2 0.0 13.5 0 1.5708 1.5708"
    angled_camera = "-6.2 -6.8 5.8 0 0.46 0.82"

    worlds = {}

    overview = ground_and_route()
    overview += corridor_walls()
    overview += [box("overview_obstacle", 0.2, 0.65, 0.60, 0.80, 0.90, 1.20, "obstacle")]
    worlds["01_m2_model_overview.sdf"] = world_document(
        "m2_model_overview", overview, angled_camera
    )

    static = ground_and_route() + corridor_walls()
    static += [box("center_obstacle", -0.15, 0.0, 0.60, 0.80, 0.90, 1.20, "obstacle")]
    worlds["02_static_avoidance.sdf"] = world_document(
        "static_avoidance_gallery", static, top_camera
    )

    crossing = ground_and_route()
    crossing += [
        vertical_actor_path("actor_crossing_path", 0.0),
        actor("crossing_actor", 0.0, -1.10),
    ]
    worlds["03_single_crossing.sdf"] = world_document(
        "single_crossing_gallery", crossing, top_camera
    )

    occluded = ground_and_route()
    occluded += [
        vertical_actor_path("occluded_actor_path", 0.0),
        box("occluder", -0.10, -1.65, 0.60, 0.80, 0.60, 1.20, "occluder"),
        actor("occluded_actor", 0.0, -2.25),
    ]
    worlds["04_partially_occluded_crossing.sdf"] = world_document(
        "partially_occluded_gallery", occluded, top_camera
    )

    head_on = ground_and_route() + corridor_walls()
    head_on += [
        horizontal_actor_path("head_on_actor_path", 1.40),
        actor("head_on_actor", 1.10, 1.40),
    ]
    worlds["05_head_on_yield.sdf"] = world_document(
        "head_on_yield_gallery", head_on, top_camera
    )

    sequential = ground_and_route()
    sequential += [
        vertical_actor_path("first_actor_path", -1.00),
        vertical_actor_path("second_actor_path", 1.35),
        actor("first_actor", -1.00, -0.95),
        actor("second_actor", 1.35, 1.25),
    ]
    worlds["06_sequential_interaction.sdf"] = world_document(
        "sequential_interaction_gallery", sequential, top_camera
    )

    width_change = ground_and_route()
    width_change += [
        box("wide_upper_entry", -2.50, 2.45, 0.50, 3.0, 0.20, 1.0, "wall"),
        box("wide_lower_entry", -2.50, -2.45, 0.50, 3.0, 0.20, 1.0, "wall"),
        box("narrow_upper", 1.00, 1.70, 0.50, 4.0, 0.20, 1.0, "wall"),
        box("narrow_lower", 1.00, -1.70, 0.50, 4.0, 0.20, 1.0, "wall"),
        box("wide_upper_exit", 4.00, 2.45, 0.50, 2.0, 0.20, 1.0, "wall"),
        box("wide_lower_exit", 4.00, -2.45, 0.50, 2.0, 0.20, 1.0, "wall"),
    ]
    worlds["07_width_change_corridor.sdf"] = world_document(
        "width_change_gallery", width_change, top_camera
    )

    WORLD_ROOT.mkdir(parents=True, exist_ok=True)
    for filename, document in worlds.items():
        (WORLD_ROOT / filename).write_text(document, encoding="utf-8")

    return sorted(worlds)


if __name__ == "__main__":
    for output_name in build_worlds():
        print(output_name)
