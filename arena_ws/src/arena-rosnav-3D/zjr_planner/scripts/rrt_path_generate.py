#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
写在前面
rrt_multi_paths.py

从 /map 获取栅格地图（nav_msgs/OccupancyGrid），从 /odom 获取起点，
用 RRT 找到三条无碰撞路径到用户指定目标点（世界坐标，米）。
结果以 JSON 格式保存，路径点保留三位小数。

运行：
    rosrun <pkg> rrt_multi_paths.py
或
    rosrun <pkg> rrt_multi_paths.py --goal X Y

如需修改调参：
--step：步长（m），如果步长太大会穿过狭缝，太小会导致搜索慢。可在 0.2 到 1.0 之间尝试。

--max_iters：RRT 每次运行的最大节点数，复杂地图适当增大（例如 10000）。

--num_paths：默认 3，可改为其它值。

若地图分辨率严格是 0.05，脚本会使用地图自身 info.resolution，通常应匹配你的场景；若你需要强制 0.05，请在 GridMap 初始化时插值/重采样（脚本当前不做重采样，直接使用原始地图分辨率）。

若 RRT 很难找到多条不同路径，可尝试增大 max_iters，增加 goal_sample_rate 或对已找到的路径在地图上"临时标记为障碍"以找到替代路径（本脚本采用多次随机重启与相似度过滤策略）。
"""

import rospy
import json
import os
import time
import math
import random
import argparse
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from collections import deque

# ---------- Simple Node structure for RRT ----------
class Node:
    def __init__(self, x, y, parent=None):
        self.x = float(x)
        self.y = float(y)
        self.parent = parent

# ---------- Utility functions for grid/world conversions ----------
class GridMap:
    def __init__(self, occupancy_grid_msg: OccupancyGrid):
        self.width = occupancy_grid_msg.info.width
        self.height = occupancy_grid_msg.info.height
        self.resolution = occupancy_grid_msg.info.resolution
        self.origin_x = occupancy_grid_msg.info.origin.position.x
        self.origin_y = occupancy_grid_msg.info.origin.position.y
        # convert data (row-major) into 2D list [row][col] where row=0 is y=0 at origin_y
        data = list(occupancy_grid_msg.data)
        # occupancy grid is row-major: data[index = y*width + x]
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        for y in range(self.height):
            for x in range(self.width):
                val = data[y * self.width + x]
                self.grid[y][x] = val  # -1 unknown, 0 free, >50 occupied typically

    def world_to_map(self, xw, yw):
        mx = int((xw - self.origin_x) / self.resolution)
        my = int((yw - self.origin_y) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        xw = self.origin_x + (mx + 0.5) * self.resolution
        yw = self.origin_y + (my + 0.5) * self.resolution
        return xw, yw

    def in_bounds_map(self, mx, my):
        return 0 <= mx < self.width and 0 <= my < self.height

    def is_occupied_map(self, mx, my):
        if not self.in_bounds_map(mx, my):
            return True
        val = self.grid[my][mx]
        return val > 50  # occupancy threshold

    def is_free_world(self, xw, yw):
        mx, my = self.world_to_map(xw, yw)
        return not self.is_occupied_map(mx, my)

    def line_collision_check(self, x1, y1, x2, y2):
        """
        Bresenham-like discrete line check between two world coordinates.
        Return True if collision exists along segment, False if free.
        """
        mx1, my1 = self.world_to_map(x1, y1)
        mx2, my2 = self.world_to_map(x2, y2)
        # if endpoints out of bounds -> collision
        if not (self.in_bounds_map(mx1, my1) and self.in_bounds_map(mx2, my2)):
            return True
        # Bresenham on grid
        dx = abs(mx2 - mx1)
        dy = abs(my2 - my1)
        x = mx1
        y = my1
        sx = 1 if mx2 >= mx1 else -1
        sy = 1 if my2 >= my1 else -1
        if dx >= dy:
            err = dx // 2
            while x != mx2:
                if self.is_occupied_map(x, y):
                    return True
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
            if self.is_occupied_map(mx2, my2):
                return True
        else:
            err = dy // 2
            while y != my2:
                if self.is_occupied_map(x, y):
                    return True
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
            if self.is_occupied_map(mx2, my2):
                return True
        return False

# ---------- RRT planner ----------
class RRTPlanner:
    def __init__(self, grid: GridMap, start, goal, step_size=0.2, max_iters=5000, goal_sample_rate=0.05):
        self.grid = grid
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.step_size = step_size
        self.max_iters = max_iters
        self.goal_sample_rate = goal_sample_rate  # probability to sample goal directly
        # sampling bounds in world coords
        self.min_x = grid.origin_x
        self.min_y = grid.origin_y
        self.max_x = grid.origin_x + grid.width * grid.resolution
        self.max_y = grid.origin_y + grid.height * grid.resolution

    def sample(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.x, self.goal.y
        x = random.uniform(self.min_x, self.max_x)
        y = random.uniform(self.min_y, self.max_y)
        return x, y

    def nearest(self, nodes, x, y):
        best = None
        best_d = float("inf")
        for n in nodes:
            d = (n.x - x) ** 2 + (n.y - y) ** 2
            if d < best_d:
                best_d = d
                best = n
        return best

    def steer(self, from_node, to_x, to_y):
        dx = to_x - from_node.x
        dy = to_y - from_node.y
        dist = math.hypot(dx, dy)
        if dist <= self.step_size:
            nx = to_x
            ny = to_y
        else:
            nx = from_node.x + dx / dist * self.step_size
            ny = from_node.y + dy / dist * self.step_size
        return nx, ny

    def build(self):
        nodes = [self.start]
        for i in range(self.max_iters):
            rx, ry = self.sample()
            nearest = self.nearest(nodes, rx, ry)
            nx, ny = self.steer(nearest, rx, ry)
            # collision check between nearest and new point
            if self.grid.line_collision_check(nearest.x, nearest.y, nx, ny):
                continue
            new_node = Node(nx, ny, nearest)
            nodes.append(new_node)
            # check if we can connect to goal
            if math.hypot(new_node.x - self.goal.x, new_node.y - self.goal.y) <= self.step_size:
                # check collision from new_node to goal
                if not self.grid.line_collision_check(new_node.x, new_node.y, self.goal.x, self.goal.y):
                    goal_node = Node(self.goal.x, self.goal.y, new_node)
                    return self.extract_path(goal_node)
        return None  # failed

    def extract_path(self, node):
        path = []
        cur = node
        while cur is not None:
            path.append((round(cur.x, 3), round(cur.y, 3)))
            cur = cur.parent
        path.reverse()
        return path

    def smooth_path(self, path, iterations=50):
        """
        Simple shortcut smoothing: try to connect random pairs directly if collision-free.
        Path is list of (x,y). Returns new path.
        """
        if path is None or len(path) < 3:
            return path
        for _ in range(iterations):
            if len(path) < 3:
                break
            i = random.randint(0, len(path) - 3)
            j = random.randint(i + 2, len(path) - 1)
            (x1, y1) = path[i]
            (x2, y2) = path[j]
            if not self.grid.line_collision_check(x1, y1, x2, y2):
                # shortcut
                new = path[:i + 1] + path[j:]
                path = new
        return path

# ---------- Top-level orchestration ----------
def compute_path_length(path):
    if not path or len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(path)):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        total += math.hypot(x1 - x0, y1 - y0)
    return round(total, 2)

def save_paths_to_file(paths_dict, filename=None):
    if filename is None:
        # save_dir = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data/ros_data")
        save_dir = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/paths.json")
        os.makedirs(save_dir, exist_ok=True)
        filename = os.path.join(save_dir, f"rrt_paths_{int(time.time())}.json")
    with open(filename, 'w') as f:
        json.dump(paths_dict, f, indent=4)
    rospy.loginfo(f"Saved paths to {filename}")
    return filename

def wait_for_map_and_build_grid(timeout=10.0):
    rospy.loginfo("Waiting for /map (OccupancyGrid)...")
    try:
        msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timed out waiting for /map")
        return None
    grid = GridMap(msg)
    rospy.loginfo(f"Map received: size {grid.width}x{grid.height}, resolution {grid.resolution}")
    if abs(grid.resolution - 0.1) > 1e-6:
        rospy.logwarn("Map resolution is not 0.05 m as you expected; using map's resolution.")
    return grid

def get_start_from_odom(timeout=5.0):
    rospy.loginfo("Waiting for /odom for start pose...")
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timed out waiting for /odom")
        return None
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rospy.loginfo(f"Start pose from /odom: ({x:.3f}, {y:.3f})")
    return (x, y)

def parse_args():
    parser = argparse.ArgumentParser(description="RRT multi-path planner")
    parser.add_argument("--goal", nargs=2, type=float, help="goal x y in map frame", required=False)
    parser.add_argument("--num_paths", type=int, default=3, help="number of paths to generate")
    parser.add_argument("--max_iters", type=int, default=5000, help="RRT max iterations per run")
    parser.add_argument("--step", type=float, default=0.5, help="RRT step size (meters)")
    parser.add_argument("--out", type=str, default=None, help="output file path")
    return parser.parse_args()

def main():
    args = parse_args()
    rospy.init_node("rrt_multi_paths", anonymous=True)
    # 1) get map
    grid = wait_for_map_and_build_grid(timeout=15.0)
    if grid is None:
        return
    # 2) get start
    start = get_start_from_odom(timeout=5.0)
    if start is None:
        return
    # 3) get goal (from CLI or ask interactively)
    if args.goal:
        goal_x, goal_y = args.goal
    else:
        # interactive input
        try:
            goal_x = float(input("Enter goal x (map frame): ").strip())
            goal_y = float(input("Enter goal y (map frame): ").strip())
        except Exception as e:
            rospy.logerr("Invalid goal input")
            return
    goal = (goal_x, goal_y)
    rospy.loginfo(f"Goal: ({goal_x:.3f}, {goal_y:.3f})")

    # Basic validation: ensure start and goal are in free space
    if not grid.is_free_world(start[0], start[1]):
        rospy.logerr("Start position is in an occupied cell. Aborting.")
        return
    if not grid.is_free_world(goal[0], goal[1]):
        rospy.logerr("Goal position is in an occupied cell. Aborting.")
        return

    # 4) Generate multiple paths
    paths_output = {}
    generated = 0
    attempts = 0
    max_attempts = args.num_paths * 10  # allow retries
    rng_seed_base = int(time.time()) & 0xffffffff
    while generated < args.num_paths and attempts < max_attempts:
        attempts += 1
        seed = rng_seed_base + attempts
        random.seed(seed)
        rospy.loginfo(f"RRT attempt {attempts}, seed={seed}")
        planner = RRTPlanner(grid, start, goal, step_size=args.step, max_iters=args.max_iters, goal_sample_rate=0.1)
        raw_path = planner.build()
        if raw_path is None:
            rospy.logwarn("RRT failed to find a path in this attempt.")
            continue
        # smooth
        smooth_path = planner.smooth_path(raw_path, iterations=10)
        # ensure collision-free for entire path
        collision = False
        for i in range(1, len(smooth_path)):
            if grid.line_collision_check(smooth_path[i-1][0], smooth_path[i-1][1], smooth_path[i][0], smooth_path[i][1]):
                collision = True
                break
        if collision:
            rospy.logwarn("Generated path has collision after smoothing; skipping.")
            continue
        # uniqueness check: ensure path not almost identical to previous ones (by comparing end-to-end distances)
        too_similar = False
        for key, val in paths_output.items():
            prev_path = [(p["position"][0], p["position"][1]) for p in val["path"]]
            # compute average pointwise distance up to min length
            L = min(len(prev_path), len(smooth_path))
            if L >= 2:
                dsum = 0.0
                for i in range(L):
                    dsum += math.hypot(prev_path[i][0] - smooth_path[i][0], prev_path[i][1] - smooth_path[i][1])
                avg = dsum / L
                if avg < 0.2:  # too similar
                    too_similar = True
                    break
        if too_similar:
            rospy.loginfo("Path too similar to earlier one; retrying with different seed.")
            continue
        generated += 1
        key = f"path_1_1_{generated}"
        path_list = [{"position": [float(round(x, 3)), float(round(y, 3))]} for (x, y) in smooth_path]
        length = compute_path_length(smooth_path)
        paths_output[key] = {"path": path_list, "length": length}
        rospy.loginfo(f"Generated path {generated}: {len(path_list)} points, length {length}")
    if generated == 0:
        rospy.logerr("Failed to generate any valid path.")
        return

    # Save results
    out_file = save_paths_to_file(paths_output, filename=args.out)
    rospy.loginfo(f"Finished. Saved {generated} paths to {out_file}")

if __name__ == "__main__":
    
    main()
