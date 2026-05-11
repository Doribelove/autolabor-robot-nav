#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
import os
import time
import math
import random
from threading import Event
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point

class Node:
    def __init__(self, x, y, parent=None):
        self.x = float(x)
        self.y = float(y)
        self.parent = parent

class GridMap:
    def __init__(self, occupancy_grid_msg: OccupancyGrid = None):
        if occupancy_grid_msg:
            self.update_from_msg(occupancy_grid_msg)
    
    def update_from_msg(self, msg: OccupancyGrid):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.grid_data = list(msg.data)
        
        # Create 2D grid representation
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        for y in range(self.height):
            for x in range(self.width):
                idx = y * self.width + x
                self.grid[y][x] = self.grid_data[idx]

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
        mx1, my1 = self.world_to_map(x1, y1)
        mx2, my2 = self.world_to_map(x2, y2)
        if not (self.in_bounds_map(mx1, my1) and self.in_bounds_map(mx2, my2)):
            return True
        
        dx = abs(mx2 - mx1)
        dy = abs(my2 - my1)
        x, y = mx1, my1
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
            return self.is_occupied_map(mx2, my2)
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
            return self.is_occupied_map(mx2, my2)

class RRTPlanner:
    def __init__(self, grid: GridMap, start, goal, step_size=0.1, max_iters=5000, goal_sample_rate=0.05):
        self.grid = grid
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.step_size = step_size
        self.max_iters = max_iters
        self.goal_sample_rate = goal_sample_rate
        self.min_x = grid.origin_x
        self.min_y = grid.origin_y
        self.max_x = grid.origin_x + grid.width * grid.resolution
        self.max_y = grid.origin_y + grid.height * grid.resolution

    def sample(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.x, self.goal.y
        return random.uniform(self.min_x, self.max_x), random.uniform(self.min_y, self.max_y)

    def nearest(self, nodes, x, y):
        return min(nodes, key=lambda n: (n.x - x)**2 + (n.y - y)**2)

    def steer(self, from_node, to_x, to_y):
        dx, dy = to_x - from_node.x, to_y - from_node.y
        dist = math.hypot(dx, dy)
        if dist <= self.step_size:
            return to_x, to_y
        return from_node.x + dx/dist * self.step_size, from_node.y + dy/dist * self.step_size

    def build(self):
        nodes = [self.start]
        for _ in range(self.max_iters):
            rx, ry = self.sample()
            nearest = self.nearest(nodes, rx, ry)
            nx, ny = self.steer(nearest, rx, ry)
            
            if self.grid.line_collision_check(nearest.x, nearest.y, nx, ny):
                continue
                
            new_node = Node(nx, ny, nearest)
            nodes.append(new_node)
            
            if math.hypot(new_node.x - self.goal.x, new_node.y - self.goal.y) <= self.step_size:
                if not self.grid.line_collision_check(new_node.x, new_node.y, self.goal.x, self.goal.y):
                    goal_node = Node(self.goal.x, self.goal.y, new_node)
                    return self.extract_path(goal_node)
        return None

    def extract_path(self, node):
        path = []
        while node:
            path.append((round(node.x, 3), round(node.y, 3)))
            node = node.parent
        return path[::-1]

    def smooth_path(self, path, iterations=50):
        if not path or len(path) < 3:
            return path
            
        for _ in range(iterations):
            if len(path) < 3:
                break
            i = random.randint(0, len(path) - 3)
            j = random.randint(i + 2, len(path) - 1)
            x1, y1 = path[i]
            x2, y2 = path[j]
            
            if not self.grid.line_collision_check(x1, y1, x2, y2):
                path = path[:i+1] + path[j:]
        return path

class RRTMultiPathPlanner:
    def __init__(self, one_shot=True):
        rospy.init_node('rrt_multi_path_planner', anonymous=True)
        self.one_shot = one_shot  # 是否只执行一次规划
        self.map_received = False
        self.start_received = False
        self.grid_map = None
        self.start_point = None
        
        # ROS subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("RRTMultiPathPlanner initialized. Waiting for map and odom...")
    
    def map_callback(self, msg):
        if not self.map_received:
            self.grid_map = GridMap(msg)
            self.map_received = True
            rospy.loginfo(f"Map received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f}m/pixel")
    
    def odom_callback(self, msg):
        if not self.start_received:
            self.start_point = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            )
            self.start_received = True
            rospy.loginfo(f"Start position received: ({self.start_point[0]:.2f}, {self.start_point[1]:.2f})")
            
            # 如果是一次性模式，取消订阅
            if self.one_shot and self.map_received:
                self.map_sub.unregister()
                self.odom_sub.unregister()
    
    def wait_for_data(self, timeout=15.0):
        """阻塞直到获取地图和起点位置"""
        rospy.loginfo("Waiting for map and start position...")
        start_time = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():
            if self.map_received and self.start_received:
                # 如果是一次性模式，取消订阅
                if self.one_shot:
                    self.map_sub.unregister()
                    self.odom_sub.unregister()
                return True
            
            if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.logerr("Timed out waiting for map and/or start position")
                return False
            
            rospy.sleep(0.1)
        return False
    
    def plan_paths(self, goal_point, num_paths=3, step_size=0.5, max_iters=5000, output_dir=None):
        """
        生成多条RRT路径
        
        参数:
            goal_point: (x, y) 目标点坐标
            num_paths: 要生成的路径数量
            step_size: RRT步长 (米)
            max_iters: 每条路径的最大迭代次数
            output_dir: JSON输出目录 (默认: ~/path_data)
        
        返回:
            (success, paths_dict, output_file)
        """
        # 等待必要数据
        if not self.wait_for_data():
            return False, None, None
        
        # 验证起点和目标点
        if not self.grid_map.is_free_world(*self.start_point):
            rospy.logerr(f"Start position {self.start_point} is occupied!")
            return False, None, None
        
        if not self.grid_map.is_free_world(*goal_point):
            rospy.logerr(f"Goal position {goal_point} is occupied!")
            return False, None, None
        
        rospy.loginfo(f"Planning {num_paths} paths from {self.start_point} to {goal_point}")
        
        # 路径生成
        paths_output = {}
        generated = 0
        attempts = 0
        max_attempts = num_paths * 10
        rng_seed_base = int(time.time()) & 0xffffffff
        
        while generated < num_paths and attempts < max_attempts:
            attempts += 1
            random.seed(rng_seed_base + attempts)
            
            planner = RRTPlanner(
                self.grid_map,
                self.start_point,
                goal_point,
                step_size=step_size,
                max_iters=max_iters,
                goal_sample_rate=0.1
            )
            
            raw_path = planner.build()
            if not raw_path:
                continue
                
            smooth_path = planner.smooth_path(raw_path, iterations=80)
            
            # 碰撞检查
            collision = False
            for i in range(1, len(smooth_path)):
                if self.grid_map.line_collision_check(*smooth_path[i-1], *smooth_path[i]):
                    collision = True
                    break
            if collision:
                continue
                
            # 相似性检查
            too_similar = False
            for existing_path in paths_output.values():
                prev_path = [(p["position"][0], p["position"][1]) for p in existing_path["path"]]
                L = min(len(prev_path), len(smooth_path))
                if L < 2:
                    continue
                dsum = sum(math.hypot(prev_path[i][0]-smooth_path[i][0], 
                                prev_path[i][1]-smooth_path[i][1]) 
                        for i in range(L))
                if dsum / L < 0.2:
                    too_similar = True
                    break
            if too_similar:
                continue
                
            # 添加新路径
            generated += 1
            key = f"path_{generated}"
            path_list = [{"position": [x, y]} for (x, y) in smooth_path]
            length = self.compute_path_length(smooth_path)
            paths_output[key] = {"path": path_list, "length": length}
            rospy.loginfo(f"Generated path {generated}: {length:.2f}m, {len(path_list)} points")
        
        if not paths_output:
            rospy.logerr("Failed to generate any valid path")
            return False, None, None
        
        # 保存结果
        output_file = self.save_paths(paths_output, output_dir)
        return True, paths_dict, output_file
    
    @staticmethod
    def compute_path_length(path):
        return round(sum(math.hypot(x1-x0, y1-y0) 
                    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:])), 2)
    
    @staticmethod
    def save_paths(paths_dict, output_dir=None):
        if output_dir is None:
            output_dir = os.path.join(os.path.expanduser("~"), "path_data")
        os.makedirs(output_dir, exist_ok=True)
        
        filename = os.path.join(output_dir, f"rrt_paths_{int(time.time())}.json")
        with open(filename, 'w') as f:
            json.dump(paths_dict, f, indent=4)
        
        rospy.loginfo(f"Saved {len(paths_dict)} paths to {filename}")
        return filename

# 示例使用方式
if __name__ == "__main__":
    # 创建路径规划器实例
    planner = RRTMultiPathPlanner()
    
    # 设置目标点 (可来自命令行参数或用户输入)
    goal_point = (5.0, 3.0)  # 示例目标点
    
    # 生成路径
    success, paths, output_file = planner.plan_paths(
        goal_point=goal_point,
        num_paths=3,
        step_size=0.5,
        max_iters=5000,
        output_dir="~/path_data"
    )
    
    if success:
        rospy.loginfo(f"Path planning completed. Results saved to {output_file}")
        # 打印第一条路径的起点和终点
        first_path = list(paths.values())[0]["path"]
        start_pos = first_path[0]["position"]
        end_pos = first_path[-1]["position"]
        rospy.loginfo(f"First path: from ({start_pos[0]:.2f}, {start_pos[1]:.2f}) to ({end_pos[0]:.2f}, {end_pos[1]:.2f})")
    else:
        rospy.logerr("Path planning failed")
    
    rospy.spin()