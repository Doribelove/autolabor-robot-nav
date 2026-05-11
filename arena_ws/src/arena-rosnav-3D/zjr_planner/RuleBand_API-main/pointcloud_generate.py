#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
import os
import math
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
DEFAULT_INDEX = 1
DEFAULT_SCAN_TOPIC = "/scan"
DEFAULT_WIDTH = 100
DEFAULT_HEIGHT = 100
DEFAULT_RESOLUTION = 0.1
DEFAULT_TIMEOUT = 10.0

def create_file_for_index(index):
    save_dir = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json")
    os.makedirs(save_dir, exist_ok=True)
    file_path = os.path.join(save_dir, f"data_start_{index}.json")
    if not os.path.exists(file_path):
        with open(file_path, 'w') as f:
            json.dump({}, f)
    return file_path

def ranges_to_pointcloud(scan_msg):
    points = []
    angle = scan_msg.angle_min
    for r in scan_msg.ranges:
        if not math.isfinite(r) or r <= scan_msg.range_min or r >= scan_msg.range_max:
            angle += scan_msg.angle_increment
            continue
        x = round(r * math.cos(angle), 2)
        y = round(r * math.sin(angle), 2)
        points.append({"x": x, "y": y})
        angle += scan_msg.angle_increment
    return points

def build_grid_map_from_points(points, grid_resolution, width, height):
    grid_map = np.zeros((height, width), dtype=int)
    half_w = width / 2.0
    half_h = height / 2.0
    for p in points:
        grid_x = int(p["x"] / grid_resolution + half_w)
        grid_y = int(p["y"] / grid_resolution + half_h)
        if 0 <= grid_x < width and 0 <= grid_y < height:
            grid_map[grid_y, grid_x] = 100
    return grid_map

def save_pointcloud_grid_to_file(file_path, index, grid_map):
    key_name = f"pointcloud{index}"
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
    except Exception:
        data = {}
    data[key_name] = {"grid_map": grid_map.tolist()}
    with open(file_path, 'w') as f:
        json.dump(data, f, separators=(',', ':'))

def generate_pointcloud_grid(index=DEFAULT_INDEX,
                              scan_topic=DEFAULT_SCAN_TOPIC,
                              width=DEFAULT_WIDTH,
                              height=DEFAULT_HEIGHT,
                              resolution=DEFAULT_RESOLUTION,
                              timeout=DEFAULT_TIMEOUT):
    """订阅一次 LaserScan 并保存为 JSON，返回保存文件路径"""
    if not rospy.core.is_initialized():
        rospy.init_node("pointcloud_generate_once", anonymous=True)

    file_path = create_file_for_index(index)
    scan_msg = rospy.wait_for_message(scan_topic, LaserScan, timeout=timeout)
    points = ranges_to_pointcloud(scan_msg)
    grid_map = build_grid_map_from_points(points, resolution, width, height)
    save_pointcloud_grid_to_file(file_path, index, grid_map)
    return file_path


def ranges_to_grid(scan_msg, resolution=0.05, width=100, height=100):
    """将激光雷达数据转换成 100/0 栅格"""
    grid_map = np.zeros((height, width), dtype=int)
    half_w = width / 2.0
    half_h = height / 2.0

    angle = scan_msg.angle_min
    for r in scan_msg.ranges:
        if not math.isfinite(r) or r <= scan_msg.range_min or r >= scan_msg.range_max:
            angle += scan_msg.angle_increment
            continue
        x = round(r * math.cos(angle), 2)
        y = round(r * math.sin(angle), 2)
        grid_x = int(x / resolution + half_w)
        grid_y = int(y / resolution + half_h)
        if 0 <= grid_x < width and 0 <= grid_y < height:
            grid_map[grid_y, grid_x] = 100
        angle += scan_msg.angle_increment
    return grid_map

def live_lidar_plot(scan_topic="/scan", resolution=0.05, width=100, height=100):
    """实时读取 /scan 并绘制 100=红色, 0=白色"""
    if not rospy.core.is_initialized():
        rospy.init_node("lidar_live_plot", anonymous=True)

    plt.ion()  # 开启交互模式
    fig, ax = plt.subplots()
    img_display = ax.imshow(np.zeros((height, width)), cmap="RdBu_r", vmin=0, vmax=100)
    ax.set_title("LaserScan Grid Map (100=Red, 0=White)")
    plt.show()

    try:
        while not rospy.is_shutdown():
            scan_msg = rospy.wait_for_message(scan_topic, LaserScan, timeout=1.0)
            grid_map = ranges_to_grid(scan_msg, resolution, width, height)

            # 把 0 映射为白色，100 映射为红色
            cmap = plt.cm.get_cmap("Reds", 2)
            img_display.set_data(grid_map)
            img_display.set_cmap(cmap)

            plt.draw()
            plt.pause(0.01)  # 更新画面
    except rospy.ROSInterruptException:
        pass
    finally:
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    try:
        path = generate_pointcloud_grid()
        rospy.loginfo(f"点云 -> 栅格 保存到 {path}")
    except rospy.ROSException:
        rospy.logerr("等待 LaserScan 超时，请确认话题存在。")
