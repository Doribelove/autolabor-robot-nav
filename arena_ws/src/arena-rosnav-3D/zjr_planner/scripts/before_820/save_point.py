import rospy
import json
import os
import random
import numpy as np
from nav_msgs.msg import OccupancyGrid
from datetime import datetime
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
import tf.transformations as tft
import time
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped  # 导入PoseStamped消息类型


# 一个预定义的局部栅格地图大小和分辨率
# 20/480 = 0.04166666666
GRID_RESOLUTION = 0.05  # 每个栅格精度和地图一样
MAP_WIDTH = 100  # 栅格地图的宽度（单位：栅格）,激光雷达扫描最大距离为3.5米，3.5/0.05=70,取90
MAP_HEIGHT = 100  # 栅格地图的高度（单位：栅格）
# 全局变量
initial_pose_pub = None
# 全局变量存储数据
point_cloud = []
new_message_received = False 
goal_pose = None
file_path = None
global_path = None
data_queue = []  # 全局队列

# 创建文件函数
def create_a_file(i):
    global file_path

    # 创建保存目录
    save_dir = os.path.expanduser("~/ros_data")
    os.makedirs(save_dir, exist_ok=True)

    file_path = os.path.join(save_dir, f"data_start_{i}.json")

    rospy.loginfo(f"文件创建成功：{file_path}")


# 保存起始点数据
def save_start_point_data_into_the_file(i, start_x, start_y):
    global file_path

    if file_path is None:
        rospy.logwarn("文件尚未创建，无法保存起始点数据！")
        return

    try:
        # 读取现有文件内容
        if not os.path.exists(file_path):
            data = {}  # 如果文件不存在，初始化为空字典
        else:
            with open(file_path, 'r') as f:
                data = json.load(f)

        # 计算start_points的键名（例如start_point1, start_point2）
        key_name = f"start_point{i}"

        # 构建起始点数据
        start_point = {
            "x": start_x,
            "y": start_y
        }

        # 将起始点数据添加到字典中，使用动态生成的键名
        data[key_name] = start_point

        # 将更新后的数据写入文件
        with open(file_path, 'w') as f:
            json.dump(data, f, indent=4)

        rospy.loginfo(f"起始点数据已保存：{start_point}")
    except Exception as e:
        rospy.logerr(f"保存起始点数据时出错：{repr(e)}")


# 保存点云到JSON文件的函数
def save_start_pointcloud_data_into_the_file(i, point_cloud):
    if file_path is None:
        rospy.logwarn("文件尚未创建，无法保存点云数据！")
        return

    try:

        key_name = f"pointcloud{i}"
        # 初始化栅格地图并映射点云
        grid_map = np.zeros((MAP_HEIGHT, MAP_WIDTH), dtype=int)
        for point in point_cloud:
            x, y = point["x"], point["y"]
            # 转换为栅格坐标
            grid_x = int(x / GRID_RESOLUTION + MAP_WIDTH / 2)
            grid_y = int(y / GRID_RESOLUTION + MAP_HEIGHT / 2)
            # 确保点在网格范围内
            if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
                grid_map[grid_y, grid_x] = 100  # 标记为障碍物

        # 加载或初始化文件数据
        data = {}
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                data = json.load(f)

        # 更新文件内容
        data[key_name] = {"grid_map": grid_map.tolist()}
        
        # 使用 compact separators 写入文件，确保紧凑的格式
        with open(file_path, 'w') as f:
            json.dump(data, f, separators=(',', ':'))

        rospy.loginfo(f"点云数据和栅格地图已保存：{key_name}")
    except Exception as e:
        rospy.logerr(f"保存点云数据时出错：{repr(e)}")


def save_goal_point_data_into_queue(i, j, goal_x, goal_y):
    try:
        # 构建目标点数据，键名为 goal_points_{i}_{j}
        key_name = f"goal_points_{i}_{j}"

        # 创建目标点数据结构
        goal_point_data = {
                "x": goal_x,
                "y": goal_y
        }

        # 将目标点数据添加到全局队列中
        data_queue.append({key_name:goal_point_data})

        rospy.loginfo(f"目标点数据已添加到队列：{goal_point_data}")
    except Exception as e:
        rospy.logerr(f"添加目标点数据到队列时出错：{repr(e)}")

# 计算路径总长度
def calculate_path_length(start_x, start_y,processed_global_path):
    if not processed_global_path or len(processed_global_path) == 0:
        rospy.logwarn("全局路径为空，无法计算路径长度！")
        return 0.0

    # 初始点到第一个航点的距离
    total_length = math.sqrt((start_x - processed_global_path[0]["position"][0])**2 +
                             (start_y - processed_global_path[0]["position"][1])**2)

    # 计算航点之间的距离
    for i in range(1, len(processed_global_path)):
        prev = processed_global_path[i - 1]["position"]
        curr = processed_global_path[i]["position"]
        segment_length = math.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)
        total_length += segment_length

    # 返回保留两位小数的路径长度
    return round(total_length, 2)

# 将路径数据保存到全局队列
def save_path_data_to_queue(i, j, k, start_x, start_y):
    global global_path
    if global_path is None:
        rospy.logwarn("路径数据为空，无法保存到队列！")
        return
    if len(global_path) > 10:
        global_path = global_path[:len(global_path) // 2]
    # 处理 global_path 中的航点数据，保留三位小数
    processed_global_path = [
        {
            "position": [
                round(point["position"][0], 3),
                round(point["position"][1], 3)
            ]
        } for point in global_path
    ]

    # 计算路径长度
    path_length = calculate_path_length(start_x, start_y,processed_global_path)

    # 创建路径数据结构
    key = f"path_{i}_{j}_{k}"
    path_data = {
        "path": processed_global_path,
        "length": round(path_length, 2)  # 路径长度保留两位小数
    }

    # 将路径数据保存到队列
    data_queue.append({key: path_data})
    rospy.loginfo(f"第 {i}_{j}_{k} 条路径数据已保存到队列：路径长度为 {path_length}")


# 监听导航目标点
def goal_pose_callback(msg):
    global goal_pose
    position = msg.pose.position
    orientation = msg.pose.orientation
    goal_pose = {
        "position": [position.x, position.y],
        "orientation": [orientation.x, orientation.y, orientation.z, orientation.w]
    }
    rospy.loginfo("导航目标已更新！")

# 监听全局路径
def path_callback(msg):
    global global_path
    global_path = [{
        "position": [pose.pose.position.x, pose.pose.position.y],
    } for pose in msg.poses]
    rospy.loginfo("全局路径已更新！")
    global new_message_received
    rospy.loginfo("接收到新的 Path 消息")
    new_message_received = True
# 激光雷达回调函数
def scan_callback(j,msg):
    global point_cloud

    # 清空点云列表以便存储新数据
    point_cloud = []

    # 获取扫描角度和范围
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    ranges = msg.ranges  # 获取激光扫描的距离值

    for i, r in enumerate(ranges):
        if r != float('Inf') and r > msg.range_min and r < msg.range_max:
            # 计算该点的相对于激光雷达的 x, y 坐标
            angle = angle_min + i * angle_increment
            x = round(r * math.cos(angle),2)
            y = round(r * math.sin(angle),2)

            # 添加到点云列表
            point_cloud.append({"x": x, "y": y})
    rospy.sleep(0.1) 
    save_start_pointcloud_data_into_the_file(j,point_cloud)#往file_path下的文件追加点云信息
    rospy.loginfo(f"扫描数据接收完毕，当前点云数：{len(point_cloud)}")

def map_callback(i,msg):
    # 将地图数据转换为二维矩阵形式
    map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width).tolist()
    rospy.loginfo(f"Map received with size {msg.info.width}x{msg.info.height}")
    return select_random_points_from_map(i,map_data)


def select_random_points_from_map(i, map_data):
    if map_data is None:
        rospy.logwarn("No map data available for selecting points.")
        return {}

    # 找到值为 0 的点的位置
    free_points = [(x, y) for y, row in enumerate(map_data) for x, value in enumerate(row) if value == 0]

    # 检查是否有足够的可用点
    if len(free_points) < i:
        rospy.logwarn("Not enough free points (value 0) in the map.")
        return {}

    # 定义一个检查点周围4格内是否有-1或100的函数
    def is_area_clear(point, map_data, radius=4):
        x, y = point
        # 检查以该点为中心的方圆4格范围
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy
                # 确保坐标在地图范围内，并检查该位置的值
                if 0 <= nx < len(map_data[0]) and 0 <= ny < len(map_data):
                    if map_data[ny][nx] in [-1, 100]:
                        return False
        return True

    # 随机选择 i 个符合条件的点
    selected_points = []
    attempts = 0
    while len(selected_points) < i and attempts < 1000:
        attempts += 1
        point = random.choice(free_points)
        if is_area_clear(point, map_data):
            selected_points.append(point)

    # 如果选择失败，输出警告
    if len(selected_points) < i:
        rospy.logwarn("Failed to select enough points with the required conditions.")

    # 构造结果字典
    points_dict = {
        f"point_{i+1}": {"x": round(point[0]/24-10, 2), "y": round(point[1]/24-10, 2)} for i, point in enumerate(selected_points)
    }
    return points_dict





# 设置机器人在 Gazebo 中的初始位置
def set_robot_position(x, y, yaw):
    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = 'turtlebot3_waffle'  # 根据实际模型名称修改
        quaternion = tft.quaternion_from_euler(0, 0, yaw)
        model_state.pose = Pose(Point(x, y, 0), Quaternion(*quaternion))
        set_model_state(model_state)
        rospy.loginfo(f"Robot moved to position ({x}, {y}) with yaw {yaw}.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

# 发布初始位姿到导航栈
def publish_initial_pose(x, y, yaw):
    global initial_pose_pub
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = rospy.Time.now()

    quaternion = tft.quaternion_from_euler(0, 0, yaw)
    initial_pose.pose.pose = Pose(Point(x, y, 0), Quaternion(*quaternion))

    # 设置协方差为一个较小的值，表示定位精度高
    initial_pose.pose.covariance = [0.1] * 36

    initial_pose_pub.publish(initial_pose)
    rospy.sleep(0.1)
    rospy.loginfo(f"Initial pose published to ({x}, {y}) with yaw {yaw}.")

# 发布导航目标点
def send_navigation_goal(x, y, yaw):
    nav_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # 等待发布器准备好

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = rospy.Time.now()

    quaternion = tft.quaternion_from_euler(0, 0, yaw)
    goal_pose.pose = Pose(Point(x, y, 0), Quaternion(*quaternion))

    nav_goal_pub.publish(goal_pose)
    rospy.loginfo(f"Navigation goal sent to ({x}, {y}) with yaw {yaw}.")

def get_point_coordinates(points_dict, i):
    # 将数字键 i 转换为字符串格式的键
    key = f"point_{i}"
    # 检查字典中是否存在键 key
    if key in points_dict:
        x = points_dict[key]["x"]
        y = points_dict[key]["y"]
        return x, y
    else:
        raise KeyError(f"Key {key} not found in points_dict.")

# 将全局队列数据写入文件并清空队列
def flush_queue_to_file():
    global data_queue, file_path

    if not data_queue:
        rospy.loginfo("队列为空，无需写入文件。")
        return

    if file_path is None:
        rospy.logwarn("文件路径未定义，无法保存队列数据！")
        return

    try:
        # 如果文件存在，读取现有数据
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                data_package = json.load(f)
        else:
            data_package = {}

        # 将队列数据合并到文件数据
        for item in data_queue:
            data_package.update(item)

        # 写回文件
        with open(file_path, 'w') as f:
            json.dump(data_package, f, indent=4)

        rospy.loginfo(f"队列数据已写入文件：{file_path}")

        # 清空队列
        data_queue.clear()

    except Exception as e:
        rospy.logerr(f"写入队列数据到文件时出错：{repr(e)}")

def has_new_path_message():
    """
    判断是否接收到新的 Path 消息。
    如果接收到新消息，则返回 True，并重置标志位为 False。
    否则返回 False。
    """
    global new_message_received
    if new_message_received:
        new_message_received = False  # 重置标志位
        return True
    return False
def calculate_start_coordinates(i):
    if i <= 25:
        start_x = round(7 - (i / 25) * 16, 2)
        start_y = 7
    elif i <= 50:
        start_x = round(7 - ((i - 25) / 25) * 16, 2)
        start_y = 5
    elif i <= 75:
        start_x = round(7 - ((i - 50) / 25) * 16, 2)
        start_y = -3
    elif i<=100:
        start_x = round(7 - ((i - 75) / 25) * 16, 2)
        start_y = -6
    elif i<=130:
        start_x = 3
        start_y = round(7 - ((i - 100) / 15) * 14, 2)
    else :
        start_x = -4
        start_y = round(7 - ((i - 130) / 15) * 14, 2)
    
    return start_x, start_y

def generate_nodes_between_points(point1, point2, num_nodes):
    if num_nodes < 2:
        raise ValueError("节点数必须大于等于 2")
    
    x1, y1 = point1
    x2, y2 = point2
    
    # 计算每个方向的步长
    step_x = (x2 - x1) / (num_nodes - 1)
    step_y = (y2 - y1) / (num_nodes - 1)
    
    # 生成节点列表
    nodes = [(round((x1 + i * step_x),3), round((y1 + i * step_y),3)) for i in range(num_nodes)]
    return nodes

if __name__ == "__main__":
    rospy.init_node('data_recording_node', anonymous=True)
    rospy.sleep(1)  # 等待发布器准备好
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    start_yaw = 1
    #生成160个点

    # 场景测试1：
    point1=generate_nodes_between_points((10.6,10.6),(-2,10.8),20)
    point2=generate_nodes_between_points((-11,10.6),(-5,5),20)
    point3=generate_nodes_between_points((-1,5),(9,3),20)
    point4=generate_nodes_between_points((10,0),(-4,0),20)
    point5=generate_nodes_between_points((0,0),(-2,-9),20)
    point6=generate_nodes_between_points((9,-5),(-2,-5),20)
    point7=generate_nodes_between_points((-10,-10.4),(10,-10.4),20)

    all_points = point1 + point2 + point3 + point4 + point5 + point6 + point7
    # 等待必要数据
    rospy.sleep(1)  # 等待数据初始化

    # 订阅路径话题
    rospy.Subscriber("/move_base/PathPlanner/plan", Path, path_callback)
    rospy.sleep(0.2)

    for i in range(1,101):
        create_a_file(i)  # 创建文件
        # 订阅/scan话题 
        start_x,start_y=all_points[i]
        rospy.sleep(0.1)
        save_start_point_data_into_the_file(i,start_x,start_y) #i用来记录是第几个点，往file_path下的文件追加起始点信息。
        # points_dict_goal=map_callback(10,latest_message_map)#用来在地图可通行区域随机获取10个点
        set_robot_position(start_x, start_y, start_yaw)
        rospy.sleep(0.2) 
        # 发布初始位姿到导航栈
        publish_initial_pose(start_x, start_y, start_yaw)
        rospy.sleep(0.2)
        latest_message_scan = rospy.wait_for_message("/scan",LaserScan)
        scan_callback(i,latest_message_scan)
        rospy.sleep(0.2)  
        n = 1
        sampled_points = random.sample(all_points, 20)
        for j in range(1,20):
            if n == 11:
                break
            goal_x,goal_y=sampled_points[j]
            goal_yaw=0
            k = 1
            h = 1
            if n<=10:
                save_goal_point_data_into_queue(i,n,goal_x,goal_y)#往file_path下的文件追加目标点信息
            rospy.sleep(0.2)      
            rospy.loginfo("等待路径更新...")

            send_navigation_goal(goal_x, goal_y, goal_yaw)
            rospy.sleep(0.2)  
            while k <= 30:
                if has_new_path_message() and h<=3 and n<=10:
                    save_path_data_to_queue(i, n, h, start_x, start_y) # #往file_path下的文件追加路径信息
                    rospy.sleep(0.1)
                    h+=1
                rospy.loginfo(f"等待路径更新 (尝试 {k}/12)...")
                rospy.sleep(0.05)  # 模拟路径更新的等待
                k+=1
                if h ==4 :
                    k=31
            if h == 3:  # 修改逻辑，让 m 的意义更加明确
                if data_queue:
                    latest_item = data_queue.pop()  # 删除队列最新元素
                    rospy.loginfo(f"删除三个最新队列元素，第一个是废物路径：{latest_item}")
                    latest_item = data_queue.pop()  # 删除队列最新元素
                    rospy.loginfo(f"删除三个最新队列元素，第二个是废物路径：{latest_item}")
                    latest_item = data_queue.pop()  # 删除队列最新元素
                    rospy.loginfo(f"删除三最新队列元素，第三个是废物目标点：{latest_item}")
            if h == 2:  # 修改逻辑，让 m 的意义更加明确
                if data_queue:
                    latest_item = data_queue.pop()  # 删除队列最新元素
                    rospy.loginfo(f"删除两个最新队列元素，第一个是废物路径：{latest_item}")
                    latest_item = data_queue.pop()  # 删除队列最新元素
                    rospy.loginfo(f"删除两个最新队列元素，第二个是废物目标点：{latest_item}")
            if h == 1:  # 修改逻辑，让 m 的意义更加明确
                if data_queue:                
                    latest_item = data_queue.pop()  # 删除队列最新元素
                    rospy.loginfo(f"删除最新队列元素，废物目标点：{latest_item}")
            if h == 4:
                n+=1
        flush_queue_to_file()
        rospy.sleep(0.5) 

    rospy.spin()