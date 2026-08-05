# Autolabor 启动交接

当前主流程：**GPS 导航 + Livox/FAST_LIO + ZED 2 深度相机 + YOLO11/FOD + RabbitMQ + Qt 操作台**。

## 1. 启动前

1. 车辆架空或停在安全区域，物理急停可随时使用。
2. 确认 CAN、GPS 和 ZED：

```bash
ls -l /dev/ttyUSB0 /dev/ttyUSB1
lsusb | grep -i 2b03
```

默认设备：`/dev/ttyUSB0` 为 CAN，`/dev/ttyUSB1` 为 GPS，ZED 序列号为 `23748636`。

修改源码后才需要重新编译：

```bash
cd /home/slam/robot_ws
BUILD_JOBS=4 ./scripts/build_workspace.sh
```

## 2. 整体启动（推荐）

```bash
cd /home/slam/robot_ws
./scripts/operator_all_in_one.sh 0.3 cruise
```

- `0.3`：最高速度，首次实车固定使用低速。
- `cruise`：开阔环境；密集固定障碍可改为 `obstacle`。
- 脚本会自动处理 ROS master，并启动导航、ZED+YOLO、RabbitMQ 和 Qt。
- 终端出现 `Robot bringup is running in gps mode.` 才表示导航完整就绪。
- Qt 中应看到 ROS、CAN、GPS、激光和 `move_base` 在线，控制模式为 `GPS_ACTIVE`。
- 运行日志位于 `log/operator_all_in_one_时间戳/`。

快速检查：

```bash
rostopic hz /gps/odom
rostopic hz /scan
rostopic hz /fod_camera/image_raw
rostopic hz /fod_camera/depth_registered
rostopic hz /fod/detections
./scripts/fod_mode.sh status
```

关闭 Qt 窗口或在启动终端按 `Ctrl+C`，会停止该脚本启动的全部进程。

常用裁剪启动：

```bash
# 不启动 RabbitMQ
OPERATOR_START_RABBITMQ=false ./scripts/operator_all_in_one.sh 0.3 cruise

# 不启动 ZED 和 YOLO
OPERATOR_START_VISION=false ./scripts/operator_all_in_one.sh 0.3 cruise
```

## 3. 分模块启动与检查

仅用于开发和故障定位。每个终端先执行：

```bash
cd /home/slam/robot_ws
source .deps/setup.bash
```

执行 `.deps/setup.bash` 后不要再次执行 `source devel/setup.bash`。

### 3.1 ROS master

```bash
roscore
```

检查：

```bash
rosnode list
```

### 3.2 CAN 与 M2 底盘

```bash
roslaunch robot_bringup can.launch port_name:=/dev/ttyUSB0 publish_tf:=true
```

检查：

```bash
rosnode info /canbus_driver
rosnode info /m2_driver
rostopic hz /odom
```

### 3.3 GPS 导航主链

```bash
NAV_START_RVIZ=false ./scripts/bringup.sh gps 0.3 cruise
```

检查：

```bash
rostopic hz /gps/fix
rostopic hz /gps/heading
rostopic hz /gps/odom
rostopic hz /scan
rosnode info /move_base
./scripts/fod_mode.sh status
```

以启动终端出现 `Robot bringup is running in gps mode.` 为最终判据。该命令已包含 CAN/M2，不能再同时启动 3.2。

### 3.4 ZED 相机（仅相机）

```bash
./scripts/zed2_camera.sh
```

检查：

```bash
rostopic hz /fod_camera/image_raw
rostopic hz /fod_camera/depth_registered
rostopic echo -n 1 /fod_camera/depth_registered | head -n 12
```

应持续收到 RGB 和深度图；深度图 `encoding` 应为 `32FC1`。

### 3.5 ZED + YOLO + 深度融合

相机未启动时：

```bash
roslaunch autolabor_fod_vision zed_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=false
```

相机已按 3.4 启动时，将 `start_camera` 改为 `false`。

检查：

```bash
rostopic hz /fod/detections
rostopic echo -n 1 /fod/detections
rqt_image_view /fod/debug/image
```

检查消息中 `depth_synchronized: true`；有 FOD 时，每个目标应包含 `depth_valid` 和 `depth_m`。多目标控制会优先选择有效深度最近的 FOD。

### 3.6 Qt 操作台

```bash
./scripts/operator_gui.sh
```

检查：

```bash
rosnode info /autolabor_operator_gui
```

Qt 单独启动不会启动导航或相机；按钮变灰时按界面提示检查上游模块。

### 3.7 RabbitMQ 桥（可选）

```bash
./scripts/rabbitmq_gps_goal_bridge.py
```

检查：

```bash
rosnode info /rabbitmq_gps_goal_bridge
rostopic echo -n 1 /rabbitmq_bridge/status
```

## 4. FOD 相机控制模式

必须先完成 GPS 导航和 ZED+YOLO 启动，并确认车辆周围安全。

```bash
./scripts/fod_mode.sh status   # 当前模式
./scripts/fod_mode.sh start    # 最近 FOD <5m 才切入；否则保持 GPS
./scripts/fod_mode.sh watch    # 持续观察状态
./scripts/fod_mode.sh stop     # 退出 FOD，恢复保留的 GPS 路线
```

辅助检查：

```bash
rostopic echo /fod_visual_servo/state
rostopic echo /fod_visual_servo/status
```

集成模式下不要直接调用 `/fod_visual_servo/set_enabled`，统一使用 `fod_mode.sh`，由 GPS/FOD 仲裁器保证 `/cmd_vel` 唯一。

入口判定：最近有效深度 FOD 严格小于 `5 m` 才暂停 GPS；目标在 `5 m` 外或连续 `1 s` 无有效深度识别时继续 GPS。进入后，目标沿图像中线向下消失，再直行 `0.5 m` 通过滚轴即完成并恢复原 GPS 路线。

## 5. 最短故障定位

- 无串口：检查供电、USB 线和 `/dev/ttyUSB*` 编号。
- 无 `/gps/odom`：检查双天线航向，正常要求 `SOL_COMPUTED + NARROW_INT`。
- 无 ZED 图像：检查 `lsusb | grep -i 2b03`，再查看启动终端报错。
- 有图像无检测：检查 YOLO Python 环境及 `/fod/debug/image`。
- 有检测无深度：检查 `/fod_camera/depth_registered` 频率和 `depth_synchronized`。
- FOD 模式被拒绝：先执行 `./scripts/fod_mode.sh status`，不要绕过安全服务。
- 一键启动异常：优先查看最新目录中的 `bringup.log`、`vision.log`、`gui.log` 和 `rabbitmq.log`。

不要同时启动两套导航、两个 ZED 节点或多个底盘控制节点；分模块排查结束后先全部停止，再恢复一键启动。
