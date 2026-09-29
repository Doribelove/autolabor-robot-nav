# Autolabor FOD Vision

这是一个与底盘控制隔离的 ROS Noetic 感知旁路，覆盖硬件到位前可以完成的三部分：

1. 独立 Ultralytics/YOLO/CUDA 环境；
2. 图片、视频和笔记本 V4L2 摄像头推理；
3. ROS 图像、检测、地面投影、多目标跟踪和调试接口。

本包没有 `/cmd_vel` 发布者，不会接管无人车。

Jetson ZED 完整启动现已默认消费 `/fod_camera/depth_registered` 和原生
CameraInfo，向每个真实检测框附加 `track_id`、`depth_valid`、`depth_m`
（光轴深度）和 `range_m`（距相机直线距离，米）。彩色和注册深度按原图时间戳
配对，中央框区域的有效深度取中位数；无可靠深度时仍输出识别框，距离无效，
不把缺失测距当成最近目标。Qt 同时显示编号与距离，当前锁定目标标注 TARGET。
控制使用独立检测跟踪编号，预览的平滑/保留框不参与控制。

ZED 相机回收每次只选择最近的一个有效目标，锁定后不因其他目标的距离或
置信度变化而切换，完成后停车。启动和测距说明见工作区 `ZED_PERFORMANCE.md`。
`FodDetection` 消息增加字段后需重新生成消息并重编译 Qt，本机已按此部署；
模型、类别和原 ROI 不变。

## 重要边界

`/home/robot/robot_ws/src/yolo/yolo11n.pt` 和 `yolo11s.pt` 是 COCO 通用
权重，只用于验证 CUDA、媒体读取和 ROS 推理链路。赵工提供的生产候选权重
位于：

```text
/home/robot/robot_ws/src/yolo/fod_yolo11n_img640_e300_orig/weights/best.pt
SHA256 7bf99d4c61343e8cdb37289f2eece6cf18342b508f9b7f80723592edce398500
```

模型的实际类别为 `Metal`、`Soft`、`Plastic`、`Wire`、`Tool`、`w`。生产
launch 会在反序列化前校验上述 SHA256、逐项校验类别并关闭
`smoke_test_only`；其中 `w` 的业务含义仍需向模型提供方确认。

`.pt` 会经过 Python pickle 反序列化，只加载可信来源并核对 SHA256。

## 安装

```bash
cd /home/robot/robot_ws
./scripts/setup_fod_yolo_env.sh
catkin_make --pkg autolabor_fod_msgs autolabor_fod_vision -j2
source devel/setup.bash
```

安装脚本默认要求 CUDA 可用，否则会失败；仅在明确进行 CPU 调试时使用
`FOD_ALLOW_CPU=1 ./scripts/setup_fod_yolo_env.sh`。

独立环境复用本机已经验证过的 CUDA PyTorch，FOD 自身依赖固定为：

```text
/home/robot/python_env/fod_yolo
torch 2.4.1 + CUDA 12.1
torchvision 0.19.1
ultralytics 8.3.0
numpy 1.24.4
opencv-python 4.10.0.84
```

该环境通过 `--system-site-packages` 只复用本机 CUDA PyTorch/ROS 基础库；
Ultralytics、Torchvision、NumPy 和 OpenCV 的 FOD 版本安装在独立环境中，
不会写入系统 Python 或已有的 `rosnav` 环境。

ROS 的 `catkin_install_python` 使用系统解释器，因此 YOLO 节点在 launch
中通过 `launch-prefix` 明确使用上述 Python；无需修改整个工作区的 Python。

## 无 ROS 冒烟测试

图片：

```bash
./scripts/fod_yolo_smoke.sh \
  --source /path/to/test.jpg \
  --output /tmp/fod_yolo_image.jpg
```

视频：

```bash
./scripts/fod_yolo_smoke.sh \
  --source /path/to/test.mp4 \
  --max-frames 300 \
  --output /tmp/fod_yolo_video.mp4
```

笔记本摄像头：

```bash
./scripts/fod_yolo_smoke.sh \
  --source /dev/v4l/by-id/usb-_Integrated_Camera_0001-video-index0 \
  --max-frames 100 \
  --output /tmp/fod_yolo_webcam.mp4
```

脚本逐帧输出 JSON 检测数量、类别和推理耗时。通用 COCO 权重没有检测到
FOD并不代表链路失败。

## ROS 笔记本摄像头

终端一：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch autolabor_fod_vision laptop_camera.launch
```

终端二：

```bash
rqt_image_view /fod/debug/image
```

关键接口：

```text
/fod_camera/image_raw       sensor_msgs/Image
/fod_camera/camera_info     sensor_msgs/CameraInfo
/fod/detections             autolabor_fod_msgs/FodDetectionArray
/fod/debug/image            sensor_msgs/Image
/diagnostics                diagnostic_msgs/DiagnosticArray
```

笔记本摄像头尚未标定，因此该 launch 明确关闭米制投影。

## 海康工业相机实时 FOD 识别

运行识别时必须关闭 `hikrobot-mvs` 桌面客户端，因为客户端和 ROS 驱动会
独占同一台相机。

如果相机驱动已经按下列方式启动：

```bash
roslaunch hikrobot_mvs_camera fod_camera.launch
```

在另一个终端启动真实 FOD 权重：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch autolabor_fod_vision hikrobot_fod_detection.launch
```

也可以在确认没有 `/fod_camera/driver` 运行时，用一个命令同时启动相机和
检测器：

```bash
roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
  start_camera:=true
```

需要让专用入口同时按地面区域自动调节曝光和增益时，显式启用图像质量控制器：

```bash
roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true
```

该功能默认关闭，因此不会改变原有启动行为。启用后，节点以配置文件中的
地面 ROI 亮度中位数为目标，优先调整曝光；达到 `5000 us` 的运动模糊上限
后才增加模拟增益，变亮时则先降低增益。它还会在 `/diagnostics` 报告过曝、
欠曝、强逆光/动态范围冲突、清晰度偏低和明显色偏。若 ROI 内同时有大量
纯黑与纯白像素，曝光无法解决问题，控制器会保持当前值（尚未接管时保留
相机原生自动模式），提示调整灯光、遮光或相机角度。

只观察指标而不更改相机：

```bash
roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true \
  image_quality_monitor_only:=true
```

运行时可暂停控制并恢复海康相机原生连续自动曝光/增益：

```bash
rosservice call /fod_image_quality_controller/set_enabled "data: false"
```

重新启用：

```bash
rosservice call /fod_image_quality_controller/set_enabled "data: true"
```

节点正常退出时也会恢复相机原生自动模式。现场安装后应按实际路面在
`config/image_quality_controller.yaml` 中校准 `roi_*`；控制器不会使用
YOLO 置信度作为曝光反馈，以免误检形成正反馈。固定焦距镜头失焦、镜头污渍
可能表现为清晰度告警，但镜头清洁、对焦和照明频闪仍需现场检查处理。

查看识别结果和结构化消息：

```bash
rqt_image_view /fod/debug/image
rostopic echo /fod/detections
rostopic hz /fod/detections
```

2026-09-28 按用户机场场景需求，生产相机 launch 默认采用
`detector_profile:=airport_recall`：置信度 `0.10`、推理尺寸 `960`、NMS IoU
`0.60`、跨类别 NMS。保留原 `Metal/Soft/Plastic/Wire/Tool/w` 六类和模型权重。
`detector_profile:=original` 恢复检测阈值 `0.25`、尺寸 `640`、IoU `0.45`，
并关闭预览稳定化；显式 `confidence:=...` 等参数仍优先。

预览对相邻帧匹配框做位置平滑和类别投票，短暂漏检最多保留 `0.30 s`，以黄色
虚线和 `hold` 标注。`/fod/detections` 仍只发布当前帧模型实际检测，置信度、
框和时间戳不被预览缓存抬高或替换；清扫控制不会收到补画的历史框。
Qt 目标计数来自当前检测，因此可以少于预览中包括保留框在内的数量。

**用户明确要求排除地面小石头。当前模型没有石头类别，也尚未使用现场石头负样本
验证或微调；上述召回配置是调试配置，不代表已具备石头排除能力。** 需用原六类
FOD 标注数据和机场石头背景负样本训练、验证；不应把增加框数等同于有效召回提升。
不按尺寸或灰色外观一刀切删除小目标，以免同时丢弃小金属件和细线。
实际结果及待补数据见工作区根目录 `FOD_RECOGNITION_TUNING.md`。

该 launch 目前只做识别和画框。虽然相机已经发布标定后的
`/fod_camera/camera_info`，但在得到并验证
`base_link -> fod_camera_optical_frame` 安装外参 TF 前，不会输出可信的
地面坐标。

需要在检测结果之上显式接管真车、将最近的单个 FOD 引导到中下回收区，并在其
消失后按底盘里程计连续前进 0.8 m 时，使用独立的 `autolabor_fod_control` 包。
控制器在线时，预览显示其实际回收区域，目标进入后从黄框变绿框。它不会由
本 launch 自动启动；完整的控制源隔离、安全检查和启动步骤见
`src/application/autolabor_fod_control/README.md`。

## 图片和视频 ROS 源

```bash
roslaunch autolabor_fod_vision image.launch image:=/path/to/test.jpg
roslaunch autolabor_fod_vision video.launch video:=/path/to/test.mp4 loop:=false
```

若录像不是 30 FPS，通过 `fps:=录像帧率` 按原速回放。

媒体源只负责发布 ROS 图像；检测器只订阅图像。海康相机已经使用
`/fod_camera/image_raw` 和 `/fod_camera/camera_info`，后级无需打开
相机 SDK。

## 确定性模拟闭环

模拟器绕过当前通用 YOLO 权重，直接生成具有准确相机内外参和准确 bbox
接地点的模拟 FOD，用于单独验证“像素到地面坐标再到跟踪目标”的数学链路：

```bash
roslaunch autolabor_fod_vision simulation.launch target_x:=2.0 target_y:=0.25
```

检查：

```bash
rostopic echo /fod/sim/ground_truth
rostopic echo /fod/ground_observations
rostopic echo /fod/target
```

预期 `/fod/target.valid=true`，目标在 `odom` 中约为 `(2.0, 0.25)`。

完整接口：

```text
/fod/ground_observations       投影后的全部候选
/fod/tracks                    未确认和已确认轨迹
/fod/target                    明确包含 valid/status/age 的目标
/fod/target_pose               仅在目标有效时发布标准位姿
/fod/projection_status         标定、TF、过滤状态
/fod/debug/projected_markers   RViz 地面投影
/fod/debug/track_markers       RViz 轨迹
```

默认连续三帧才确认目标。出现多个已确认目标时，默认状态为 `AMBIGUOUS`
且 `/fod/target.valid=false`，必须由后续任务层给出选择规则。

## 真实相机接入要求

真实米制投影必须同时满足：

- `CameraInfo.K` 是实际分辨率下的有效标定；
- 图像、CameraInfo、detections 使用同一采集时间和光学 frame；
- 存在采集时刻的 `base_link <- fod_camera_optical_frame` TF；
- `camera_init <- base_link` 可用，以便在稳定坐标系中跟踪；
- 检测 anchor 是 FOD 的地面接触点；Detect 模型默认使用 bbox 底边中心，
  Segment 模型则保留 mask 轮廓。

未标定、尺寸/frame 不一致、TF 缺失、射线反向或目标超距时，投影节点会
发布明确的无效状态，不会输出伪造距离。
