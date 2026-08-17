# Autolabor FOD Vision

这是一个与底盘控制隔离的 ROS Noetic 感知旁路，覆盖硬件到位前可以完成的三部分：

1. 独立 Ultralytics/YOLO/CUDA 环境；
2. 图片、视频和笔记本 V4L2 摄像头推理；
3. ROS 图像、检测、地面投影、多目标跟踪和调试接口。

本包没有 `/cmd_vel` 发布者，不会接管无人车。

## 重要边界

`/home/slam/robot_ws/src/yolo/yolo11n.pt` 和 `yolo11s.pt` 是 COCO 通用
权重，只用于验证 CUDA、媒体读取和 ROS 推理链路。赵工提供的生产候选权重
位于：

```text
/home/slam/robot_ws/src/yolo/fod_yolo11n_img640_e300_orig/weights/best.pt
SHA256 7bf99d4c61343e8cdb37289f2eece6cf18342b508f9b7f80723592edce398500
```

模型的实际类别为 `Metal`、`Soft`、`Plastic`、`Wire`、`Tool`、`w`。生产
launch 会在反序列化前校验上述 SHA256、逐项校验类别并关闭
`smoke_test_only`；其中 `w` 的业务含义仍需向模型提供方确认。

`.pt` 会经过 Python pickle 反序列化，只加载可信来源并核对 SHA256。

## 安装

```bash
cd /home/slam/robot_ws
./scripts/setup_fod_yolo_env.sh
catkin_make --pkg autolabor_fod_msgs autolabor_fod_vision -j2
source devel/setup.bash
```

安装脚本默认要求 CUDA 可用，否则会失败；仅在明确进行 CPU 调试时使用
`FOD_ALLOW_CPU=1 ./scripts/setup_fod_yolo_env.sh`。

独立环境复用本机已经验证过的 CUDA PyTorch，FOD 自身依赖固定为：

```text
/home/slam/robot_ws/.venv/fod_yolo
torch 2.0.0+nv23.05 + Jetson CUDA 11.4
torchvision 0.15.1
ultralytics 8.3.0
numpy 1.24.4
opencv-python 4.10.0.84
```

该环境通过 `--system-site-packages` 复用 ROS 基础库；Jetson ARM64 版
PyTorch/Torchvision 及 Ultralytics、NumPy 和 OpenCV 安装在独立环境中，
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
cd /home/slam/robot_ws
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
/fod_camera/depth_registered sensor_msgs/Image (32FC1, metres)
/fod/detections             autolabor_fod_msgs/FodDetectionArray
/fod/debug/image            sensor_msgs/Image
/diagnostics                diagnostic_msgs/DiagnosticArray
```

笔记本摄像头尚未标定，因此该 launch 明确关闭米制投影。

## ZED 2 实时 FOD 识别

当前生产入口使用序列号 `23748636` 的 ZED 2。一个命令同时启动 ZED、真实
FOD 权重和检测器：

```bash
roslaunch autolabor_fod_vision zed_fod_detection.launch \
  start_camera:=true
```

ZED 原生话题由 launch 映射到稳定的应用接口：

```text
/zed2/zed_node/rgb/image_rect_color -> /fod_camera/image_raw
/zed2/zed_node/rgb/camera_info     -> /fod_camera/camera_info
/zed2/zed_node/depth/depth_registered -> /fod_camera/depth_registered
```

消息的 `frame_id` 不伪装，保持为 `zed2_left_camera_optical_frame`。视觉伺服
会同时校验发布者必须是 `/zed2/zed_node`、分辨率必须是 `640x360`。

生产 launch 会把同一时刻的注册深度图融合进 YOLO 结果。每个
`FodDetection` 包含 `depth_valid`、`depth_m`、`depth_mad_m`、
`depth_sample_count` 和 `depth_valid_fraction`；数组消息还包含深度帧头与
RGB/深度时间差。Segment 模型在掩膜内部采样，Detect 模型在 bbox 内缩区域
采样，再用中位数/MAD 排除 NaN、飞点和边缘背景。`/fod/debug/image` 会在每个
检测框旁标出米制深度，无可靠深度时明确显示 `depth:N/A`。

默认保留 ZED 原生联动自动曝光/增益。需要试验路面 ROI 控制器时显式启用：

```bash
roslaunch autolabor_fod_vision zed_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true
```

该功能默认关闭。启用后，节点通过 ZED dynamic-reconfigure 接口，以配置文件
中的地面 ROI 亮度中位数为目标，优先调整曝光百分比，达到上限后才增加增益
百分比，变亮时则先降低增益。它还会在 `/diagnostics` 报告过曝、
欠曝、强逆光/动态范围冲突、清晰度偏低和明显色偏。若 ROI 内同时有大量
纯黑与纯白像素，曝光无法解决问题，控制器会保持当前值（尚未接管时保留
相机原生自动模式），提示调整灯光、遮光或相机角度。

只观察指标而不更改相机：

```bash
roslaunch autolabor_fod_vision zed_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true \
  image_quality_monitor_only:=true
```

运行时可暂停控制并恢复 ZED 原生联动自动曝光/增益：

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

默认置信度阈值为 `0.25`。现场已知没有 FOD 却出现低置信度框时，可先用
`confidence:=0.4` 重新启动；正式阈值应通过有标注的现场测试集确定，而
不是只观察单幅画面。

该 launch 做识别、注册深度融合和画框，但不把轴向深度冒充地面坐标。虽然相机已经发布标定后的
`/fod_camera/camera_info`，但在得到并验证
`base_link -> zed2_left_camera_optical_frame` 安装外参 TF 前，不会输出可信的
地面坐标。

需要在检测结果之上显式接管真车、从多个 FOD 中锁定最近目标、将其引导到画面底部并在其消失后按
底盘里程计前进 0.5 m 时，使用独立的 `autolabor_fod_control` 包。它不会由
本 launch 自动启动；完整的控制源隔离、安全检查和启动步骤见
`src/application/autolabor_fod_control/README.md`。

## 图片和视频 ROS 源

```bash
roslaunch autolabor_fod_vision image.launch image:=/path/to/test.jpg
roslaunch autolabor_fod_vision video.launch video:=/path/to/test.mp4 loop:=false
```

若录像不是 30 FPS，通过 `fps:=录像帧率` 按原速回放。

媒体源只负责发布 ROS 图像；检测器只订阅图像。ZED 生产入口已经使用
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
- 存在采集时刻的 `base_link <- zed2_left_camera_optical_frame` TF；
- `camera_init <- base_link` 可用，以便在稳定坐标系中跟踪；
- 检测 anchor 是 FOD 的地面接触点；Detect 模型默认使用 bbox 底边中心，
  Segment 模型则保留 mask 轮廓。

未标定、尺寸/frame 不一致、TF 缺失、射线反向或目标超距时，投影节点会
发布明确的无效状态，不会输出伪造距离。
