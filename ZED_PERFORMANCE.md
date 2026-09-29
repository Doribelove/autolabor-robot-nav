# ZED 2 成像与识别性能配置

更新日期：2026-09-29。目标机为 Jetson AGX Orin 64GB、JetPack 5 / L4T 35.5、
ZED SDK 4.2.5。本次围绕机场地面 FOD 识别、低延迟回收优化，使用当前六类模型。

## 当前启动配置

完整入口继续使用 `OPERATOR_CAMERA_DRIVER=zed ./scripts/operator_all_in_one.sh`，
CAN 和 GNSS 的本机固定端口见 [AGX_STARTUP.md](AGX_STARTUP.md)。默认导航速度为
1.6 m/s，相机伺服标称速度为 0.6 m/s。

| 项目 | 当前配置 | 用途 |
| --- | --- | --- |
| 相机采集 | HD720，60 FPS | 保持宽视野和现有像素坐标，同时提高采集频率 |
| 彩色输出 | 1280×720，30 FPS 上限，缩放因子 1 | 给识别提供完整分辨率图像 |
| 图像标定 | ZED 原生矫正与 CameraInfo，启动自标定开启 | 使用本机相机标定；不手填其他相机的内参 |
| 曝光、增益 | ZED 原生联合自动控制 | 随场景光照调整，Qt 可改手动档位 |
| 白平衡 | ZED 原生自动白平衡 | 随光源调整，Qt 可固定色温 |
| 锐度 / 伽马 | 4 / 8 | 采用原生成像默认值，支持现场调节 |
| 深度 | ULTRA，米制浮点注册深度，检测器主动订阅 | 给每个 FOD 测距，用于单次最近目标选择 |
| 后台定位、建图、通用物体识别 | 默认关闭 | 当前定位由 GNSS / 激光完成，FOD 使用专用六类模型 |
| 深度时序稳定 | 0 | 避免为了深度稳定而隐式启动后台定位 |
| YOLO 后端 | 本机 TensorRT FP16，224×960 输入 | 对现有地面识别带执行同等比例缩放与填充 |
| YOLO 取帧 | 原有单帧队列、只处理最新图像 | 高帧率输入下避免旧画面积压 |

`/fod_camera/image_raw` 实际承接 ZED 的矫正彩色图，配套
`/fod_camera/camera_info` 使用同一像素坐标；深度接口为
`/fod_camera/depth_registered`。原 YOLO 区域仍为 720 高度中的第 180～449 行，
检测结果和锚点仍使用完整图像坐标。TensorRT 使用相同的 960 像素推理宽度，
识别带按比例缩为约 203 像素高，再填充到 32 的倍数 224。

ZED 完整启动默认给识别框附加深度。RGB、注册深度和 CameraInfo 使用相同光学
坐标系，取时间差不超过 60 ms 的最近深度帧，对框内中央区域的有效深度取中位数，
再用原生内参计算距相机的直线距离。空洞、零、NaN、Inf、过期或不匹配的深度
不会当作零距离目标；框仍显示“深度暂无”，不参与首次最近目标选择。

相机回收从原有置信度和转向捕获范围内，选择有有效深度的最近一个目标；确认
后锁定其编号，其他框仍显示，距离/置信度排序变化不会转移本次目标。独立检测
跟踪器只为真实本帧识别结果分配编号，不把界面保留框当成控制观测。
锁定后的短时深度空洞可继续按同一编号的像素位置跟踪，丢失处理仍按原流程执行。
原像素转向、0.6 m/s 回收速度及目标进入盲区后的 0.50 m 穿越流程保留。

每次仅回收一个。完成后停在 `FOD_COMPLETE_STOP`，结束本次自动清扫，GPS 路线
保持暂停；点击“取消相机回收 / 恢复导航”才继续，再次回收需重新点击启动。
未启用相机回收时仍正常 GPS 导航；启用后，小于 5 m 无有效深度 FOD 时仍按 GPS
导航和原清扫逻辑运行。测距值是相机距离，未冒充已经标定的底盘三维坐标。
深度并不提供材质分类，也不据此宣称“小石头已排除”。

主要配置文件：

- `src/scripts/robot_bringup/config/zed2_fod.yaml`：ZED 原生图像和计算模块参数。
- `src/scripts/robot_bringup/launch/zed2_camera.launch`：加载顺序、采集与输出频率。
- `config/fod_zed_ground_roi.yaml`：共享识别带。
- `~/.cache/autolabor_fod/fod_zed_roi_224x960_fp16.engine`：本机生成的推理缓存；
  相邻 `.sha256` 和 `.json` 记录校验值、来源模型、形状与运行库版本。

原 `.pt` 模型保留。完整入口仅在 ZED + `airport_recall` 且本机缓存存在时选择
TensorRT；其他场景、缺少缓存时继续加载原 `.pt`。如需明确使用原后端：

```bash
OPERATOR_CAMERA_DRIVER=zed OPERATOR_ZED_TENSORRT_ENGINE=off \
  ./scripts/operator_all_in_one.sh
```

完整入口在启动导航和识别之前确定同一个模型校验值，通过
`FOD_RECOVERY_EXPECTED_MODEL_SHA256` 传给视觉控制器，并将相同值传给检测器。
这避免检测器使用 TensorRT 时，控制器仍要求原 `.pt` 指纹而在接管后超时停车。
切回原模型时两端一起恢复原指纹；独立 launch 未指定覆盖值时沿用其 YAML 配置，
需要外部检测器时可用 `expected_model_sha256` launch 参数明确指定。

频率和深度模式可由 `OPERATOR_ZED_GRAB_FPS`、`OPERATOR_ZED_PUBLISH_FPS`、
`OPERATOR_ZED_DEPTH_MODE` 覆盖，例如采集 30、输出 15：

```bash
OPERATOR_CAMERA_DRIVER=zed OPERATOR_ZED_GRAB_FPS=30 OPERATOR_ZED_PUBLISH_FPS=15 \
  ./scripts/operator_all_in_one.sh
```

## Qt 使用

在“视觉”页选择“ZED 原生参数”。完整 ZED 启动会自动选中该页签。

1. 点击“读取 ZED 参数”，读取原生驱动的当前配置和采集频率设置。
2. 自动曝光和增益使用同一个开关。取消勾选后，可以分别设置曝光、增益的
   0～100 档位。它们不是微秒或 dB；自动模式下显示的数值是手动预设档位，
   不代表实时自动曝光读数。
3. 自动白平衡关闭后可调色温，28～65 表示 2800～6500 K。
4. 亮度、对比度、饱和度、锐度、伽马和深度置信度可分别调节，点击
   “应用 ZED 参数”交给原生驱动。深度阈值越小，过滤越严格；它不改变 YOLO 阈值。
5. “恢复自动曝光与白平衡”只恢复这两个自动开关，不覆盖其他成像设置。

界面在后台操作参数，服务未就绪时显示原因并可重新读取。设置服务应答表示
驱动接收配置，实际硬件效果以实时图像和驱动日志为准。手动调节作用于当前会话；
需要重启后保持的设置写入 `zed2_fod.yaml`。原海康控件在独立页签中保留。

## 本次实际测量

在本机对已有 ZED 高清录像抽取 55 帧，使用相同地面区域、六类模型、0.10 检测
阈值和 0.60 NMS 阈值，交替运行两个后端，计时包含预处理、推理和结果取回：

| 指标 | 原 CUDA / PyTorch FP16 | TensorRT FP16 |
| --- | --- | --- |
| 单帧中位耗时 | 28.99 ms | 14.21 ms |
| 单帧平均耗时 | 28.82 ms | 14.72 ms |
| 单帧 P95 耗时 | 31.81 ms | 19.00 ms |

中位耗时约降低 51%，处理速度约为原来的 2.04 倍。原后端 65 个检测框均找到
同类别且 IoU ≥0.8 的对应框；其中 56 个置信度 ≥0.30 的框全部匹配，匹配 IoU
中位数为 0.976，所测匹配框最大置信度差约 0.0043。这是录像上的后端对比，
不代表整个机场数据集的精度验收。

实际 ROS 检测节点加载该引擎后，在独立 master 上以 30 Hz 回放 45 帧，收到
44 帧检测消息、63 个框；从图像发布到检测回调的中位延迟为 20.63 ms，最大
25.73 ms。原图尺寸、时间戳和 ROI 坐标保持正确；此测量不包含曝光和 USB 采集。

当前系统查询为 MAXN 模式。转换使用安装在本机的 TensorRT 8.5.2.2 和 CUDA，
修正了该版本绑定与 NumPy 1.24 的 `np.bool` 兼容问题，未更换全局 Python 库。
本次没有检测到相机 USB 连接，采集 60 FPS、输出 30 FPS 是配置值，尚未进行
当前相机与完整导航并发运行时的帧率、曝光、运动模糊测量。

相同硬件、相同软件和现有 ROI 下，可重新生成缓存：

```bash
cd /home/slam/robot_ws_agx
~/python_env/fod_yolo/bin/python3 scripts/build_zed_fod_engine.py
```

更新 GPU / JetPack / TensorRT 或训练模型后应重新生成缓存；改变推理分辨率或
ROI 时，还需要同步调整导出脚本的输入形状。TensorRT 引擎不作为跨平台源码备份。

验证记录与备份：`log/zed_native_controls_20260929_012730/`。

原生接口依据：[Stereolabs ROS 参数](https://docs.stereolabs.com/docs/integrations/ros/zed-node)、
[ZED 相机控制](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/camera-controls)。
具体可用参数以本机 `zed_nodelets/cfg/Zed.cfg` 与 SDK 4.2.5 头文件为准。
TensorRT 导出参考 [Ultralytics 文档](https://docs.ultralytics.com/integrations/tensorrt)，
本机命令按已安装的 Ultralytics 8.3.0 API 执行。
