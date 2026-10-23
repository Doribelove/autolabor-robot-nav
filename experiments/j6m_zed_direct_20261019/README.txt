ZED 直连 J6M 独立环境（vision-bpu，2026-10-19）

部署根目录：/map/j6m-dev-b/zed-direct
本次版本：releases/20261019_5m7c286u
管理入口：/map/j6m-dev-b/zed-direct/zed-direct

本环境包含
- 固定提交的 ZED Open Capture 0.6.0，启用视频和传感器采集；无需 CUDA。
- 私有 HIDAPI 0.14.0 libusb 后端；不要求系统 hidraw 驱动。
- 限时、无图形界面的原始图像/IMU 接收探针，固定预期相机 SN23748636。
- 该相机原始标定文件的独立副本和 SHA256。
- 复用已有只读 Noetic rootfs 的 OpenCV 4.2 CPU 立体匹配检查和 ROS 消息导入检查。
- 独立版本指针、健康检查、每次运行的新证据目录和回退入口。

本环境不包含
- NVIDIA ZED SDK，不覆盖 /usr/local/zed 或原 zed-ros-wrapper。
- 生产 ROS 相机节点、完整标定补偿、实机深度验收或正式视觉后端切换。
- 内核、设备树、固件或开机服务的修改。

当前板端限制（实际出图前必须解决）
- /proc/device-tree/soc/usb@39820000/dwc3@39820000/dr_mode 为 peripheral。
- CONFIG_MEDIA_SUPPORT 未启用，缺少 UVC/V4L2 摄像头支持。
- 当前未枚举到 USB Host 设备或 ZED 视频节点。
因此“采集用户态环境已安装”不等于“ZED 直连已出图”。
HID 也未启用，但本次选择 libusb 后端，不能把缺少 hidraw 单独误报为该后端的必要条件。
后续应取得这块 Matrix 板的可用 USB Host 接口/供电说明及匹配
6.1.158-rt58-g4f685f73e030 的内核模块/配置方案，不混用 AGX 内核驱动。
本任务不刷内核，也不试改设备树来猜测接口。

板端命令（通过现有 SSH 登录 J6M 后执行）
  /map/j6m-dev-b/zed-direct/zed-direct status
  /map/j6m-dev-b/zed-direct/zed-direct self-test
  /map/j6m-dev-b/zed-direct/zed-direct check
  /map/j6m-dev-b/zed-direct/zed-direct probe --seconds 10
存在多个 ZED 视频节点时，可用 --device /dev/videoN 明确选择已识别的设备。
必须按检查结果和相机序列号选择，不能凭历史 videoN 编号猜测设备。

status 显示安装哈希、旧版本保护、标定文件与硬件缺口；硬件未就绪时 check/probe
返回 20，并在打开摄像头前退出。self-test 只执行库版本、设备枚举、隔离 CPU 立体
匹配和 ROS 消息导入；不启动 ROS master、导航或相机采流。
probe 最多 60 秒，接收原始左右拼接图和 IMU 并输出统计，不发布 ROS、不计算深度。
上游初始化会设置流格式并重置部分曝光/图像控制；本次无设备，未发生这些相机操作。
原始 IMU 角速度单位为度/秒；未来接 ROS sensor_msgs/Imu 时必须转换为弧度/秒，
并处理标定、坐标系和原始采集时间，不直接复用未经转换的数据。

回退
  /map/j6m-dev-b/zed-direct/zed-direct rollback
本次部署前没有旧的 zed-direct 候选，回退会撤掉本实验的 current 指针。
文件和所有成功/失败证据保留；原 AGX SDK、原板端源码、Livox 和导航 current 从未切换。
回退不启动原导航、不恢复任何历史车辆任务；只恢复这套实验的启用状态。
重新启用本次版本：
  python3 -B /map/j6m-dev-b/zed-direct/releases/20261019_5m7c286u/zed_direct.py activate
控制器只操作本任务登记的指针，发现别的版本已接管会拒绝覆盖；运行期间也拒绝并发切换。

构建和追溯
- upstream.lock.json 固定厂商源码提交、下载 URL 和 SHA256。
- 本地源码归档/部署包/日志在 runtime/zed_direct_deploy_20261019_5m7c286u/。
- 在 J6M 原有 Ubuntu 20.04 chroot 内原生构建，独立 mount/network namespace，
  基础 rootfs 只读、无底盘或摄像头设备映射；仅本候选目录可写，并发 -j2。
- 采集探针可在 J6M Debian 原生环境运行；OpenCV/ROS 自检通过受控 chroot 运行。
- 不更改全局 LD_LIBRARY_PATH、ldconfig、apt 包、udev 或系统 Python 环境。
- 原上游源码不修改；保留上游对 aarch64 的 Cortex-A72 编译选项，
  该用户态构建尚未做 J6M 上的实际相机吞吐优化或精度验收。
- 不要直接在已验证版本里重新编译；修改应创建新的 releases 版本并另存证据。

厂商来源
https://github.com/stereolabs/zed-open-capture
https://github.com/libusb/hidapi
