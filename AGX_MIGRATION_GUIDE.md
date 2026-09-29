# Jetson AGX Orin 64GB 迁移指南

编写日期：2026-09-27。目标已经由用户确认：**Jetson AGX Orin 64GB、Ubuntu 20.04、单主机、原来同一套硬件，继续完成同一个导航和视觉任务**。

本文件说明迁移；项目运行原理见 [AGENT_GNSS_ONBOARDING.md](AGENT_GNSS_ONBOARDING.md)。给目标机 Codex 的完整任务见 [AGX_CODEX_PROMPT.md](AGX_CODEX_PROMPT.md)。

## 1. 迁移基线与交付内容

- 原机是 x86_64 / Ubuntu 20.04 / ROS Noetic；目标机是 aarch64。采用目标机原生 ROS Noetic、Python 3.8 和 catkin，在 Jetson 上重新编译。
- 以移动盘内的**实际工作区快照**为基线。打包时分支是 `gnss_0913_lianxinag`，HEAD 是 `eb2353ad`；存在尚未提交的导航修改和脏子模块。不能用 GitHub 的 `main` 或重新 clone 的 HEAD 替代快照。
- 保留双天线 GNSS 定位、MID360 避障、TEB、M2 底盘、Qt 内嵌 RViz、海康相机、FOD YOLO 和 RabbitMQ。暂不开展 FAST_LIO 定位模式；GNSS 模式依赖的 FAST_LIO 点云处理仍保留。
- 所有 ROS 节点、图形界面和推理均在 Jetson 上运行；旧电脑退出运行链路。已有 RabbitMQ 服务仍按项目配置连接，单主机不表示要在 Jetson 新建一个 RabbitMQ 服务。

移动盘包的结构：

| 文件 | 用途 |
| --- | --- |
| `robot_ws_source.tar.gz` | 完整源码工作区、当前修改、未跟踪源码、模型、配置、Git 历史及全部子模块历史 |
| `robot_ws_reference_data.tar.gz` | 原机 rosbag、工作区运行日志、测试结果，仅供查阅，不是启动依赖 |
| `AGX_CODEX_PROMPT.md` | 交给目标机 Codex 的完整执行指令 |
| `AGX_MIGRATION_GUIDE.md` | 本指南 |
| `AGENT_GNSS_ONBOARDING.md` | 双天线导航上手和当前启动流程 |
| `RESTORE_ON_AGX.sh` | 校验源码包并恢复到一个尚不存在的本地工作区 |
| `START_HERE.md` | 最短使用步骤 |
| `metadata/` | 原机环境、依赖版本、Git 状态、子模块版本、文件清单与校验值 |
| `host_reference/` | 原机相机 udev、库搜索配置及 SDK 版本资料，仅供目标机适配参考 |
| `SHA256SUMS`、`VERIFICATION_REPORT.md` | 交付文件的校验值及实际打包验证结果 |

源码包不携带原机 `build/`、`devel/`、`install/`、Livox SDK 的构建目录、Python 字节码和工作区内的 Codex/npm 二进制。原机系统 ROS、CUDA、MVS 二进制、系统库、Python 虚拟环境也不能直接用于 ARM；对应依赖在目标机安装或重建。模型 `.pt`、标定与源码随包迁移。具体排除项见 `metadata/exclusions.txt`。

这是一份源码、模型和数据迁移包，**不是离线 ARM 系统镜像**。目标机首次配置通常需要联网获取匹配其 JetPack 的 NVIDIA PyTorch、海康 aarch64 SDK 和 ROS/系统依赖。原机账户的 SSH 私钥和 Codex 登录状态不在迁移范围内。

## 2. 恢复工作区

Doribelove 是 exFAT。压缩包保留 Linux 权限和符号链接；请解压到 Jetson 的 ext4 等本地 Linux 文件系统，不能直接在移动盘的散文件目录编译运行。

在 Jetson 上找到本包目录。下例中 `BUNDLE_DIR` 必须替换为实际路径，用户名、挂载点和包名以该机为准：

```bash
BUNDLE_DIR='/media/实际用户名/Doribelove/实际迁移包目录'
bash "$BUNDLE_DIR/RESTORE_ON_AGX.sh" "$HOME/robot_ws"
cd "$HOME/robot_ws"
git status --short --branch
git submodule status --recursive
```

恢复脚本不要求用户名必须是 `robot`，不会覆盖已有目标目录。如果该目录已有工作，先检查并选另一个空路径；已经成功恢复的工作区应继续使用。脚本只恢复源码，不安装软件或启动设备。

也可手动恢复到不存在的 `$HOME/robot_ws`：

```bash
cd "$BUNDLE_DIR"
sha256sum -c SHA256SUMS
tar --extract --gzip --file robot_ws_source.tar.gz \
  --directory "$HOME" --no-same-owner --same-permissions
```

参考数据解压后顶层为 `robot_ws_reference_data/`，不回灌到运行工作区：

```bash
tar --extract --gzip --file "$BUNDLE_DIR/robot_ws_reference_data.tar.gz" \
  --directory "$HOME" --no-same-owner
```

主仓库和 14 个已初始化子模块的 `.git` 数据均保留，子模块路径为相对引用。迁移不需要先拉取远端。不执行 `git reset --hard`、`git clean`、切到 `main`、强制更新子模块或覆盖本地修改。先对照 `metadata/git/` 了解基线，尤其是 FAST_LIO 和 arena 子模块的未提交修改，以及项目维护的 TEB 分支。

## 3. 先识别目标机系统

在目标机记录下列信息到迁移报告：

```bash
uname -m
cat /etc/os-release
cat /etc/nv_tegra_release
dpkg-query -W nvidia-l4t-core nvidia-jetpack 2>/dev/null || true
python3 --version
readlink -f /usr/local/cuda
nvcc --version
df -h "$HOME"
```

应识别为 `aarch64`、Ubuntu 20.04、系统 Python 3.8。`nvidia-jetpack` 元包或 `nvcc` 未安装不等于板卡没有 JetPack/L4T，结合 L4T 和实际库判断。Ubuntu 20.04 不能唯一确定 JetPack 小版本。例如 NVIDIA 的 [JetPack 5.1.5](https://developer.nvidia.com/embedded/jetpack-sdk-515) 使用 Ubuntu 20.04 和 L4T 35.6.1，支持 AGX Orin；本任务没有要求给目标机刷成这个版本。

保留现有可用的 JetPack，按实际 L4T/CUDA 选择配套组件。无需为了迁移直接刷机、换 Ubuntu 22.04、改 ROS 2 或改多主机架构。Noetic 的平台和 Python 基线可查 [ROS REP 3 原始文档](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0003.rst)。

## 4. 原生依赖与 ARM 编译

### 4.1 ROS、系统与 Python

优先复用目标机已安装的 Noetic `/opt/ros/noetic`。未安装时配置适用于 Ubuntu Focal arm64 的 ROS 官方软件源；源或密钥失效时修复实际源问题，并记录处理方式。

安装 `build-essential`、`cmake`、`git`、`pkg-config`、`python3-rosdep`、`python3-venv`、`python3-pip`，以及 ROS Noetic 桌面/RViz、导航、消息与编译依赖。使用工作区清单解析依赖：

```bash
source /opt/ros/noetic/setup.bash
# 仅在 /etc/ros/rosdep/sources.list.d/20-default.list 不存在时执行 sudo rosdep init
rosdep update --rosdistro noetic --include-eol-distros
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
```

源码包的部分历史 `package.xml` 依赖声明不完整，rosdep 完成后仍需按真实构建/launch 错误补齐。常见依赖如下；实际安装版本由目标机软件源和构建结果确定：

| 类别 | 常见依赖 |
| --- | --- |
| 导航、点云 | Noetic `navigation`、`costmap-converter`、`libg2o`、`pointcloud-to-laserscan`、`pcl-ros`、`tf2-sensor-msgs`、`serial` |
| 相机、Qt | `cv-bridge`、`image-transport`、`camera-info-manager`、`rviz`、`rqt-gui-py`，系统 `python3-pyqt5`、`python3-pyqt5.qtopengl` |
| C++ 库 | `libusb-1.0-0-dev`、`libeigen3-dev`、`libpcl-dev`、`libopencv-dev`、`libsuitesparse-dev`、`libyaml-cpp-dev`、`libgoogle-glog-dev`、`libgflags-dev`，必要时 NLopt/qpOASES |
| Python 运行 | `rospy`、`python3-serial`、`python3-yaml`、`python3-pika`；图像推理使用下节单独环境 |

`metadata/host/` 提供原机 apt 和 Python 版本作为比对依据；它不是可以整体安装的 arm64 锁文件。旧仿真/RL 依赖只有当前实际构建或运行需要时才处理。保留当前源码 TEB 及其修改，不能用发行版 TEB 替换项目中的实现。

### 4.2 Livox SDK2

包内已有 SDK2 v1.2.5 源码。ROS1 驱动寻找 `/usr/local/lib/liblivox_lidar_sdk_static.a`，先在 ARM 本机编译并安装：

```bash
cmake -S src/perception_livox/Livox-SDK2 -B "$HOME/.cache/autolabor-livox-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build "$HOME/.cache/autolabor-livox-build" -j4
sudo cmake --install "$HOME/.cache/autolabor-livox-build"
sudo ldconfig
```

原机 SDK build 里的 x86 `.a/.so` 已排除。无需为了迁移升级 SDK 协议版本或雷达固件。

### 4.3 海康 MVS

从海康官方获取 **Linux aarch64** 版 MVS SDK，并确认 `MvCameraControl.h`、`libMvCameraControl.so` 的真实安装位置和架构。厂商有独立的 [Linux aarch64 下载项](https://www.hikrobotics.com/en/machinevision/visionproduct/?id=168&pageNumber=1&pageSize=500&typeId=78)；不要拿原机的 x86_64 `.so` 替代。原机参考版本：MVS 5.0.1，Camera SDK 4.8.0.3，详细版本说明随包保留。

当前 `src/perception_camera/hikrobot_mvs_camera/CMakeLists.txt` 将库搜索目录及 RPATH 写死为 `/opt/MVS/lib/64`。在目标机按实际 SDK 布局适配，ARM 安装目录可能是 `/opt/MVS/lib/aarch64`，不能只凭目录名假设成功。用 `file`/`readelf` 确认 ELF 为 AArch64，随后用 `ldd` 确认相机节点解析到 ARM 库。

`host_reference/etc/udev/rules.d/80-hikrobot-mvs.rules` 是 USB vendor `2bdf` 的用户访问规则，可按目标机现有 SDK 规则合并。原机 `hikrobot-mvs.conf` 中 `/opt/MVS/lib/64` 仅供参考，目标机应使用其真实 ARM 路径。CAN/GNSS 用户通常需要 `dialout`，相机规则使用 `plugdev`；新增组后让用户的新会话生效。按实际设备处理，不用 `chmod 777` 整个设备目录。

### 4.4 FAST_LIO 的系统 libusb 路径

`scripts/bringup.sh` 的 `FAST_LIO_SYSTEM_LIBRARY_DIR` 默认是 `/lib/x86_64-linux-gnu`。目标机应改成按架构选择默认值，并继续支持既有变量覆盖。可根据 `dpkg-architecture -qDEB_HOST_MULTIARCH` 或 `uname -m` 选择 ARM 的 `/lib/aarch64-linux-gnu`。

临时验证可使用现有覆盖能力：`FAST_LIO_SYSTEM_LIBRARY_DIR=/lib/aarch64-linux-gnu`。交付时应让正常入口自动适配。保留 FAST_LIO 进程独立的系统 libusb 选择，避免 MVS 内置旧 libusb 干扰 PCL。不要为解决此问题全局覆盖系统库或删除海康库。

FAST_LIO 子模块当前 CMake 中显式链接 PCL 发现的系统 libusb 的修改也必须保留。已有 ARM 编译分支先沿用。若调整默认库目录，同步调整 `src/scripts/robot_bringup/test/test_teb_profile_tuning.py` 中硬编码 x86 字符串的相关断言。

### 4.5 工作区编译

完成上述依赖和路径适配后，在系统 ROS 环境中构建，避免使用 Conda/Python 推理环境运行 catkin：

```bash
cd "$HOME/robot_ws"  # 如果恢复到其他路径，使用实际路径
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4 -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES=""
source devel/setup.bash
```

`-j4 -l4` 是首次编译的资源选择，不是新的运行限制。按实际剩余内存调整并行度。原机历史上完整构建包含 73 个 catkin 包，清单随包提供。若遇到可选旧仿真/RL 包错误，解决具体依赖；可以先编译导航所需闭包推进，但要明确记录尚未构建的包，不能以隐藏或删除源码包来声称完成全工作区构建。

## 5. Jetson GPU 推理迁移

当前 `scripts/setup_fod_yolo_env.sh` 为桌面 CUDA 写死了 `torch 2.4.1`、`torchvision 0.19.1` 和 `cu121` wheel 源，**在 Jetson 上需要先适配再执行**。不要直接运行其中的虚拟环境清空逻辑覆盖目标机已有环境。

1. 根据第 3 节探测出的 JetPack/L4T，从 NVIDIA 的 [安装文档](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html) 和 [兼容矩阵](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html) 选择 Linux aarch64、Python 3.8、匹配 JetPack 的 CUDA PyTorch wheel。保留可用的现有 Jetson GPU PyTorch，避免桌面 cu121 包或 CPU 包覆盖。
2. 在 `$HOME/python_env/fod_yolo` 建立 Python 3.8 的 `--system-site-packages` 环境；若目标机已有同名可用环境，先检查再复用。安装匹配该 torch 的 torchvision；缺少适配 wheel 时按对应版本源码构建 CUDA 扩展，不随意换成最新版。
3. 应用依赖参考 `src/application/autolabor_fod_vision/requirements-yolo.txt`：ultralytics 8.3.0、numpy 1.24.4、OpenCV 4.10.0.84、scipy 1.10.1。以保留模型可加载、ROS Python 3.8 和 GPU 正常运行为目标处理 ARM 兼容性；pip 安装过程中明确约束 torch/torchvision，避免依赖解析器偷偷替换 NVIDIA 包，记录最终版本和来源。
4. 同一推理 Python 中验证 `rospy`、`cv_bridge`、`cv2`、`torch`、`torchvision`、`ultralytics` 可同时导入。检查 CUDA 可用，并实际执行一次 CUDA `torchvision.ops.nms` 和项目 `best.pt` 的 `device=0` 推理。仅能 import torch 或自动回落 CPU 不算 GPU 迁移完成。
5. 可使用已存在的 `scripts/fod_yolo_smoke.sh --help` 查看本项目自检参数，在正确环境中验证模型。图像可选包内样例，无相机时亦可验证模型加载和 CUDA 运算；无真实相机的检测结果不能宣称实际 FOD 识别验收。

必须使用 `src/yolo/fod_yolo11n_img640_e300_orig/weights/best.pt`，SHA256：

```text
7bf99d4c61343e8cdb37289f2eece6cf18342b508f9b7f80723592edce398500
```

类别为 `Metal,Soft,Plastic,Wire,Tool,w`。保持现有 640 推理尺寸、标定和类别语义。无需为首次迁移转 TensorRT、重新训练或替换通用 COCO 模型。

## 6. 用户目录、设备和单主机网络

### 6.1 目录可迁移性

目标机沿用自己的登录用户。`/home/robot` 是原机路径，不是必须新建的用户名。实际工作区通常是 `$HOME/robot_ws`。

检查并最小修改当前会被执行的硬编码路径，包括：

- `scripts/operator_all_in_one.sh` 的 `OPERATOR_DETECTOR_PYTHON`。
- `scripts/setup_fod_yolo_env.sh`、`scripts/fod_yolo_smoke.sh` 的环境目录。
- `src/application/autolabor_fod_vision/launch/` 中模型和 Python 默认路径。
- `scripts/record_rosbag.sh` 以及实际使用的诊断输出路径。

Shell 优先根据脚本位置计算工作区，虚拟环境根据用户目录或已有变量定位；launch 采用包路径/传入参数，保持原入口和参数兼容。不要对整个 Git 历史或全部历史文档做无差别字符串替换。

### 6.2 原设备对应关系

| 设备 | 沿用配置 | 目标机要确认的主机差异 |
| --- | --- | --- |
| M2 CAN 串口转换器 | `CAN_PORT=/dev/ttyUSB0` | USB 身份、实际端口和用户访问权限 |
| 双天线 GNSS | `GPS_PORT=/dev/ttyUSB1`，115200 | USB 编号可能互换，按真实设备身份映射 |
| Livox MID360 | 雷达 `192.168.1.125`；接收主机 `192.168.1.50` | 给实际直连网卡配置对应静态地址，通常 `/24`，不填无关默认网关 |
| 海康 USB 相机 | 序列号 `DA7535899`，1280×1024，20 Hz | ARM SDK、USB3 带宽、udev 权限、相机未被厂商 GUI 占用 |
| 清扫控制器 | 独立 TCP `192.168.0.197:50003` | 源码 `src/SweepDeviceControl/` 已打包；确需使用时配置相应链路 |

MID360 地址来自 `src/perception_livox/livox_ros_driver2/config/MID360_config.json`；静态地址应使用 Jetson 的实际网卡名持久配置，不能照抄原机 `enp8s0`。需要联网安装依赖时保留已有互联网连接和默认路由。

GNSS 主天线 x=-0.30 m、y=-0.05 m；`base_link → body` 静态 z=+0.6 m。海康标定来自 `src/perception_camera/hikrobot_mvs_camera/config/`。同一套硬件且安装位置不变时保留原标定和航向约定；更换主机不构成重新标定理由。

所有 ROS 进程使用同一个本机 master：检查并清除指向旧电脑的 `ROS_MASTER_URI`、`ROS_IP`、`ROS_HOSTNAME`。仅本机使用时可在启动终端设置：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_IP ROS_HOSTNAME
```

保持 Qt 所在桌面的有效 `DISPLAY` / `XAUTHORITY`。已有 RabbitMQ 连接参数从原代码及环境读取，报告中不复制凭据；无需安装本地消息服务器。清扫脚本按单独任务调用，启动导航不自动切换清扫设备。

## 7. 启动与实际完成标准

### 没有接硬件时

完成 ARM 构建和依赖配置后，在 Jetson 桌面打开操作台：

```bash
cd "$HOME/robot_ws"
./scripts/operator_gui.sh
```

这会启动 Qt、内嵌 RViz、静态 GPS 误差监视器及必要的 ROS master。设备状态显示离线是预期。可以完成软件构建、模型 GPU 推理和界面验证，报告应明确“未接设备，完整硬件导航未启动”。

### 原硬件接入后

退出单独运行的 Qt，避免重复同名节点。在目标机图形桌面启动：

```bash
cd "$HOME/robot_ws"
OPERATOR_DETECTOR_PYTHON="$HOME/python_env/fod_yolo/bin/python3" \
  ./scripts/operator_all_in_one.sh 0.3 cruise
```

适配好默认路径后正常入口仍为 `./scripts/operator_all_in_one.sh 0.3 cruise`。需要只开 GNSS 导航时仍用 `./scripts/bringup.sh gps 0.3 cruise`，不能省略 `gps`。

先沿用既有启动链和判定，不增加新的门控、强制验收脚本或额外许可步骤：

1. CAN/M2 → MID360 → FAST_LIO 点云（`odom_pub_en=false`、`tf_pub_en=false`）→ `/scan`。
2. GNSS 双天线 → `/gps/odom` 与 `camera_init → base_link` → GPS 长距离目标管理与既有模式管理器 → move_base/TEB。
3. 相机、FOD、RabbitMQ 和 Qt 由现有一体化入口启动，与导航初始化并行。

本次 `log/operator_all_in_one_*/bringup.log` 打印 `Robot bringup is running in gps mode.`，再结合实际节点和数据判断导航完成。相机图像、GPU 检测、RabbitMQ 连接和 Qt 状态分别报告；窗口出现不代表导航、感知和外部消息服务均正常。RabbitMQ 收到坐标仍先缓存，等待原有人工确认。

迁移启动不要求自动发送 GPS 目标、启用视觉行驶或切换清扫。完整导航等待用户操作；没有设备、没有 RTK 双天线有效解或外部服务不可达时，准确写出实际阻塞，不造数据或以旧日志声称成功。

## 8. 目标机交付记录

目标机 Codex 最终在工作区写 `AGX_MIGRATION_REPORT.md`，列出：实际 JetPack/L4T/CUDA/ROS/Python、软件来源及 torch/torchvision 版本、修改文件、构建结果、GPU 推理结果、实际工作区与启动命令、设备/网络映射、界面和各模块运行状态、必要的下一步。

将影响日后运行的路径和启动说明同步进上手文档；保留原机基线和当前未提交修改。只针对 ARM、SDK、用户路径等真实改动进行必要验证，不把旧历史记录里的待验收项扩大为本次整车重新验收。
