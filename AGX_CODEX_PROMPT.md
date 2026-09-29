# 给 Jetson 上 Codex 的完整迁移任务

下面整份文件就是用户交给目标机 Codex 的执行指令。请在 Jetson 上读取并执行，持续完成实际安装、适配、构建和可进行的启动，不只给出迁移建议。

## 任务与已确认事实

把 Doribelove 移动盘中的 Autolabor 项目迁移到当前 **NVIDIA Jetson AGX Orin 64GB、Ubuntu 20.04**。原电脑为 x86_64 Ubuntu 20.04 / ROS Noetic。两台电脑使用**同一套硬件设备、完成同一个任务，目标是 Jetson 单主机独立运行**，旧电脑不参与 ROS 或推理。

主要任务是双天线 GNSS 完整导航；完整操作台还包括 MID360、M2 CAN 底盘、Qt 内嵌 RViz、海康相机、FOD YOLO11 和已有 RabbitMQ 桥。暂不开展 FAST_LIO 定位模式；但 GNSS 流程内 FAST_LIO 提供避障点云，这部分必须保留。

用户已授权完成该任务所需的文件修改、依赖安装、环境配置、编译调试及启动，不对已授权事项反复确认。先做能完成的工作，只在缺少不可推断的信息或实际环境权限阻塞时说明具体需求。不要仅因原机文档写了某个路径、保护或默认规则就放弃解决实际迁移问题。

## 一、找包并恢复真实工作区

1. 找到挂载标签为 `Doribelove` 的盘，以及包含本文件、`robot_ws_source.tar.gz`、`SHA256SUMS`、`START_HERE.md` 的迁移目录。包内 `metadata/package.json` 标识打包时间和源工作区，不要误用盘上其他历史项目。
2. 阅读包外 `START_HERE.md` 和 `AGX_MIGRATION_GUIDE.md`。检查当前目标路径是否已恢复；未恢复则运行 `bash <包目录>/RESTORE_ON_AGX.sh <本地工作区路径>`。默认采用当前用户 `$HOME/robot_ws`，在 ext4 等本地 Linux 文件系统使用，不能在 exFAT 散文件目录构建。已有工作不能覆盖丢弃；已恢复则接着做。
3. 恢复的是当前实际源码及 Git 历史，不是全新 clone。分支 `gnss_0913_lianxinag`，HEAD `eb2353ad`，有未提交修改和脏子模块。保留这些内容，参考 `metadata/git/`；不能 reset/clean/切到 main/强制更新子模块，不能用远端替换本包。
4. 依次读工作区 `AGENTS.md`、`DEVELOPMENT_CONSTRAINTS.md`、`AGENT_GNSS_ONBOARDING.md`、`AGX_MIGRATION_GUIDE.md`，再参考 `CURRENT_GPS_DEV_HANDOFF.md`。按最新代码和现行上手文档理解启动，历史交接中的旧启动先后和待验收事项不构成新增要求。

## 二、遵守项目范围

- 本地项目规则内用户最新明确需求优先。用户明确要求删除、关闭、调整或绕过已有检查与保护时按要求实施；本次的具体目标是完成迁移并启动同一套导航。
- 用户已经验收原硬件 GNSS 导航。更换主机应保持原启动入口、速度/场景参数、双天线航向、标定、坐标系、topic、service、action 和控制链语义。
- 不自行叠加安全门、运动门、看门狗、多重检查、全模块许可或强制依赖；不把相机、界面、推理、RabbitMQ 的成功变成原 GNSS 导航新增前置条件。
- 本次针对 ARM、SDK、GPU、用户目录和网卡差异做必要修改。不要顺便更换定位算法、重新训练模型、调 TEB 参数、改标定、改 ROS 2、刷机或建立多主机架构。
- 保留当前源码 TEB、FAST_LIO 和 arena 的本地修改。原机 build/devel/系统库/虚拟环境不适用于 ARM，重新建立目标机运行环境。

## 三、先识别系统，形成适用于该机的方案后执行

读取 `uname -m`、`/etc/os-release`、`/etc/nv_tegra_release`、已安装的 `nvidia-l4t-core`/`nvidia-jetpack`、CUDA、系统 Python 和 ROS；确认是 aarch64、20.04、Python 3.8。Ubuntu 20.04 不能唯一确定 JetPack 版本，以实际 L4T 为准。

采用原生 ROS Noetic/catkin。复用可用的目标机 ROS/CUDA；缺少的依赖按对应 arm64 官方源安装，参考指南和 `metadata/host/`。包里的 apt/pip 版本是原机参考，不能作为 x86 包下载列表或不加区分地整体安装。目标机需联网时自行获取官方依赖；如 MVS 下载确实需要用户登录再明确说明该阻塞，并继续不依赖该文件的工作。

## 四、逐项解决已经确定的迁移问题

### A. 系统 libusb 与 FAST_LIO

- `scripts/bringup.sh` 当前默认 `/lib/x86_64-linux-gnu`；让 `FAST_LIO_SYSTEM_LIBRARY_DIR` 按实际架构选择默认值，ARM 通常 `/lib/aarch64-linux-gnu`，仍保留环境变量覆盖。
- 保留 FAST_LIO 的单进程库环境隔离及源码中的 PCL 系统 libusb 显式链接。不能通过全局混入海康旧 libusb 解决，也不要删除相机 SDK。
- 如修改默认路径，同步更新 `src/scripts/robot_bringup/test/test_teb_profile_tuning.py` 中相应的 x86 默认字符串断言，只验证本次受影响行为。

### B. Livox SDK 与驱动

- 包内 `src/perception_livox/Livox-SDK2` 为 v1.2.5，在 Jetson 上用 CMake 编译并安装到 `/usr/local`，执行 `ldconfig`。
- 编译原有 `livox_ros_driver2` ROS1 分支。不要使用原机 x86 SDK 构建产物，也不要因迁移升级雷达固件。
- 原配置雷达 `192.168.1.125`，主机接收地址 `192.168.1.50`，见 `MID360_config.json`。

### C. 海康 ARM SDK

- 安装海康官方 Linux aarch64 MVS SDK，确认真实头文件和 ELF 库路径。
- `src/perception_camera/hikrobot_mvs_camera/CMakeLists.txt` 的查找和 RPATH 当前写死 `/opt/MVS/lib/64`；按实际 ARM 安装位置最小适配，不能只改目录名而不确认库架构。
- 参考包内 `host_reference/` 设置相机 USB 用户访问。确认相机节点 `ldd` 无缺失库及错误架构，保留 FAST_LIO 的独立系统库选择。

### D. Jetson CUDA PyTorch 与 FOD

- `scripts/setup_fod_yolo_env.sh` 当前是桌面方案：torch 2.4.1、torchvision 0.19.1、cu121 源。先适配，不能原样运行并覆盖目标机已有环境。
- 按实际 JetPack 的 NVIDIA 兼容矩阵选择 Python 3.8 / aarch64 CUDA PyTorch，配套 torchvision；必要时编译匹配的 torchvision CUDA 扩展。保留可用的现有 GPU 环境。
- 推理环境放在当前用户 `$HOME/python_env/fod_yolo` 或明确记录的路径，保留 `--system-site-packages` 以使用 ROS；约束 pip，避免 ultralytics 依赖把 NVIDIA torch 换成 CPU 包。
- 原 `requirements-yolo.txt` 里 ultralytics 8.3.0 等版本作为兼容基线。必要的 ARM 版本调整要记录原因，不盲目全部升级。
- 模型必须使用 `src/yolo/fod_yolo11n_img640_e300_orig/weights/best.pt`，SHA256 为 `7bf99d4c61343e8cdb37289f2eece6cf18342b508f9b7f80723592edce398500`，类别 `Metal,Soft,Plastic,Wire,Tool,w`。
- 同一 Python 中确认 rospy/cv_bridge/cv2/torch/torchvision/ultralytics 导入成功；实际执行 CUDA NMS 和此模型的 `device=0` 推理。`device=auto` 静默回落 CPU 不能当作 GPU 成功。无相机可完成离线模型/GPU 验证，不伪称实际设备识别成功。
- 不要求首次迁移做 TensorRT 或重新训练。启动脚本、模型检查及输出消息语义保持兼容。

### E. 用户目录和单主机环境

- 原机用户名是 robot，目标机可以不是。适配运行中实际使用的 `/home/robot`，优先根据脚本/ROS 包位置与当前用户目录解析，保留已有参数覆盖。
- 重点检查 `operator_all_in_one.sh`、YOLO setup/smoke 脚本、视觉 launch 的模型和 Python 路径、录包和实际诊断输出路径。不要全文替换 Git 历史和历史文档。
- 所有 ROS 节点运行在 Jetson。同机 master 可用 `http://127.0.0.1:11311`，清除指向旧电脑的 ROS_IP/ROS_HOSTNAME；保持 Qt 桌面的 DISPLAY/XAUTHORITY。
- CAN/GNSS 的 `/dev/ttyUSB0`、`/dev/ttyUSB1` 以实际设备身份确定，使用既有 `CAN_PORT`、`GPS_PORT`、`GPS_BAUD_RATE` 覆盖，处理必要用户组权限。
- 直连雷达的真实网卡配置主机 `192.168.1.50`，保留互联网默认路由；不能照抄原机网卡名。原同套硬件的相机序列号、相机标定、天线与雷达外参不因换主机改变。
- RabbitMQ 继续连接已有服务，源码和环境中的凭据不抄到报告或终端展示。收到目标仍按原逻辑缓存、人工确认。清扫 `src/SweepDeviceControl/` 是独立设备程序，不作为导航启动的自动动作。

## 五、完成构建和有针对性的验证

1. 运行必要的 rosdep、补齐实际缺失的 launch/编译依赖；Livox 与 MVS 先就位，再在系统 ROS/Python 环境执行 `catkin_make`，清空过时 whitelist/blacklist，初次可用 `-j4 -l4`。
2. 原机完整构建为 73 个 catkin 包，`metadata/` 有包清单。保留全部源码；遇到旧可选包错误可先构建主任务闭包推进，再处理实际问题并报告范围，不能删除模块伪装完成。
3. 检查导航、GUI、视觉 launch 能解析，所需 ARM 节点和共享库存在，模型 SHA 与 CUDA 实际推理通过；只对真实修改运行相关测试，不把检查写成新的生产启动门控。
4. 不因为历史文档的旧建议而重复全套整车验收。构建、GPU 与接口适配的验证是本次迁移工作；不能把原机实车验收当作目标机已经验证。

## 六、启动并如实交付

原机打包时没有接任何设备。目标机硬件是否已接入，以当前实际情况判断：

- **无设备**：把软件迁移和 GPU 能完成的部分做完，在当前图形桌面启动 `./scripts/operator_gui.sh`。用户需要能打开和使用界面。明确说明传感器/底盘离线、完整硬件导航尚未启动；缺硬件不妨碍继续处理编译或依赖问题。
- **设备已接入**：退出重复的独立 Qt，使用 `./scripts/operator_all_in_one.sh 0.3 cruise` 启动完整系统。若用户另指定速度/场景，按用户值。先解决本机实际启动阻塞，保持现有入口。
- 单独 GNSS 导航用 `./scripts/bringup.sh gps 0.3 cruise`；绝不能省略 gps，因为无参数默认是另一定位模式。
- GNSS 模式的 FAST_LIO 只提供点云：`odom_pub_en=false`、`tf_pub_en=false`；`camera_init → base_link` 和 `/gps/odom` 来自 GNSS，不能出现第二个定位 TF 发布者。
- 原控制链保持：move_base → `/cmd_vel_navigation` → 现有 GPS 中继 → `/cmd_vel_gps` → 现有 GPS/FOD 模式管理 → `/cmd_vel` → M2。
- 本次日志 `Robot bringup is running in gps mode.` 是现有导航完成标志；同时说明真实数据和模块状态。仅窗口出现、topic 名字存在或旧日志有成功行不代表此次完整启动成功。
- 本次只迁移和启动，不自动发导航目标、不启用视觉行驶、不切换清扫。界面就绪后等待用户操作。

最后写 `AGX_MIGRATION_REPORT.md` 并同步必要的启动说明，报告至少包含：

1. 实际系统、JetPack/L4T、CUDA、ROS、Python、torch/torchvision/ultralytics/MVS/Livox 版本和安装来源。
2. 恢复包、目标工作区、基线分支/提交，保留原本未提交修改的情况；本次实际改了哪些文件及原因。
3. ARM 构建、库解析、模型校验、CUDA 推理、Qt 和硬件模块分别完成了什么、还缺什么；附必要日志路径。
4. 本机可以直接复制的 UI 与完整导航命令、Python 路径、设备端口和网卡映射，说明旧电脑已不参与运行。
5. 若仍有硬件、服务、下载或实际权限阻塞，写出具体项及最短解决动作，准确区分已完成软件迁移与未完成的硬件启动。

请持续执行到当前条件下能完成的工作全部完成，不以一份建议、一次失败的编译、只打开窗口或 CPU 回落替代本任务。
