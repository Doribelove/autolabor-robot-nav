# Coverage navigation for ROS 1 / Ackermann robots

适用于**受阿克曼运动学约束的四轮前轮转向机器人**的 ROS 1 覆盖导航算法。
支持在已知静态地图上自选多边形覆盖区域；**区域外需额外预留足够的已知可通行空间，
供机器人入场、转弯、换向和转场使用**。车辆不能按差速底盘的方式原地旋转，规划需要
满足车体外廓、轴距、最大转向角、最小转弯半径及前进/倒车速度约束。

算法由多边形生成清扫线，Navfn 首次入场，分层并行 Hybrid A* 完成线间转场，
定制 TEB 跟踪固定挡位路径，覆盖管理器处理暂停、取消、换向及续扫。

## 适用条件与区域选择

* 底盘能用前轮转向的阿克曼/自行车运动学模型描述；本实现没有验证四轮独立转向、
  全向或原地旋转模型。驱动轮布置本身不替代运动学和控制接口适配。
* 已有静态 OccupancyGrid、可靠 map→base_link 定位、里程计和实时障碍物输入。
* 用户可选择覆盖多边形，管理器按已知可达自由空间生成清扫线；覆盖多边形是作业区域，
  **不是禁止车辆越出的 geofence**，入场和转场可能走到区域外。
* 区域外余量必须满足实际车辆的最小转弯半径、带余量外廓、入口姿态和倒车许可。
  不能只留一条与车宽相当的走廊；也没有对所有车辆通用的固定外扩距离。
* 墙边、狭窄空间或禁止倒车的区域可能无法完成全部转场。允许倒车的任务还需要后方
  空间及障碍感知。若项目要求车辆始终不越过多边形，需额外实现硬边界约束。
* 配置中的 5 m 是清扫前方致命障碍检查窗口，**不是要求区域外统一留 5 m**。
  默认轴距/最小半径等来自原 M2 平台，移植时须按目标车辆标定。

此仓库包含可构建的算法源码、ROS 接口、定制 TEB、底盘接口依赖和无硬件示例。
无需 J6M、NVIDIA、雷达、相机、私有地图或模型即可运行下列示例。实车使用需要按
[移植接口](docs/PORTING.md)接入真实定位、里程计、障碍物与底盘；本仓库不提供
完整设备驱动部署、Qt、FAST-LIO、视觉推理或现场运动授权配置。

目前基于 **Ubuntu 20.04 / ROS Noetic / Python 3 / C++14**。Noetic 已于
2025-05-31 结束官方支持，因此提供容器构建入口固定使用 Focal 用户态；这不是 ROS 2
版本。[ROS 官方说明](https://www.ros.org/blog/noetic-eol/)

## 克隆与最快示例

```bash
git clone https://github.com/cl4mat/autolabor-coverage-navigation.git coverage-navigation
cd coverage-navigation
python3 tools/geometry_demo.py
python3 src/application/autolabor_coverage/test/test_coverage_geometry.py
```

几何示例只依赖 Python 标准库，调用实际 CoveragePlanner，输出
`validation/geometry_demo/plan.json` 与 `swaths.svg`。SVG 显示清扫线，不把几何
估算连接线当成已完成碰撞检查的 Hybrid 路径。

## 完整 ROS 构建、测试与示例

安装 Docker 后，在仓库根目录执行：

```bash
docker build -t coverage-navigation:local .
docker run --rm --network none coverage-navigation:local
docker run --rm --network none coverage-navigation:local bash tools/run_demo.sh sweep
docker run --rm --network none coverage-navigation:local bash tools/run_demo.sh preview
```

构建会编译仓库中的定制 TEB。单元测试容器默认执行 `tools/test.sh`；另外两个命令
运行真实 move_base/规划插件与合成地图：分别验证 5 米清扫碰撞窗口，以及首入口预览、
取消和换图。网络隔离后只有 lo，示例内部创建私有 ROS master，不连接实车 ROS 图。

如需保留示例结果，在运行时追加 `-v "$PWD/validation:/coverage_ws/validation"`；
Linux Docker 默认用户为 root，生成目录可能需要本机调整所有者。

已有干净 Noetic 环境时也可本机构建：

```bash
source /opt/ros/noetic/setup.bash
# 首次使用 rosdep 的机器先执行 sudo rosdep init。
rosdep update --rosdistro noetic --include-eol-distros
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
sudo apt-get install python3-nose python3-yaml libgtest-dev libsuitesparse-dev bubblewrap
bash tools/build.sh
bash tools/test.sh
bash tools/run_demo.sh sweep
bash tools/run_demo.sh preview
```

构建默认两任务，可用 `BUILD_JOBS` 调整。测试细节、源工作区专用检查和证据边界见
[TESTING.md](docs/TESTING.md)。容器镜像 tag 和 apt 依赖是构建输入，首次构建需要网络；
`SOURCE_MANIFEST.json` 固定本仓库源码，不宣称 apt 下载永久位级可复现。

## 提交内容

| 路径 | 内容 |
|---|---|
| `src/application/autolabor_coverage/` | 几何算法、管理器、Hybrid A*、Reeds–Shepp、全局插件、命令 mux、msg/srv/action、YAML、launch 与测试 |
| `src/navigation_arena/forks/navigation/local_planner/teb/` | 与固定挡位转场协议配套的 TEB 源码和许可证 |
| `src/autolabor_core/autolabor_canbus_driver/autolabor_canbus_driver/` | 管理器必需的底盘消息/服务定义及原驱动源码；示例不启动驱动 |
| `src/platform/autolabor_dual_host/scripts/move_base_pause_bridge.py` | 导航与覆盖目标所有权、暂停桥；保留对应单元测试 |
| `tools/`, `Dockerfile`, `.github/workflows/` | 构建、测试、合成示例与 CI |
| `docs/`, `LICENSE`, `THIRD_PARTY_NOTICES.md` | 架构、测试、移植、已知问题与许可证 |
| `SOURCE_MANIFEST.json` | 来源提交和逐文件 SHA-256 |

不提交 `build/devel/install`、ROS 日志、实测地图、rosbag、模型、凭据、主机私有
`.env`、运动授权标记或原整机仓库的 Git 历史。目录层级保留源项目位置，以便比较更新。

## 当前实现边界

单元测试、规划插件合成场景与实车结果分别报告；示例通过不等于新底盘实车通过。
已知换向点接续/执行无进展问题及未记录获选搜索线程的问题没有在此发布准备中修复。
详见 [KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md)。

架构见 [ARCHITECTURE.md](docs/ARCHITECTURE.md)，配置和实车接口见
[PORTING.md](docs/PORTING.md)。原始项目代码按现有 MIT 声明整理；TEB、Reeds–Shepp
保留各自 BSD 许可证，具体范围以 [第三方声明](THIRD_PARTY_NOTICES.md) 为准。
