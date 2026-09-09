# 配置与实车接入

本实现面向四轮前轮转向、受阿克曼运动学约束的机器人，不能原地旋转。自选覆盖区域
之外要有已知自由转弯空间，满足车体外廓、最小转弯半径和入场姿态要求；倒车还需要
后方空间和障碍物输入。覆盖多边形不作为硬边界。5 m 清扫检查窗口不是区域外扩距离。

## 已包含的依赖

ROS Noetic 消息和通用导航依赖通过 rosdep/apt 安装。仓库直接包含定制 TEB 和
autolabor_canbus_driver，因为当前覆盖管理器导入后者的消息及 ChassisParameterServer。
包含驱动用于编译闭环；默认示例不启动 canbus_driver、m2_driver 或任何硬件。

只复用几何算法可直接导入 `autolabor_coverage.coverage_geometry`，不依赖 ROS。
C++ Hybrid 库依赖 ROS pose/costmap 类型，但不依赖 J6M/BPU/CUDA。

## 必须提供的运行接口

以 `autolabor_coverage/config/coverage.yaml` 和实际源码参数为准：

| 输入/入口 | 要求 |
|---|---|
| `/map` 与 map→base_link TF | 已知静态地图和真实定位，坐标系一致；不能把 fixture 的 LOCALIZED 用于实车 |
| `/odom` | 新鲜底盘里程计及速度反馈，配置须与 TEB/mux 一致 |
| 定位状态、scan、避障状态 | 管理器预检需要的真实、持续更新状态；其他定位/传感器通过适配器提供 |
| M2 状态与参数服务 | 当前状态机使用的底盘就绪、急停和运动学字段；新底盘需实现适配或独立接口重构 |
| `/navigation_pause/set_coverage_owner` | 运行随仓库附带的所有权桥，保证普通导航与覆盖不抢占 |
| move_base + CoverageGlobalPlanner + 定制 TEB | 全局插件模式交接、action 及强制路径必须匹配 |
| 下游最终命令链 | 真实定位/障碍保护、急停、指令失联看门狗及唯一底盘输出；算法 mux 不替代这些环节 |

在自己的 bringup 中先加载 move_base 和两个插件，再启动所有权桥与 coverage.launch。
所有权桥脚本导出自原平台包，未注册为独立 catkin 包，可用 Python 直接启动：

```bash
source devel/setup.bash
python3 src/platform/autolabor_dual_host/scripts/move_base_pause_bridge.py \
  __name:=navigation_pause _require_coverage_state:=true
```

move_base 普通 simple-goal 输入应按桥协议重映射，TEB 原始速度输出接 `/cmd_vel_teb`，
mux 输出接自己的受保护导航链。完整参考交接逻辑在桥源码和 fixture 中；fixture 自身
只用于隔离演示，不是实车 launch。

## 配置归属

* `coverage.yaml`：作业宽度、重叠、速度/加速度、入口容差、恢复及超时等管理器参数。
* `coverage_factory_defaults.yaml`：配置重置基准；不得用目标机器人未验证的值覆盖其标定。
* `coverage_global_planner.yaml`：`sweep_collision_check_horizon: 5.0`，加载到
  `/move_base/CoverageGlobalPlanner`，在插件初始化时读取。
* Hybrid 分层窗口、两路启发权重等插件参数在源码有默认值，可由自己的 move_base launch
  显式配置；本仓库默认示例使用合成地图参数，不包含原整机 launch。
* footprint、wheelbase、最小半径、速度/曲率、定位状态和控制 topic 必须按真实车辆适配。

收到 `COMPLETED` 表示状态机完成任务，不证明清扫执行器效果或外部绝对定位精度。
参数调优应固定场地、路线和起点，记录规划时间、转场执行时间、清扫速度与每次人工干预。
