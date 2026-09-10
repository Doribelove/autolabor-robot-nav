# 待清扫垃圾：Vision Qt 合入 0909 与导航/视觉接管

工作区 `/home/slam/robot_j6m_ws_navigation_20260907`，分支 `0909`。
最新状态：已于 2026-09-10 部署为 J6M `20260910_233119`，程序保持停止；部署结果见文末。
随后用户已要求持久开启 FOD 运动授权，两端配置 true/true；仍由用户手动完整启动 Qt。
源码基线 `1bb3366`。本次源码修改留在工作树，未提交、未推送。
Vision 来源为 `/home/slam/robot_j6m_ws_vision_0908` 的当前工作树
（`feature/vision-map-localization-0908`），来源工作树未修改。

## 已实现的流程

Qt 的“待清扫垃圾”页保留 Vision 的地图、五帧确认、稳定地图判定、清扫栏和仅本次会话
存储逻辑。选中已确认垃圾后，“清扫”异步调用独立协调节点：

1. 检查当前导航所有权、FOD 待机状态和视觉运动授权；FOD 未授权时在粗导航前拒绝。
2. 若垃圾距车辆超过 1.50 m，向原有 `/move_base_simple/goal` 入口发送一次普通导航
   目标：在当前位置到垃圾的方向上保留 1.50 m 距离，朝向垃圾。
3. 绑定所有权桥返回的唯一新 GoalID，等待该目标成功，并等待桥禁止重发已完成目标，
   再进入视觉接管。取消使用该确切 GoalID，不发送取消全部目标。
4. 校验到达后垃圾距离不超过 2.20 m、方向误差不超过 35°；将选中的 object_id 和
   map 坐标交给视觉伺服，经原 FOD 仲裁器切换控制来源。
5. 像素框与米制检测按源时间戳匹配，再按唯一边框关联绑定 object_id。只有所选垃圾
   可被锁定；身份更换、坐标跳变、缺失/过期检测等情况沿现有停止路径处理。
6. 在视觉锁定时将选中 map 坐标换算到独立轮式 odom，沿车辆前方清扫参考点轨迹计算
   点到线段最小距离。有效前进轨迹在 0.10 m 内覆盖该坐标，并完成停车确认，才算完成。
7. 协调器要求本次的新完成序号、相同垃圾 ID、相同目标坐标、有效前向线段和不大于
   0.10 m 的覆盖证据，同时等待 FOD 恢复导航待机。单独恢复 GPS_ACTIVE 不代表成功。
8. Qt 按完成的确切 ID 删除垃圾点，并抑制同一检测流中的旧帧重新加入；拒绝/中止时
   保留目标并恢复操作状态。旧的锁存 COMPLETE 不会被当作新任务结果。

当前一次点击只处理所选垃圾，不自动执行整个清扫栏队列。目标太近但不在视觉可接管
方向时会拒绝；尚未增加自动倒车调整或额外搜索动作。

## 隔离与参数

新增 `pending_litter_cleaner_node.py` 不发布速度，只调用既有导航入口和 FOD 模式服务。
未修改 move_base/TEB、覆盖管理器、导航所有权桥或底盘驱动的移动逻辑。
10 cm 坐标判定及地图检测要求只在显式设置待清扫目标时启用；普通视觉/智能语音入口
继续使用原来的像素接近及消失后 0.50 m 判据。

轨迹参考点偏移 `cleaning_reference_offset_m=0.52` 是候选配置，仍需现场核对是否对应
实际清扫装置有效中心。判据同时要求几何前进、正速度依据，排除倒车和方向切换采样段；
静止漂移不算清扫。现有消失后 0.55 m 总行程限制、速度限制、里程计/转角新鲜度、
急停、CAN 故障和控制租期保护保留。

相机标定随直接依赖同步了 Vision 中操作员于 2026-09-09 确认的数值：左目相对
MID360 前 0.42 m、右 0.13 m、低 0.03 m、俯角 42°。现有 `zed_mount_args.py` 和
地图定位模块读取同一配置；没有额外叠加双目基线。这与清扫参考点 0.52 m 是两个参数。

## 主要文件

- `src/application/autolabor_operator_gui/`：待清扫页、注册/存储、Qt 回调整合、异步服务和测试。
- `src/application/autolabor_fod_control/scripts/pending_litter_cleaner_node.py`：独立任务协调器。
- `src/application/autolabor_fod_control/scripts/fod_visual_servo_node.py`：选中目标绑定、可选覆盖判据与完成证据。
- `src/application/autolabor_fod_control/src/autolabor_fod_control/{pending_litter,visual_servo}.py`：几何与前向覆盖。
- `src/application/autolabor_fod_msgs/`：两种地图坐标消息及启动/目标设置服务。
- `src/application/autolabor_fod_vision/`：地图定位、稳定性判断、检测发布与标定校验及对应测试。
- `src/application/autolabor_bpu_perception/config/zed_mount.yaml`：与 Vision 一致的实测相机标定。
- `src/platform/autolabor_dual_host/launch/j6m_fastlio_navigation.launch`：协调器随 FOD 仲裁器启动。
- `scripts/deploy_j6m.sh`：补齐共享消息包同步/编译白名单、新服务 MD5 和协调器安装检查。

## 本机验证

证据目录：`validation/pending_litter_cleaning_20260910/`。ROS 集成测试通过 bubblewrap
建立独立 network/PID namespace，仅启用 loopback；检测、TF、里程计、轮角和 CAN
均为合成输入，没有启动实车驱动或连接 J6M ROS master。Qt 页测试使用 offscreen。

| 检查 | 结果/证据 |
|---|---|
| Release 全量构建及最终 Qt 重链 | `build.log`、`final_control_qt_build.log`，退出 0 |
| 控制/几何/协调器单元测试 | 90 项；`control_tests_final.log` |
| 地图几何、Qt 契约、运行解释器包装检查 | 75 项（22+52+1）；`map_gui_contract_tests_final.log` |
| 真实检测节点的合成输入发布测试 | 15 项，已包含于上一行包装检查；`map_runtime_cases.log`，不重复计数 |
| 检测契约、BPU 几何、FOD 仲裁辅助逻辑 | 24 项；`dependency_tests.log` |
| Qt 页面及回调联动 | 12 项；`qt_page_final.log` |
| Qt 注册/存储 | 20 项；`qt_tests.log` |
| 真实视觉控制节点的隔离 ROS 测试 | 10 项；`visual_selected_ros_final.log` |
| 静态健康检查 | `static.log`，退出 0；其中汇总 XML 含历史测试，不作为本轮新增计数 |

隔离 ROS 包括：所选点前向覆盖完成、30 cm 横向偏离拒绝完成、旧直接视觉完整闭环、
偏心目标转向、里程计/轮角失效及瞬态急停/CAN/旁路控制停止。选点测试覆盖距离小于
10 cm 时进入停车，消失后约 0.20 m，明显小于旧 0.50 m；偏离测试在原 0.55 m
上限中止。单元测试另覆盖倒车、零速抖动、方向切换段、人工退出、旧结果和错误身份。

初轮失败证据保留：地图解释器测试最初误用不存在的工作区内 venv，现读取本项目
`setup_env.sh` 配置的生产解释器；第一次隔离 ROS 调用缺少 `/dev` 专用挂载，测试未启动，
补齐后通过。最终采用上述带 `final` 的证据。

## 部署和待现场验收

本轮未部署、未启动真实 Qt/导航、未发送实车初值或目标，也未修改私有运动授权配置。
只读核对两服务 `autolabor-navigation-20260907` / `autolabor-vision-0908` 均 inactive、
MainPID=0。J6M 本导航版本 `current` 仍指向 `20260909_003347`。
主运动门 true、FOD 门 false 保留；当前不能直接开始本功能的实车移动。

本次涉及 J6M 控制包、共享消息与 launch，实机启用前须经统一入口原生部署，核对两端
服务/消息定义一致，再完整启动并由操作员给出本次真实初值。按用户“改完后询问是否
打开 Qt”的要求，部署/启动留待确认；FOD 运动授权须单独明确。
部署脚本已补齐新消息包，避免沿用 J6M 基础 install 的旧接口；本轮仅通过 shell 语法
和本机接口解析校验，没有运行部署脚本或切换远端 current。

本地测试尚不代表真实 move_base 到点后的双机交接、实际图像中的目标重捕获、10 cm
物理精度或真实垃圾清扫效果已经验收。清扫装置偏移、地图/深度误差和轮式里程计误差
需要现场量测。既定判据按轨迹推定已清扫，没有新增清扫机构电机控制或垃圾消失复检。

## 2026-09-10 部署完成（取代上节未部署状态）

用户授权部署，并明确由其自行打开 Qt。执行 `./scripts/optimized.sh deploy`，使用
原 J6M 原生构建缓存，53 秒退出 0，新 release 为 `20260910_233119`。板端 `current`
已指向新版本，之前的 `20260909_003347` 历史版本保留。

- 板端标准静态健康检查退出 0。
- 本机与板端 FodDetectionArray、FodVisionDetectionArray、FodMapPositionArray、
  SetFodCleaningTarget、StartPendingLitterCleaning 的 5 个 MD5 全部相同。
- 对照安装空间的 11 个关键文件全部一致：控制节点、几何模块、视觉配置、主 launch、
  相机标定及新增消息/服务。两个 Python 入口对比时忽略 catkin 改写的首行 shebang。
- 在板端安装环境实际导入两个控制脚本成功，未调用 main 或初始化 ROS 节点；安装
  launch 展开包含 `/pending_litter_cleaner`，没有仅凭源码存在就推断部署完成。
- 最后两套本机服务均 inactive/MainPID=0，J6M 11311 无监听，无本导航运行 PID 文件。
  私有配置继续主门 true、FOD=false。未打开 Qt、发送实车初值或目标，未做移动验收。

日志为同一证据目录内 `deploy_preflight.log`、`deploy.log`、`deploy_health.log`、
`deploy_stopped_state.log`，哈希/接口/安装导入明细为 `deploy_verification.json`。
后续由用户通过当前导航工作区的统一入口启动完整 Qt 会话，并提供新会话真实初值；
FOD 运动门仍须单独明确开启后才能执行待清扫垃圾移动。

## 2026-09-10 后续：FOD 运动授权已开启

用户随后明确要求开启运动授权，供之后自行打开 Qt 使用。主运动门和原授权标记保留，
将私有 `config/dual_host.env` 中 `FOD_MOTION_ENABLED` 改为 true 并同步板端；两端
SHA256 均为 `2d9433d96cc9404d229074a3f6d0718dca50e806835c4cbf33f820267327522e`。
这不是一次性环境变量覆盖，下一次正常完整启动即可加载；无需重新编译或部署代码。

验证了本机无临时覆盖时读取主门/FOD均为 true，以及板端安装版本启动参数展开为
`/fod_visual_servo/allow_motion=true`、`external_estop_override=false`。速度、急停、
定位、传感器保护均未调整，语音/解析/AI控制仍走Qt各自的授权按钮。

程序仍停止，没有实际启动或发出运动指令；不能将本次参数验证记为运行态/实车验收。
新会话仍需操作员设置真实初值并等待 LOCALIZED 后再发任务。备份和验证日志位于
`runtime/fod_motion_authorization_20260910_DgGBPN/`，板端旧配置保留为
`dual_host/config/dual_host.env.before_fod_motion_20260910_DgGBPN`。
