# 完整项目启动记录（2026-09-06 15:53）

## 16:34 完整重启追加记录（当前状态）

用户先暂停第二条链路开发，再单独要求重启整个程序。本轮使用现有业务代码，通过
`./scripts/optimized.sh restart --map-set global_maps/map_sets/map_20260829_221335_gimp_cleaned`
完整冷重启，入口返回0，运行态健康检查通过。停机时旧服务余留一个SSH子进程，按既有
180秒停机超时及J6M的PID归属清理流程完成退出；未修改系统服务、未宽泛终止未知进程。
旧BPU PID=10033已确认退出，新常驻服务PID=46646。

- 候选服务active，MainPID=1131832；原版inactive。Qt PID=1136854，地图READY。
- NVIDIA FCOS RGB-D PID=1136962；J6M CenterPoint PID=46795、候选适配器PID=46796。
- J6M current仍为 `/opt/autolabor/dual_host/releases/20260906_152402/install`，本轮没有部署。
- 启动检查最初为WAITING_INITIAL_POSE，随后现场在新会话设置初值。16:34最新检查已为
  LOCALIZED，overlap=0.9998、RMSE=0.1704、inliers=4462。助手未发送初值，不需重复本次已完成的操作。
- 主运动运行态true、最大线速1.60、超时0.25秒；FOD=false、授权标记保留。
- 20秒被动检查faults=[]：995条最终指令、左轮899条/右轮894条反馈全部为零，末次无导航目标。
- 当前场景FCOS RGB-D有效结果264帧，约13.10 Hz、平均帧龄212.79 ms；CenterPoint有效结果
  101帧，约4.99 Hz、平均帧龄114.08 ms，仍为0个检测框。没有修改性能参数，不能把本次少框
  场景的表现解释为下方完整负载多框瓶颈已经修复。
- 第二条导航接入开发仍暂停；CenterPoint navigation_enabled=false，Qt仍未新增该链路展示。
  本次只是重启现有版本，没有修改业务代码、构建或执行助手实车移动测试。

新证据：`log/full_project_restart_20260906.log`、
`validation/full_restart_observation_20260906.json`、
`validation/full_restart_ui_perception_20260906.json`。
观测JSON的age_at_end个别值小幅为负，是结束标记后到取消订阅前仍有回调进入的采样边界，
不代表传感器未来时间戳。

下方为15:53首次启动历史，定位等待状态及旧PID不代表本次重启后的状态。

用户要求启动完整项目，并随后确认 MID360 已接线供电。
首次检查无载波，统一入口在网络前检退出，未绕过检查；连接供电后网口恢复 carrier=1，
既有 NetworkManager 自动连接配置恢复 192.168.1.50/24，两条专网双向检查通过。

实际执行候选统一入口：

```bash
./scripts/optimized.sh start --map-set global_maps/map_sets/latest
```

本次没有改动业务代码或配置，没有构建、部署、发送初始位姿、目标、非零速度或恢复旧目标。
地图为 `map_20260829_221335_gimp_cleaned`（fused），J6M current 仍为
`/opt/autolabor/dual_host/releases/20260906_152402/install`。

## 已运行的状态

- 候选 `autolabor-optimized-20260905.service`：active，MainPID=919104；原版服务 inactive。
- NVIDIA Qt PID=927465；地图显示状态 READY，实际尺寸1311×670、分辨率0.1 m。
- NVIDIA FCOS RGB-D PID=927737；Qt 已订阅新 BPU 预览图。
- J6M CenterPoint ROS PID=10182，候选适配器 PID=10183，原生常驻双模型服务 PID=10033。
- 正式 detect_and_classify、ZED、MID360/IMU、前后LD19、融合scan、FAST-LIO、已知地图定位、
  move_base/TEB、覆盖管理、FOD仲裁、最终速度看门狗及Qt/AI随完整主栈启动。
- 启动入口运行态健康检查通过并返回0，完整服务保持运行，不依赖本轮启动终端。
- 实测 watchdog motion_enabled=true、线速上限1.60、指令超时0.25 s；FOD运动门仍false。
- 定位最新为 WAITING_INITIAL_POSE：本次冷启动尚未收到操作员根据真实位置设置的初值。
  不能把地图READY、FAST-LIO正常或运动授权true等同于已完成全局定位。

## 20秒被动检查

`validation/full_start_observation_20260906.json`：faults=[]。
996条最终速度指令全零，左右轮826/818条反馈全零；无助手运动测试。
去畸变 MID360 `/cloud_registered_body` 约10.00 Hz、融合 `/scan` 约10.00 Hz。

CenterPoint 本次已收到真实 MID360 点云并持续执行。两个20秒窗口均约5 Hz、100个有效结果，
平均源帧年龄分别约84/122 ms；BPU推理约10 ms。10 Hz输入预算采样为5 Hz，状态中的
`dropped`包含主动覆盖未处理旧帧，不是输入断流。当前窗口检测框数量为0，不能据此声称场景
无障碍或检测准确；模型精度仍未验收，仍只发布候选，控制接入保持关闭。

**新的实际限制：FCOS在完整负载下的有效深度输出明显下降。**

初次被动观测约2.42 Hz，随后仅订阅小体积结果/状态的20秒检查约1.66 Hz，
30个有效结果、486次带深度的目标观测，平均帧龄325.64 ms。
该窗口末次BPU推理4.25 ms、传输连同推理25.96 ms、深度融合123.64 ms、进入处理时源帧龄180.49 ms。
许多结果在发布前超出0.35 s限制并被丢弃，同时最新结果槽覆盖积压。
这是完整负载和当前多框场景下的实测性能限制，不能沿用之前独立测试10–14 Hz作为整栈保证。
尚未定位各项资源竞争的因果占比；本轮未改动帧龄阈值、正式后端或感知算法。

## 证据

- `log/full_project_start_20260906.log`：首次缺载波前检失败。
- `log/full_project_start_connected_20260906.log`：完整启动及运行态检查通过。
- `validation/full_start_network_20260906.log`：两条网络通过。
- `validation/full_start_ui_perception_20260906.json`：节点实际主机/PID及Qt订阅。
- `validation/full_start_observation_20260906.json`：传感器、零速和轮速被动检查。
- `validation/full_start_perception_light_20260906.json`：低观测开销的感知性能结果及定位状态。

项目保持运行。下一步现场使用前，由操作员在本次Qt中设置真实初始位姿；达到LOCALIZED后才
具备地图坐标定位。用户未在本轮授权助手发目标或实车运动，本轮没有执行这些操作。

## 17:12 Qt崩溃后完整重启与现场实验

用户要求重启后再次实验。候选统一入口使用同一静态地图完成完整冷重启，退出码0；启动自检
共997项通过，runtime健康检查通过。候选服务MainPID=1261687，Qt PID=1266732，原版服务
inactive。J6M运行二进制仍来自release `20260906_152402`，本轮没有构建或远端部署。

Qt地图状态为READY（1311×670、0.1 m/格）。操作员提交真实初值后定位进入LOCALIZED；
17:12采样overlap=0.9846、RMSE=0.1786、inliers=4339。Qt本轮未再出现此前的
Ogre `PassGroupRenderableMap`断言。

操作员先在Qt发起28.00 m²批次任务。该操作与初值提交时间接近，首次被动观察发现非零命令后，
助手为排除旧任务自动恢复只调用一次`/coverage/set_paused=true`；服务返回成功，随后指令及
左右轮反馈均归零。日志时序显示计划在17:05:14已由Qt发起、初值在17:05:52接受，不是初值
本身自动恢复旧任务。操作员随后在Qt新建独立计划`9f6498e3e44c457987140dddbc8d21d7`，
面积61.49 m²、无旧batch标识；17:12状态为TRANSITING、5/6段、进度66.21%，采样命令
linear=0.4776 m/s、angular=-0.3326 rad/s，车辆正在执行本轮现场实验。助手没有发送初值、
导航目标或非零速度，也没有再次干预控制。

主运动门保持true，最终线速上限1.60 m/s，FOD=false，既有授权标记保留。FCOS RGB-D与
CenterPoint候选链路随全栈运行；CenterPoint仍只发布候选观察数据，未接入正式导航避障控制。
重启日志为`log/full_project_restart_after_qt_crash_20260906.log`，首次观察记录为
`validation/full_restart_after_qt_crash_observation_20260906.json`；该观察窗口与操作员运动
实验重叠，其中非零指令是现场任务执行证据，不能当成静止启动验收。
