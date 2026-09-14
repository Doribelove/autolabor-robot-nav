# 0912 源码保存、视觉副本与 AI 结果反馈修复

## 1. Git 保存与复制

按用户要求先保存再修改，未推送任何远端。

| 位置 / 分支 | 保存内容 | 提交 |
| --- | --- | --- |
| 当前导航目录 / `0912` | 修改前完整可跟踪源码、文档、已有导航/视觉/Qt/AI修改 | `5cb44aa` |
| 旧视觉备份 / `backup/vision-0908-before-0912` | 原视觉分支及其 46 个文件的未提交修改 | `41f32d4` |
| `/home/slam/robot_j6m_ws_vision_0908` / `robot_j6m_ws_vision_0912` | 与 `0912` 修改前完全相同的源码快照 | `5cb44aa` |

两个目录是同一 Git 仓库的独立 worktree，因此通过保存旧分支再检出新分支复制源码；
未复制或替换 `.git`。两份 `HEAD:src` 对象均为
`7cefdbb52eec8ead3f3555629890ec72eca506d7`。旧命名分支
`feature/vision-map-localization-0908` 仍为 `452d59b`，
`robot_j6m_ws_vision_0911` 仍为 `e648da6`，均未移动。

复制不包括忽略的私有配置、地图、模型、运行日志、授权和构建目录；这些原地保留。
本轮后续修复只在当前导航目录 `0912` 工作树中，视觉目录保持修改前快照且干净。
`scripts/optimized.sh` 有当前导航目录的绝对路径保护，复制后的视觉目录**不是已迁移、
已构建或可直接启动的独立运行版本**；后续若要从该目录运行，须另行核对启动/部署路径、
构建环境和服务隔离，不能绕过路径保护或共用旧构建产物启动。

## 2. 本次修复范围与日志依据

原视觉控制器已有完整完成判定，不应再添加一套“是否扫完”的云端猜测。本轮不改
控制器或仲裁器，只修 NVIDIA MCP/AI 对它们已有结果的转交和摘要。

现场 AI 日志：`log/nvidia_ui_20260912_012721/ai.log`。
板端同会话视觉日志：
`/map/robot_j6m_navigation_20260907/logs/dual_host/20260912_012714/08082da2-ae06-11f1-a760-5e9bc489939f/fod_visual_servo-16.log`。
被动采样：`validation/coverage_live_20260912/20260912_013815/events.jsonl`。
以下时间为本机 Asia/Shanghai；板端原日志需结合 UTC 换算。

- 01:34:26 的 AI 视觉请求没有确认新视觉会话，01:34:29 失败。随后 01:35:21 的
  Qt 手动视觉启动是另一请求，不能拿它的移动证明前一个 AI 请求已经启动。
  原程序丢掉模式服务响应原因，不能追溯断言当时究竟无有效深度目标还是目标太远。
- 01:43:01 已启用视觉，进入 APPROACH，01:43:10 控制器 ABORT：
  `detections source stamp age is invalid: 0.518s`。当时门限为 0.50s。
- 01:49:15 已启用视觉，经过 APPROACH、EDGE_ARMED，01:49:24 ABORT：
  `FOD is still visible below the motion-confidence threshold; blind advance is inhibited`。
  该分支表明有检测但没有满足运动候选条件的目标；不能据错误文案臆断具体置信度，
  深度/匹配等筛选也应后续核查。

后两次没有 COMPLETE：AI 判“未完成”是对的，但汇总成“未执行清扫”不准确。
原 MCP 只传 `mode.reason`，丢失了 `visual.reason`，进一步掩盖具体原因。

## 3. 源码修改

- `src/sweeper_mcp/src/sweeper_mcp/ros_backend.py`：视觉查询保留
  `visual_reason`、`mode_reason`；没有新会话时保留 `service_message` 和模式/视觉
  快照，仅明确 KEEP_GPS 且新鲜停用时标记 `not_started`。无回复或不明确应答标记
  `start_unconfirmed`。未改变原错误/成功标志、启动超时、运动门和所有权释放条件。
- `src/sweeper_mcp/src/sweeper_mcp/agent.py`：已取得新 ID 后中止/超时/失联等结果为
  `started_not_completed`，保存退出前原因。模式故障且视觉未中止时优先用仲裁原因。
  输出结构化清理状态；应答仅为 `exit_requested`，错误为 `exit_unconfirmed`。
  对字符串分别限长和脱敏，不从中截断 JSON 导致执行状态丢失。最终视觉失败摘要由
  本地证据生成，区分已完成步骤、失败步骤和尚未执行步骤，不请求云端重写执行事实。
- `test_visual_completion.py` 新增 10 项测试；`scripts/simulate_ai_control.py`
  增加两条现场错误的状态接口故障注入，核对原因、摘要和第四步阻断。
- 项目与包 README 同步更新接口说明。

原 COMPLETE 提交点、会话/完成序号/停车证据、0.50s 数据时效、候选置信度与深度筛选、
5cm 停车保护、底盘/定位/急停/导航/覆盖/Qt 控制功能均未改。
未把“有运动”当“清扫成功”；无有效深度目标也不代表周围没有垃圾。

## 4. 验证

证据目录：`validation/ai_visual_feedback_0912/`。

- `ai_regression.log`：11 个测试文件隔离运行，合计 216 项通过（原 206 + 新增 10）。
- `scenarios.jsonl`：49/49，含原 47 个场景及新增两种现场 ABORT 后禁止第四步导航；
  正常完成仍可继续。18 份云端计划是此前真实返回的存档重放，本轮没有新调用云端。
- `visual_qt_contract.log`：原视觉单元测试 84 项、Qt 源码契约 52 项通过；不等于实车/窗口验收。
- `build.log`：本目录 Release 构建退出 0。
- `static.log`：静态健康检查通过；639 是已有 catkin 结果汇总，不是本轮新增 639 项测试。
- 测试网络命名空间仅 `lo`；机器人接口为内存替身，没有真实导航目标、速度、服务调用、
  麦克风录音或 ASR 模型推理。现场错误注入不等于重现整个传感器/车辆物理过程。
- 初始 `before_fix.log` 完整保留：3 项暴露缺失字段/结构，另外 3 项首先遭遇测试替身
  缺少 `initialize` 的夹具错误；夹具修复后再验证，不把夹具错误冒充业务复现。

## 5. 运行与未完成项

本轮未部署 J6M、未启停 Qt/导航、未发真实初值/目标/速度、未恢复任务或改变授权。
只读核对本候选 J6M current 为 `20260912_011633`；查询应使用
`/map/robot_j6m_navigation_20260907/rootfs/opt/autolabor/dual_host/current`，
不是历史基础目录 `/map/autolabor_runtime/rootfs` 的 current。
本机导航服务仍 active（所见 MainPID=282115），旧视觉服务 inactive；未接管它们。
本轮只变更 NVIDIA 代码，无新增 J6M 接口；以后统一重启前仍需按正常流程结束当前任务，
不能据本机测试宣称新逻辑已在运行会话中完整验收。

两目录私有配置校验值未变：导航 `dual_host.env` 为
`2d9433d96cc9404d229074a3f6d0718dca50e806835c4cbf33f820267327522e`，
视觉为 `d4d5ee418b4ffbe7604f70a25ab1a33d42f002802e6d6c4f2b69d171754728a1`；
两份私有 MCP 配置为 `fe9741bea708e8e2c0e2d0b43436c55524a5d247049df0f0a476b7a24fb9722f`。
运动授权标记与真实区域配置保留，没有新增实验区域。当前区域文件 SHA256 为
`3d38f903fe6ea06a35ccef8ec006026655a96f6672be0bb1b44342432aa3133d`，修改时间为本轮
保存源码之前的 01:41:32；不把前次修复报告的旧哈希当成当前文件哈希。

尚未实施：短暂检测超龄如何停车等待后恢复、近距离候选不稳定如何在不放宽保护的前提下
恢复。这些属于视觉运动策略修改，不应在修结果提示时一并静默改变。
