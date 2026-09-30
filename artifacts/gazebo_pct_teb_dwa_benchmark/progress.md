# 进度账本

更新时间：2026-08-10（Asia/Shanghai）

## 当前状态

- 分支：`research/pct-teb-dwa-gazebo-20260810`
- 当前阶段：P03 四方法共享合同与 development 公平调优
- 仿真权限：仅新 development 身份；历史 V6-03/R1/R2/V6-04 身份禁止复跑
- formal 状态：新 Gazebo 研究未预注册、未冻结、未访问；仅离线重放历史 V6 快速正式程序
- 实车状态：禁止；未修改或 source `/home/robot/robot_ws`

## 阶段清单

- [x] P00 完整输入/仓库/进程/证据审计
- [x] P01 冻结 V6-02-F1 80-block/320-episode 结果独立复现
- [x] P02 Gazebo 固定输入、导出、ROS、候选、权限、跟踪、影子和时延准入
- [ ] P03 四方法共享合同及 TEB/DWA/PCT/解析先验开发集公平调优
- [ ] P04 development 消融、版本冻结、pilot 和功效分析
- [ ] P05 独立正式研究第 1 轮
- [ ] P06 若未过门，仅用 development 数据执行单因素修复和独立正式研究第 2 轮
- [ ] P07 统计、完整性、图表和中文论文回填闭环

## 已完成动作

1. 已确认工作区、分支、remote、submodule 状态和无残留 ROS/Gazebo/训练进程。
2. 已完整读取根 `AGENTS.md`、根 README、主实验提示、V6 当前交接、V6-06 论文交接与 R1
   证据包说明、通用 TEB/RL 合同、实验书、T12 冻结交接、V4/V5/V6 架构交接。
3. 根目录指定输入 `CODEX_HANDOFF_PCT_TEB_DWA_GAZEBO_BENCHMARK.md` 在工作树、Git 历史和
   `/home/robot`（排除稳定工作区）均未找到。该输入缺口保留，不伪造内容；现有 V6 权威证据
   足以继续只读审计和重建新的独立研究合同。
4. 已创建独立分支和独立结果目录；未覆盖历史 artifacts。
5. P00 审计于 `2026-08-10T01:32:20+08:00` 完成：21 个关键输入中 20 个存在，唯一缺口为
   指定根交接文件；ROS/Gazebo 未启动，新正式种子未访问，稳定工作区未修改。
6. 已校验 V6 component/validation/formal 三份清单共 11/11 项，以及 V6-06 R1 终端内容清单
   77/77 项；`src/application/v5_e2e_scope_rl/tests` 为 7/7 通过。
7. P01 在独立目录重算冻结 V6-02-F1：320 行科学字段逐项一致，bootstrap CSV SHA256
   `97d5dea6761dacdd0c1f09756ecbc2e184dbea7450c74da7ccf282685a77b79a` 逐字节一致；
   差异仅为预声明不确定的墙钟推理时延。主效应 8.402444%，双侧 97.5% CI
   [3.484003%, 13.713459%]，但范围严格限于历史四类快速传感器总体。
8. P02 离线诊断复现了历史迁移失配：激活位置跳过 1.4 m、actor 到达冲突点早 4.0053 s、
   训练/旧运行动态占据膨胀与终点语义不一致。诊断报告为
   `diagnostics/p02_frozen_migration_diagnosis.json`。
9. 新增训练/运行同源的 72-beam 候选适配器、scan/odom/TF fail-closed 合同、
   suggestion→revalidation→commit 事务、方向 LaserScan 跟踪、响应延迟、停车后 1.5 s
   不变性验证、终点速度包络和候选在 `x=2.0 m` 前回归全局路线。旧测试与新增测试为
   19/19 通过，`catkin_make -j2` 通过。
10. 当前 PCT 版本按冻结顺序完成 6/6 fresh-development Gazebo 准入：shadow seeds
    196021/196019/196022 与 takeover seeds 196023/196020/196024 在 static/single/occluded
    场景均到达、contact=0、mask violation=0、完整性通过；最低保守净空 +0.051338 m，
    最大 episode P99 推理 6.878368 ms（门限 20 ms）。报告 SHA256
    `61cc9d7a80fbe1b5fb71d62b87e65cad9c97398b5e0fff25845754c21c614c41`。
11. P02 共保留 39 个 development identities：13 个目标/完整性通过结果、19 个保守净空
    失败、6 个 timeout/nonarrival 和 1 个首次 Python/torch 基础设施失败；没有删除或重跑
    相同身份。`run_manifest.csv` 与 `episode_results.csv` 已分别更新至 41/359 行并通过唯一性检查。
12. Actual TEB 与 DWA 的静态 ROS smoke 均通过。动态调优仍未完成：TEB v0 在配对
    development seed196030 的最近净空为 -0.000523 m；降低 TEB 速度上限的 v2/v3 违反
    “不得靠降低基线速度通过准入”的选择边界，已明确淘汰但保留证据。TEB/DWA 遮挡 smoke
    为零 contact 但保守净空分别 -0.104607/-0.127211 m，必须继续公平调优或作为正式失败计入。

## 当前未完成与下一步

扩展到主合同要求的六类核心场景，生成四方法同 seed、顺序平衡的 development 矩阵；在相近
试验预算下完成 TEB/DWA 搜索空间、选择准则与最终 YAML，同时评估方向跟踪/候选条件预测的
训练-运行一致性并决定是否需要 development-only PPO 重训。pilot/formal seeds 仍未访问。

## P03 静态候选修复检查点

- development seeds196041--196074 共 34 个身份全部保留：1 goal、24 timeout/nonarrival、
  8 execution-integrity failures、1 infrastructure failure；没有相同身份 retry。
- seed196074 首次通过核心静态绕障门：goal、integrity pass、20.45 s、contact=0、最小保守净空
  0.224907 m。该结果仅证明当前解析先验在一个 development 静态身份上的链路可行。
- 修复内容包括 scan 时刻 TF、完整静态地图验证、候选覆盖/平台、停车后侧向承诺、共同 gate
  原子交接和几何可行性闭合；27/27 单元/回归测试通过。
- 本检查点之前的 development episode 未在启动前绑定逐文件源码 SHA，已声明为完整性限制；
  runner 自下一身份起生成 `preflight_bindings.json`。pilot/formal 仍未访问或授权。

## P04 development 检查点

- 新 benchmark PPO identity 完成 40,008 环境步；训练 571/575 到达、0 contact、0 mask violation。
- 互斥离线验证 1,200 episodes：新 PCT 400/400、解析先验 400/400、历史 PCT 392/400；均保留逐 episode CSV。
- 首个六场景同 seed 四方法开发矩阵：PCT 6/6、解析 6/6、TEB-v0 4/6、DWA-v0 3/6；失败统一按 22 s。
- 六块失效计入均时：PCT 15.6917 s、解析 15.9083 s、TEB-v0 13.2833 s、DWA-v0 16.1167 s。仅为自适应 development 描述，不是优势证据。
- TEB/DWA 的保守、homotopy、LiDAR 动态转换、inflation 与长视野变体均完整保留；当前 development 选择仍为各自 v0，禁止用较慢变体弱化基线。
- static 条件高速脉冲正向，持久高速承诺造成安全停车死锁并回滚；方向 tracker 的关联劫持已修复。34/34 测试通过。
- pilot/formal seeds 未访问，正式执行仍未授权。

## P05 development 冻结候选

- PCT R2 头对头标签与 R3 评估足迹重训均完成 40,008 步并封存；独立离线开发验证分别为 1,200/1,600 回合，R1 仍以 400/400、16.7863 s、0 contact/mask violation 获选。
- 评估足迹修复把空间候选中心从 1.40 m 调整到 1.35 m，并让无扫描墙面回退、训练 rank oracle 和物理净空统一计入 0.33 m 膨胀；安全门未放宽。41/41 测试通过。
- seed196115 因外层 PYTHONPATH 覆盖导致 roslaunch 导入失败，按基础设施失败保留且未重跑；fresh seed196116 静态验证 15.75 s、净空 0.20995 m。
- 最终六场景四方法开发矩阵：PCT 6/6、14.9500 s；解析 4/6、17.0750 s；TEB-v0 4/6、13.3000 s；DWA-v0 3/6、16.1083 s。均为失效计入均时。
- PCT 相对 DWA 的开发点改善 7.1903%，低于 8% 实效门；相对 TEB 为 -12.4060%。这只是自适应开发描述，不能支持优势声明。
- 代码/actor/TEB-v0/DWA-v0 已作为 pilot 候选固定；pilot/formal seeds 尚未访问。

## P06 preregistered pilot

- 唯一 release SHA `967c4d688405f0a40a95ae99d9b856948dc79632a42636dde72eddc90f70fd7b` 按预注册日程完成 20 blocks / 80 episodes，全部 attempt=1，哈希/报告/身份闭包通过，ROS/Gazebo 清场。
- 失败计入均时：PCT 15.1550 s（19/20）、解析 16.7850 s（17/20）、TEB 14.8525 s（11/20）、DWA 16.0775 s（10/20）；全方法 contact=0，PCT mask violation=0。
- PCT 相对 DWA 点改善 5.7378%，相对 TEB 为 -2.0367%；pilot 不支持优势声明且未用于调参。
- 8% 目标下，DWA 比较 80% 功效约需 153 blocks；TEB 约需 253，超过 200 上限。正式 R1 选择 160 blocks，对 TEB 功效不足明确保留为限制。
- formal seeds 尚未访问，正式 release 尚未创建。

## P07 formal R1 release

- 在任何 formal outcome 访问前冻结 160 blocks / 640 episodes，fresh seeds197001--197160；四方法在四个顺序位置各 40 次。
- 日程 SHA `89adaa3ad8c6175c2223b91f3b457c789b94858d2b606515413da6bac8c496e8`，预注册 SHA `13785745d0ff40596aacb19c8cb5880baeb9480fce8ba8e3a6714a984078156b`，唯一 release SHA `af9ce1b2e5887cc4b41ef822dd877a50eb222bcdc67107163cc2de920c1cd93f`。
- 主指标、8% 实效门、97.5% 配对 bootstrap、完成率非劣 -5pp、contact/mask 门和 22 s 失败记账已锁定；formal outcome 尚未访问。

## P08 formal R1 result

- 160 blocks / 640 episodes 全部 attempt=1 完成；无 retry、replacement 或删除，正式 R2 seeds 未访问。
- 失败计入均时：PCT 15.0634 s（153/160）、解析 16.1231 s（146/160）、TEB 14.9225 s（89/160）、DWA 16.0881 s（80/160）。
- PCT 对 TEB 改善 -0.9442%（97.5% CI [-8.3569%, 5.6359%）；对 DWA 改善 6.3693%（97.5% CI [1.0871%, 11.1411%）。
- `formal_result=true`；`performance_improvement_demonstrated=false`；阳性比较：none。
- 完整性 review SHA `f2fb67d34998c17adba1c174608a37613696187cdd882f758d9dbc641cec9422`。

## P09 R2 development 单因素候选覆盖

- 12 个预注册 development 配对块 / 24 回合全部 attempt=1 终止；首个 control 为预语义导入失败，按 22 s 保留且未重试，其余 23 个报告通过哈希闭包。
- 失败计入均时：R1 control 15.5917 s（10/12），0.6 m/s 候选 14.3917 s（12/12），改善 1.2000 s。
- 静态场景改善 5.6250 s；四个动态场景均值变化 -0.4187 s（负值为改善）；treatment contact=0、mask violation=0、最小保守净空 0.225445 m。
- 全部预注册门限通过；仅推广该候选速度单因素进入新的 pilot/R2 冻结候选。正式 R2 seeds198001--198200 尚未访问。

## P10 preregistered R2 pilot

- release `61bd7379d9c7cd105fff5f02169971bd42569a75179d9d6160bee63600d6d3c7` 下 20 blocks / 80 episodes 全部 attempt=1 完成，无重试、替换或删除。
- 失败计入均时：PCT 14.2750 s（20/20）、解析 16.1750 s（14/20）、TEB 14.5376 s（12/20）、DWA 16.0700 s（10/20）。
- PCT 对 TEB/DWA 的 pilot 点改善分别为 1.8064% / 11.1699%；pilot 不用于调参或优势声明。
- 冻结功效规则选择 formal R2 为 200 blocks；正式 seeds198001--198200 尚未访问。

## P11 formal R2 release

- 在任何 R2 formal outcome 访问前冻结 200 blocks / 800 episodes，fresh seeds198001--198200；四方法在四个顺序位置各 50 次。
- 日程 SHA `9b83713ee4fabbd86e9009a2c7b6bb89f533142824b5c1f24820d55cae2b065c`，预注册 SHA `fabdc7034f4e352e258873e76b754cd890ff3fbef099a198c3ee2395ef31cffb`，唯一 release SHA `c64dee01acbdf58ece4b9eb2056d039c6d2ee61121dbbbcffbc717c8c5253137`。
- 相对 R1 只改变 actions4/5 空间候选速度 0.4→0.6 m/s；PPO actor、解析公式、硬安全门、TEB-v0、DWA-v0、统计门和失败记账均不变。
- 正式 R2 outcome 尚未访问。

## P12 formal R2 result

- 200 blocks / 800 episodes 全部 attempt=1 完成；无 retry、replacement 或删除。
- 失败计入均时：PCT 14.8413 s（180/200）、解析 16.3988 s（150/200）、TEB 14.4470 s（119/200）、DWA 16.0925 s（100/200）。
- PCT 对 TEB 改善 -2.7288%（97.5% CI [-9.8083%, 3.5914%）；对 DWA 改善 7.7754%（[2.5521%, 12.6148%）。
- `performance_improvement_demonstrated=false`；阳性比较：none；完整性 review SHA `a937f5272756bb2b1a10526046d03ff9cbf0dd6f8d41be3658e7b8419b7a6265`。

## P13 thesis result transfer

- 中文论文回填、四张 SVG、四张机器表与终端完整性清单已生成；无新仿真、训练或硬件访问。
- `THESIS_RESULT_TRANSFER.md` SHA 将由 `final_transfer/final_transfer_integrity.json` 绑定。
- 封存后分层诊断将 head-on/single-crossing 的 stop/switch 负担与 sequential 超时保留为可证伪机制假设；任何后续只能在新 development 数据上检验，不得使用 R1/R2 formal seeds 调参。

## P14 external paper-writer handoff

- 已生成面向另一论文写作系统的外层权威交接目录，嵌入历史 V6-06 包并新增两轮 Gazebo formal、
  claims contract、canonical JSON、LaTeX 章节、图表、配置/代码和原始证据 SHA 索引。
- 全量索引覆盖 23,744 个 benchmark 文件（约 806 MB）；便携包不重复复制逐 episode ROS 日志。
- 本阶段为纯离线派生：无训练、仿真、formal seed、ROS/Gazebo、硬件或实车；episode 总账保持
  6,145 行且唯一。旧 V6-06 包保持字节历史，只由外层 supersession notice 更新 Gazebo 写作口径。
