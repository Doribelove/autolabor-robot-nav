# FAM-TEB / V3-SCOPE 论文实验完整复现说明

日期：2026-08-06

## 1. 复现目标与两种模式

本说明把“复核既有论文证据”和“独立运行新的科学复现”明确分开。

1. **证据复核**：不启动 ROS/Gazebo，逐字节检查原始数据、统计结果、图表和 PDF。该路径可在当前冻结工作区立即执行，且应得到相同 SHA-256。
2. **新鲜种子复现**：复用四方法运行框架，但必须建立新的阶段名、release、seed 区间、预算与独立审查。不得重跑 V3-19/V3-20/V3-21 的身份，也不得覆盖其 artifact 目录。

历史 V3-11、V3-19、V3-20 和 V3-21 都是一次性冻结阶段。复现研究应复现方法和设计，不应伪造同一 attempt 的第二次执行。

## 2. 项目边界

- 论文工作区：`/home/robot/robot_ws_base_rl`
- 稳定实车工作区：`/home/robot/robot_ws`
- 当前论文分支：`base_rl`
- ROS：Noetic 1.17.4
- Gazebo：11.15.1
- TebLocalPlannerROS：0.8.4
- Python：3.8.10
- 操作系统：Linux x86_64，Ubuntu 20.04 兼容 ROS Noetic 环境

所有命令都应在论文工作区运行。不要在同一终端同时 source 两个工作区。本文没有实车、CAN、串口、在线 TEB 写入或硬件运动；任何实车动作必须另获现场批准。

## 3. 获取仓库与子模块

```bash
git clone --recurse-submodules git@github.com:Doribelove/autolabor-robot-nav.git robot_ws_base_rl
cd robot_ws_base_rl
git switch base_rl
git submodule update --init --recursive
```

若收到的是论文归档包，应保持 `artifacts/`、`config/`、`docs/`、`experiments/` 和 `src/` 的相对路径不变。生成目录 `build/`、`devel/`、`install/`、`log/` 和 rosbag 不属于版本化科学输入。

## 4. 构建与环境检查

```bash
cd /home/robot/robot_ws_base_rl
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack find v3_scope_planner
rospack find m2_gazebo
rosversion teb_local_planner
```

预期 `rosversion teb_local_planner` 为 `0.8.4`。四方法 episode 启动入口是：

```text
src/simulation/m2_gazebo/launch/m2_v3_19_thesis_benchmark_episode.launch
```

方法语义实现位于：

```text
src/application/v3_scope_planner/src/v3_scope_planner/thesis_benchmark_methods.py
src/application/v3_scope_planner/scripts/v3_scope_thesis_benchmark_episode.py
```

## 5. 系统架构与数据流

系统目标是在动态交互中输出 Ackermann 可执行、可解释、可回退的局部轨迹选择，而不是在线自由写入高维 TEB 参数。

```text
scan / odom / GPS / TF / global path / costmap / health
                         │
                         ▼
             immutable scene snapshot
                         │
                         ▼
       Ackermann-feasible candidate generator
       (left/right/straight/slow/certified stop)
                         │
                         ▼
       candidate-conditioned actor predictor
                         │
                         ▼
          calibrated reachable occupancy
                         │
                         ▼
     hard verifier: geometry + braking + commit
                         │
                         ▼
        analytic ranker or certified-stop fallback
                         │
                         ▼
                     tracker
                         │
                         ▼
               /cmd_vel_navigation
                         │
                         ▼
       GPS limiter → FOD arbiter → M2 driver
```

论文实验只运行到 Gazebo 执行边界。完整机器可读架构在：

```text
artifacts/v3/thesis/v3_18_a0_project_audit/system_architecture.yaml
```

## 6. 标准化采集模式

统一 episode 模式包含六类字段：

- identity：stage、sequence、block、attempt、seed、scene、actor policy、method；
- execution：完成状态、终止原因、wall/sim time、retry/resume；
- task：goal success、22 s restricted time、path length、final error；
- safety/mechanism：contact、collision、minimum clearance、fallback、候选身份、规划延迟；
- provenance：contract/release/schedule/report/raw-manifest SHA-256；
- authority：simulation-only、training steps、hardware/real-vehicle flags。

数据模式实现：

```text
src/tools/thesis_experiment/src/thesis_experiment/thesis_dataset.py
artifacts/v3/thesis/v3_18_a0_project_audit/standard_collection_schema.yaml
artifacts/v3/thesis/v3_18_b0_canonical_dataset/data_dictionary.yaml
```

历史证据已经适配为 825 行统一数据集：V2-P1 270、V3-05 270、V3-09 180、V3-11 60、V3-16 45。V3-21 另有本阶段的 120 行 canonical dataset。

## 7. 纯离线证据复核

### 7.1 项目审计

```bash
python3 src/tools/thesis_experiment/scripts/build_v3_18_a0_thesis_evidence_audit.py --check
python3 src/tools/thesis_experiment/scripts/review_v3_18_a0_thesis_evidence_audit.py --check
sha256sum -c artifacts/v3/thesis/v3_18_a0_project_audit/terminal_freeze.sha256
```

### 7.2 历史统一数据集

首版 manifest 因生成器源文件在封存后漂移而留下一个预期失败，不能删除；终态使用 R1 manifest 修复，dataset 字节不变。

```bash
python3 src/tools/thesis_experiment/scripts/repair_v3_18_b0_r1_dataset_manifest.py --check
python3 src/tools/thesis_experiment/scripts/review_v3_18_b0_r1_canonical_dataset.py --check
sha256sum -c artifacts/v3/thesis/v3_18_b0_canonical_dataset/terminal_freeze_r1.sha256
```

若专门重放历史失败，可单独运行 `build_v3_18_b0_canonical_dataset.py --check`；它应因首版
`source_and_output_manifest.sha256` 的生成器源哈希漂移返回非零。该失败是已保留证据，不属于
终态复核失败。

### 7.3 V3-21 统计复算

```bash
python3 src/tools/thesis_experiment/scripts/assess_v3_21_f1_characterization.py --check
python3 src/tools/thesis_experiment/scripts/review_v3_21_f1_characterization.py --check
sha256sum -c artifacts/v3/thesis/v3_21_f1_execution/terminal_freeze.sha256
```

预期关键结果：

```text
episodes/paired blocks = 120/30
V3 success/time = 28/30, 7.785 s
Actual TEB success/time = 30/30, 7.770 s
V3 vs Actual TEB = -0.193%, 95% CI [-19.867%, 12.982%]
V3 vs Fixed-v1.20 = 64.614%, 95% CI [57.682%, 69.250%]
V3 vs no conditioning = 49.454%, 95% CI [37.001%, 58.800%]
contact count = 0 for all methods
```

### 7.4 图和三线表复核

```bash
python3 src/tools/thesis_experiment/scripts/build_v3_22_p1_paper_materials.py --check
python3 src/tools/thesis_experiment/scripts/review_v3_22_p1_paper_materials.py
sha256sum -c artifacts/v3/thesis/v3_22_p1_paper_materials/terminal_freeze.sha256
```

输出目录：

```text
artifacts/v3/thesis/v3_22_p1_paper_materials/
├── figures/        # 5 图 × PDF/SVG/300-dpi PNG
├── tables/         # 5 个 booktabs 三线表
├── RESULTS_NARRATIVE_ZH.md
├── RESULTS_NARRATIVE_EN.md
├── paper_claims.yaml
└── paper_materials_manifest.sha256
```

## 8. 四方法实验设计

### 8.1 方法

1. `v3_spatial_safe`：主方法，候选条件预测、保守占据、硬验证、解析排序。
2. `actual_teb`：工作区真实 `teb_local_planner/TebLocalPlannerROS` 0.8.4 主基线。
3. `fixed_v120_shielded`：固定 1.20 m/s 屏蔽比较策略；certified stop 保持有效。
4. `v3_no_candidate_conditioning`：单因素消融，只替换预测头；其余 V3 链保持不变。

### 8.2 场景和随机化

- `open_crossing_90`
- `wide_corridor_crossing_70`
- `bounded_corridor_crossing_110`
- actor policy：`cautious`、`hesitant`
- 六个 scene×policy strata，每层 5 个 block
- 每个 block 四方法共享一个 simulator seed
- 方法顺序使用确定性平衡 Latin rotation
- V3-21 seeds 8201–8230 已消费，禁止复用

### 8.3 统计

- 独立单位：paired scene-seed block；
- 主结果：V3 与 Actual TEB 的失效计入式平均受限时间相对差；
- 任务失败：保留为 22.0 s；
- paired block bootstrap：20000 次、双侧 95% CI；
- 三个比较报告双侧 bootstrap p，并作 Holm 校正；
- 同时报告成功率、净空、contact、路径、回退率、P99 规划延迟和六层结果；
- 禁止把规划帧或轨迹采样点当作额外独立样本。

## 9. 新鲜种子独立复现协议

以下是框架级流程，不是对 V3-21 的续跑授权。

1. 从 `v3_21_a0_ephemeral_port_repair_contract.yaml` 派生新合同，使用新的 stage 名和未使用 seed 区间。
2. 预注册场景、方法、timeout、失败处理、bootstrap seed、停止规则和 claim 边界。
3. 生成 exact schedule、execution release、simulation authorization 和 path+SHA closure。
4. 由不同于生成器的 review 脚本确认 release；review 通过前不得启动 ROS/Gazebo。
5. 使用低于 ephemeral range 的成对 master 端口；每个 identity 只允许 attempt 1。
6. 运行完整矩阵；失败必须保留，禁止 retry、replacement 或 selective exclusion。
7. assessment 从 raw receipts 重建 canonical CSV/JSONL，再由独立 reviewer 复算。
8. 最后才生成图表和论文声明。

运行器接口形式为：

```bash
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws_base_rl/devel/setup.bash
python3 src/tools/thesis_experiment/scripts/run_v3_21_ephemeral_port_repair_characterization.py \
  --phase formal \
  --output-root artifacts/v3/thesis/NEW_STAGE_EXECUTION \
  --execute
```

不得直接执行上例，除非新合同、release、authorization、fresh schedule 和独立 review 已经完成，并且运行器绑定已更新到新阶段。V3-21 的 release SHA `94fee18f...cc963`、seeds 8201–8230 和 artifact 目录都是终态只读证据。

## 10. 论文 PDF 构建

```bash
cd /home/robot/robot_ws_base_rl/docs/thesis_experiment/thesis_draft
latexmk -xelatex -interaction=nonstopmode -halt-on-error FAM_TEB_V3_THESIS_DRAFT.tex
pdfinfo FAM_TEB_V3_THESIS_DRAFT.pdf
pdftotext FAM_TEB_V3_THESIS_DRAFT.pdf -
```

终态导出脚本会在独立 artifact 目录中编译，不修改稳定实车工作区。PDF 审查至少检查：页数、A4、字体嵌入、关键统计文本、五图五表、声明边界、PDF SHA 和 evidence manifest。

## 11. 结果解释规则

- V3-11：限定单 actor/oracle track 的历史正式阳性，12.9288%；
- V3-21：新的完整失效计入式表征，没有证明相对 Actual TEB 的时间优势；
- 两阶段不得合并或只选有利 seeds；
- V3-21 相对去候选条件化的 49.454% 可写为冻结总体内的探索性机制证据；
- contact=0 和经验净空不是形式化安全；
- 不得声称感知、多人/OOD、部署、实车或普遍 TEB 优势；
- 训练步数为 0，不得声称强化学习收益。

## 12. 故障排查

- `rospack find m2_gazebo` 失败：确认只 source 本工作区 `devel/setup.bash`，并重新 `catkin_make`。
- ROS master 端口占用：新阶段必须分配明确的低端口 namespace，不得使用 V3-20 的 34000+2×sequence。
- `/clock` 不前进：先建立正进展 clock barrier，再等待 move_base/dynamic-reconfigure readiness。
- Gazebo 残留进程：终止当前合法 identity，记录失败并按合同停止；不得以 retry 覆盖。
- manifest hash 不一致：先保留失败，判断是证据字节漂移还是仅 manifest/source closure 漂移；不得覆盖原始证据。
- PDF 缺图：必须从 `thesis_draft` 目录执行 XeLaTeX，使 `../../../artifacts/...` 相对路径可解析。

## 13. 冻结证据入口

- 项目审计：`artifacts/v3/thesis/v3_18_a0_project_audit/`
- 历史统一数据：`artifacts/v3/thesis/v3_18_b0_canonical_dataset/`
- 四方法资格阴性：`artifacts/v3/thesis/v3_19_q1_execution/`
- 端口碰撞终止证据：`artifacts/v3/thesis/v3_20_f1_execution/`
- 120 回合结果：`artifacts/v3/thesis/v3_21_f1_execution/`
- 图表与论文材料：`artifacts/v3/thesis/v3_22_p1_paper_materials/`
- 终态论文导出：`artifacts/v3/paper/v3_22_p2_thesis_export/`
