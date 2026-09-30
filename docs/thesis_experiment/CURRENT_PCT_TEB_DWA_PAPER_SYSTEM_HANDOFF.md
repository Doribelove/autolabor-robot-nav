# PCT-SCOPE-RL / TEB / DWA 论文系统完整移交终态

日期：2026-08-10

## 状态

外层论文写作交接包已经完成并通过完整性检查。该包整合历史 V3/V4/V5/V6 论文材料、V6-06
41 页整合技术稿、最新 Gazebo 迁移修复、公平基线配置、development/pilot/preregistration、两轮
正式研究、逐 episode CSV、统计、图表、失败归因、声明边界和可直接交给另一系统的提示词。

目录：

```text
artifacts/paper_writer_handoff/FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810/
```

便携压缩包：

```text
artifacts/paper_writer_handoff/FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810.tar.gz
```

压缩包 SHA256：

```text
c22a369561f1e9041d02fa76a564f3640cd7528a871293be9b163a374bdc4b29
```

## 使用顺序

首先读取外层 `00_READ_ME_FIRST.md`、`claims_contract.yaml`、`canonical_results.json` 和
`data/CURRENT_COMPLETE_RESULT_SUMMARY.csv`。随后读取 01--08 号说明、最新 benchmark transfer、
CH11 LaTeX 章节，再把 `upstream_v6_06_historical/` 用作旧 V3/V5/V6 章节基础。

旧 V6-06 包本身没有改字节。其“未运行 Gazebo formal”是 2026-08-08 的历史状态；外层包记录的
后续 R1/R2 已完成，但二者均没有达到预注册 superiority 门。Gazebo 写作发生冲突时，外层
`claims_contract.yaml` 优先；其他历史冻结结果保持有效。

## 当前关键结果

- V6 fast-population：8.402444%，双侧 97.5% CI [3.484003%,13.713459%]，限定阳性。
- Gazebo R1：160 blocks / 640 episodes；PCT vs TEB -0.9442%，vs DWA 6.3693%，均阴性。
- Gazebo R2：200 blocks / 800 episodes；PCT vs TEB -2.7288%，vs DWA 7.7754%，均阴性。
- 两轮正式身份均 attempt=1，无 retry/replacement；全部失败保留并按 22 s 计入。
- R2 PCT 保留 18 次超时、2 次保守净空失败；不能声称形式安全。
- 实车仍为 `pending_no_evidence`。

## 校验

- 外层 manifest：`0589ae9ef00c30b975929630eade9994d1396eba9ff7ce8b90c88d410ec22978`
- handoff integrity：`ac3d3dbf49fb9e5cb3311bb313f7e00cb9f026e2f4fe874e7df91b50b9b356b9`
- archive：`c22a369561f1e9041d02fa76a564f3640cd7528a871293be9b163a374bdc4b29`
- 241/241 外层内容通过；76/76 历史包内容通过；CH11+表格编译为 3 页 A4。

本阶段没有训练、仿真、正式种子访问、ROS/Gazebo、硬件、实车或在线 TEB 写入；稳定工作区
`/home/robot/robot_ws` 未修改。
