# 论文写作系统移交入口

日期：2026-08-10

这是当前项目交给另一套论文写作系统的唯一外层入口。它在历史 V6-06 跨电脑论文包之上，加入了
最新完成的 PCT-SCOPE-RL、解析先验、Actual TEB 和 DWA 两轮 Gazebo 公平正式研究。

## 交付路径

- 可直接读取的目录：
  `artifacts/paper_writer_handoff/FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810/`
- 可发送到另一系统的压缩包：
  `artifacts/paper_writer_handoff/FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810.tar.gz`
- 压缩包 SHA256：
  `c22a369561f1e9041d02fa76a564f3640cd7528a871293be9b163a374bdc4b29`
- 校验文件：
  `artifacts/paper_writer_handoff/FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810.tar.gz.sha256`

另一系统解压后必须首先阅读 `00_READ_ME_FIRST.md`，再按其中顺序读取 `claims_contract.yaml`、
`canonical_results.json` 和 01--08 号文件。`08_PROMPT_FOR_PAPER_WRITING_SYSTEM.md` 可以直接作为
另一系统的任务提示词。

## 当前论文结论

- V6 在冻结的 fast sensor-level population 上相对解析先验有 8.402444% 限定阳性；这不是
  Gazebo、传统规划器或实车证据。
- 最新 Gazebo R1/R2 共 360 paired blocks / 1,440 episodes，全部 attempt=1，无补跑或替换。
- R1/R2 均未按预注册综合门建立 PCT 优于 Actual TEB 或 DWA。
- R2 对 DWA 为 7.7754%，双侧 97.5% CI [2.5521%,12.6148%]，但未达到冻结 8% 点门。
- R2 对 Actual TEB 为 -2.7288%，区间 [-9.8083%,3.5914%]。
- 不支持形式安全、一般感知/OOD、多人泛化、部署或实车声明。

## 完整性

- 包内 content manifest SHA：
  `0589ae9ef00c30b975929630eade9994d1396eba9ff7ce8b90c88d410ec22978`
- handoff integrity SHA：
  `ac3d3dbf49fb9e5cb3311bb313f7e00cb9f026e2f4fe874e7df91b50b9b356b9`
- canonical results SHA：
  `90420db7a532be733a71d1c735df00e0ad08b2ca97b12ca57aac41de95b41b40`
- claims contract SHA：
  `a42bf2e3c2dfd90700f8595297abb2d4bf3dc67f6b1df8f6fe98a14881f5a6cc`

包内 241 个 manifest 条目全部通过，历史 V6-06 内层 manifest 76/76 通过；新 LaTeX 章节及表格
已独立编译为 3 页 A4。完整索引覆盖原 benchmark 的 23,744 个文件；约 806 MB 原始日志不重复
复制，但保留原路径、字节数和 SHA256。

历史 V6-06 包保持原字节并放在 `upstream_v6_06_historical/`。其中“尚无 Gazebo formal”仅描述
2026-08-08 当时状态；新论文必须加入后续两轮正式阴性研究，不能把时间性更新写成 Gazebo 阳性。
