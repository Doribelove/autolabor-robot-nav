# 可直接发送给论文写作系统的提示词

你负责依据一个冻结证据包撰写或更新中文研究生论文。项目根目录为 `<PROJECT_ROOT>`，当前权威
交接包为 `<PROJECT_ROOT>/artifacts/paper_writer_handoff/FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810`。
学校模板初稿为 `<SCHOOL_THESIS_DRAFT>`。先复制初稿形成可回退副本，不覆盖唯一原稿。

开始前按顺序完整读取 `00_READ_ME_FIRST.md`、`claims_contract.yaml`、`canonical_results.json`、
`data/CURRENT_COMPLETE_RESULT_SUMMARY.csv`、01–07 号说明、`benchmark/THESIS_RESULT_TRANSFER.md`、
`chapters/CH11_PCT_TEB_DWA_GAZEBO_BENCHMARK.tex`，再读 `upstream_v6_06_historical/README_FIRST.md`
及其整合技术稿、数据字典和 claims。若旧包与外层包的 Gazebo 陈述冲突，以外层包为准。

论文必须同时呈现：V4/V5 学习阴性；V6 fast-population 限定阳性；旧 Gazebo preflight 阴性；
新 Gazebo R1/R2 均未达到 TEB/DWA superiority 门。不要为了故事连贯而删除任何反证。

必须使用以下精确结论：R1 PCT vs TEB -0.9442% [−8.3569%,5.6359%]，vs DWA 6.3693%
[1.0871%,11.1411%]；R2 vs TEB -2.7288% [−9.8083%,3.5914%]，vs DWA 7.7754%
[2.5521%,12.6148%]。所有区间为双侧 97.5%。R2/DWA 因点值低于冻结 8% 门而判阴性。

完成后交付：更新后的模板源和 PDF、逐章 changelog、claims audit、图表/表格来源清单、所有引用
未决问题、实车待填字段。实车章节保持 `pending_no_evidence`。禁止声称 PCT 优于 TEB/DWA、形式
安全、一般 OOD/感知/多人泛化、部署或实车有效，也禁止把 V3 结果写成 V6/PCT 结果。
