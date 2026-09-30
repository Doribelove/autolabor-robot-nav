# 数据字典

- `canonical_results.json`：全部核心历史结果和最新两轮 Gazebo 结果的机器权威总表。
- `claims_contract.yaml`：当前允许、未证明和禁止声明；写作前必须读取。
- `data/CURRENT_COMPLETE_RESULT_SUMMARY.csv`：研究级一行一结论汇总。
- `benchmark/formal/round_*/formal_r*_assessment.json`：正式统计、门限与逐身份哈希证据。
- `benchmark/formal/round_*/formal_r*_episode_results.csv`：R1 640、R2 800 个正式 episode。
- `benchmark/formal/round_*/formal_r*_schedule.csv`：配对区块、种子、场景、方法和顺序。
- `benchmark/formal/round_*/formal_r*_preregistration.yaml`：结果访问前冻结的指标与门。
- `benchmark/formal/round_*/formal_r*_execution_release.yaml`：执行资源绑定与 SHA。
- `benchmark/final_transfer/*.csv`：方法、效应、场景和失败机制的论文表。
- `benchmark/final_transfer/*.svg`：四张可编辑新结果图。
- `figures/pct_gazebo/*.png`：上述四图的可直接插入版本，像素尺寸与 SVG viewBox 一致。
- `benchmark/ledgers/episode_results.csv`：全研究 6,145 行 episode 总账，不等于只有正式数据。
- `benchmark/ledgers/run_manifest.csv`：运行/离线阶段总账。
- `indices/full_raw_benchmark_evidence_index.csv`：未复制的 868 MB 原始证据逐文件路径和 SHA。
- `indices/portable_source_index.csv`：包内复制品与原仓库路径的映射。
- `upstream_v6_06_historical/data/`：V3、V5、V6 fast-population 历史机器数据。

布尔 `valid_goal` 同时要求 goal、执行完整性、非负冻结保守净空和无 contact。主时间指标把所有
失败统一替换为 22 s。相对降低定义为 `1 - mean(PCT)/mean(baseline)`；正数表示 PCT 更快。
