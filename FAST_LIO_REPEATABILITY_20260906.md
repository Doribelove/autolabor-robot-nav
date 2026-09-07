# FAST-LIO 已知地图定位修复与重复定位验收（2026-09-06）

## 结论

已修复人工初值附近仍被宽范围粗 ICP 拉入重复走廊错误极值、持续 `LOST` 的问题，
并部署到 J6M candidate release `20260906_190656`。实机完成 5 个完整组，每组固定
人工初值重复定位 12 次。按追加试验前固定的规则，选择
`max(X P75, Y P75)` 最小的完整组；最佳为第 4 组：

| 指标 | 实测 | 条件 | 结果 |
|---|---:|---:|---|
| 最佳组 X 轴绝对偏差 P75 | 0.01442 m | ≤ 0.05 m | 通过 |
| 最佳组 Y 轴绝对偏差 P75 | 0.02330 m | ≤ 0.05 m | 通过 |
| 全部 60 次 X 轴绝对偏差 P75 | 0.01289 m | ≤ 0.05 m | 通过 |
| 全部 60 次 Y 轴绝对偏差 P75 | 0.03592 m | ≤ 0.05 m | 通过 |
| 完整组成功重定位 | 60/60 | — | 完整 |

单组“偏差”是每次稳定窗口位置中位数相对该组 12 次总体中位数的轴向绝对偏差；
全部 60 次汇总值则相对 60 次总体中位数计算。
现场没有外部真值设备，因此该结论只证明重复定位一致性，不证明相对地图真值的
绝对坐标误差。

## 修复内容

原实现每次都先执行最大对应距离 5 m 的粗 ICP，再执行精 ICP。在具有相似几何的
室内走廊中，同一份实机数据会被粗搜索从正确人工初值拉到数米外的错误局部极值，
最终 RMSE 超过 0.35 m 并持续 `LOST`。

新实现先从操作员初值执行最大对应距离 0.50 m 的局部精 ICP，并继续使用原有
内点数、重叠率和 RMSE 质量门。局部结果未通过时才回退原粗到精搜索，因而保留
较差初值的恢复能力。隔离回放中，新路径连续 9 次接受，RMSE 为 0.253–0.262 m、
重叠率为 0.347–0.444、内点为 1297–1649。

## 实机测量

- 操作员在本次冷启动后重新提交真实初始位姿；
- 每个完整组 12 次，5 组共 60 次；每次均重新发布同一人工初值，并必须观察到
  `ALIGNING → LOCALIZED`；
- 每次进入 `LOCALIZED` 后等待 0.25 秒，再取 1.5 秒 `/localization` 的中位数；
- 全部 60 次收敛时间 P50 4.24 秒、P75 6.21 秒、最大 12.25 秒；
- 全部 60 次 ICP 最小重叠率 0.5290、最大 RMSE 0.2690 m、最少内点 1916；
- 试验期间活动导航目标为 0，最终速度及左右轮反馈最大值均为 0；
- 5/5 个完整组均满足两轴 P75 不超过 0.05 m；
- 操作员说明此前扰动压力试验期间有人走动，该数据已单独保留且不纳入完整组；
- 另有一次中止尝试在旧的 24 秒等待窗口内第 3 次未收敛；该尝试单独保留，没有
  将前两次结果拼接进任何完整组。等待上限后来改为 35 秒，定位质量门未改变。

全部五组结束后的 10 秒复核中，202 个定位样本全部处于 `LOCALIZED`；X/Y 连续
定位绝对偏差 P75 分别为 0.01171 m 和 0.00987 m，最终速度与左右轮反馈仍为 0。

## 产物

- 最佳组数据图：`validation/fast_lio_repeatability_20260906/best_group_p75/repeatability_p75_dashboard.png`
- 五组对比图：`validation/fast_lio_repeatability_20260906/multi_group_final/multi_group_p75_comparison.png`
- 多组报告：`validation/fast_lio_repeatability_20260906/multi_group_final/REPORT.md`
- 多组汇总：`validation/fast_lio_repeatability_20260906/multi_group_final/groups_summary.json`
- 逐组指标：`validation/fast_lio_repeatability_20260906/multi_group_final/group_metrics.csv`
- 5 个完整组原始数据：`validation/fast_lio_repeatability_20260906/fixed_seed_trials_*`
- 最终运行复核：`validation/fast_lio_repeatability_20260906/final_runtime_check_after_groups/`

## 部署与验证

- NVIDIA 本机构建通过；
- 定向 localizer contract 测试 6/6 通过；
- 完整静态检查 997 项全部通过；
- 隔离 ROS master 上的真实数据回放通过；
- J6M ARM64 原生构建、安装空间检查和静态健康检查通过；
- J6M `current` 指向 `/opt/autolabor/dual_host/releases/20260906_190656/install`；
- 实际运行 PID 201467 来自该 release；
- 运行参数 `/fast_lio_map_localizer/local_max_correspondence=0.5`；
- 最终定位为 `LOCALIZED`，Qt 地图显示状态为 `READY`。
