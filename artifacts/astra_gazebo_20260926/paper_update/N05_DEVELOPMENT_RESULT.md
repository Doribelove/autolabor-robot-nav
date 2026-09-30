# N05 解析预览可选性开发诊断（终态）

N05 使用六个**已用过的** development 父几何、每个几何两个新执行种子，在真实 Gazebo 中完成 12 个对照/处理配对块（24/24 attempt=1）。对照是原解析路径选择，处理只在移动动作的原始未取整全预览净距不小于零时允许其参与解析排序；真正的 backup mask 和 0.33 m 保守余量未变。处理使 504 次候选选择改变，但两方法均只有 2/12 到达目标，10/12 超时。

| 指标 | 原解析对照 | N05 预览可选性 |
|---|---:|---:|
| 目标完成/12 | 2 | 2 |
| trace 消息数 | 4689 | 4698 |
| 执行停止消息 | 3118（66.50%） | 3788（80.63%） |
| backup 再验证失败消息 | 2046 | 1201 |
| backup 停止请求消息 | 896 | 2425 |
| 终态地图系目标距离均值 | 5.6014 m | 6.1451 m |

配对终态距离差（处理减对照）均值为 +0.5437 m、中位数为 +0.0778 m；这里每个终点都取自该方法自己的精确终止源时间，不能当作同一时刻状态或总体效应。事件率以 trace 消息数为分母，类别可以重叠，不是停止时长。b04/b05 两个配对块的四次回合都未通过保守工程净距门（最低约 −0.1644 至 −0.1648 m），尽管接触日志中没有非法机器人接触。因而该处理没有通过完整工程安全门，不晋级训练或正式评估，也没有性能提升声明。

可直接使用的开发诊断图是 [NAV04 PDF](figures/NAV04_n05_paired_diagnostic_r02/NAV04_n05_paired_diagnostic.pdf)；源数据为 [完整回放](../audits/navigation_n05_terminal_replay/report.json)、[结果解释](../audits/navigation_n05_result_interpretation/report.json) 与 [独立审查](../audits/navigation_n05_terminal_independent/review.json)。48 张原生 2560×1440 Gazebo 截图的逐帧哈希/尺寸和实际视检记录在 [图像 QC](raw_qc/navigation_n05_contact_sheets_r03/actual_view_qc.json)。这些图仅是开发诊断，不支持六族总体、训练收益、形式安全或实车结论。

补充图组将不同问题分开呈现：[NAV09](figures/NAV09_n05_executed_pairs_r02/NAV09.pdf)取六族各首个配对的12条真实模型轨迹，包含10个超时回合；[NAV10](figures/NAV10_n05_goal_progress_r02/NAV10.pdf)覆盖全部24回合的源时间目标距离历程，保留20次超时与4次完成；[NAV11](figures/NAV11_n05_actual_speed_r02/NAV11.pdf)使用全部24回合的23,678个真实里程计样本，报告近零速样本比例和绝对纵向速度第90百分位。NAV11 的0.05 m/s阈值只作探索性描述，不是预注册决策门，也不证明超时的因果机制。[NAV12](figures/NAV12_n05_clearance_timeline_r02/NAV12.pdf)显示b04/b05四条工程净距失败回合的4,400个20 ms下界区间；负下界仅表示工程门失败，不能推作实际接触。六族原生Gazebo场景图为F06–F11，每图均单独绑定原始帧、输入几何与视检记录。全部图的索引、限制和可重生成脚本见[30图组阶段图册](working_atlas_r12/FIGURE_INDEX.md)，图册不是正式方法优越性结论。

发布 SHA：`e9879ee6ca8ca050b9ee036db87c4728f5f1aa38895b69c2ca2d429bbcf6c628`。独立结果审查 SHA：`552cd67545e1618164bce9a3406a4d31c6281e88147c523ec0c88f4a08b7bfad`。不重跑或复用 N05 种子与预算。
