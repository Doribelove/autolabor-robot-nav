# D04 r05：联合分割与静态栅格闭包（仅离线）

最终输入在 `scenes/development_d04_geometry_r05/`：六族各 4 个、共 24 个 development 父几何／实际 actor-cell 组合。17 个保留 r04 的原几何，7 个为按事先声明的有限坐标网格生成的替代几何（4 个静态避障、3 个变宽走廊）。相对 A01/A02 与其 N04/N05 派生父组，24 个父几何仍互异；r04 标签和文件没有回写。没有分配任何新几何或执行种子，没有创建 episode、release、ROS/Gazebo 或训练。

## 分组约定与适用范围

`design.json` 在筛选前落盘，SHA256 为 `f680b01930e12c2291a56c25b17c1859e1d46e344369ed8319c7b0b4e09b67ac`。精确散列输入为 UTF-8（无 BOM、无结尾换行）：

```text
astra_gazebo_20260926|canonical_parent_geometry_id|actor_parameter_cell_id|allocation_v1
```

取 SHA256 前 16 个十六进制字符的无符号整数模 100，分箱为 train `[0,60)`、development `[60,80)`、validation `[80,90)`、final_test `[90,100)`。静态场景只用固定 `actor_none_v1`。多 actor 单元按实际 `benchmark_actor_1, ...` 的顺序对真实原子 cell ID 元组散列；不能用未使用参数改变分组。

既有 parent 和原子 actor-cell 归属优先，联合散列还必须一致。旧 r04 四个静态父组在固定 sentinel 下分别落到 35、16、16、38，全部拒绝而保留原 development 历史标签；生成新几何寻找合法新组合。所有 24 个最终组合的桶位在 `[60,80)`，20 个实际 actor 原子 cell 均为本批新 development cell。`initial_ownership.json` 和 `final_ownership.json` 保留归属增量，后继分配必须加载它们。全局 registry 未改；此审查范围为已绑定 A01/A02、N04/N05 输入与 r04 的 48 个已知父组，不是全历史数据库或未来 train/validation/final 的完备审计。没有读取这些保留集的新 outcomes。

## 原样保留的失败与检查

- 150 个联合散列不属于 development 的组合全部保存完整 payload、digest、bucket 和实际 cells。
- 12 个几何候选被拒绝：6 个原始完整车体／有限参考路径失败，6 个与已登记／已选几何过近。
- 唯一一次初始作者测试错误来自 Python 3.8 不提供 `ast.unparse`，保留测试快照和失败日志；只修测试兼容性。最终 15/15 测试通过。
- 作者回放重新编码 24 个联合散列、核对 20 个实际参数映射和所有旧归属，重做起点／gate／终点实际 9 碰撞组件、actor 扫掠及遮挡检查；221 个当前资源和 r04 manifest 的 151 个资源哈希一致。此为作者回放，独立复核待 root 安排。

原始静态 witness 使用实际转换后的九组件外包络、所有前轮角／轮转保守包络及明确的小倾斜界，保留 `.33 m` 裕量、`.4964 rad` 转角界与 `.75 rad/s` 转角速率界。901 个路径采样配解析 Lipschitz 区间扣减；存在性只针对理想平面 rear-bicycle 静态几何。

## 四个静态导出的栅格与共同 guide

使用冻结的 `static_map_export`（`.05 m` 像素、`[-20,-15,20,15]` 边界），无 carving；共同 `common_route` guide 半径 `1.0 m`。检查包括 occupied/unknown 像素的精确并集覆盖原障碍、导出 guide 每段恒定朝向的完整包络平移扫掠、每个转角的全朝向外包圆，以及重新选择的平滑 witness 的区间余量。

| 静态 slot | 原 native witness 经栅格后的余量 m | 栅格合格 witness 余量 m | 导出 guide 全包络最小余量 m |
|---|---:|---:|---:|
| 000 | 0.1215731331 | 0.1215731331 | 0.0812323546 |
| 001 | **−0.0117327195** | 0.1229223736 | 0.0743573926 |
| 002 | 0.0430718496 | 0.0430718496 | 0.0743573926 |
| 003 | 0.0992670632 | 0.0992670632 | 0.0903181273 |

slot 001 的负例没有删除：原 `reference_witness.csv` 仍在，不能把它当作栅格合格路径。替代路径来自预先声明的有限 profile，并保存在 `raster_and_guide/raster_witness.csv`。以后若派生 N06，须绑定实际 `raster_and_guide/common_bundle/` 和合格的 raster witness，不可只复用原 native witness。四个 guide 的独立几何检查均通过，但 A* 折线转角不是 Ackermann 可行转弯或真实跟踪证据。其余 20 个父组尚未取得这一栅格导出资格。

## 未取得的资格

本批仅 4 个父几何／族，远少于冻结完整 development 24 个／族及后继独立 validation/final 要求。actor nominal arrival、两个姿态的离散 lidar 遮挡、静态平滑路径只提供输入存在性；不证明实际动态冲突可解、传感器持续揭露、22 秒完成、物理跟踪或所有基线可达。yaw/translation 派生、runtime renderer、真实 actor plugin 参数对应和执行身份仍需单独审核。

最终选择与生成早于收到 N05 终态汇总；本批未用其方法结果重筛选。`execution_authorized=false`、`runtime_ready=false`、`independent_review=false`。root 将独立检查 split/raster 后另行决定是否建立新 release；此目录不授权 N06。
