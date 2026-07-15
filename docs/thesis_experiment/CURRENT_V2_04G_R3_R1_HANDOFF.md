# CURRENT V2-04G-R3-R1 HANDOFF

日期：2026-07-14

## 结论

V2-04G-R3-R1 是从失败的 R3 派生出的独立 calibration-only、readiness-only 单因素修复阶段。
它将 rule supervisor 对 geometry、tracks、health 的三个独立 latest-message 输入替换为有界
原子缓存 join，并用全新 seeds 5041--5046 完成 6/6 readiness。

180 条测量事务全部为 CLEAN：transaction valid、activated 和 mechanism join valid 均为
180/180；`world_model_sequence_mismatch`、world-model input join fault、backend fault 和
unknown fault 均为 0。R3 seed4996 暴露的非原子输入竞态已在本阶段出口门下消失。

权威机器结论：
`artifacts/v2/calibration/v2_04g_r3_r1/v2_04g_r3_r1_readiness_freeze_report.yaml`。

## 单因素实现

新增 `world_model_input_join.py`：

- geometry、tracks、health 每流最多缓存 32 条；
- 三个 payload 必须共享完全相同的 `world_model_seq`；
- 只选择最新的完整三元组，禁止跨序号合成；
- 到达年龄上限 1.0 s，新序号滞后上限 2，source timestamp spread 上限 0.05 s；
- 更新中的较新序号尚未形成完整三元组时，可继续使用边界内的上一完整三元组；
- 丢流、过期、时间戳异常和无完整序号全部 fail closed；
- 仿真时间回退清空三个缓存；重复序号只替换对应流；
- subscriber queue 从 2 增加到 32。

继续冻结 R1 transaction join、R2 candidate bank、幂等 typed runtime、R2-R1 taxonomy、
Anchor Bank、supervisor 阈值、动力学和 evaluator。

## readiness 结果

| Profile | seed | stable | valid | activated | join | taxonomy | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| balanced | 5041 | 10 | 1.0 | 1.0 | 1.0 | 30 clean | PASS |
| balanced | 5042 | 10 | 1.0 | 1.0 | 1.0 | 30 clean | PASS |
| balanced | 5043 | 10 | 1.0 | 1.0 | 1.0 | 30 clean | PASS |
| aggressive | 5044 | 10 | 1.0 | 1.0 | 1.0 | 30 clean | PASS |
| aggressive | 5045 | 10 | 1.0 | 1.0 | 1.0 | 30 clean | PASS |
| aggressive | 5046 | 10 | 1.0 | 1.0 | 1.0 | 30 clean | PASS |

每个 probe 只执行一次，没有重试或预算扩张。

## 验收与边界

- join、乱序、部分发布、丢流、重复、缓存、时间戳和时钟回退测试通过；
- R3-R1/R3/R2-R1 相关测试与核心 supervisor 测试共 45/45 通过；
- `teb_mode_manager`、`thesis_experiment`、`m2_gazebo`、`nav_world_model` 定向构建通过；
- TTC 未启动，navigation episode 为 0，SAC 未启动，实车未使用；
- 5001--5010 未消费，5021--5035 未复用；
- `runtime_ready=false`、`formal_result=false`。

原始 summary 复用了 R3 helper 的 `ttc_probe_authorized/navigation_authorized` 字段名并写为
true；其含义只允许预注册未来的 full calibration，不覆盖本阶段合同中的 TTC/navigation 禁令。
freeze report 已机器可读地固定此解释，且实际 `ttc_started=false/navigation_started=false`。

## 下一入口

阶段一已完成。下一步只能新建全新的 full calibration-only 预注册，继续冻结本轮 input join、
R1 transaction join、R2 candidates、typed runtime 和 taxonomy，使用新的 readiness/navigation
seeds，依次执行 readiness、TTC 和 60 个导航比较。本阶段本身不授权候选冻结、held-out、
V2-05、SAC 或实车。
