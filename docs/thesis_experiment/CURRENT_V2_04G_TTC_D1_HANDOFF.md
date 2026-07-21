# V2-04G-TTC-D1 Offline Diagnosis Handoff

更新时间：2026-07-19

## 1. 阶段结论

`V2-04G-TTC-D1` 已完成，是独立于 R5 execution 的
`offline-only / diagnosis-only` 阶段。

本阶段只读复用了已经消费过的 R5 readiness seed5111 证据，没有产生新实验
evidence unit，没有启动 ROS、Gazebo、`move_base`、component、navigation 或训练，
没有修改任何 threshold、scene、evaluator、R5 合同、R5 脚本或 R5 artifact。

机器结论：

- R5 仍保持 `terminally stopped`，不得 retry 或 resume；
- seed5111 的 `NO_CONFLICT_IN_HORIZON` 可由冻结 trace 离线复现；
- runtime `CROSSING` overlay 与 evaluator circle-contact TTC 不是同一语义；
- 5.0/4.5/4.0 秒三个 R5 horizon 在该 trace 的 21 个可达 `CROSSING`
  样本上完全同判，目标因素没有集成辨识度；
- actor 比机器人早约 `4.0052595847 s` 到达交叉点；
- trace、evaluation 和 truth-proxy 均没有 finite TTC；
- 5 秒圆包络最小正余量为 `0.6848243632 m`；
- 冻结 Gazebo truth box clearance 为 `1.6251341292 m`；
- 六项要求的执行完整性风险全部写入 machine-readable audit；
- 1.5/1.0 秒在单条冻结 trace 上具备辨识度，但只能作为未来预注册候选，
  不构成适用性、安全性、性能或 winner 结论，也没有创建 R6 authorization。

`formal_result=false`，`runtime_ready=false`。

## 2. 不可跨越的冻结边界

R5 的最终状态没有被 D1 改写：

- 唯一已尝试 identity：
  `r5-readiness-r5_ttc_h450-s5111`，`attempt=1`；
- expected TTC status：`OBSERVED_CONFLICT`；
- observed TTC status：`NO_CONFLICT_IN_HORIZON`；
- R5 消费：`1/69` evidence units；
- 剩余 `68` units 已随 terminal stop 放弃；
- component：`0/3`，未启动；
- navigation：`0/60`，未启动；
- held-out seeds `5001--5010`：未使用；
- passing candidate：无；
- winner：无。

继续禁止：

- retry/resume R5 seed5111；
- 消费 R5 剩余 68 units；
- 执行 component 或 navigation；
- freeze winner；
- 使用 held-out 5001--5010；
- 创建 R6 execution authorization；
- 启动 V2-05、SAC 或任何训练；
- 连接实车、让实车运动或写实车 TEB 参数。

## 3. 新增 D1 文件

| 文件 | SHA256 | 用途 |
| --- | --- | --- |
| `config/thesis_experiments/v2/v2_04g_ttc_d1_offline_diagnosis_contract.yaml` | `7b6639362e28e1fb8c553e1d9a4333867142036e6d94f4a8816ac11bc5993913` | offline-only 合同、输入 hash、权限和禁令 |
| `src/tools/thesis_experiment/scripts/diagnose_v2_04g_ttc_d1.py` | `879ea2e1454ab7356af6aee25265a0399e25108edb9906046f7c844d6cc82fe7` | 可重复、fail-closed、原子写报告的诊断脚本 |
| `src/tools/thesis_experiment/tests/test_v2_04g_ttc_d1.py` | `790e5edc8957fb90baeeebfb0f6f18bb6d5c072e740637622ee6279b998dfb9b` | 9 项定向测试 |
| `artifacts/v2/diagnosis/v2_04g_ttc_d1/v2_04g_ttc_d1_report.yaml` | `e8983d6bb9fc805c807d289cb65949b5d08b4eab8984a72760febde91d6bb063` | machine-readable diagnosis report |
| `docs/thesis_experiment/CURRENT_V2_04G_TTC_D1_HANDOFF.md` | 本文件，提交前重新计算 | 本阶段交接 |

`src/tools/thesis_experiment/CMakeLists.txt` 仅新增 D1 script 安装项和 D1
nosetest 项；未修改 R5 source。

## 4. 输入完整性与 R5 原样保持

D1 脚本在导入纯 Python 几何模块前先执行以下检查：

1. 严格 YAML loader 拒绝 duplicate key；
2. 校验合同声明的冻结 R5 输入路径和 SHA256；
3. 复核 R5 preregistration 的 39/39 resource closure；
4. 对 readiness compiled index 的 14/14 子文件逐项复核；
5. 对 navigation compiled index 的 30/30 子文件逐项复核；
6. 验证 seed5111 compiled instance 的 canonical instance hash；
7. 在诊断前后对整个 R5 artifact tree 做逐文件快照并要求完全相等。

结果：

- D1 frozen input hashes：全部一致；
- R5 preregistered closure：`39/39`；
- compiled child hashes：`44/44`；
- R5 artifact tree：`68` 个文件；
- R5 artifact tree SHA256：
  `ecb1f33093dee469008c2ad2d783b3e8ffd1c0739db7903b5df273717e270984`；
- 诊断前后 tree snapshot：完全一致；
- R5 文件改动数：`0`。

机器报告中保留了 68 个 R5 artifact 的逐文件路径和 SHA256，便于下一会话再次
只读核验。

## 5. Runtime overlay 与 evaluator TTC 的语义差异

### 5.1 Runtime 路径

冻结 world-model classifier 先用 `prediction.horizon_s=2.0` 判断 motion class。
`CROSSING` 只有在中心线 crossing time 落入这 2 秒窗口时才会向 supervisor 传播。

supervisor 随后：

- 对 `CROSSING` 使用中心线交叉时间；
- 对 `HEAD_ON`、`FOLLOWING`、`UNKNOWN` 使用 point closest approach
  减 actor radius；
- 再用 5.0/4.5/4.0 秒 `predicted_ttc_max_s` 过滤。

### 5.2 Evaluator 路径

冻结 evaluator 使用相对运动的第一次圆包络接触：

- horizon：`5.0 s`；
- robot radius：`0.62 m`；
- actor radius：由 `0.55 x 0.55 m` box 的半对角线得到
  `0.3889087297 m`；
- interaction radius：`1.0089087297 m`；
- minimum confidence：`0.45`。

因此 runtime 的 `CROSSING` 只表示“预测会穿过机器人中心线附近”，不等于
“robot/actor 圆包络会在 5 秒内接触”。实际执行记录有 14 个 runtime
`CROSSING` context samples，但 evaluator finite TTC 为 0，这两项可以同时为真。

## 6. 5.0/4.5/4.0 秒集成辨识度

离线 proxy replay 使用 frozen trace 的 193 行机器人状态、compiled actor trajectory、
冻结 world-model classifier 和冻结 supervisor `_overlay`。

motion class 计数：

| Class | 样本数 |
| --- | ---: |
| STATIONARY | 113 |
| CROSSING | 21 |
| HEAD_ON | 14 |
| DEPARTING | 15 |
| UNKNOWN | 30 |

overlay replay：

| Horizon | NONE | CROSSING | OVERTAKE_OR_YIELD |
| --- | ---: | ---: | ---: |
| 5.0 s | 168 | 21 | 4 |
| 4.5 s | 169 | 21 | 3 |
| 4.0 s | 172 | 21 | 0 |

pairwise 差异：

- 5.0 vs 4.5：1 个样本；
- 4.5 vs 4.0：3 个样本；
- 5.0 vs 4.0：4 个样本；
- 以上差异全部来自 `UNKNOWN -> OVERTAKE_OR_YIELD/NONE`；
- 21 个可达 `CROSSING` 样本上，三个 horizon 的差异数均为 0。

结论：R5 选择的 5.0/4.5/4.0 秒因子无法在该 seed 的目标 `CROSSING`
路径上形成集成辨识；上游 2 秒 classification cap 先于更长的 supervisor horizon
生效。

## 7. 到达时间、TTC、圆包络和 clearance

### 7.1 交叉点到达时间

- 交叉点：`x=8.0 m`；
- actor 中心过中心线时间：`10.9171370891 s`；
- 此时机器人：`x=4.4992469538 m`；
- 机器人到 `x=8.0 m`：`14.9223966738 s`；
- `robot_time - actor_time = 4.0052595847 s`。

actor 先通过交叉点，机器人约 4.005 秒后才到达。

### 7.2 TTC 与预测圆包络

- evaluation finite TTC samples：`0`；
- trace finite `predicted_ttc_s` samples：`0`；
- truth-proxy circle-contact finite TTC samples：`0`；
- 5 秒窗口内最小预测中心距：`1.6937330928 m`；
- 最小圆包络余量：
  `1.6937330928 - 1.0089087297 = 0.6848243632 m`。

这支持 `NO_CONFLICT_IN_HORIZON`，不是 tracker invalid：
tracker messages 为 290，health valid fraction 为 1.0。

### 7.3 Truth clearance

- preserved asynchronous Gazebo truth box clearance：
  `1.6251341292 m`；
- trace-synchronous trajectory proxy：
  `1.6251709611 m`；
- 两者绝对差：`0.0000368320 m`；
- signed scan clearance：`1.6614924435 m`；
- contact count：`0`；
- runtime policy received truth：`false`。

执行期异步 Gazebo model-state audit 仍是权威 truth 指标，trace proxy 只是紧密的
离线交叉核验。

## 8. 六项机器可读风险审计

| Risk ID | 确认结果 | 影响边界 |
| --- | --- | --- |
| `D1-RISK-READINESS-DIRECT-COUNTS` | immediate combined gate 未直接比较 evaluation tracker/context message count 与 minimum 20 | 实际计数足够，不改变 R5 stop；未来必须直接绑定并硬检 |
| `D1-RISK-COMPILED-SCENE-TOCTOU` | execution load 时未逐项重验 compiled index child SHA256 | D1 当前复核 44/44；未来应在 launch 前即时重验并绑定 attempt |
| `D1-RISK-SIGINT-IN-PROGRESS` | `except Exception` 不捕获 `KeyboardInterrupt`，可能留下 `in_progress` 且合同禁止 resume | 本次 R5 是正常 terminal recorder 路径；未来需原子记录 `terminal_interrupted` |
| `D1-RISK-ASSESSMENT-RAW-BINDING` | terminal summary 的 `reports=[]`，assessment 未直接绑定 activation/evaluation/trace | stage report 另行绑定了它们；未来 assessor 必须自行绑定失败 attempt 原始证据 |
| `D1-RISK-EXECUTION-HASH-CLOSURE` | R5 runner 动态加载的 R4-R1→R4→R3→R2→R1→legacy 链未完整进入 39 项闭包 | 顶层 runner 在 stage report 有 hash；未来必须机械生成完整执行闭包 |
| `D1-RISK-TEARDOWN-RESTORE` | goal 完成后 startup typed TEB profile 恢复失败 | 只能声称 measurement window 内接口门通过，不能声称 teardown 完整通过 |

六项均为 `CONFIRMED`，且 `changes_r5_terminal_stop=false`。它们是未来 execution
合同的修复要求，不是恢复 R5 的依据。

## 9. 1.5/1.0 秒未来候选审查

D1 仅在内存中创建 counterfactual supervisor copy，没有生成 runtime config：

| Horizon | CROSSING | 与前一比较的总差异 | 其中 CROSSING 差异 |
| --- | ---: | ---: | ---: |
| 5.0 s | 21 | — | — |
| 1.5 s | 16 | 9（vs 5.0） | 5 |
| 1.0 s | 11 | 5（vs 1.5） | 5 |

结论限定为：

- 1.5/1.0 秒在这条冻结 trace 上具备 integrated distinguishability；
- 可以进入未来 R6 设计候选讨论；
- 当前不能判断是否“适合执行”；
- 不能从单 trace 推断安全、性能、泛化或 winner；
- 必须先修复六项执行完整性风险，再做独立 preregistration、fresh-scene
  design review 和用户 execution authorization；
- 本阶段没有创建 R6 文件或 execution authorization。

## 10. 可重复命令

只读复算并原子重建唯一 D1 report：

```bash
cd /home/robot/robot_ws_base_rl
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/tools/thesis_experiment/scripts/diagnose_v2_04g_ttc_d1.py \
  --workspace /home/robot/robot_ws_base_rl
```

D1 定向测试：

```bash
cd /home/robot/robot_ws_base_rl
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  src/tools/thesis_experiment/tests/test_v2_04g_ttc_d1.py
```

全部 V2 测试必须只 source 当前 thesis workspace：

```bash
cd /home/robot/robot_ws_base_rl
source /home/robot/robot_ws_base_rl/devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  src/tools/thesis_experiment/tests/test_v2*.py
```

当前结果：

- D1 directed：`9 passed`；
- all `test_v2*.py`：`151 passed`；
- `git diff --check`：通过；
- ROS/Gazebo/`move_base`/训练进程：最终检查为 0。

## 11. 下一阶段入口

当前没有获准的在线或仿真执行动作。若用户希望继续，下一轮只能先建立独立的
R6 **设计/预注册审查**，不能执行。设计至少要：

1. 选择一个能穿过上游 2 秒 classification cap 的单因素，或显式对齐 runtime
   overlay 与 evaluator circle-contact 语义；
2. 说明 1.5/1.0 秒只是候选，不预先选 winner；
3. 修复并测试本 handoff 第 8 节的六项风险；
4. 使用新的 scene/seed firewall 和新的 evidence budget；
5. 保持 held-out 5001--5010 不可见；
6. 在新的用户明确授权前，所有 execution 字段保持 `false`。

不得把未来阶段写成 R5 retry/resume，也不得直接启动 ROS/Gazebo。
