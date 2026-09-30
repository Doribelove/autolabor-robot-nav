# 第八章补充：四方法新鲜种子复现与消融表征

## 实验定位

在 V3-11 获得限定单演员正式阳性结果后，本研究建立了统一 episode 数据模式，并运行独立的四方法新鲜种子研究。四种方法包括主方法 V3 spatial-safe、Actual TebLocalPlannerROS 0.8.4、Fixed-v1.20 shielded 比较策略以及去候选条件化单因素消融。消融仅替换预测头，候选生成、硬验证、解析排序、跟踪和 certified-stop 回退保持不变。

V3-19 的 24 回合资格研究完整执行，但主方法目标成功率为 5/6，没有达到 0.95 资格门，故正式 seeds 未启动并退役。V3-20 改用失效计入设计，但在第 78 个序列因 ROS master 端口落入 Linux ephemeral range 而终止。V3-21 只把端口命名空间修复为 24002–24240，使用 fresh seeds 8201–8230 完成 30 个 scene-seed blocks、120 个一次性回合。

## 总体结果

V3、Actual TEB、固定策略和消融的目标成功率分别为 93.33%、100%、0% 和 43.33%；受限均时分别为 7.785 s、7.770 s、22.000 s 和 15.402 s。所有已经消费的失败都以 22 s 保留，四方法 contact 均为 0。

V3 相对 Actual TEB 的配对相对降幅为 -0.193%，双侧 95% block-bootstrap CI 为 [-19.867%, 12.982%]，Holm p=0.755412。因此本研究没有复现 V3 相对 Actual TEB 的失效计入式时间优势。V3 相对 Fixed-v1.20 和去候选条件化消融分别缩短 64.614% 和 49.454%，对应双侧 95% CI 为 [57.682%, 69.250%] 和 [37.001%, 58.800%]。

## 分层和机制

Actual TEB 在六层均达到 100% 成功；V3 在 bounded-110/hesitant 与 open-90/hesitant 两层为 80%，其余四层为 100%。去候选条件化消融在 hesitant actor 下明显退化，open-90/hesitant 成功率为 0%。该差异支持候选条件预测在冻结总体内具有探索性机制价值，但不能替代与 Actual TEB 的主要比较。

V3 的最小经验净空为 0.4764 m，Actual TEB 为 0.0395 m；V3 P99 规划延迟为 10.756 ms。这些均为有限仿真的描述量，不构成形式化安全或实车证据。

## 协议偏差和敏感性

唯一协议偏差为 sequence 107 的 gate x 误差 0.032286 m，比 0.03 m 容差多 2.286 mm。该回合成功且其余完整性门通过，因此按 intention-to-treat 保留。排除 block 27 后，V3 相对 TEB 点降幅为 -0.643%，结论不变。由于存在偏差，V3-21 的 `formal_result=false`，但完整 characterization 可报告。

## 与 V3-11 的关系

V3-11 与 V3-21 是不同合同、不同 seeds 的独立研究。前者双方 100% 成功并得到 12.9288% 正式阳性；后者保留两个 V3 certified-stop timeout 后没有证明时间优势。两者必须并列报告，不得合并、挑 seed 或删除失败。最稳健的论文结论是：候选条件机制证据明确，历史限定阳性存在，但新的严格失效计入复现未证明 V3 普遍优于 Actual TEB。

图、三线表和中英文材料位于：

```text
artifacts/v3/thesis/v3_22_p1_paper_materials/
```

