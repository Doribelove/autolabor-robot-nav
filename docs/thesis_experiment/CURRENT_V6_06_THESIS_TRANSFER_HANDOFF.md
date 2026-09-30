# V6-06-P1-R1 论文更新与跨电脑移交终态

日期：2026-08-08

## 目的与权限

本阶段响应“把 V3 学校模板初稿更新到当前 V5/V6 结果，并移交给另一台电脑 Codex”的要求，
只执行纯离线派生、排版、复制和哈希审查。没有训练、仿真、formal seed 访问、ROS/Gazebo、硬件、
实车或在线 TEB 写入；`/home/robot/robot_ws` 未修改。

V6-06-P1 首次生成 41 页整合稿并通过科学内容审查。视觉检查发现封面标题孤行、页眉重叠和一个
长区间贴边，因此 V6-06-P1-R1 只修复这三处排版；任何实验数据、图表、声明和上游证据均未改变。
P1 初版保留，R1 是交付入口。

## 最终入口

目录：

```text
artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/
```

便携压缩包：

```text
artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package.tar.gz
```

压缩包 SHA：`16ea155b47189dd449a2c7bb7920a1a63fe9b8274cc03966466267516eaadfe2`。

首先阅读：

1. `README_FIRST.md`
2. `FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.pdf`
3. `CURRENT_THESIS_INTEGRATION_GUIDE.md`
4. `CODEX_PROMPT_FOR_SCHOOL_TEMPLATE.md`
5. `combined_claims_evidence_matrix.yaml`
6. `DATA_DICTIONARY.md`
7. `CODEX_HANDOFF_MACHINE_CONTEXT.md`

## 包内容

- 41 页、A4、可搜索、字体嵌入的 V3+V6 真正整合技术稿，而不是旧 PDF 与附录拼接；
- 可独立编译的整合 LaTeX 源和 V6 独立章节；
- 明确为空的实车实验章节、合同表和结果表；
- V3-11/V3-21、V5-03-F1、V6-02-F1 和 V6 Gazebo 负结果的机器可读数据；
- V3 的 5 张 PDF/SVG/PNG 图和 5 个 booktabs 三线表；
- V6 的 6 张 PDF/SVG 图及方法、效应、场景、逐 episode、Gazebo CSV；
- 统一结果 CSV、combined claims/evidence YAML、数据字典、源文件索引；
- 可以原样发送给另一台电脑 Codex 的学校模板更新提示词；
- 上游证据 manifest、独立 review 和 77 文件终态 content manifest。

## 当前论文主结论

- V3-11：历史限定 Gazebo actual-TEB 阳性，12.9288%，one-sided 97.5% lower 12.7253%；
- V3-21：新鲜失效计入式 V3/TEB 优势未证明，-0.1931%，95% CI
  [-19.8668%,12.9820%]；
- V5：6.0936%，95% CI [1.7867%,11.0069%]，未达到冻结 8% 实效门；
- V6：80 blocks/320 episodes，目标 60/80 对 49/80，均时 13.115625/14.318750 s，
  相对解析先验 8.402444%，双侧 97.5% CI [3.484003%,13.713459%]；
- V6 对 V5：3.161052%，95% CI [0.493580%,6.576508%]；
- V6/no-history 完全相同，history benefit 未证明；
- 三次 V6 Gazebo 预检失败，不存在 Gazebo/Actual TEB/部署阳性；
- 实车状态为 `pending_no_evidence`，所有结果单元留空。

## 冻结哈希

- integrated TeX SHA：`b16f5cdbd503357382c5a0b13af743ed373044f6ddf068ecbd8c857573f74532`
- integrated PDF SHA：`a2fcacc82ee8f7fc290415661b34bd5324bbc157158d54a0780705a7737eadf1`
- combined claims SHA：`6d757e5806361c3e8e684a06474a1bd38a0caf2d48b7d4711504973e2eb783a6`
- Codex prompt SHA：`245b5b2daadc8ab08d406b01e3501b6d77e36cf9974ee0f8e34f23104413e0f3`
- independent review SHA：`00e4705f783632723018c6920f8e1b68dc6096a08cc28018d72e286e783d1f1b`
- terminal content manifest SHA：`3353fdb3af3edff3b04cc339a66830e8f4382c35bedd509d6cb4e1e964149553`

独立审查状态：`v6_06_p1_r1_thesis_transfer_independent_review_pass`，全部检查通过；PDF 41 页
A4、28/28 字体行嵌入、无 LaTeX error/undefined reference，实车数值为空。

## 另一台电脑的操作边界

另一台电脑只需把整个项目和已有学校模板初稿放在同一可访问环境，替换提示词中的
`<PROJECT_ROOT>` 与 `<SCHOOL_THESIS_DRAFT>` 后执行。它不需要重跑任何训练或 Gazebo。若学校
初稿和包内数字冲突，以 `combined_claims_evidence_matrix.yaml`、CSV 和 assessment 为准。

不得把 V3 的 Gazebo 结果写成 V6 结果，不得删除 V5/V3-21/Gazebo 阴性，不得补写实车数字。
未来获得真实数据后，实车章节必须作为新的、有原始日志和独立评价真值的研究更新。
