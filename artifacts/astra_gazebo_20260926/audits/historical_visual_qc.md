# 历史离线诊断图 QC

状态：**通过，采用 r03 的 H01–H03**。只使用冻结历史 R2 数据和独立复算 CSV，无新增 Gazebo 运行、截图或性能结论；没有修改主 figure_plan/figure_manifest。

三图均已用 `view_image` 实际逐张检查 PDF 的 170 mm 版心渲染。字体嵌入，最小字 7.2 pt，曲线 1.2 pt，底注/图例/轴标签清楚，所有文字边界检查通过。原生 PNG 均 600 dpi；PDF/SVG、中文独立图注、LaTeX 插入片段、原始来源 SHA 和命令全部保留于 `visuals/plots/historical_diagnosis/r03/`。图件有源 CSV，不用手写数字。

| 图号 | 主题 | 关联主计划 | 物理尺寸 |
|---|---|---|---|
| H01 | 两次净空失败：旧保守圆与独立二维几何、速度、非停 mask | F04/F30/F39 | 170×185 mm |
| H02 | 扫描中心/安全方向估计与实际 actor 位移方向 | F29/F39 | 170×140 mm |
| H03 | 全部 18 超时和另一个总体的 21 TF 完整性失败 | F32/F33/F39 | 170×164 mm |

r01 的生成异常与底注重叠/曲线裁切都保留；r02 排版修复后补充轨迹/汇总净空差异说明并修正离轴自动刻度的机器检查；r03 通过。修复史见 `revision_history.json`。r01 源快照是在精确编辑记录上重建的，已明确标记；r02/r03 为原样快照。

证据细节：全部运行期日志重新提取的 TF 缺口是 **2–20 ms**，24 条日志与 24 个拒绝计数对应；早先抽样的 4–20 ms 不是完整范围。seed198146 的 control_trace 最小值为 −0.269610573 m，result 累计最小值为 −0.269571175 m，两者独立记录，不重写历史结果。1.36 mm 二维投影相交不宣称已确认物理接触。

可复现命令（原目录防覆盖；再次离线生成须选新 revision）：

```bash
python3 artifacts/astra_gazebo_20260926/visuals/plots/historical_diagnosis/generate_figures.py --workspace /home/robot/robot_ws_base_rl --revision r03
python3 artifacts/astra_gazebo_20260926/visuals/plots/historical_diagnosis/finalize_qc.py
```

SVG 保留可编辑文本，需要 Noto Sans CJK JP；PDF 字体已嵌入且经 `pdffonts` 验证。实际检查结果、完整 SHA 与机器检查明细见同名 JSON。
