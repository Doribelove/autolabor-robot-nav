F04 固定选取 `passive_sweep_q05_static`、seed260926031 的首个正式源时刻和完整正式窗口。

`generate_F04.py` 只读闭合 receipt、assessment、原 sweep 输出、实际 bag 和转换几何；核对身份及 SHA。脚本不会运行 ROS、重算资格、访问其他新试验或用 Q04 替换 Q05。`test_F04.py` 只测试内存中的字段和失败边界，不生成图或实验数据。

收到 root 的正式 Q05 结果路径后，用系统 Python（含 ROS bag 读取依赖）执行：

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 generate_F04.py --revision r01
```

独立复核状态只保存在来源 manifest 中，不放入图版副标题，以免发布流程状态过时。只有给定报告明确绑定该 run/assessment 且提供独立 pass/fail 时才在 manifest 确认其状态。每版独占新目录，不覆盖前版。全局 manifest/atlas 不修改，只输出索引建议。

输出为170 mm宽、最小7 pt中文的 PDF、字体轮廓内嵌SVG、600 dpi PNG，另有首时刻9组件几何、全窗口端点/步进数据、来源 SHA、图注、LaTeX 插入和150 dpi纸面检查图。最终需实际查看渲染图并独立复核；代码/schema通过不等于图像已通过。

物理投影带符号间距、减0.33 m保守间距和声明Gazebo正常Euler模型下界明确区分。源数据缺失保持断线，不补帧；原结果失败原样标注。该图不主张形式证明或真实连续物理安全。
