# 视觉质检

尚无新截图或数据图。机器/逐图视觉/论文实际尺寸质检均pending；未声明通过。

## A01首次真实场景诊断
17张2560×1440原始帧已登记，0张机器QC通过。所有帧轮底+17.8至+20.0mm，17帧odom时间晚于画面。root实际打开frame00008：无多机器人重影，但纵向路线/墙端裁切，不适合作论文图；其余16张不标visual pass。原图仅保留诊断。A02改z=0、按帧时间之前取状态、调整真实相机而不修图。

2026-09-26 H01–H03 historical diagnostic figures accepted at r03: 25/25 machine checks and three actual per-figure views at 170 mm paper width. All earlier failed revisions retained. These are three data-figure groups, zero new Gazebo screenshots. Review: audits/historical_visual_qc.json.

A02/Q01：69张原生截图，65 machine-pass、4早期时钟未稳定fail；5页总览实际查看，按中点规则选出的4张逐张查看，170mm四面板PDF实际渲染通过。其余65张未逐张批准。单机器人、完整墙体及标注通过；俯视截图不替代轮地高度证据。生成脚本、PDF/SVG/600dpi PNG、图注和LaTeX位于 visuals/panels/bootstrap_a02/r01/。

AQC02 r02: root actually viewed all six panels and independent170mm PDF150dpi rendering; Chinese labels/units/limits/scopes readable. One nonduplicative actuator diagnosis group added. AQC01 r05 supporting figure viewed, but panel(c) legend crosses threshold line; not a separate delivered group. This is a data figure, not a screenshot.

TIM01 r02：root实际查看170×180mm PDF渲染，六面板和中文标注可读、图例不遮挡数据；独立14项QC及全部源/输出SHA复核通过。记录P01/P02测量协议差异和保留失败，不声称统计因果或性能提升。新增1组数据图，累计6个非重复交付图组；PHY01的三pair预注册选样未完成，未替代。

F04 r07：独立17项数值/几何/产物检查通过；root实际查看170×155mm纸面PDF渲染及600dpi PNG，各panel清晰，caption明确图示与模型范围。固定Q05static031首正式帧及完整窗口，计一组新增图，不将迭代版本计数。Root QC：audits/F04_root_visual_qc_r07.json。

Q05/N01/N02/N03原生素材更新：所有拟记录帧均已root逐张实际查看，QC绑定于audits/*_root_visual_qc.json；原N01 DWA为捕获失败回合，图片仅诊断用途。无裁剪或像素修改。原生素材尚未排入论文尺寸面板，不增加非重复图组，当前仍8组。

NAV01 r02：独立14/14核对通过，root已实际查看全分辨率PNG及170×165mm纸面预览，四面板无图例遮挡。PDF/SVG与400dpi PNG、中文图注及LaTeX include齐备；未宣称LaTeX已编译。预测净空、命令请求与实际物理量严格区分，超时保留；仅一次开发失败的描述性案例。新增1组，累计9组独立交付图组，原生支持帧不另计。

F03 r02：M08/N05源码流程示意，root实看600dpi PNG和170mm PDF；r01箭头文字交叉保留失败。该图无实时/物理安全资格。QC：paper_update/figures/F03_candidate_transaction_r02/actual_view_qc.json。

F06–F11 r04：六类既有N05开发任务各取一个有效原生8.50s帧，固定宽度裁切保留全高；root逐张打开六张成图，另实看六份170mm PDF渲染。每图分离标出源清单的二维几何和原生截图，机器人1台、参与者0/1/1/2/1/0，原清单水平初始交叠为零；不据此证明竖直轮地接触或安全。r01生成器异常、r02/r03裁切下缘参与者均保留。六份QC在paper_update/figures/F06_n05_native_scene_r04至F11_n05_native_scene_r04；累计26组独立交付候选，图册r07逐页总览通过。

NAV09 r02：12个N05旧开发真实model-state bag中提取16421坐标，六族每族首个配对；root实看170mm PDF与图册A4页，六面板及中文标注清晰。r01 PNG可读但PDF字体损坏，保留拒收。r02与r01源CSV逐字节一致；10超时、2完成保留。仅描述性开发轨迹，不推断安全/总体性能。QC见 paper_update/figures/NAV09_n05_executed_pairs_r02/actual_view_qc.json；累计27组，r08图册新增页实看通过。

NAV10 r02：N05全部24回合旧开发真实Gazebo model-state bag提取23654个目标距离样本，根代理实看170mm PDF及A4图册第28页；六族每族四线，20超时和4完成完整显示。r01底部图注与横轴冲突被保留拒收；r02与r01源数据逐字节一致，源哈希96项通过。仅描述时序和停滞，不推断正式性能/安全。QC在 paper_update/figures/NAV10_n05_goal_progress_r02/actual_view_qc.json；累计28图组，r09图册新页实看通过。

NAV11 r02：N05全部24回合旧开发真实`/odom`，23678个50Hz源时间速度样本，root实看170mm PDF和A4图册第29页。双面板展现低速样本比例与实际绝对纵向速度第90百分位，全部20超时/4完成保留。r01图间家族标签碰撞拒收；r02与r01两份源CSV逐字节一致。0.05m/s仅探索性阈值，不作因果/安全/正式性能推断。QC见 paper_update/figures/NAV11_n05_actual_speed_r02/actual_view_qc.json；累计29图组，r10图册新页实看通过。

NAV12 r02：四条N05 b04/b05旧开发真实工程保守净距失败，共4400个20ms区间；root实看170mm PDF及A4图册第30页，完整22秒与局部5.5–9.5秒双面板，原始负下界和工程/物理界限均清晰。r01横轴与脚注冲突被保留拒收，r02源CSV逐字节一致。QC见 paper_update/figures/NAV12_n05_clearance_timeline_r02/actual_view_qc.json；累计30个不同质检图组。r11图册第30页实看及前29页源/QC哈希复核通过，图组交付不代表全研究目标完成。
