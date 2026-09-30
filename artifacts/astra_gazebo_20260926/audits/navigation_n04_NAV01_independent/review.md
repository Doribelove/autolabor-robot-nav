NAV01 独立数据与语义复核通过（14/14）。

439 条控制日志、88 条决策日志逐行与 CSV 一致；41 次空 mask、206 次 backup_revalidation_failed、111 次 backup_stop_requested、15 次 deadline drop 均由原 JSON 复算。任务 timeout 和 22 s 上限保留，不推断成功或反事实因果。

候选横轴是本地 ROS 仿真时钟在决策开始的读数，控制横轴是发布前读数；不是 Gazebo 驱动接收时间。事务纵轴为 perf_counter 壁钟毫秒。预测净空已经扣除 0.33 m 裕量及候选区间修正，零线是模型可行边界，不是实测物理净空。

13 个 manifest 资源当前 SHA 全匹配，未重读 bag 或重跑 dense。实际查看既有 170×165 mm 预览，图例没有盖住散点。PDF 版心尺寸精确一致；现有 PNG 为 2677×2598、400 dpi，未称为 600 dpi。英文图注属于现有版本。

这是单独的 NAV01 解释性负面诊断，不改变 F06–F11 冻结选图规则。完整计数、源行和边界见 review.json。
