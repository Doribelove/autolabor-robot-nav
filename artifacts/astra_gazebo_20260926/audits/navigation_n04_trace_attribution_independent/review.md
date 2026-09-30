N04 r01 attribution独立复核已完成：计数通过，终点距离不通过。原脚本、JSON、CSV和全部历史结果不改，精确副本在r01_original。

5个完整analytic bag/assessment/raw/receipt SHA均重新核验，逐消息重算2190条trace：1509条action0/零请求、1017条backup_revalidation_failed、414条backup_stop_requested、195条no_prediction_model_backup_pass、73条transaction drop、0条inference drop，与r01逐行完全一致。

明确错误在原脚本47–48行：把odom坐标直接减map目标。所有bag /odom.header.frame_id均为odom；共同SHA绑定common_route.json的map_from_odom_xy_yaw为[-5.4,0,0]，并与bag实际map→odom TF一致；目标map=(5,0)。因此须先x_map=x_odom−5.4，再求距目标。

| seed | r01错误跨frame距离(m) | exact终点map距离(m) | bag尾map距离(m) | bag尾晚于终点(ms) |
|---|---:|---:|---:|---:|
| 263001001 | 1.731432 | 5.878025370 | 5.877904891 | 220 |
| 263001002 | 1.758514 | 6.057741378 | 6.057590946 | 240 |
| 263001003 | 1.272951 | 5.679422122 | 5.678871225 | 120 |
| 263001004 | 1.229842 | 5.668530984 | 5.667420750 | 260 |
| 263001005 | 2.357000 | 7.756985787 | 7.757000250 | 220 |

所有正式gate为7.460s、终点为29.460s；五个bag都存在唯一完全相同source_ns的真实odom，终点数值无需插值。原脚本使用bag最后odom，晚120–260ms；这里另列尾距，不能把它作为终态即时距离。

2190是全bag trace，适合复现r01消息分类口径。按trace stamp_ns局部ROS仿真clock取半开[7.460,29.460)，则2182条、1501零请求、1014backup失败、414backup stop、73drop。边界外8条包含5条timeout终止输出和3条running尾消息，全部零请求，完整保留；不修改原报告总数。stamp_ns来自节点prepublish rospy.Time.now()，不是driver收到命令的真实world source时间，也不保证均匀50ms。

样本仅包括b00–b04的5个完整timeout：静态2、crossing2、head-on1，不能概括全六族。b05 seed263001006的receipt明确为assessment KeyboardInterrupt()、task_outcome=null、无assessment，排除但保留其消费；b06–b11共6个analytic身份退休未运行。整四方法matrix则24 consumed/24 retired，不能把analytic子集的6退休误写成stage总数。

本次仅描述性复算；失败原因类别与停车输出可重叠，零请求不等于物理静止；没有证明因果或N05收益。原timeout/安全评估不重判，没有dense500Hz重算、ROS/仿真、训练或新身份。详见recomputed.json和2190行CSV；正确终态位置/距离均以完整精度保存在corrected_endpoint_diagnostic.csv。
