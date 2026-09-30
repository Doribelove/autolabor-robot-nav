P04 已按冻结规则以基础设施失败终止：1单元消费、5单元退休，无恢复、重跑或替代。原121门完成、119/121通过；失败为实际中心转角变化率与command_loss_no_relay_publication_after_drain。PHY02原三对代表规则不可执行，明确not_complete，P03 PHY01也保持未完成。

失败机制不是37.108的浮点上界：唯一被原loss gate判入区间的headerless relay回调为t=37.106、v=w=0。对应源terminal_zero的Header为37.108s，bag command_event也为37.108s；bag导航和relay零命令均在37.110s接收。驱动source status于37.100s仍timeout=true，37.120s才false。源命令创建已因果地位于正式结束后，probe的缓存/clock尚落后2ms，因此给下游回调标上结束前的时间。将37.106换算整数ns仍落在区间内，不能用整数化冒充此缺陷的修复。bag接收时刻本身也不是精确发布时刻，P04没有可逐消息验证的driver ingress source记录。

loss开始后的最后运动命令实际位于33.058s（loss起点33.108s）；source和bag在loss内部没有命令。原失败不据此重分类或删除，而以独立诊断解释其测量限制。实际20ms中心转角峰值0.879587593rad/s依然超过冻结0.85门，物理负面也保留。

暂停水位修复本次独立15/15通过：GetWorldProperties两次37.144s稳定，GetPhysicsProperties.pause=true，2ms步长一致；bag末/clock为37.144s，全部10条源流从首包到各自最终slot完整连续、全序列raw/bag及payload/header相同。world50Hz末包37.142s、driver50Hz末包37.140s、500Hz末包37.144s。四项源join真，暂停后drain耗时0.344101s，小于原5s。该测量修复有效不等于整回合资格通过。

冻结assessor121门逐字段重放一致。正式协议子集23/24通过，其保留失败正是raw receipt loss计数；源640条命令间隔全部50ms，命令事件整数边界有效。450/450 release当前路径SHA与归档SHA在root随后集成P05前全部复核通过，完整包含历史420 path+SHA；row bindings、raw/report/bag/receipt、registry和cleanup均核验，预算绑定14/14。该“当前文件匹配”是集成前快照，后续新P05编辑共享构建文件不应回溯改写本快照。

全部phase/tail membership额外审查：正式窗口float/ns归属0差异，细分phase发现15组差异位于7.108/6.358s，原因accelerate上界浮点为7.1080000000000005。涉及dense/contacts/commands；当前物理phase门采用的50Hz world/odom集合没有变化，而命令计数原本已用integer。它不解释本次loss失败；新阶段可统一整数归属而不改变原数值门。

剩余接收时钟语义问题已列入relay_clock_diagnosis.json：旧command_relay_observed只是120ms内重复值的存在性匹配，630/640源事件有多个候选，631观测可被重复匹配，并非一对一传输证明。Bool接收间隔仅描述，anytrue故障保留；schedule_start marker以data整数验证而非receipt_t；warmup是接收者观察顺序，与物理源时间需区分。新P05建议由source-stamped driver callback记录承担实际driver接收门，保留旧raw作为诊断。loss drain grace原为0.12s，命令发布间隔门原为0.075s，不可混淆或放宽。

本目录包含独立源/实际运动CSV、完整bag对照、服务水位、phase归属、预算绑定、精确命令及SHA。未修改run原证据，没有启动ROS/Gazebo。PHY02预注册及0.75s尾窗勘误均在任何P04 outcome读取之前保存，未改变代表pair规则。
