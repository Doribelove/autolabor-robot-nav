# N06 manager r05 独立离线审查

结论：**受限入口与 manager 环境执行链组件通过；最终 N06 source/lifecycle 和 runtime disk 闭包仍待独立封存。** 本报告不授予 release、种子或 ROS/Gazebo 执行权限。

实际复跑 75 项作者测试，通过。14 组独立检查完成，r01/r02/r03/r04/r05 的 352/733/764/791/866 项资源在测试前及封存时全部匹配。递归本地静态 imports 也全部位于作者清单内；这不替代动态 loader/Python/ROS 依赖闭包。

独立测试调用真实的外层 env -i → bash --noprofile --norc → 系统 Python -I -S → bootstrap。用冻结的实际 execution_environment，而非空字符串 namespace，构造临时 synthetic 输入。无害 BASH_ENV/sitecustomize 标记均未执行；bootstrap 的 inspect 与拦截最终 execve 的真实 main 分支均确认 manager/ROS 尚未导入且传入精确绑定字典。8 类错误（SHA、遗漏/错哈希入口、重复资源、未绑定 runtime、错 stage、错解释器 prefix、禁用字段）均在导入 manager 前拒绝。另复测直接 bootstrap 多环境字段拒绝。

实际 live_preflight 对所有禁用键、未知键和每个已声明字段的改变均在 publish/consume/Popen 前拒绝。旧 LD_PRELOAD/LD_AUDIT/PYTHONHOME/PYTHONUSERBASE/EGL 继承反例在字典层拒绝，未实际加载这些注入值。实际 supervisor 的 worker Popen 收到完整固定字典；模拟 spawn 失败后 0 consumed、8 retired，没有伪造子进程退出观察。

实际 execute_unit 用未 stub 的环境 helper 执行四例：正常、启动前父字典改变、assessment 前父字典改变、assessment 异常。仅 ROS 子进程被替身截获；两次 assessment 内真正启动无害系统 Python，观测完整环境仅包含固定字段及 run 的 ROS_HOME/ROS_LOG_DIR/nonce。正常和异常退出均恢复原精确父字典；两个漂移用例拒绝相应启动/评估，保留失败回执。临时运行路径和 synthetic 报告不能充当真实导航证据。

r04 晚到 assessment 恢复的两个函数 AST 与 r05 完全一致。又在实际 reconcile_worker_exit 上重放 complete/timeout/fault/stopped 四种 retained assessment，均闭合 1 consumed infrastructure failure + 7 retired；保留 assessment 字节/SHA、-9 错误与 task 标签，不将 complete 标签提升为成功。再次 reconcile 对 bytes/inode/mtime 为只读。非法/不完整 assessment 仍是已披露 unresolved 范围，不声称任意损坏磁盘可自动收口。

最终 gazebo_ros wrapper 的八份 shell 观测由另一位审查者独立重放，本报告绑定其 r05_wrapper_precheck 并明确继承；未重复运行 wrapper，也未把环境观测当成完整 disk 闭包。它保留旧空模型 URI 恢复远程地址的负例；当前非空 localhost URI 在观测中保持。完整 runtime 仍待单独报告和最终候选 roster。

精确启动方式包含外层 clean env；直接从任意已加载环境导入 manager 不是替代入口。此方案是可信本机、受控唯一写入者工作流的一致性门，不是抵御恶意同账户修改或未知 OS/loader 攻击的证明。哈希自举依赖外部审过并绑定的入口字节。子进程与评估门会捕获后续父环境漂移，但若漂移在已分配身份之后才出现，则会消费该身份并失败，不宣称回收预算。

真实三张 CSV 的 bytes/inode/mtime 与前审观察一致，未修改作者源码、旧报告或真实账本。N05 outer-shell 143 原因未知等既有限制保留。
