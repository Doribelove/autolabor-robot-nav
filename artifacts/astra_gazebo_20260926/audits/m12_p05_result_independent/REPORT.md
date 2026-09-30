# P05 独立终止审计

P05 在首个启动回合终止：1 consumed、5 retired，未进入36 s命令日程，PHY03保持 `not_complete`。527项release依赖与全部归档blob、全部回合绑定、receipt/hash/预算/清理经独立复核。

实际 raw 中所有运动、命令、状态、ingress数组均0，没有schedule_start、final_drain或bag；因此不能声称复算122门或无物理运动。没有可用于物理对照的P05测量结果。

同一份实际 robot_description 含P05 driver，probe验证通过；冻结command source第188–189行仍要求P02 driver。独立调用原函数精确复现 `ValueError: unique P02 physical plugin required`。source required node因此exit1；probe随后readiness wall deadline、spawn在shutdown中失败。roslaunch0不掩盖该错误，manager正确保留infrastructure_failure。

前次独立审查的明确盲点：验证了stage-only source差异和probe/manager，却没有让source与probe两套启动验证函数同时接收真实展开URDF。旧离线pass报告保留不改；该报告不能当作runtime集成保证。后继新stage必须以实际展开URDF调用两套验证器，并验证新版本插件名、seed及参数，不能只做launch dump或重复合成运动门。

原预算不可续跑、换seed或补样；PHY01/PHY02也保持未完成。审计JSON：`terminal_startup_failure_audit.json`；所有命令和哈希在其中。
