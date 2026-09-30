# 四方法 bootstrap smoke 独立预检

状态：执行前需修复。这是源码审查与纯离线 renderer 检查，不是实际 Gazebo 运行通过。审查范围为新 seed260926001、四个方法、static corridor，保留旧 R2 行为，只增加无碰撞体相机与地面外观。实际源哈希见 JSON。

## S01 / blocking

`scripts/astra_gazebo/run_smoke.py:129-147,149-160`

Attempt directory is created before the exception/receipt boundary. Renderer, binding, git diff, port reservation, or ledger errors produce a partial identity without durable failed receipt.

Required: All operations after identity allocation must finalize a receipt with explicit preflight/not-started status. No automatic rerun of partial identity.

## S02 / blocking

`scripts/astra_gazebo/run_smoke.py:111-121`

After leader exit, remaining process group receives SIGTERM without waiting, escalating, or verifying isolation ports. Cleanup can return while Gazebo children remain.

Required: Wait on actual owned group membership and identity, escalate to SIGKILL if needed, confirm no owned descendants/occupied ports before next row. Preserve final cleanup result.

## S03 / high

`scripts/astra_gazebo/run_smoke.py:94-108`

Runtime dependency binding is incomplete: Python executable recorded as path only; no environment manifest/lock binding; omitted move_base and several Gazebo sensor/API plugin binaries; TEB runtime plugin path resolution not demonstrated.

Required: Bind environment manifest and actual executable/library/module provenance for this smoke, or explicitly mark incomplete closure and avoid a full dependency integrity pass.

## S04 / high

`scripts/astra_gazebo/run_smoke.py:157-170,194-195`

If report exists after KeyboardInterrupt or wall timeout, status becomes completed; loop can start next method.

Required: Interruption/timeout remains a terminal queue reason regardless of report presence; finalize evidence and stop schedule.

## S05 / medium

`src/simulation/m2_gazebo/launch/astra_smoke_episode.launch:14-18`

Passive monitor death or no camera frames does not affect episode completion; all_four_completed is only report presence, not successful capture or simulation validation.

Required: Separate execution_complete, navigation_pass, capture_complete, machine_qc and human_qc. Do not claim smoke pass from all_four_completed alone.

## S06 / medium

`scripts/astra_gazebo/run_smoke.py:126-128`

Existing receipt is accepted without validating identity, bindings, report hash, or retained evidence.

Required: Validate recorded identity/evidence before idempotent reuse; never restart corrupted historical attempt.

## S07 / observation

`src/simulation/m2_gazebo/scripts/astra_evidence_monitor.py:127-148`

Legacy spawn z=0.02 and repeated SetWorldPose may yield wheel bottom around +0.02m; QC correctly requires <=0.01m, so bootstrap may expose floating geometry.

Required: Measure live link/visual evidence, retain failed QC, introduce any physical correction under a new identity/version; do not loosen QC merely for pass.

## S08 / observation

`src/simulation/m2_gazebo/scripts/astra_evidence_monitor.py:105-148`

Images retain exact sensor stamp and receipt-time model/link context separately, which is correct provenance; callback delay can make latest odom/scan newer than image and machine checks then fail.

Required: Do not claim exact synchronization from receipt-time states; use bounded timestamped queues or offline association for later stronger evidence.

独立实际检查：runner/monitor AST parse通过；调用新runner的render(260926001)，产生3640字节world，4个静态world模型、加机器人后预期5个，原图2560×1440。没有调用execute或启动ROS/Gazebo。原图以native sensor header时间、receipt时间状态、相机参数和SHA绑定，且visual_qc/final_size_qc保留pending，这是正确边界。

## 修订复核通过（仅本次 bootstrap）

原始发现保留于上文和 JSON revision_history。S01–S05 已处理；S06 既有receipt再验证为续跑前必需，当前首次新身份不构成阻断；S07/S08属于本次实际运行待测诊断。

实际运行非ROS进程fixture：父进程先exit0，两个子进程各自setsid，其中一个忽略SIGTERM；调用真实stop_group，约 5.117 秒后所有专属环境标记的活进程清空，证明SIGKILL升级路径；不同标记的control进程保持活着。端口14820/14821可bind。仅将cleanup.json写入函数替换为内存sink，信号/进程检测/等待/端口检查均为真实执行。没有启动ROS/Gazebo。
