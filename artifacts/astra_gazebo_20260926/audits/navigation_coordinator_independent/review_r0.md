当前r0独立审查不通过：三个实际代码竞态已复现。

1. complete/timeout/stopped会被迟到fault覆盖；完整node夹具已先发complete再因method故障或暂停5秒wall stall改成fault。
2. 合法odom header比本地/clock领先2ms时永久fault。应源消息排队待clock覆盖，不放宽200ms新鲜度。
3. 29.000s目标odom早于29.020s截止，同一消息两种跨topic回调顺序得到complete/timeout。超时需由有序实际odom水位裁决。

新route仅部署传感器输入，scan使用exact-time TF；R2保持算法但仍以Gazebo model_states作bootstrap/内部goal break，且latest TF健康与future age clamp不同，不能称四方法全链sensor-only。私有Twist只证实receipt，不证明生成时刻；queuedpreack防止范围仅已收到callbacks。完整源码快照、8个针对性案例和行号见JSON。无ROS/新运行/运行源修改。
