# 结果、统计与失败案例

## 历史学习结果

- V4：0.0980%，95% CI [0,0.2941%]，未达到 3% 学习门。
- V5：6.0936%，95% CI [1.7867%,11.0069%]，未达到 8% 实效门。
- V6 fast population：PCT/解析 13.115625/14.318750 s、60/80 对 49/80；降低 8.402444%，
  双侧 97.5% CI [3.484003%,13.713459%]，限定阳性。
- V6/no-history 完全相同，不支持时序历史收益。

## 新 Gazebo R1

160 blocks / 640 episodes 全部 attempt=1。PCT/解析/TEB/DWA 有效目标为 153/146/89/80，均时
15.0634/16.1231/14.9225/16.0881 s。PCT 对 TEB 为 -0.9442%，97.5% CI
[-8.3569%,5.6359%]；对 DWA 为 6.3693%，CI [1.0871%,11.1411%]。无阳性比较。

## 新 Gazebo R2

200 blocks / 800 episodes 全部 attempt=1。PCT/解析/TEB/DWA 有效目标为 180/150/119/100，均时
14.8413/16.3988/14.4470/16.0925 s。PCT 对 TEB 为 -2.7288%，97.5% CI
[-9.8083%,3.5914%]；对 DWA 为 7.7754%，CI [2.5521%,12.6148%]。DWA 区间下界为正，
但点值低于冻结 8% 门，因此仍是 superiority 阴性。

## 必须保留的失败

R2 PCT 有 18 次 timeout/nonarrival 与 2 次保守净空失败；解析先验有 29 次 timeout 和 21 次
LaserScan 时间戳 TF 消息合同失败；Actual TEB 有 81 次、DWA 有 100 次负保守净空失败。所有
失败均进入 22 s 主指标。PCT R2 最小保守净空 -0.2696 m，因此不能声称形式安全。

## 分层机制证据

PCT 相对 DWA 在 partially-occluded/static-avoidance 快 8.3061/7.0338 s，但在 head-on-yield 与
single-crossing 慢 5.2666/4.6618 s。head-on 平均 24.97 次 certified stop；single-crossing
平均 14.00 次切换、15.53 次 stop；sequential 有 11 次 timeout，平均 23.24 次切换、38.67 次
stop、9.00 次恢复。这支持“保守停车/候选切换抖动”机制假设，但不是随机化因果证明。
