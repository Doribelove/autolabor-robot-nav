# M13 read-only M08 build profile

2026-09-27 local. This is one cProfile run for each of two frozen synthetic inputs, with no ROS, Gazebo, actor, training, or hard-real-time inference. It is a hotspot guide only; cProfile cumulative times overlap and must not be summed. No candidate source was changed or qualified.

| Case | Build profiled cumulative | Rollout | Verify-many | Route projection |
|---|---:|---:|---:|---:|
| sparse1440_retained2048 | 43.50 ms | 20.18 ms | 19.37 ms | 9.52 ms |
| dense1440 | 35.97 ms | 20.34 ms | 12.78 ms | 9.65 ms |

The frozen M08 benchmark remains negative on the 50 ms full-chain gate. The separate M12 lazy-query candidate also failed its dense regression gate and had two sparse observations above 50 ms. Neither candidate is promoted. Future optimization must use a new version and semantic parity plus fresh paired latency tests before live qualification.

Inputs and raw cProfile call data: [profile_report.json](profile_report.json).
