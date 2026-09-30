# V3-SCOPE ROS interface boundary

This package is an interface-only V3-00 foundation. It contains no planner,
predictor, tracker, ROS node, launch file, execution entrypoint, model or
training code.

The five messages preserve a direct audit chain:

```text
scene snapshot
  -> CandidateSet
  -> CandidateEvaluation[] (including explicit rejection codes)
  -> CandidateDecision
  -> future trajectory tracker
  -> /cmd_vel_navigation
```

The future tracker may publish only through `/cmd_vel_navigation`. It must not
publish directly to `/cmd_vel`, bypass the GPS limiter/FOD arbiter, or address
the M2 driver. The message package alone does not authorize ROS, Gazebo,
training, parameter writes or vehicle motion.
