# teb_mode_manager

FAM-TEB V2 factorized context, transition and parameter-transaction package.

- `teb_mode_manager_skeleton_node.py` remains the default invalid fail-closed heartbeat.
- `rule_context_supervisor_node.py` is the V2-03 label-free simulation candidate. It consumes
  matching world-model geometry/tracks/health, applies confidence, confirmation, dwell and release
  hysteresis, and publishes `ContextState` plus transition audit events.
- `rule_anchor_transaction_node.py` is the V2-04 zero-residual rule candidate. It resolves a typed
  Anchor/profile, decodes it into the feasible domain, starts each rate-limited transition from the
  previous acknowledged `executed` profile and publishes all four action stages.

Health or sequence faults immediately force invalid `BALANCED/NONE/FAULTED`. The V2-03 node never
reads a scene manifest/family label, publishes `/cmd_vel`, or writes a parameter transaction.
Current thresholds remain uncalibrated and `runtime_ready=false`.

The V2-04 executor is deliberately `deterministic_shadow`: it models atomic request, ack, readback
and activation but has no dynamic-reconfigure or real-vehicle writer. Integer and boolean slow
profile parameters commit only after continuous values converge.
