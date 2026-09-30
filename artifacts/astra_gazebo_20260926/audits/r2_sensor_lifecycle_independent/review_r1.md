R2 sensor lifecycle r1 independent repair review
================================================

Offline repair review passed; live integration remains pending. Final nodeSHA765238653925e232243c1caa01a58d9034e1ee7171f0f39b1d1032d31ddd3712; helper and all original algorithm/weights are unchanged. Author62tests replayed successfully;3 independent real-parent-callback probes pass.

The discovered second exact-TF lookup failure now latches `frozen_scan_history_append_failed` and emits zero before any queue pop. The original660ms-old history reproduction now raises and remains unready, with one actual parent failure instead of repeatedly dropping scans. A transient first lookup miss still recovers at its exact original source stamp, and a failed sixth real history cannot arm. The new readiness gate separately checks the latest committed source stamp at the unchanged200ms boundary.

The former negative review/source blobs/counterexample remain frozen. No simulator, ROS master, training, build or old algorithm edits occurred. Route availability remains audit_only; this does not qualify all-five shared-path consumption or live use.
