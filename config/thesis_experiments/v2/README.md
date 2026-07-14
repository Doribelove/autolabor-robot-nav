# FAM-TEB V2 configuration root

This directory is isolated from the frozen V1/T00--T12 configuration files.

- `architecture_contract.yaml` freezes the V2 module and execution semantics.
- `parameter_registry.yaml` separates fast, slow-profile and startup parameters.
- `mode_thresholds.yaml` defines fail-closed design placeholders; it is not runtime-ready.
- `state_contract.yaml` requires LaserScan angular metadata and rear-coverage auditing.
- `simulation_contract.yaml` freezes the V2-02 uncalibrated dynamics candidate.
- `evaluation_contract.yaml` freezes the five-family trace and metric semantics.
- `world_model_contract.yaml` freezes the V2-03 world-model, tracking, prediction, health and
  label-free rule-supervisor component boundary.
- `action_pipeline_contract.yaml` freezes the V2-04 typed Anchor/profile, feasible decoder,
  previous-executed transaction and zero-training shadow-loop boundary.
- `v1_frozen_baseline.yaml` records the pre-implementation V1 evidence and hashes.

No file in this directory authorizes training, real-vehicle motion, or real TEB writes.
