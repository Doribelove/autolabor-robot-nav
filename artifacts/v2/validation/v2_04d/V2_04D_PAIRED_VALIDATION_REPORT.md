# V2-04D No-Training Paired Validation

Stage 1 passed: **True**. Performance effectiveness proven: **False**. V2-05 authorized: **False**.

| Method | Success | Collision | Minimum clearance (m) | Total time (s) | Anchors |
|---|---:|---:|---:|---:|---|
| fixed_teb | 10/10 | 0 | 0.254 | 209.1 | n/a |
| balanced_anchor | 10/10 | 0 | 0.415 | 277.8 | anchor_balanced |
| rule_multi_anchor | 10/10 | 0 | 0.433 | 344.0 | anchor_balanced, anchor_corridor, anchor_cruise, anchor_static_dense |

## Rule Multi-Anchor paired median change

Positive time/path percentages are regressions; positive clearance is improvement.

| Family | Time vs Fixed | Time vs Balanced | Path vs Fixed | Clearance vs Fixed (m) |
|---|---:|---:|---:|---:|
| CRUISE | +62.5% | +24.4% | +0.0% | +0.005 |
| DYNAMIC | +96.6% | +52.9% | +0.2% | -2.230 |
| STATIC_DENSE | +51.4% | +16.3% | -2.8% | +0.079 |
| CORRIDOR | +66.4% | +28.6% | -0.2% | -0.003 |
| MANEUVER | +54.1% | +10.9% | -7.7% | +0.182 |

## Blockers

- `rule_navigation_time_regressed_in_all_five_families`
- `rule_supervisor_never_activated_maneuver_anchor`
- `cross_method_dynamic_observed_conflict_fraction_below_preregistered_threshold`

The frozen Anchor Bank was not modified. SAC training and real-vehicle execution remain unauthorized.
