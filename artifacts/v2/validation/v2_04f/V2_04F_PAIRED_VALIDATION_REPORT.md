# V2-04F Fresh Held-Out Three-Method Validation

Success non-degradation: **True**. All hard gates: **False**. Performance effectiveness: **False**.

| Method | Success | Collision | Minimum clearance (m) | Total time (s) | Anchors |
|---|---:|---:|---:|---:|---|
| fixed_teb | 10/10 | 0 | 0.254 | 203.0 | fixed_teb |
| balanced_anchor | 10/10 | 0 | 0.000 | 293.4 | anchor_balanced |
| rule_multi_anchor | 10/10 | 0 | 0.429 | 268.4 | anchor_balanced, anchor_corridor, anchor_cruise, anchor_maneuver_forward, anchor_static_dense |

## Rule paired medians (descriptive only)

Stage-2 claims are not authorized because hard gates failed.

| Family | Time vs Fixed | Time vs Balanced | Path vs Fixed | Clearance vs Fixed (m) |
|---|---:|---:|---:|---:|
| CRUISE | +3.9% | -20.4% | -0.0% | +0.000 |
| DYNAMIC | +4.0% | -25.5% | -0.0% | -0.789 |
| STATIC_DENSE | +73.3% | +12.7% | +0.6% | +0.118 |
| CORRIDOR | +60.7% | +26.0% | +0.0% | -0.001 |
| MANEUVER | +51.2% | -10.8% | -7.2% | +0.182 |

## Blockers

- `successful_episode_minimum_clearance_below_0_25_m`
- `held_out_rule_anchor_switch_count_exceeded_preregistered_maximum`
- `dynamic_observed_conflict_fraction_below_preregistered_threshold`
- `rule_navigation_time_regressed_vs_fixed_in_all_five_families`

The frozen supervisor and Anchor Bank were not modified. SAC training and real-vehicle execution remain unauthorized.
