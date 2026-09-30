# D04 — 24 additional development parent geometries

Final geometry-only inputs are `scenes/development_d04_geometry_r04/`: six core families × four additional parents. All 24 canonical parent IDs and all 24 unquantized geometry hashes are different. They do not duplicate the 24 registered A01/A02 parents. N04 and N05's 24 derived manifest rows were authenticated and reduce to the same six A02 parents; they were not counted as additional geometries. A01's original feasibility failures are not requalified by this identity audit.

The final author checks pass, with independent review pending. No ROS, Gazebo, training, actor execution, execution seed allocation or global registry mutation occurred. No final outcomes were accessed. The deliverable deliberately contains geometry manifests and reference-path CSVs, not ready-to-execute episode worlds. Actor policies in the manifests are offline existence witnesses, not released runtime configurations.

## What changed geometrically

| Family | New parents | Included structural forms |
|---|---:|---|
| Static avoidance | 4 | Different central-block progress, offset, width/length and corridor width |
| Single crossing | 4 | Perpendicular and oblique paths, distinct conflict locations and offset approach edge |
| Head-on yielding | 4 | Symmetric corridor, left/right bays; new variable-depth/length bay with return walls |
| Sequential interaction | 4 | Alternating and same-side crossings with distinct separated paths |
| Occluded emergence | 4 | Three distinct single screens and one actual two-box L screen |
| Width-change corridor | 4 | Symmetric, offset and two-stage throats |

The inherited reflection-canonical route-frame descriptor excludes labels and runtime noise. Same-structure parents additionally require a maximum normalized structural difference of at least two cells from prior/new peers, stricter than the existing one-cell near-duplicate rejection. Positions use 0.20 m cells and dimensions 0.10 m cells. Different box/path counts are separate structural forms. World rotations or translations were not used to manufacture independent layouts; current manifests remain canonical yaw-zero inputs with gate −4, goal 5 and nominal longitudinal route length 9 m.

## Per-parent geometric checks

Each parent has `geometry_manifest.json`, `geometry_checks.json`, `reference_witness.json` and `reference_witness.csv`. `r04/parents_summary.csv` indexes all 24 results.

The reference model uses the existing expanded physical robot URDF and its authenticated converted SDF: seven native bodies and nine collision primitives. Spawn, gate and goal are checked using every projected collision primitive against static shapes and initial actors. Initial actor pairs are checked explicitly. Actors use complete 0.72 m square sweeps along their entire start/end translation; they are not replaced by inscribed circles. Across the accepted inputs, the smallest actor-sweep/static gap is **0.12095 m**, above the retained 0.05 m geometry gate.

Static path witnesses use an outward 16-plane envelope of all nine converted collision components, including all front steering orientations, the validated wheel-spin quantization pad and a declared roll-plus-pitch envelope of 0.05 rad. The geometric reference is the rear axle; all stored base poses include the exact 0.325 m offset. Sin-fourth profiles and separately versioned asymmetric quintic profiles have zero endpoint slope/curvature. Sample gaps of 0.01 m receive an analytic point-displacement Lipschitz deduction. Steering stays within 0.4964 rad, and a sufficiently slow positive witness speed bounds central steering rate below 0.75 rad/s. The minimum final conservative **interval lower bound after subtracting 0.33 m** is **0.00649963 m**. All 901 samples of each selected path were read back and their reported minimum independently recomputed by the author's artifact check.

This is an existence result for ideal static planar geometry. It does not demonstrate actual physical tracking, elapsed time within 22 s, collision avoidance with moving actors, controller action support, or continuous physical safety. The witness's detour arc length may exceed the nominal 9 m longitudinal task distance. Slow witness speeds are mathematical feasibility parameters, not proposed method speeds or measured outcomes. Rasterized common-map/guide derivation and its extra discretization effect remain untested; the narrowest positive margin must not be assumed sufficient for an arbitrary raster export.

Every actor has a gate-pose 720-ray visibility diagnostic. The four occlusion parents additionally require ≥5 actor-directed rays, initial visible fraction ≤0.20, revealed visible fraction ≥0.80 and a collision-clear reveal pose. The largest accepted initial fraction is **0.185185**, and every revealed fraction is **1.0**. These are two static sensor poses with the actor held at its initial position, using the real beam angles and 0.60 m laser height. Noise, rolling/tilted sensor effects and temporal emergence in a reactive episode remain pending.

`family_actor_timing_witnesses.json` adds development-only parameter-cell existence examples with absolute unreactive arrival difference ≤2 s. Sequential second actors use the intersection of the generic gate-delay range [0,1] s and family second-delay range [0.8,2.8] s. The original generic parameter examples remain unchanged. Neither set assigns episode seeds or proves reactive timing, perception or dynamic feasibility.

## Preserved generator counterexamples and source lineage

1. **r01:** 11 accepted parents; the fourth symmetric head-on slot exhausted 2,000 draws. Of those, 1,449 hashed outside development and 551 failed the retained structural distinctness rule. These are input-generation rejections, not failed simulator episodes.
2. **r02:** The exact 11 accepted inputs were copied without byte changes. A new source version sampled bay length [1.8,3.0] m and depth [0.6,1.2] m, with actual return walls. It reached 17 parents but exhausted the next single-screen slot. Ray semantics and finite symmetric route witnesses rejected the relevant candidates; their geometries and checks remain in `counterexamples.json`.
3. **r03:** The exact 17 inputs were carried forward. Asymmetric quintic path witnesses expanded the finite search without changing scene inputs or clearance/steering thresholds. It reached 19 parents, then exhausted the fourth single-screen slot. This is not a proof that the rejected geometries admit no path.
4. **r04:** The exact 19 inputs were carried forward. The fourth occlusion slot implements the already declared L-corner topology with a real return screen; both screen centers must remain 0.5–1.4 m from conflict. It filled the remaining five slots, reaching 24. Final thresholds are unchanged.

All four generator source versions, their tests, logs, partial output directories, rejected draws and bounded failures remain present. The full local lineage retains **8,418 geometry RNG draws**, including first-free collision handling and rejected candidates. Every new record has purpose `geometry`; there are zero new execution-stream allocations. This finite offline input search does not retry, replace or relabel any experiment.

The frozen v2 generator itself was not edited. Its L-corner/staggered requests collapse to `single_screen`; D04 preserves an actual counterexample and repairs only the new D04 L input. Other excluded forms remain explicit: static staggered blocks exceed the generic central-obstacle lateral range in the inherited implementation, and `crossing_then_oncoming` has an unqualified second-conflict lateral offset. Open crossing scenes also retain the inherited absence of explicit side-boundary walls; D04 does not claim a complete implementation of every grammar parameter or topology.

## Split separation and remaining work

All 24 new parent IDs map to `development` under the existing `allocation_v1` geometry hash. All supplied actor-parameter cells independently map to development. All cosmetic/rigid transforms and parameter repeats must retain their parent identity and split. No episode-row random split is permitted. The combined A01/A02/D04 registry has no parent/near-duplicate conflicts.

This retains the actual implemented parent-hash and actor-cell-hash rule. The frozen split contract's literal joint `(parent, actor cell)` hash expression is not implemented by the old generator; that discrepancy is disclosed rather than called resolved. Future train/validation/final generation must check the combined parent and near-duplicate registry before allocation and enforce actor-cell separation. D04 has not inspected all future registries and cannot certify absent data. It neither opens nor constructs final outcomes.

Four new parents per family do not complete the contract's 24-parent-per-family development target, 100-parent training design, fresh validation tranches or ≥360 final blocks. Required next work includes independent source/geometry review, complete grammar and actor-timing integration, common raster/guide conversion, actual physical/reference tracking and temporal visibility checks, and a separately bound execution stage. Every failure must remain visible. N05 and all inherited runtime resources were left unchanged.

Author evidence: 18 small component tests passed, plus all 24 final artifact/901-sample rechecks. `r04/implementation_report.json` and its source/evidence manifest freeze this bounded offline result; they are not an execution release or independent approval.
