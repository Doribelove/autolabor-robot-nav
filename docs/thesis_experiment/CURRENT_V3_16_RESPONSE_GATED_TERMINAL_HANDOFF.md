# V3-16 Response-gated Multi-actor Terminal Handoff

Date: 2026-08-06

## Terminal result

V3-16-D1 completed a pure-offline attribution of the terminal V3-14 common-prefix failure. All
40 scientific receipts retained correct action identity and ego pairing. Maximum actor coordinate
spread was only 0.00512 m at the warmup gate but grew to 0.066074 m at the 12.5 s intervention
boundary. The evidence supports pre-branch reactive-response amplification as the bounded root
cause and rejects action/ego mismatch as the primary explanation.

V3-16-S1 preregistered exactly one factor: both actors retained their frozen paths, base speeds,
policies and geometry, but reactive response was disabled until simulation time 12.55 s, 50 ms
after the 12.50 s candidate branch. Seed 7741 supplied five non-scientific preflight actions;
fresh seeds 7742--7749 supplied eight five-action scientific blocks. All 45 units completed at
attempt 1 with no retry, resume, replacement, selective exclusion, forfeiture, action substitution,
hardware access or real-vehicle operation.

The factor passed its speed-mechanism prediction but failed the complete scientific gate. Maximum
actor speed spread fell from the V3-14 value 0.072081 m/s to 0.008597 m/s, an 88.07% reduction and
below the frozen 0.05 m/s gate. Maximum actor coordinate spread was 0.067472 m, above the frozen
0.05 m gate; only 5/8 scientific blocks passed the coordinate gate. Consequently:

```text
evidence_collection_gate_pass=false
predictor_fit_authorized=false
actual_TEB_execution_authorized=false
perception_execution_authorized=false
training_authorized=false
hardware_accessed=false
real_vehicle_used=false
```

This supports only the scoped mechanism statement that delayed reactive response removed the
speed mismatch. It does not qualify common-prefix support, multi-actor prediction, performance,
generalization, safety or deployment.

## Integrity repair

The frozen assessor's first call failed only because it resolved raw episode reports through an
indirect index into a mixed resource list. The failed assessment and review are preserved. R1
changed only path resolution to each receipt's sibling `episode_report.yaml`; no simulation byte,
identity, inclusion, threshold or scientific result changed. R1 replayed all 45 report hashes and
action/response bindings and passed independent review.

## Frozen evidence

- D1 attribution report SHA: `1f84987b14a9a8721eea5823e965d7c0283f0a4250f7efb7137cf060b08e7396`
- D1 independent review SHA: `2a1b92099420861074b73540b9c4a9ff9322ef93c7f4607eda7c248c0c8cbd73`
- S1 contract SHA: `90398235f01cbc97e0279242261436f1dbb0853095b166896e61f0981ccf9c6d`
- S1 execution release SHA: `8bdb5eebebaf1cded5b2712fd51d92cce134858255f7f8776cfb1f7076618a71`
- S1 evidence summary SHA: `088483a89952f157b51f5411a96e270f3d639c214a244d1d6fbdbc4b9e116590`
- preserved failed assessment SHA: `32569fc7b7ff51a90919a0dcc6facca3df09e0da18c7a3fc805eee39d0649a26`
- R1 assessment SHA: `6a30f54abea88949722cf10e58e4da59aaa11a631f9eece2ee373a9f766ebaf5`
- R1 evidence manifest SHA: `571ea959b33cca9ca4718e05ddb546e11a343d3d94807b0166d3e7abd945cae0`
- R1 independent review SHA: `c4469571a1a8f8e999e733138d42466fa0c0a6bd472bcebe5d044fe512a4765c`
- R1 terminal freeze SHA: `1338cd90bc0712ac58ad72776bd87bfd74a3f50eb0d82710d4389aa7a3abc628`

## Prohibitions and future entry

Do not rerun/resume V3-16, reuse seeds 7741--7749, alter the frozen 0.05 thresholds, append units,
fit the r3 multi-actor predictor, start actual-TEB multi-actor/perception/OOD experiments, train,
or access a real vehicle from this evidence. A future multi-actor study would require a genuinely
new actor position snapshot/reset design, fresh seeds and budget, independent review and separate
explicit simulation authorization. Paper-only interpretation and export are safe.

