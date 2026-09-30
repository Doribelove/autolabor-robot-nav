# V3-15-P1 Thesis Export Terminal Handoff

Date: 2026-08-06

## Export status

The pure-offline paper export is complete. The canonical output is the 23-page A4 PDF:

`artifacts/v3/paper/v3_15_p1_thesis_export/FAM_TEB_V3_THESIS_DRAFT.pdf`

The manuscript contains Chinese and English abstracts, system goals and architecture, candidate-
conditioned prediction and hard verification, experiment-integrity methods, the V2/T12 negative
background, the V3-11 actual-TEB formal positive result, the V3-14 multi-actor terminal negative
result, limitations, conclusions, and evidence/claim appendices.

## Claim state

- qualified primary result: in 30 fresh paired blocks / 60 episodes from three frozen
  single-reactive-actor Gazebo templates, V3 spatial-safe reduced restricted gate-to-goal time
  versus actual TebLocalPlannerROS 0.8.4 by 12.9288%, with one-sided 97.5% lower 12.7253%; both
  methods had 100% goal success and zero contact states;
- terminal extension result: V3-14 completed 45/45 units with correct action identity, but failed
  the 0.05 m actor-coordinate and 0.05 m/s actor-speed common-prefix gates at 0.066074 m and
  0.072081 m/s; multi-actor support and predictor fitting were not demonstrated;
- forbidden: perception robustness, multi-actor performance, OOD generalization, formal safety,
  learning benefit, deployment readiness, all-TEB superiority, and real-vehicle performance.

## Frozen export evidence

- PDF SHA: `be4fe262001a874c1e1d4ada8c8196a209a5fba833863e0d69ce0c987858c59a`
- paper evidence manifest SHA: `9a919b61574c007d3d52f6555ab33962877341daf81cfc38d41fd398c12ffde7`
- claims SHA: `421f9654977be606313da3dbef04c52f310df57b810dd4ad68b56a99dfcaa264`
- result summary SHA: `892e8d12dbf8dda020be97f86ea58292a5c3c85da2a8e7eddd7e96ff60f1e7f1`
- preserved first review failure SHA: `79e4ee1bf8b4e953dada8d9c784b274bd6a838d99e0b1f1c0d89b3d98992a3c0`
- R1 independent review SHA: `cc3b1cb69ddef8fedcbb2929e879d0b282bd6d1dbb9526985a828a8f449bebab`
- R1 terminal freeze SHA: `6eb74645674c00f5eea5ec3825f18345b7dbd6a58acdb61ca5f7b07b7b77411a`

The first paper review failed only because its `pdfinfo` A4 assertion depended on an exact number
of spaces. R1 repaired only that parser, preserved the failed review, and changed no PDF, evidence,
metric, claim, or threshold. All ten R1 checks pass.

## Authority and future work

This stage grants no execution, training, hardware, deployment, or real-vehicle authority. The
current user objective through paper export is complete. Any future stronger journal study must be
a separate effort with a preregistered actor-state synchronization factor, independent geometries,
fresh multi-actor/perception/OOD evidence, and a new bounded release. Real-vehicle work remains
subject to separate on-site approval.

