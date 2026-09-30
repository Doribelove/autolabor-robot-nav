# V3-04 Learning Audit Handoff

Date: 2026-07-24

## Terminal status

V3-04-A0 is terminally complete with status
`v3_04_learning_not_justified_offline_closure_pass`. The prescribed sequence
was audited in order: supervised ranker value, offline contextual-bandit
support, then sequential offline-RL support.

No learning layer was activated:

- the C6 analytic ranker selected an accepted candidate in 308/308 decisions;
- 183 decisions exposed more than one safe candidate and 890 alternative
  accepted actions in total;
- the analytic choice had zero immediate progress regret in all 308
  decisions;
- the exact C6 closed-loop contexts have zero logged propensity for every
  unexecuted alternative;
- D1 contains 135 balanced one-step cloned interventions at propensity 0.2,
  but it does not contain sequential closed-loop action/transition support.

Therefore the contextual positivity and sequential support gates fail, while
the fail-closed no-training decision passes. Training steps and learned model
artifacts are both zero. The C6 analytic ranker remains the only qualified
V3 policy.

The audit report is
`artifacts/v3/learning/v3_04_a0/v3_04_a0_learning_value_support_audit.yaml`;
the terminal freeze is
`artifacts/v3/learning/v3_04_a0/v3_04_a0_terminal_freeze.sha256`.

## Claim boundary

This is a positive integrity result and a negative learning-activation
result. It does not show learned-policy benefit, offline-RL validity,
performance improvement, deployment readiness or real-vehicle readiness.

The absence of an RL result must not be disguised as a learned-policy
comparison. It is methodologically valid to carry the qualified analytic
planner into V3-05.

## Next safe entry

V3-05 must preregister a one-time, fresh, paired formal simulation evaluation
of the frozen C6 analytic V3 planner against fair fixed baselines. Geometry /
actor-policy blocks, sample size, statistical unit, family split, safety and
task noninferiority gates, primary efficiency effect, multiplicity handling,
failure policy and execution budget must be frozen before any new outcome is
accessed. V3-03 and V3-04 evidence must remain read-only.
