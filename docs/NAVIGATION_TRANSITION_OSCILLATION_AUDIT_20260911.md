# Latest navigation and transition oscillation audit — 2026-09-11

## Scope and clock

This is a read-only audit of the live session started at 2026-09-11 00:22 CST.
The J6M release is `20260910_233119`. The ROS master is on J6M at
`192.168.10.100:11311`. J6M Python log prefixes render eight hours behind CST;
the times below are ROS epoch timestamps converted to Asia/Shanghai.

No navigation goal, velocity command, pause/resume command, parameter update,
build, deployment, or business-code change was issued during the audit.

## Current state observed after the experiment

- Coverage batch `coverage-batch-0ef10340c856402087c7498bd47fff5c` is
  `COMPLETED` for region `研究院大楼前大厅`.
- Coverage is inactive and navigation is paused.
- Localization remains `LOCALIZED` (`overlap=0.9969`, `rmse=0.1594`,
  `inliers=3888`).
- A five-second passive sample recorded 245 final command samples, 100 safe
  command samples, 253 samples from each wheel and 250 steering-angle samples.
  All command and wheel velocities were zero and the steering angle was zero.

## Coverage timeline

Mission plan `b9000178ef224e2f8291661174f6bd3a` ran from approximately 00:27:45
to 00:32:53. It contained four swaths and eight entry/transit/sweep segments and
eventually completed.

Hybrid A* produced 20 valid direct connector paths. No Hybrid search timeout,
localization loss, coverage collision rejection, or entry-recovery 4 m rejection
appears in this mission's log.

The two problematic line-to-line transitions were:

1. The transition after sweep segment 4 started with a 4.80 m forward path.
   It made less than 0.10 m progress for two seconds twice. Replanning repeatedly
   produced a 2.10 m path containing a 0.30 m reverse part. Four measured cusps
   invalidated the cached suffix with join errors of 41.4, 23.2, 37.5 and 43.9
   degrees. A final 2.10 m forward-only path entered the next swath at 00:29:46.
2. The transition after sweep segment 6 started as 4.20 m in three parts, with
   0.60 m total reverse. Replans made less than 0.10 m progress five times.
   Four cached suffixes were rejected (8.1, 19.8, 22.8 and 29.2 degrees, versus
   the configured 5.7 degree join limit). The mux held zero during the interval
   in which its TF/TEB/path lease or safety permit was stale. A final 2.40 m
   forward-only path reached the swath-entry gate at 00:31:07.

Across the coverage mission the log contains:

- 588 `optimized command violated the fixed-gear contract` warnings;
- 32 `Hybrid TEB mux is holding zero` warnings;
- 9 cached cusp-suffix invalidations;
- 7 Hybrid attempts canceled after less than 0.10 m progress in 2 seconds;
- 7 TEB subgoals reported as reached during repeated cusp handling;
- one TEB oscillation detection at 00:31:15;
- one stale transition callback after an already replaced goal handle.

Of the 588 fixed-gear command violations, 31 occurred while a Hybrid transition
was active and 557 while a forward-only sweep was active. The last sweep alone
accounts for 521. On each violation the local planner clears its optimized band,
zeros linear and angular output, and returns `NO_VALID_CMD`. Valid cycles may
then rebuild a differently curved band. This repeated reset explains intermittent
stops and is consistent with the operator's observed steering correction from
side to side. TEB's own failure detector also observed multiple angular-command
zero crossings before emitting its oscillation warning.

The strongest causal chain is therefore:

1. Hybrid A* finds a path with short fixed-gear parts and cusp changes.
2. TEB deforms the first optimized edge across the requested gear sign, or a
   forward sweep temporarily develops a small backwards fold.
3. The fixed-gear guard correctly refuses that command and outputs zero, but it
   also clears the complete local band every control cycle.
4. The manager sees no physical progress after two seconds, cancels the attempt,
   and replans from a pose only slightly changed from the previous one.
5. The new path often contains another short reverse cusp; the measured stop no
   longer joins the cached suffix within 5.7 degrees, so the suffix is discarded
   and the cycle repeats.

This is an execution and hand-off failure. It is not evidence of Hybrid A*
failing to find a route. It is also unrelated to the 4 m entry-recovery envelope
and the 5 m sweep collision-check horizon.

## Events after coverage completed

The coverage manager restored ordinary Navfn + TEB at 00:32:53. A separate
ordinary navigation/FOD sequence then generated infeasible TEB trajectories whose
footprints intersected lethal costmap cells. TEB detected oscillation at 00:33:21
and move_base aborted at 00:33:27 because it could not produce a valid control.
Another goal at 00:36:30 first hit a roughly 24 ms transform extrapolation error,
then temporarily had an infeasible local trajectory, but reached its goal at
00:36:42. These are separate from the coverage line-to-line transition problem.

At the end of this audit the FOD navigation-mode status was `FOD_ABORTED`, with
GPS navigation held paused until an explicit stop/return command. No automatic
resume was performed.

## Evidence boundary

The ROS logs contain planner decisions and failure reasons but do not contain a
time-series archive of `/m2_driver/wheel_angle`, both wheel velocities, or each
published TEB command during the mission. The logs prove repeated zero-command
rejection and TEB's oscillation decision. The physical steering amplitude and the
exact number of left/right steering reversals cannot be reconstructed from this
session.

Raw evidence copied from the live J6M process root is under
`validation/navigation_transition_audit_20260911/20260911_005900/`.
