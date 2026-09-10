# Coverage / TEB action hand-off fix — 2026-09-11

## Saved baseline and scope

The user requested saving the current source to branch `0911`, then implementing
the transition marker and handling logic. Commit `8bdd8e2` is the complete source
checkpoint before this fix. It includes the previously uncommitted pending-litter,
vision and Qt work. Ignored runtime assets and private configuration were not
added. No remote repository was changed or pushed.

The preceding field evidence is in
[`NAVIGATION_TRANSITION_OSCILLATION_AUDIT_20260911.md`](NAVIGATION_TRANSITION_OSCILLATION_AUDIT_20260911.md).
That mission had 588 fixed-gear rejections (31 in transitions, 557 in sweeps).
The old controller cleared its elastic band on every rejection. The logs establish
repeated command rejection and replanning; they do not reconstruct the physical
steering amplitude or establish a single cause for every observed stall.

## Final behavior

1. Every successful synchronous `set_enforced_path` installation allocates a new
   action sequence, even when a paused sweep resumes with the same plan ID,
   segment and gear. The sequence is internal to the global-planner process.
   Repeated planner queries and topic lease refreshes retain that sequence.
2. The coverage global planner places the sequence in each returned
   `PoseStamped.header.seq`. The top two bits (`0xC0000000`) identify this local
   protocol; the remaining 30 bits form an installation counter. No ROS message
   definition or MD5 is changed. The counter wraps after 2^30 installations.
   This is an execution-state marker, not an authorization or ownership token.
3. On a new marked action in fixed-gear mode, TEB clears its elastic band,
   infeasibility history, oscillation detector, preferred turning direction and
   last command. A gear change also clears this state. Thus a same-gear path
   replacement cannot silently retain the preceding action's band or recovery
   preference. Same-action refreshes preserve the warm start.
4. A wrong-sign optimized command still returns zero and `NO_VALID_CMD`. Its first
   occurrence starts a recovery episode and clears the band once. Subsequent
   wrong-sign results in that episode keep zero without repeatedly clearing the
   band. TEB continues optimization each cycle.
5. During recovery, release requires five consecutive feasible command solutions
   in the required direction, with signed longitudinal speed above 0.001 m/s.
   Zero and another wrong-sign result reset that count. Transform, planning,
   divergence, footprint-feasibility and velocity-extraction failures also break
   the consecutive evidence. At the configured 10 Hz this requires roughly half
   a second of stable solutions. It does not certify physical progress; the
   coverage manager's existing progress timeout still handles persistent stalls.
6. Plan installation, reconfiguration and command computation serialize access
   to the planner state so a callback cannot clear the graph during optimization.

The existing global planner already anchors its output at the current pose and
omits the nearest cached anchor when a subsequent sample exists. That behavior
was retained; this patch does not introduce another path-pruning algorithm.
The coverage manager, entry acceptance and sweep completion state machine are
unchanged. Transition lookahead remains 2 m; entry recovery remains 4 m and sweep
collision checking remains 5 m. Wrong-direction commands, footprint collision
checks, turning-radius enforcement and the existing command/ownership gates
remain in effect.

## Validation and reproduction

Run commands from the source workspace:

```bash
./scripts/optimized.sh build
./scripts/optimized.sh run bash -c '
  source scripts/setup_env.sh
  make -C build run_tests_teb_local_planner_gtest_test_teb_basics \
    run_tests_autolabor_coverage -j2
'
bash src/application/autolabor_coverage/test/run_fixed_gear_handoff_simulation.sh
./scripts/optimized.sh check --static
```

The integration test launches real coverage-global-planner and TEB plugins through
move_base, with a synthetic map, stationary pose and odometry. Its wrapper creates
a network namespace with only loopback and a private ROS master at port 11494;
synthetic action goals and command outputs cannot reach the real vehicle. Cases
include a forward transition, another action in the same gear, a 0.30 m reverse
part, a forward sweep and resuming the same sweep. Each case checks the action
marker's transport, one reset per installation, stability across planner/lease
refreshes, usable same-direction output and the hard turning-radius bound.

Test logs and results are stored under
`validation/fixed_gear_handoff_20260911/`; the latest simulation directory is named
in its `latest_run.txt`.

Final validation passed:

- Full workspace Release build: exit 0 (`build.log`).
- 220 independent test cases: 19 TEB C++, 8 Hybrid A* C++, and 193 coverage
  Python cases (149 state-machine, 26 geometry, 15 contracts, 3 command mux).
  Zero failures and zero skipped. Counted actual `testcase` elements rather than
  Catkin's nested-suite totals; XML copies and counts are in `verification.json`
  and `test_results/`.
- Five real-plugin hand-off scenarios plus the loopback-only isolation check:
  passed. Every scenario had one action-marker reset, at least 13 command
  samples, no wrong-direction command and no hard-radius violation. Final run:
  `20260911_014531_845048/results.json`, console `integration_final.log`.
- Static health check, shell/Python syntax and `git diff --check`: passed.

An initial integration-test call tried the public `move_base/make_plan` service
while a synthetic action was active. move_base intentionally rejects that use;
the test now samples TEB's actual published plans during the action, and makes
the service query after exact-goal cancellation. The failed-run evidence is kept
in `integration.log`. Earlier direct test linking also required sourcing this
workspace's `scripts/setup_env.sh`; the final commands above include it.

## State before deployment

This is a local source and build change. After the reported power interruption,
read-only SSH confirmed J6M `current` still points to
`/opt/autolabor/dual_host/releases/20260910_233119/install`.
The fix has not been deployed, and no real navigation goal, velocity, restart or
task-resume command was issued. Persistent motion authorizations were preserved.
Physical tracking and steering behavior still require a later deployment and
operator-supervised experiment; a stationary synthetic test is not that acceptance.
The fix and this report are retained as uncommitted changes in the `0911` working
tree, separately from the user-requested pre-change checkpoint `8bdd8e2`.

## Deployment completed — 2026-09-11

The user's subsequent instruction was to deploy. The unified stop command
returned 0 and cleared stale PID records from the power-interrupted session.
There was no reachable candidate ROS master or active goal to cancel.
`./scripts/optimized.sh deploy` then returned 0 after 169 seconds and installed
J6M release `20260911_014849`; `current` now points to that release's install
directory. The previous `20260910_233119` release remains available.

The board's clock had reset to 2025-06-26 after power loss, causing initial Make
clock-skew warnings. The existing `scripts/sync_j6m_time.sh` restored host time
(measured midpoint offset -2 ms). Build logs confirm native recompilation of both
modified planner translation units. Six source/configuration SHA256 values match
the source workspace; the installed TEB header and two coverage YAML files also
match. Both installed ELF libraries contain the new marker/recovery code and load
successfully with immediate symbol resolution in the J6M chroot.

Board static health passed. `EnforcedPath` and `SetEnforcedPath` MD5 values match
NVIDIA. Installed launch expansion confirms 2.0 m transition lookahead and 4.0 m
entry-recovery length; the installed sweep collision YAML retains 5.0 m. Private
configuration and the local motion-authorization marker hashes are unchanged;
the remote private configuration matches NVIDIA.

Final observation: both local navigation/vision services are inactive with
MainPID=0; the candidate J6M run directory is empty and port 11311 has no listener.
No real initial pose, motion goal or velocity was sent, and no navigation task
was resumed. The operator may start through the workspace's unified Qt entry,
set the real initial pose for the new session, and wait for LOCALIZED before
testing. This deployment does not itself validate physical steering improvement.

Evidence: `validation/fixed_gear_handoff_20260911/deploy_20260911_014837/`, including
`stop.log`, `deploy.log`, `time_sync.log`, `verification.json`, `board_health.log`,
`library_and_launch.log` and `final_state.log`. Source fix remains uncommitted on
`0911`, with no push and no source-code changes during deployment.
