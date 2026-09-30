# N06 manager r03: focused repair, no execution authority

The immutable r01 and r02 sources, reports and independent negative reviews are retained. New modules are `run_navigation_n06_manager_r03.py`, `navigation_n06_integrity_r03.py`, `navigation_n06_process_r03.py`, and `navigation_n06_admin_plan_r03.py`. The process module changes only its integrity import; the manager changes only its three module imports. No physical/controller/evidence/prepared source changes occur.

The original r01 API, process ownership and administrative roll-forward model remain applicable, with these repairs:

1. Administrative apply rehashes plan, review and all plan resources before each table, after each replacement, and before completion; existing-completion returns also rehash. `publish_plan` adds the receipt's bound assessment and any source/sweep bindings. Input drift after the first table leaves that after-version visible and stops; no completion is emitted. There is no automatic rollback or claim of joint atomic publication. Exact restored inputs permit the same reviewed deterministic roll-forward; changed evidence is not silently re-approved.
2. `validate_terminal_receipt` is shared by ledger close and administrative projection. It checks stage/run/seed/method/attempt/release identity, boolean integrity, error and exit-code type, bound assessment schema/identity/task, and consistency of its boolean integrity checks. Positive receipt integrity requires no error, integer process code 0 and a bound positive assessment. Negative assessment or a later retained error remains a valid infrastructure failure. A task outcome without assessment is rejected. This is consistency validation, not an independent physical replay; independent terminal review remains necessary.
3. Every exact prepared conversion sidecar passes frozen `dense_sweep_n05.authenticate_conversion(..., {}, diagnostic=True)` before allocation. Its two converter lifetimes, command/output pairing, saved maps/library roster, reproduction and sdformat correspondence provenance are checked. Diagnostic pass does not assert live mapped converter correspondence; the inherited collector/runtime closure still must qualify that later.

Future release schema/identity/budget is unchanged. Root must bind these r03 modules instead of the four r01 manager modules and provide a new exact source review. No release, registration, or execution exists yet. Future input paths remain as proposed in r01 API; the runtime disk closure task is paused until this repair freezes. Root/independent reviewer, not this author report, decides readiness.

Explicit future read-only verifier invocation:

```bash
source /home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/workspace/scripts/astra_env.sh
/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/workspace/.venv/bin/python /home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/workspace/scripts/astra_gazebo/run_navigation_n06_manager_r03.py --verify --release /ABS/FUTURE_RELEASE.json --release-sha256 EXACT_SHA
```

Author tests use `-m unittest discover -s .../scripts/astra_gazebo/tests -p test_navigation_n06_manager_r03.py -v` under that interpreter. Structural positive-release fixtures explicitly stub native facts and converter authentication. A separate negative runs the actual authenticator on the three-field fake and rejects it, while a separate positive authenticates the preserved Q05 conversion without spawning anything. Historical Q05 data are fixtures only; they are not new N06 input/output evidence.

No final-window guarantee against a malicious concurrent writer is claimed; repeated checks close the observed accidental-drift counterexamples within the root-sole-writer workflow. Filesystem/kernel/supervisor failures may still leave an unresolved transaction. Original N05 outer-shell 143 remains unexplained, and host-shell exit remains unknown in this manager.


## r03 receipt closure

Administrative metadata has exactly 15 fields and cannot add run_id or any identity field. Receipt/report/raw/heartbeat paths, launch command, workspace, release/code/config hashes and exit values derive from the canonical run and frozen prepared contract. pid/pgid must match the retained process record. The administrator also refuses a metadata override of run_id even after validation.

Every assessment, including a retained negative assessment, must be at runs/RUN_ID/assessment.json and use the actual assessor schema, exact nine integrity checks, six canonical source bindings, and canonical sweep report. The sweep binds its complete eleven-source roster; common source bindings must match the assessment. World and runtime URDF bytes must equal the frozen prepared contract. Collector identities/completion and closure identity are joined. All these dependencies are returned to the administrative transaction for repeated checks. Prelaunch infrastructure failures may retain no assessment and no task outcome, with a nonempty error and complete administrative metadata; they are not invented timeouts.

This is exact source/receipt consistency, not a replacement for replaying the bag, physical evaluator or independent terminal assessment. Synthetic temporary fixtures exercise this schema but never constitute simulation evidence. The two orchestration modules are byte-equivalent to r02 after dependency-name replacement. r01/r02 remain ineligible for release; future runtime disk closure must bind all four r03 modules.
