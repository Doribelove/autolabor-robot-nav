The evaluator now recognizes the new Astra actor plugin and reports its reviewed target/current speed envelope. The gate only truncates integration after activation; the policy dynamics remain the retained V3 rule. The bound is max(initial, base, 0.12, 1.35, 0.32, 0.78×base) m/s, including initial speeds above every target. Invalid configurations, additional unknown plugins or incomplete actor bounds fail closed.

61 tests pass: 44 evaluator and 17 geometry tests. New checks include 180,000 updates through the production C++ header, all 12 live-v1 scene world rosters, a high initial speed, invalid gates/configurations and an observed actor jump despite a known plugin filename. No ROS, Gazebo or training was started.

The exact previous source/tests/reports are preserved in `evaluator_actor_compat_prior/` with hashes. Source, current library and tested world hashes are in the JSON report; a future run must separately bind its actual loaded binary. No historical evidence was changed.

Continuous safety remains pending: the actor still uses SetWorldPose and sets reported twist to zero. A conditional model envelope does not prove physical velocity continuity. This change does not make fully_safety_qualified_success true or lower its failure-inclusive 22 s time.
