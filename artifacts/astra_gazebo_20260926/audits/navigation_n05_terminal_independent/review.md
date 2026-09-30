# N05 terminal result: independent offline review

The terminal replay and result interpretation agree with an independent
all-24-bag recomputation. The original release, ledger, reports and runtime
files were not edited. This review supports the descriptive negative result
and no-promotion decision; it is not a fresh execution or safety qualification.

All 24 attempt-1 receipts match the terminal ledger and exact prepared order:
12 counterbalanced fresh-seed pairs, two methods per pair, over six reused
development parent geometries. All 1179 release resources and 696 receipt
artifact bindings were hash-checked. There are 24 consumed, zero retired, no
retry, and 24 receipts reporting valid capture integrity. Failed navigation
and engineering-clearance outcomes are retained rather than filtered out.

The independent code does not import or call the production replay or
navigation assessor. It reads every message on six selected topics in all
24 bags: 328354 source-stamped 2 ms `end_links` packets, 32826 odometry
messages, 9417 runtime traces, 1890 candidate records, 24 common gates and
3901 coordinator status messages. These are complete reads of the selected
topics, not samples. Within the formal windows, 236564 base-link poses form
complete increasing 2 ms lattices. Other bag topics were not semantically
replayed in this review.

Both methods complete 2/12 and time out in 10/12. The four independent first
geometry arrivals are 8.114 and 8.274 s for control, 8.310 and 8.350 s for
treatment. First qualifying 50 Hz actual odometry terminates at 8.120,
8.280, 8.320 and 8.360 s respectively. The small difference is retained:
restricted successful time uses the actual 2 ms geometry arrival; the
coordinator terminal uses its actual odometry source. Every timeout is kept
at the full 22 s cap. All 12 failure-inclusive paired restricted-time
differences and their mean **+0.0226666667 s** (treatment minus control)
match the original report and CSV exactly.

Terminal odometry is transformed using each saved bundle's map-from-odom
rigid transform. These endpoint poses agree with the same-source actual
world base-link poses within 8.89e-16 m in XY. All 24 goal distances and
12 pair differences match the original rounded-to-9-decimal report fields.
Mean treatment-minus-control final distance is **+0.5436998409 m**, median
**+0.0777742410 m**. Each endpoint is at that method's own terminal source;
these are not matched-elapsed-time or matched-state comparisons.

| Formal-window mechanism | Control | Treatment |
| --- | ---: | ---: |
| Candidate decisions | 945 | 945 |
| Runtime trace records | 4689 | 4698 |
| Executed-stop records | 3118 | 3788 |
| Backup revalidation failed | 2046 | 1201 |
| Backup stop requested | 896 | 2425 |
| Transaction deadline drop | 176 | 162 |
| Treatment selection changes | 0 | 504 |

All saved treatment eligibility vectors were independently reconstructed
from the true backup masks and unrounded full-preview clearances, including
the stop exception and empty-set fallback. Masks agree with the saved
backup-clearance signs; choices remain inside eligibility or explicitly
fall back to stop. The 504 changes, moving-backup sum 4833 and moving-eligible
sum 719 match exactly. This verifies recorded decision structure, not a
fresh rerun of all planner geometry or analytic-prior calculations.

Trace-window counting uses `[gate, terminal)`. Candidate timestamps are
decision-visible clock time; runtime trace timestamps are the prepublication
end clock, not the driver's own ingress source. Executed-stop fractions
0.664960546 and 0.806300553 are fractions of recorded trace messages, not
time stopped. The stop total overlaps its reason subclasses. The largest
observed trace gap is 134 ms and candidate gap 290 ms; no sample-regularity
or causal explanation of task failure is inferred from the counts.

All four b04/b05 head-on episodes remain engineering-clearance failures.
Saved scoped swept minima are approximately −0.1646891162, −0.1645462950,
−0.1643676676 and −0.1648170819 m. Independently aggregating the saved
44000 two-millisecond variation budgets and endpoint clearances reproduces
all 4400 coarse bounds within 2.23e-16 m. This checks the reported reduction
and signs; it does not independently rederive compound collision geometry
or body displacement budgets from the raw dynamics. Zero recorded illegal
contact is confirmed against the hash-bound assessment summaries, not by a
new contact-topic replay, and does not erase the four negative clearances.
Thus neither method passes every scoped safety row. PNG visual review,
native process-map semantic review, and a full physical-assessor replay
were not repeated here.

The durable `navigation_n05_manager_return.json` records return code 0,
matching the manager stdout and terminal ledger. Source inspection places
that write after `execute()` returns and before the Python entry point
finishes. Each of the 24 episode receipts separately reports child exit 0.
The saved shell wrapper would write `navigation_n05_outer_exit_code.txt`
after the Python command; that file is absent. Parent `/root` observed
`write_stdin(session_id=1158)` return exit code 143 and saved a provenance
note. No original independently accessible tool-response artifact exists,
so 143 is explicitly a parent tool observation, not an independently
recovered filesystem fact. These records establish manager logical return
and episode closure, not a normal host-shell exit. No signal sender,
termination cause or manager-exit failure is inferred.

The r04 preflight review is bound for its stated scope: 56 synthetic
fixtures, two real receipts/assessments and six candidate records. Its
earlier negative revisions remain preserved. The present all-bag task and
mechanism replay adds the full-matrix coverage described above, without
claiming to repeat that prior source audit or all physical qualification.

The additional interpretation report matches all method totals, endpoint
statistics, four negative clearance records, 504 selection changes, exit
limitations and the no-promotion decision. The evidence does not support
promoting N05 preview eligibility into training/evaluation policy, claiming
performance improvement, general or formal safety, live readiness, learned
benefit, deployment or real-vehicle readiness. Historical identities and
the budget remain terminal.
