# M08 compatible-environment narrow latency diagnosis

The exact retained sparse negative remains negative in the compatible
interpreter. Ten frozen full-core trials have median **51.594417 ms**, P95
**57.88787855 ms**, maximum **57.912548 ms**; all 10 exceed 50 ms. The historical
author environment's maximum **69.239986 ms** is retained unchanged. Different
environments and measurement times are not a paired optimization experiment,
so the lower maximum is not attributed to any particular package change.

Only `combined1440_sparse_reobserved_and2048_retained` was rerun: the unchanged
benchmark's `bench` function, its 10 trials, 400×400 synthetic grid, straight
route, source stamps, pose, 2048 prior sparse supports, 1440 reobserved points,
and actual saved nine-component robot URDF. Runtime construction and prior
support ingestion remain outside the measured interval, exactly as in the
frozen benchmark. The synthetic six-by-72 history is a fixture, not real scans.
No ROS callbacks, exact TF transport, actor inference or live scheduling are
included. This is neither a live transaction measurement nor a WCET result.

| Uninstrumented phase | Median ms | P95 ms | Maximum ms |
| --- | ---: | ---: | ---: |
| Current scan accept | 18.471630 | 22.093725 | 24.941904 |
| Eight previews + eight backups + features | 29.080047 | 29.518640 | 29.677550 |
| Selection + current backup recheck | 3.911840 | 7.526833 | 10.333144 |
| Whole measured chain | 51.594417 | 57.887879 | 57.912548 |

Medians/maxima from different trials are not additive. Every raw phase timing
is retained in `reproduction.json`; no outliers were removed. At the maximum
whole trial the three phases were 24.941904, 29.091845 and 3.878799 ms.

The unchanged scalar-v1 geometric reference agrees conservatively over all
491 trajectory queries (248 preview, 243 backup): maximum batch-minus-v1 is
−9.99991200956174e−11 m, maximum absolute difference 1.000008964524568e−10 m.
There are nine physical footprint components, 16 support planes, 3488 declared
support terms and 2048 evaluated persistent terms after the existing exact
dominance removal. This stress input has **all eight backup masks false**;
selection/command is zero with `backup_revalidation_failed`, not a navigable
or physically safe success fixture. No mask or support was removed to speed it.

## Hotspots, from a separate instrumented run

A single additional call per phase was profiled with cProfile. These timings
include instrumentation overhead and must not be substituted for the 10 raw
measurements. Inclusive call times overlap and must not be summed as disjoint
costs. A fresh unprofiled direct call and the profiled call have identical
snapshot/observation/rollout/mask/clearance/command hashes after excluding only
the tracker's descriptive `processing_ns` statistic.

In accept, `_clusters` took about 9.01 ms inclusive and static-map classification
about 8.19 ms inclusive in the instrumented call. There are **1439 separate
`cKDTree.query_ball_point` calls**, 4.76 ms inclusive, for isolated returns.
Static-map classification calls `clearance` 1440 times, including repeated
scalar shape/finite checks and coordinate conversions. JSON receipt hashing
is also retained (about 1.40 ms inclusive), because dropping provenance is not
an optimization option.

In build, 16 rollout calls took about 20.20 ms inclusive; their 475 steps include
route projection and kinematics. `verify_many` took about 18.81 ms inclusive,
including roughly 9.05 ms dynamic batch distance work and 8.69 ms static
footprint lookup/construction. These profile totals exceed the uninstrumented
phase and expose relative work, not calibrated wall-time fractions. The current
command repeats occupancy construction (about 3.25 ms inclusive), but avoiding
that work would require additional snapshot/time/cache invariants and is not
the single factor proposed below.

## One proposed, falsifiable factor: bounded batched KD neighborhoods

Propose a new isolated ingestion helper that changes **only retrieval of the
existing exact KD broad-phase neighbor lists**. Keep the old module frozen.
Predeclare 32 consecutive query rows per lazy tile, one cached tile at a time,
single worker, `eps=0`, `return_sorted=True`, and the unchanged radius
`cluster_gap_m + 1e−9`. On demand, issue one vectorized `query_ball_point` for
that tile and use its row for the original current point. Preserve the original
visited/queue/component ordering and the exact scalar connectivity predicate,
including `np.linalg.norm` within the existing 1e−12 m boundary band. No new
threshold, approximation, frame coalescing or point reduction is introduced.

The current sparse sequence would replace roughly 1439 Python-to-native query
calls by at most 45 tile calls. This is a hypothesis about dispatch overhead,
not a measured speedup. Dense or interleaved graph traversals can perform extra
unused/prefetched queries or thrash the one-tile cache and could regress; that
possibility must be tested and retained. Tile size is fixed before any such
comparison, not searched until timing passes. At the existing maximum 1440
points, the cached tile has at most 32×1440=46080 neighbor indices; this count
bound does not claim a fixed byte size for Python objects. No unbounded all-pairs
neighborhood cache is allowed.

The equivalence argument is narrow: the same tree, immutable points, radius,
sorted exact query and original scalar predicate induce the same adjacency
graph; unchanged traversal yields identical connected components and ordering.
Availability/source-time checks, snapshot commit, retained supports, ambiguity,
CV diagnostics, map classification, JSON receipts and every downstream
candidate/backup calculation remain unchanged. Bulk-query equivalence itself
must be independently tested on the actual pinned SciPy backend, including
threshold equality/adjacent floats, duplicates, chains, singletons, dense
components and interleaved traversal. Do not replace this with an approximate
radius search or change the 200/20/50 ms gates.

A future bounded offline check should preregister 10 paired cold-core trials
per implementation with a fixed alternating order, the same 1440/2048 input
and setup exclusion as here, every result retained. Primary factor criterion:
at least 5% reduction in paired median full-chain time, with byte-identical
policy inputs, masks, support receipts and commands (descriptive processing
time excluded). Add the existing 1440-dense fixture as a prespecified regression
check; more than 5% median full-chain regression rejects this candidate. This
is a proposed test protocol, not an authorization or achieved result. The
unchanged full-chain 50 ms gate remains separate: even a factor-level improvement
that still exceeds 50 ms is a retained runtime negative, not qualification.
Ten measurements cannot establish a worst-case execution-time bound.

No optimization source was implemented or benchmarked in this diagnosis.
All old M08/N05 files remain unchanged. New method/environment closure and
independent review would precede any future integration. A learned actor can
add inference time; this analytic full-core test has no actor and cannot qualify
the learned complete chain.

## Environment and failures retained

Command: source the frozen `workspace/scripts/astra_env.sh`, then explicitly
invoke `workspace/.venv_train_cuda/bin/python -B reproduce.py` with user site
disabled and OPENBLAS/OMP/MKL/NUMEXPR thread counts set to 1. This does not
activate CUDA: Torch was not imported and no GPU operation was issued.

Observed versions: Python 3.8.10, NumPy 1.24.4, SciPy 1.10.1, Shapely 1.7.0,
PyYAML 5.3.1 and pytest 4.6.9; CPU is Intel Core i7-14650HX. `environment.json`
binds the interpreter, package entry files, loaded local source modules and 89
actually mapped shared libraries, and retains `/proc/self/maps`, selected thread
settings and affinity. Lock/manifest files are additionally bound in the final
report. This is observed diagnostic dependency evidence, not a complete future
execution release or proof of every arbitrary installed package's provenance.

The original unsourced author environment was Python 3.8.10 with user-site
NumPy 1.21.0 and system SciPy 1.3.3. Its historical timing remains separate.
The existing `workspace/.venv` NumPy1.24/system-SciPy1.3.3 `np.typeDict` import
failure remains bound from the earlier independent review; that environment
was neither changed nor reclassified as compatible.

The primary 10-trial timing and geometric reference completed on the first
attempt. Two subsequent **audit serialization** defects were retained: the
first profiling wrapper used `stats` instead of the actual `statistics` field;
the next hit NumPy int64 JSON serialization. Old wrapper bytes and both error
logs remain. Only this new audit wrapper was repaired. The valid 10-trial
result was not rerun or overwritten; `--profile-only` completed the remaining
diagnostic and environment evidence. These are harness failures, not failures
of the frozen M08 core, and are not hidden by the final report.
