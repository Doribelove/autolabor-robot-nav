# M12 lazy tiled KD retrieval: preserved negative factor result

This new isolated implementation preserves the tested M08 semantics, but it
**fails the prespecified combined factor criterion**. The sparse full-chain
case improves by 5.4524% at the paired median; the dense regression case slows
by 5.5766%, beyond the allowed 5%. The source, tile size, thresholds and trial
budget were not adjusted after seeing these results. No integration or live
qualification is claimed.

| Frozen input | M08 whole median / maximum ms | M12 whole median / maximum ms | Paired median change |
| --- | ---: | ---: | ---: |
| 1440 reobserved sparse returns + 2048 retained sparse supports | 51.705633 / 58.194465 | 48.876401 / 55.762028 | 5.4524% faster |
| 1440 returns on the original dense circular fixture | 35.133019 / 41.615454 | 37.072931 / 43.239917 | 5.5766% slower |

The sparse M12 case still exceeds 50 ms in 2 of 10 trials. The dense case is
below 50 ms in these 10 observations. Neither this sample nor a median below
50 ms is a live runtime or worst-case execution-time qualification. These are
unchanged analytic full-core synthetic fixtures, without ROS/TF callbacks,
actor inference, transport or live scheduling. The sparse case retains all
eight backup masks false and a zero command; it is not a navigation-success
or physical-safety fixture. All measurements, including high-latency rows,
are retained.

## Single implemented factor

`sensor_tracks_ingestion_m12.py` adds `IngestionSensorTracksM12` and a private
32-row lazy neighborhood cache. The exact M08 `accept` body is copied
byte-for-byte. The clustering traversal, visited/queue behavior, component
sorting and scalar connectivity predicate are AST-identical after normalizing
the two retrieval substitutions. The scalar boundary fallback remains
`np.linalg.norm` within 1e-12 m of the original gap. The original broad-phase
radius `gap + 1e-9`, `eps=0`, ascending neighbor order and single-worker query
are retained. No point, support, candidate, geometry query, receipt hash or
freshness/deadline check is removed.

On first access to row `i`, the cache queries the consecutive tile beginning
at `(i // 32) * 32`. It retains one tile and explicitly drops the previous
tile before allocating the next query result. At the frozen 1440-point limit,
the cached neighbor lists contain at most 32 × 1440 = 46080 indices; this is
not a byte bound for Python objects or native temporary storage. Traversal
can revisit tiles and fetch neighborhoods that are never used. Native query
failure propagates without committing a new tracking snapshot.

The separate profile exposes the tradeoff: sparse native KD calls fall from
1439 to 45, and profiled query time from roughly 4.62 to 1.01 ms. On the dense
fixture the original DFS needs only 123 scalar queries, while 45 full tiles
prefetch 1440 rows; profiled native query time rises from 0.61 to 2.61 ms.
Profile timing is instrumented and is excluded from the paired measurements.
This explains the observed direction of the regression; it is not a general
latency model. No conditional dense fallback or cache tuning is implemented.

## Validation and measurement boundary

Twenty tests pass. They include a separate all-pairs union-find reference,
inclusive threshold and adjacent-float cases at positive/negative large map
coordinates, duplicates, empty/singleton/sparse/connected/dense clouds, seeded
random inputs, interleaved components, ascending/random/reverse scalar-versus-
bulk KD rows, cache eviction/native failure, accepted history/ambiguity/lost
supports, and identical validation failures without snapshot commit. These
tests support exactness on the pinned backend and tested numerical domain;
they are not a proof of all floating-point inputs on all SciPy versions.

`protocol.json` was written before tests/timing. Each of the two unchanged
frozen benchmark inputs has 10 pairs: M08 then M12 on even pair indices and
M12 then M08 on odd indices. The sparse case runs first. Each trial has a fresh
core; constructor and prior retained-support accepts are excluded exactly as
in the old benchmark. Current scan acceptance, eight previews, eight backups,
selection and current backup recheck are measured. Thread counts are one;
there is no warmup deletion, outlier exclusion, repeated search or winner
selection. The primary criterion uses the median of paired fractional whole-
chain reductions; the ratio of the two medians is separately reported, with
the same pass/fail outcome here.

Every pair has equal per-field hashes for all snapshot fields except the
descriptive `processing_ns`, observation array dtype/shape/raw bytes, previews,
backups, full precision clearances, prediction audit, true masks, selection,
source stamps and command. Values are also deterministic across all pairs.
Full prediction outputs and masks retain the existing semantics. The separate
test sequences exercise future state evolution, not only a current snapshot.

The benchmark injects a new tracker into a newly constructed **local instance**
of the frozen core, before any scan acceptance, and adds explicit offline
implementation identity. It never replaces M08 module globals. This is a test
harness, not a new deployable ROS runtime, checkpoint identity or actor loader.
All protected frozen benchmark dependencies match before and after the run;
only the newly assigned M12 source/test/audit paths were written.

## Environment, artifacts and next boundary

The command sources frozen `workspace/scripts/astra_env.sh`, then explicitly
uses `.venv_train_cuda/bin/python -B`, with user site disabled and
OPENBLAS/OMP/MKL/NUMEXPR thread counts set to one. Actual imports are Python
3.8.10, NumPy 1.24.4, SciPy 1.10.1, Shapely 1.7.0, PyYAML 5.3.1 and pytest
4.6.9. The raw process maps, actually mapped native libraries, local imported
modules, package entry files, interpreter, CPU model, affinity and per-pair
load averages are retained. Torch was not imported and no GPU was used.
This is observed diagnostic dependency evidence, not a complete future
deployment release.

`paired_trials.jsonl` was flushed and fsynced after every pair. The full paired
reports, all phase timings, first-to-last semantic receipts, profile and logs
are preserved. The first test and benchmark invocations completed without
exceptions. The factor regression and 50 ms failures are retained negative
results, not hidden harness failures. Earlier M08 author-environment and
compatible-environment negative reports, including the original incompatible
venv import failure, remain unchanged and bound through their saved reports.

The component is frozen for independent review. It should not replace M08:
the preregistered dense regression gate failed. Any subsequent optimization is
a separately named factor with a new protocol, not an edit or retest of this
candidate. ROS/Gazebo, training, new scene execution, and existing N05 release
resources were not changed or invoked.
