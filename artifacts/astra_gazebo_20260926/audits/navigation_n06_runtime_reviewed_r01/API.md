# N06 reviewed disk envelope and candidate roster

This is an offline derived envelope, not a release. Original author r01/r02 and
the independent negative result remain unchanged. `closure.json` changes only
review lineage/status/limits; the 5217 runtime bindings and fixed environment
remain exact. The independent increment report qualifies disk dependencies only.

`probe_runtime_interface.py` calls the actual frozen r05 `verify_release` using
temporary author-test scaffolding. Its runtime document, resource bytes, Python
identity, environment and canonical executable checks are real. The separate
source/native/converter scaffolding is explicitly synthetic. A positive probe
therefore does not qualify a complete release or live runtime.

`build_candidate_roster.py --reviewed-runtime ABS/closure.json --output NEW_DIR`
creates a read-only candidate `resources.json` and immutable baseline snapshot.
It never changes the live registry, reserves identities, creates a release or
starts any ROS process. It requires that all proposed identities remain unused.
The final independent `source_review` must include every path/SHA in
`source_config_roster`; its own JSON is added separately by root afterwards.
All `required_manager_paths`, preparation, conversion, predecessor, qualification,
engine evidence and runtime resources are included. No live administrative CSV
is a release resource. Historical source review is referenced for engine evidence
only and is not reused as N06 stage approval.

Root owns the subsequent release constructor. The future actual preflight must
use the restricted entry documented in `navigation_n06_manager_r05/API.md`,
including the outer `env -i`, rather than an unrestricted Python invocation.
The exact final source review, native checks, conversion authentication and
environment/registry gates must then all pass with no test stand-ins.

GAZEBORC `.do_not_use_gazeborc` absence is a current disk observation. There is no
dedicated absence gate in the frozen manager and no native SystemPaths execution
in this review. Future live checks still need the original actual mapped-ELF and
source/native closure; this envelope adds no new live absence guarantee. The
unused exported model/media trees are outside qualification because the exact
prepared inputs contain no external model/media URI.
