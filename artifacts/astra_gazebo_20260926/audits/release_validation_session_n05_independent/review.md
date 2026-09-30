# N05 release-validation cache: independent review

The unintegrated component passes this scoped source review. Eight author tests independently reran successfully; eleven additional small checks passed. Nested qualification, review and conversion evidence remains hashed even when omitted from the fixture’s top-level resource list. Conflicting/noncanonical paths, unbound dependencies, missing helper, changed rosters and restored-after-failure bytes are rejected. Returned data cannot mutate the cached copy.

The inherited N04 full verifier was inspected as a bound-file semantic validator. No additional current file dependency or stateful runtime check was found that this helper silently omits. The new helper still hashes the complete release and explicit nested evidence on every call and rebuilds current dependency/schedule rosters.

This is not a generic cache for arbitrary validators. A deliberate external-state fixture demonstrates that changing an unbound authorization flag can make a fresh full verifier reject while the cache returns its prior value. Future integration must preserve the pure, deterministic manager contract and leave ledger, process, service, environment/liveness and terminal checks outside cached work. A permanently closed session must not be replaced to resume its failed matrix.

The actual frozen N04 release does not bind this helper, so it cannot be used there. A new stage must bind both helper and manager and independently review integration at the original validation boundaries. No release, ledger, source or live run was changed. No performance benchmark or speedup claim was made.
