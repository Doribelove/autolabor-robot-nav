"""Reuse deterministic release semantics only while every bound byte is unchanged.

This helper does not allocate identities, execute a process, or grant authority.
The manager's full verifier runs once. Each later call still opens and hashes
every release resource, control document and declared prerequisite, and rebuilds
the active path roster. Only repeated parsing/reconstruction of *all other*
prepared episodes is avoided. A caller must retain its existing one-shot ledger,
per-episode runtime inventories and terminal evidence checks.

Not wired into frozen N04. Use in a new stage requires independent review.
"""
from copy import deepcopy
from pathlib import Path


class UnchangedReleaseSession:
    def __init__(self, manager, release_path, release_sha256):
        self._manager = manager
        self._q = manager.evidence
        self._release_binding = dict(path=str(Path(release_path).resolve()),
                                     sha256=release_sha256)
        self._closed = False
        release, prepared, review = manager.verify_release(release_path, release_sha256)
        self._values = deepcopy((release, prepared, review))
        bindings = {}

        def add(item):
            path = str(Path(item['path']).resolve())
            if path != item['path']:
                raise ValueError('session requires canonical bound paths')
            binding = dict(path=path, sha256=item['sha256'])
            if path in bindings and bindings[path] != binding:
                raise ValueError('conflicting immutable resource bindings')
            bindings[path] = binding

        add(self._release_binding)
        for item in release['resources']:
            add(item)
        for key in ('prepared', 'source_review', 'passive_geometry_qualification'):
            add(release[key])
        # Full verification must already have closed these paths. Explicitly
        # retain them even if a manager permits redundant prerequisite bindings.
        import json
        qualification = json.loads(self._q.read_bound(release['passive_geometry_qualification']))
        for item in qualification['evidence_bindings']:
            add(item)
        for item in review['resources'] + review['engine_module_bindings'] + review['engine_source_bindings']:
            add(item)
        for key in ('native_engine_report', 'native_engine_records', 'upstream_source_manifest'):
            add(review[key])
        for unit in prepared['units']:
            for item in unit['files'] + unit['conversion_resources'] + [unit['conversion_manifest'], unit['converted_sdf']]:
                add(item)
        self._bindings = tuple(bindings[k] for k in sorted(bindings))
        if str(Path(__file__).resolve()) not in bindings or str(Path(manager.__file__).resolve()) not in bindings:
            raise ValueError('session helper and manager source must be release-bound')
        self._required = tuple(sorted(str(Path(p).resolve()) for p in manager.required_resources()))
        if len(self._required) != len(set(self._required)):
            raise ValueError('duplicate active resource roster')
        if any(path not in bindings for path in self._required):
            raise ValueError('active dependency outside immutable closure')
        self._schedule = tuple(tuple(row) for row in manager.identities())
        self.verify_unchanged()

    def verify_unchanged(self):
        """Hash all resources; any failure permanently closes this session."""
        if self._closed:
            raise RuntimeError('release validation session is permanently closed')
        try:
            for item in self._bindings:
                self._q.read_bound(item)
            roster = tuple(sorted(str(Path(p).resolve()) for p in self._manager.required_resources()))
            if roster != self._required:
                raise ValueError('active dependency roster changed')
            schedule = tuple(tuple(row) for row in self._manager.identities())
            if schedule != self._schedule:
                raise ValueError('active schedule changed')
        except BaseException:
            self._closed = True
            raise
        return deepcopy(self._values)

    @property
    def resource_count(self):
        return len(self._bindings)

    def close(self):
        self._closed = True
