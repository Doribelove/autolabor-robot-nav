"""Small fault tests; no ROS, simulator, training or performance benchmark."""
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import release_validation_session_n05 as candidate


@pytest.fixture
def session_fixture(tmp_path):
    def bind(path):
        return dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest())

    def read_bound(item):
        path=Path(item['path'])
        if path.is_symlink():raise ValueError('symlink')
        data=path.read_bytes()
        if hashlib.sha256(data).hexdigest()!=item['sha256']:raise ValueError('changed bound bytes')
        return data

    def document(name,value):
        p=tmp_path/name;p.write_text(json.dumps(value));return bind(p)

    source=document('runner.py', 'fixture manager source')
    runtime=document('runtime.py', 'fixture runtime source')
    native=document('native.json',{})
    helper=bind(Path(candidate.__file__))
    prepared={'units':[dict(files=[runtime],conversion_resources=[native],conversion_manifest=native,converted_sdf=native)]}
    review=dict(resources=[source,runtime,helper],engine_module_bindings=[],engine_source_bindings=[],
                native_engine_report=native,native_engine_records=native,upstream_source_manifest=native)
    qualification=document('qualification.json',dict(evidence_bindings=[native]))
    release=dict(resources=[source,runtime,native,helper],prepared=document('prepared.json',prepared),
                 source_review=document('review.json',review),passive_geometry_qualification=qualification)
    release_binding=document('release.json',release)
    calls=[];roster=[Path(x['path']) for x in (source,runtime,helper)];schedule=[('fresh_name',123,'method')]

    def full_verify(path,digest):
        calls.append((str(path),digest));read_bound(dict(path=str(path),sha256=digest))
        return release,prepared,review

    manager=SimpleNamespace(evidence=SimpleNamespace(read_bound=read_bound),verify_release=full_verify,
                            required_resources=lambda:roster,identities=lambda:schedule,__file__=source['path'])
    session=candidate.UnchangedReleaseSession(manager,release_binding['path'],release_binding['sha256'])
    return SimpleNamespace(session=session,calls=calls,roster=roster,schedule=schedule,runtime=runtime,
                           release_binding=release_binding,release=release,prepared=prepared,review=review)


def test_full_semantics_run_once_while_every_byte_is_rehashed(session_fixture,monkeypatch):
    f=session_fixture;original=f.session._q.read_bound;seen=[]
    monkeypatch.setattr(f.session._q,'read_bound',lambda row:(seen.append(row['path']),original(row))[1])
    a=f.session.verify_unchanged();b=f.session.verify_unchanged()
    assert a==b==(f.release,f.prepared,f.review)
    assert len(f.calls)==1 and len(seen)==2*f.session.resource_count
    assert f.runtime['path'] in seen and f.release_binding['path'] in seen


def test_caller_cannot_mutate_cached_semantics(session_fixture):
    f=session_fixture;result=f.session.verify_unchanged();result[0]['resources'].clear();result[1]['units'].clear()
    again=f.session.verify_unchanged();assert again==(f.release,f.prepared,f.review)


def test_changed_resource_permanently_closes_even_after_bytes_restored(session_fixture):
    f=session_fixture;p=Path(f.runtime['path']);original=p.read_bytes();p.write_text('changed')
    with pytest.raises(ValueError):f.session.verify_unchanged()
    p.write_bytes(original)
    with pytest.raises(RuntimeError):f.session.verify_unchanged()


def test_unbound_added_source_rejected(session_fixture,tmp_path):
    f=session_fixture;f.roster.append(tmp_path/'late_import.py')
    with pytest.raises(ValueError,match='roster'):f.session.verify_unchanged()


def test_removed_required_source_rejected(session_fixture):
    f=session_fixture;f.roster.pop()
    with pytest.raises(ValueError,match='roster'):f.session.verify_unchanged()


def test_schedule_mutation_rejected(session_fixture):
    f=session_fixture;f.schedule.append(('foreign',124,'method'))
    with pytest.raises(ValueError,match='schedule'):f.session.verify_unchanged()


def test_release_change_rejected(session_fixture):
    f=session_fixture;Path(f.release_binding['path']).write_text('{}')
    with pytest.raises(ValueError):f.session.verify_unchanged()


def test_explicit_close_rejects_all_reuse(session_fixture):
    f=session_fixture;f.session.close()
    with pytest.raises(RuntimeError):f.session.verify_unchanged()
