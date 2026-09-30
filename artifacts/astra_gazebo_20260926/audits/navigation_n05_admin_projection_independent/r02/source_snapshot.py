#!/usr/bin/env python3
"""Recoverable, failure-inclusive N05 administrative projection; no simulation.

Only this writer may modify the three tables during closure. A durable immutable
transaction records every before/after byte string before the first replacement.
Reentry accepts only those two versions of each table; other edits fail closed.
"""
from datetime import datetime, timezone
import csv
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import sys
import tempfile

sys.dont_write_bytecode = True
S = Path(__file__).resolve().parents[1]
SCHEMA = 'astra_navigation_n05_admin_transaction_r02'
REPLAY_SHA = '5554ddf0e5a956704ba5192a696ca2164ecb0c59535a3e5c56f5d01bc5f51cce'
TABLES = ('seed_registry.csv', 'run_manifest.csv', 'EXPERIMENTS.csv')
HEADERS = (
    ('namespace','split','purpose','seed','run_id','status','source_binding'),
    ('run_id','pid','process_group','command_json','cwd','started_utc','ended_utc','exit_code','heartbeat_path','receipt_path','bindings_sha256'),
    ('run_id','split','candidate_id','method','scene_id','layout_id','seed','training_seed','status','started_utc','ended_utc','exit_code','code_sha256','config_sha256','model_sha256','report_path','raw_path'),
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def regular_bytes(path):
    path = Path(path)
    if path.is_symlink() or not stat.S_ISREG(path.stat().st_mode):
        raise ValueError('regular nonsymlink file required: '+str(path))
    return path.read_bytes()


def binding(path):
    return dict(path=str(Path(path).resolve()), sha256=digest(regular_bytes(path)))


def json_bytes(value):
    return (json.dumps(value, indent=2, allow_nan=False)+'\n').encode()


def fsync_dir(path):
    fd = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def write_new(path, data, mode=0o444):
    fd = os.open(str(path), os.O_CREAT | os.O_EXCL | os.O_WRONLY, mode)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(data); stream.flush(); os.fsync(stream.fileno())


def load_dependencies(stage):
    sys.path.insert(0, str(stage/'workspace/scripts/astra_gazebo'))
    import run_navigation_n05 as manager
    path = stage/'audits/navigation_n05_terminal_replay_r04.py'
    data = regular_bytes(path)
    if digest(data) != REPLAY_SHA:
        raise ValueError('independently reviewed r04 receipt validator changed')
    namespace = {'__file__': str(path), '__name__': 'n05_admin_bound_receipt_validator'}
    exec(compile(data, str(path), 'exec'), namespace)
    if namespace['S'] != stage:
        raise ValueError('receipt validator stage root differs')
    return manager, namespace['validate_consumed_receipt'], [binding(path), binding(Path(manager.__file__))]


def inputs(stage, manager, validate_receipt, dependency_bindings):
    ledger_path = stage/'queue/navigation_n05_ledger.json'
    release_path = stage/'contracts/navigation_n05_release.json'
    design_path = stage/'contracts/navigation_n05_preview_selection_design.json'
    ledger_binding = binding(ledger_path); release_binding = binding(release_path)
    ledger = json.loads(regular_bytes(ledger_path))
    if ledger.get('status') != 'terminal' or ledger.get('stage') != manager.NAME:
        raise ValueError('exact terminal N05 ledger required')
    release, prepared, _ = manager.verify_release(release_path, release_binding['sha256'])
    if ledger.get('release_sha256') != release_binding['sha256']:
        raise ValueError('terminal release differs')
    design_binding = binding(design_path)
    release_resources = {str(Path(r['path']).resolve()): r['sha256'] for r in release['resources']}
    if release_resources.get(design_binding['path']) != design_binding['sha256']:
        raise ValueError('design absent from verified release')
    design = json.loads(regular_bytes(design_path)); schedule = design['schedule']
    units = ledger['units']
    identities = lambda rows: [(u['run_id'], u['seed'], u['method']) for u in rows]
    if (design.get('stage') != manager.NAME or len(units) != 24 or len(schedule) != 24 or
            len({u['run_id'] for u in units}) != 24 or
            identities(units) != identities(schedule) or
            identities(units) != identities(prepared['units']) or
            identities(units) != manager.identities()):
        raise ValueError('exact ordered24 identity schedule differs')
    if any(u['status'] not in ('consumed', 'retired_unrun') for u in units):
        raise ValueError('nonterminal unit status')
    consumed = sum(u['status'] == 'consumed' for u in units)
    if type(ledger.get('consumed_units')) is not int or ledger['consumed_units'] != consumed:
        raise ValueError('ledger consumed count differs')
    if [u['status'] for u in units] != ['consumed']*consumed+['retired_unrun']*(24-consumed):
        raise ValueError('one-shot consumed-prefix/retired-tail differs')
    resources = [ledger_binding, release_binding, design_binding]+list(dependency_bindings)
    receipts = {}; metadata = {}; failed_seen = False
    for unit, prepared_unit in zip(units, prepared['units']):
        root = stage/'runs'/unit['run_id']
        if unit['status'] == 'retired_unrun':
            if root.exists() or unit.get('receipt'):
                raise ValueError('retired identity has attempt evidence')
            continue
        if failed_seen:
            raise ValueError('consumed unit after terminal infrastructure failure')
        receipt = validate_receipt(unit, release_binding['sha256'])
        if type(receipt['attempt']) is not int or receipt['attempt'] != 1:
            raise ValueError('exact attempt1 required')
        receipts[unit['run_id']] = receipt
        resources.append(binding(Path(unit['receipt']['path'])))
        if receipt.get('report') is not None:
            resources.append(binding(Path(receipt['report']['path'])))
        failed_seen = receipt['integrity_pass'] is False
        artifacts = {}
        for item in receipt.get('artifacts', []):
            path = str(Path(item['path']).resolve())
            if path in artifacts:
                raise ValueError('duplicate receipt artifact path')
            artifacts[path] = item['sha256']
        values = {}
        for name in ('allocated.json', 'process.json', 'contract.json', 'collector_raw.json'):
            path = root/name
            if path.exists() or str(path.resolve()) in artifacts:
                item = binding(path)
                if artifacts.get(item['path']) != item['sha256']:
                    raise ValueError('unbound administrative metadata: '+str(path))
                resources.append(item); values[name] = json.loads(regular_bytes(path))
        if 'contract.json' in values:
            expected = prepared_unit['files'][0]
            if artifacts[str((root/'contract.json').resolve())] != expected['sha256']:
                raise ValueError('attempt contract differs from released prepared bytes')
        if receipt['integrity_pass'] is True and 'contract.json' not in values:
            raise ValueError('valid attempt missing contract')
        metadata[unit['run_id']] = values
    # Preserve exact bytes consumed during this validation; never snapshot a new
    # ledger version after checking the old one.
    for item in resources:
        if binding(Path(item['path'])) != item:
            raise ValueError('input changed during validation')
    return ledger, schedule, receipts, metadata, resources


def table(data, fields):
    reader = csv.DictReader(io.StringIO(data.decode(), newline=''))
    rows = list(reader)
    if reader.fieldnames != list(fields) or any(None in r or any(v is None for v in r.values()) for r in rows):
        raise ValueError('administrative CSV schema differs')
    return rows


def table_bytes(rows, fields):
    stream = io.StringIO(newline=''); writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader(); writer.writerows(rows)
    return stream.getvalue().encode()


def plan(stage, ledger, schedule, receipts, metadata, resources):
    before = [regular_bytes(stage/name) for name in TABLES]
    rows = [table(data, fields) for data, fields in zip(before, HEADERS)]
    reg_rows, run_rows, exp_rows = rows
    scheduled = {r['run_id']: r for r in schedule}
    if any(r['run_id'] in scheduled for r in run_rows+exp_rows):
        raise ValueError('N05 administrative rows already present without this transaction')
    relevant = [r for r in reg_rows if r['run_id'] in scheduled]
    release_sha = ledger['release_sha256']
    if (len(relevant) != 24 or len({r['run_id'] for r in relevant}) != 24 or
            any(r['status'] != 'reserved_exact_n05_release' or r['source_binding'] != release_sha for r in relevant)):
        raise ValueError('exact24 untouched reservations required')
    registry = {r['run_id']: r for r in relevant}
    new_run = []; new_exp = []; summaries = []; valid = failed = retired = 0
    runner_path = stage/'workspace/scripts/astra_gazebo/run_navigation_n05.py'
    runner_sha = binding(runner_path)['sha256']
    for unit in ledger['units']:
        name = unit['run_id']; root = stage/'runs'/name; reg = registry[name]
        if reg['seed'] != str(unit['seed']) or reg['split'] != 'development':
            raise ValueError('reservation seed/split differs')
        started = ended = exit_code = config = report = raw = code = ''
        summary = dict(run_id=name, seed=unit['seed'], method=unit['method'], status=unit['status'])
        if unit['status'] == 'retired_unrun':
            retired += 1; reg['status'] = 'retired_unrun_after_terminal_n05'; status = 'retired_unrun_after_terminal_stop'
        else:
            receipt = receipts[name]; meta = metadata[name]; receipt_path = Path(unit['receipt']['path'])
            started = meta.get('allocated.json', {}).get('started_utc', '')
            ended = datetime.fromtimestamp(receipt_path.stat().st_mtime, timezone.utc).isoformat()
            config = binding(root/'contract.json')['sha256'] if 'contract.json' in meta else ''
            report = receipt['report']['path'] if receipt.get('report') else ''
            raw = str(root/'collector_raw.json') if 'collector_raw.json' in meta else ''
            code = runner_sha; exit_code = receipt.get('exit_code')
            if 'process.json' in meta:
                process = meta['process.json']
                if any(type(process.get(k)) is not int or process[k] <= 0 for k in ('pid','pgid')):
                    raise ValueError('actual process identity malformed')
                new_run.append(dict(run_id=name,pid=process['pid'],process_group=process['pgid'],
                    command_json=json.dumps(['roslaunch',str(root/'fixture.launch')]),cwd=str(stage/'workspace'),
                    started_utc=started,ended_utc=ended,exit_code=exit_code,heartbeat_path=str(root/'heartbeat.json'),
                    receipt_path=str(receipt_path),bindings_sha256=release_sha))
            if receipt['integrity_pass']:
                valid += 1; reg['status'] = 'consumed_terminal_n05'; status = 'terminal_'+receipt['task_outcome']
            else:
                failed += 1; reg['status'] = 'consumed_integrity_failure_n05'; status = 'terminal_integrity_failure'
            summary.update(receipt=unit['receipt'],task_outcome=receipt['task_outcome'],error=receipt.get('error'),
                           integrity_pass=receipt['integrity_pass'],report=receipt.get('report'))
        row = scheduled[name]
        new_exp.append(dict(run_id=name,split='development',candidate_id=unit['method'],method=unit['method'],
            scene_id=row['scene_id'],layout_id=row['layout_id'],seed=unit['seed'],training_seed='',status=status,
            started_utc=started,ended_utc=ended,exit_code=exit_code,code_sha256=code,config_sha256=config,
            model_sha256='',report_path=report,raw_path=raw))
        summaries.append(summary)
    after = [table_bytes(rr, fields) for rr, fields in zip((reg_rows,run_rows+new_run,exp_rows+new_exp), HEADERS)]
    manifest = dict(schema=SCHEMA,stage=ledger['stage'],created_utc=datetime.now(timezone.utc).isoformat(),
        release_sha256=release_sha,input_bindings=resources+[binding(Path(__file__))],
        tables=[dict(name=name,before_sha256=digest(b),after_sha256=digest(a),mode=stat.S_IMODE((stage/name).stat().st_mode))
                for name,b,a in zip(TABLES,before,after)],
        consumed=valid+failed,valid_assessment=valid,integrity_failed=failed,retired_unrun=retired,
        run_manifest_rows=len(new_run),experiment_rows=len(new_exp),units=summaries,
        interpretation='Administrative projection only; failed and retired identities retained. No simulation retry/resume.',
        ended_utc_semantics='Receipt filesystem mtime observed during transaction planning; not exact process or physics completion time.')
    return manifest, before, after


def closure_bytes(manifest, manifest_binding):
    return json_bytes(dict(schema=SCHEMA,stage=manifest['stage'],status='administrative_terminal_projection',
        transaction_manifest=manifest_binding,release_sha256=manifest['release_sha256'],
        before={t['name']:t['before_sha256'] for t in manifest['tables']},
        after={t['name']:t['after_sha256'] for t in manifest['tables']},
        **{k:manifest[k] for k in ('consumed','valid_assessment','integrity_failed','retired_unrun','run_manifest_rows','experiment_rows','units','interpretation','ended_utc_semantics')}))


def commit(stage, txn, output, validated_resources):
    manifest_path = txn/'manifest.json'; manifest = json.loads(regular_bytes(manifest_path))
    if manifest.get('schema') != SCHEMA or [t['name'] for t in manifest['tables']] != list(TABLES):
        raise ValueError('transaction schema/targets differ')
    expected_inputs = validated_resources+[binding(Path(__file__))]
    if manifest['input_bindings'] != expected_inputs:
        raise ValueError('transaction original source/ledger/receipt bindings differ')
    snapshots = []
    for item in manifest['tables']:
        b = regular_bytes(txn/('before_'+item['name'])); a = regular_bytes(txn/('after_'+item['name']))
        if digest(b) != item['before_sha256'] or digest(a) != item['after_sha256']:
            raise ValueError('transaction snapshot changed')
        current = binding(stage/item['name'])['sha256']
        if current not in (item['before_sha256'],item['after_sha256']):
            raise ValueError('table differs from both transaction states: '+item['name'])
        snapshots.append(a)
    expected_closure = closure_bytes(manifest, binding(manifest_path))
    if output.exists():
        if regular_bytes(output) != expected_closure or any(binding(stage/t['name'])['sha256'] != t['after_sha256'] for t in manifest['tables']):
            raise ValueError('completed closure/table evidence differs')
        return json.loads(expected_closure), False
    for item, data in zip(manifest['tables'], snapshots):
        target = stage/item['name']; current = binding(target)['sha256']
        if current == item['after_sha256']:
            continue
        if current != item['before_sha256']:
            raise ValueError('table changed before replacement')
        fd, pending = tempfile.mkstemp(prefix='.'+item['name']+'.n05-r02-', dir=str(stage))
        with os.fdopen(fd,'wb') as stream:
            os.fchmod(stream.fileno(),item['mode']); stream.write(data); stream.flush(); os.fsync(stream.fileno())
        os.replace(pending,target); fsync_dir(stage)
    if any(binding(stage/t['name'])['sha256'] != t['after_sha256'] for t in manifest['tables']):
        raise ValueError('committed table hash differs')
    # Hard-link publication is exclusive and atomic: a crash cannot expose a
    # partial JSON closure. Leftover temporary files never block recovery.
    fd, pending = tempfile.mkstemp(prefix='.n05-closure-r02-',dir=str(output.parent))
    with os.fdopen(fd,'wb') as stream:
        stream.write(expected_closure); stream.flush(); os.fsync(stream.fileno())
    os.link(pending,output); fsync_dir(output.parent)
    os.unlink(pending); fsync_dir(output.parent)
    return json.loads(expected_closure), True


def close(stage=S, dependencies=None):
    stage = Path(stage).resolve(); queue = stage/'queue'
    txn = queue/'navigation_n05_admin_transaction_r02'
    output = queue/'navigation_n05_admin_closure_r02.json'
    lock = queue/'navigation_n05_admin_r02.lock'
    with lock.open('a') as lock_stream:
        fcntl.flock(lock_stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        manager, validator, dependency_bindings = load_dependencies(stage) if dependencies is None else dependencies
        ledger, schedule, receipts, metadata, resources = inputs(stage,manager,validator,dependency_bindings)
        if output.exists() and not txn.exists():
            raise ValueError('closure exists without transaction')
        if not txn.exists():
            manifest,before,after = plan(stage,ledger,schedule,receipts,metadata,resources)
            staging = Path(tempfile.mkdtemp(prefix='.navigation_n05_admin_r02-',dir=str(queue)))
            for name,b,a in zip(TABLES,before,after):
                write_new(staging/('before_'+name),b); write_new(staging/('after_'+name),a)
            write_new(staging/'manifest.json',json_bytes(manifest)); fsync_dir(staging)
            if txn.exists():
                raise FileExistsError('transaction concurrently appeared')
            os.rename(staging,txn); fsync_dir(queue)
        result, changed = commit(stage,txn,output,resources)
        print(json.dumps(dict(status=result['status'],consumed=result['consumed'],valid=result['valid_assessment'],
            retired=result['retired_unrun'],tables_written=changed,closure_sha256=binding(output)['sha256'])))
        return result


def main():
    close()


if __name__ == '__main__':
    main()
