"""Lossless administrative plan and reviewed, recoverable three-table commit.

Publishing the transaction directory is atomic. Updating the three original
CSV paths is NOT jointly atomic: durable snapshots/hash checks permit roll-
forward after a partial commit. No automatic CSV write belongs in execution.
"""
import csv
import fcntl
import io
import json
import os
from pathlib import Path
import shutil
import tempfile

import navigation_n06_integrity_r05 as q

FIELDS={
 'seed_registry.csv':q.REGISTRY_FIELDS,
 'run_manifest.csv':('run_id','pid','process_group','command_json','cwd','started_utc','ended_utc','exit_code','heartbeat_path','receipt_path','bindings_sha256'),
 'EXPERIMENTS.csv':('run_id','split','candidate_id','method','scene_id','layout_id','seed','training_seed','status','started_utc','ended_utc','exit_code','code_sha256','config_sha256','model_sha256','report_path','raw_path')}

def records(data,fields):
    text=data.decode('utf-8');lines=text.splitlines(keepends=True);reader=csv.reader(io.StringIO(text,newline=''));header=next(reader)
    if tuple(header)!=tuple(fields):raise ValueError('administrative header differs')
    offset=reader.line_num;head=''.join(lines[:offset]).encode();out=[]
    for values in reader:
        end=reader.line_num
        if len(values)!=len(fields):raise ValueError('malformed administrative CSV')
        out.append((dict(zip(fields,values)),''.join(lines[offset:end]).encode()));offset=end
    return head,out

def row_bytes(row,fields,newline='\r\n'):
    if set(row)!=set(fields):raise ValueError('complete administrative row required')
    out=io.StringIO(newline='');csv.DictWriter(out,fieldnames=fields,lineterminator=newline).writerow(row);return out.getvalue().encode()

def projected_tables(before,ledger,receipts,release_sha):
    if ledger.get('stage')!=q.NAME or ledger.get('status')!='terminal' or ledger.get('release_sha256')!=release_sha:raise ValueError('exact terminal ledger required')
    rows=ledger['units'];schedule=q.prepared_api.schedule()
    if len(rows)!=8 or any(any(not q.same_typed(row[k],s[k]) for k in s) for row,s in zip(rows,schedule)):raise ValueError('terminal ordered identities differ')
    statuses=[r['status'] for r in rows];consumed=statuses.count('consumed')
    if statuses!=['consumed']*consumed+['retired_unrun']*(8-consumed):raise ValueError('consumed prefix/retired tail required')
    if not q.same_typed(ledger.get('maximum_units'),8) or not q.same_typed(ledger.get('consumed_units'),consumed):raise ValueError('terminal budget totals differ')
    names={r['run_id'] for r in rows};tables={};parsed={}
    for name,fields in FIELDS.items():parsed[name]=records(before[name],fields)
    registry=parsed['seed_registry.csv'][1];selected=[r for r,_ in registry if r['run_id'] in names]
    if len(selected)!=8 or len({r['run_id'] for r in selected})!=8:raise ValueError('exact eight unique reserved registry rows required')
    for row in rows:
        own=next(r for r in selected if r['run_id']==row['run_id'])
        expected=dict(namespace='astra_gazebo_20260926',split='development',purpose='n06_static_lookahead_pilot',
            seed=str(row['seed']),run_id=row['run_id'],status='reserved_exact_n06_release',source_binding=release_sha)
        if own!=expected:raise ValueError('reservation changed before projection')
    if any(r['seed'] in {str(u['seed']) for u in rows} and r['run_id'] not in names for r,_ in registry):raise ValueError('foreign paired seed reservation')
    for table in ('run_manifest.csv','EXPERIMENTS.csv'):
        if any(r['run_id'] in names for r,_ in parsed[table][1]):raise ValueError('N06 administrative identity already projected without this transaction')
    updates={};run_rows=[];experiment_rows=[];first_failed=None
    for index,row in enumerate(rows):
        name=row['run_id'];rec=receipts.get(name)
        if row['status']=='consumed':
            if rec is None:raise ValueError('consumed receipt identity missing')
            q.validate_terminal_receipt(rec,row,release_sha)
            if not q.same_typed(row.get('integrity_pass'),rec['integrity_pass']) or not q.same_typed(row.get('task_outcome'),rec['task_outcome']):raise ValueError('ledger and receipt outcomes disagree')
            if first_failed is not None:raise ValueError('consumed after infrastructure failure')
            if not rec['integrity_pass']:first_failed=index
            status='terminal_'+str(rec['task_outcome']) if rec['integrity_pass'] else 'terminal_integrity_failure'
            updates[name]='consumed_terminal_n06' if rec['integrity_pass'] else 'consumed_integrity_failure_n06'
            meta=rec.get('administrative_metadata',{})
        else:
            if rec is not None:raise ValueError('retired identity has receipt')
            updates[name]='retired_unrun_after_terminal_n06';status='retired_unrun_after_terminal_stop';meta={}
        run={k:'' for k in FIELDS['run_manifest.csv']};run.update(run_id=name)
        for k in run:
            if k!='run_id' and k in meta:run[k]=str(meta[k])
        run_rows.append(run)
        exp={k:'' for k in FIELDS['EXPERIMENTS.csv']}
        exp.update(run_id=name,split='development',candidate_id=row['method'],method=row['method'],scene_id=row['scene_id'],layout_id=row['layout_id'],seed=str(row['seed']),status=status)
        for key in ('started_utc','ended_utc','exit_code','code_sha256','config_sha256','model_sha256','report_path','raw_path'):
            if key in meta:exp[key]=str(meta[key])
        experiment_rows.append(exp)
    head,old=parsed['seed_registry.csv'];parts=[head]
    for row,raw in old:
        if row['run_id'] in updates:
            row=dict(row,status=updates[row['run_id']]);parts.append(row_bytes(row,FIELDS['seed_registry.csv'],'\r\n' if raw.endswith(b'\r\n') else '\n'))
        else:parts.append(raw)
    tables['seed_registry.csv']=b''.join(parts)
    for table,newrows in [('run_manifest.csv',run_rows),('EXPERIMENTS.csv',experiment_rows)]:
        if not before[table].endswith(b'\n'):raise ValueError('table must end with newline')
        tables[table]=before[table]+b''.join(row_bytes(r,FIELDS[table]) for r in newrows)
    return tables

def publish_plan(stage,transaction,ledger_binding,receipt_bindings,extra_bindings):
    """Write a new complete plan directory, never touch the three CSVs."""
    stage,transaction=Path(stage).resolve(),Path(transaction).resolve()
    if transaction.exists():raise FileExistsError('retain earlier transaction; use commit recovery')
    ledger=q.json_bound(ledger_binding);receipts={}
    resources=[ledger_binding]+list(extra_bindings)
    for row in ledger['units']:
        if row['status']=='consumed':
            item=receipt_bindings.get(row['run_id'])
            if item!=row['receipt']:raise ValueError('receipt is not ledger-bound')
            receipts[row['run_id']]=q.json_bound(item);resources.append(item)
            resources.extend(q.validate_terminal_receipt(receipts[row['run_id']],row,ledger['release_sha256']))
    for item in resources:q.read_bound(item)
    before={name:q.regular_bytes(stage/name) for name in FIELDS}
    after=projected_tables(before,ledger,receipts,ledger['release_sha256'])
    temporary=Path(tempfile.mkdtemp(prefix='.n06-admin-plan-',dir=str(transaction.parent)))
    try:
        table_rows=[]
        for name in FIELDS:
            for label,data in (('before',before[name]),('after',after[name])):q.publish(temporary/(label+'_'+name),data)
            table_rows.append({'target':str(stage/name),'before_file':'before_'+name,'after_file':'after_'+name,
                'before_sha256':q.digest(before[name]),'after_sha256':q.digest(after[name])})
        manifest={'schema':'astra_navigation_n06_administrative_plan_v1','stage':q.NAME,'release_sha256':ledger['release_sha256'],
            'tables':table_rows,'resources':resources,'three_original_paths_jointly_atomic':False,
            'requires_separate_independent_terminal_and_plan_review':True,'execution_authorized':False}
        q.publish(temporary/'manifest.json',manifest);q.fsync_dir(temporary)
        os.rename(str(temporary),str(transaction));q.fsync_dir(transaction.parent)
        return q.binding(transaction/'manifest.json')
    except BaseException:
        # Published plan survives; unpublished staging is also retained for
        # diagnosis and can never be mistaken for a committed transaction.
        raise

def apply_reviewed_plan(manifest_binding,review_binding):
    """Future explicit administrative commit; safe to reenter after a crash.

    Independent review is mandatory. This function is not called by manager.
    Root must remain sole writer of these original tables; this lock protects
    cooperating N06 commits, not unrelated writers.
    """
    manifest=q.json_bound(manifest_binding);review=q.json_bound(review_binding)
    expected={'schema':'astra_navigation_n06_admin_review_v1','stage':q.NAME,'status':'independently_reviewed',
        'plan':manifest_binding,'terminal_evidence_review_pass':True,'administrative_apply_authorized':True}
    if not q.same_typed(review,expected):raise ValueError('exact independently reviewed administrative authorization required')
    if manifest.get('schema')!='astra_navigation_n06_administrative_plan_v1' or manifest.get('stage')!=q.NAME or len(manifest.get('tables',[]))!=3:raise ValueError('closed three-table plan required')
    if set(manifest)!={'schema','stage','release_sha256','tables','resources','three_original_paths_jointly_atomic','requires_separate_independent_terminal_and_plan_review','execution_authorized'} or manifest['three_original_paths_jointly_atomic']is not False or manifest['execution_authorized']is not False:raise ValueError('plan scope or authority changed')
    if {Path(row['target']).name for row in manifest['tables']}!=set(FIELDS) or len({str(Path(row['target']).parent) for row in manifest['tables']})!=1:raise ValueError('exact co-located administrative targets required')
    if {row['target'] for row in manifest['tables']}!={str(q.STAGE/name) for name in FIELDS}:raise ValueError('exact canonical stage administrative targets required')
    for row in manifest['tables']:
        name=Path(row['target']).name
        if set(row)!={'target','before_file','after_file','before_sha256','after_sha256'} or row['before_file']!='before_'+name or row['after_file']!='after_'+name:raise ValueError('closed table snapshot mapping required')
    directory=Path(manifest_binding['path']).parent
    with (directory/'commit.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        def recheck_inputs():
            # Include the review/plan themselves; a crash or source drift after
            # an earlier table replacement must never publish completion.
            q.read_bound(manifest_binding);q.read_bound(review_binding)
            for item in manifest['resources']:q.read_bound(item)
        recheck_inputs()
        targets=[]
        for row in manifest['tables']:
            target=Path(row['target']);before=q.regular_bytes(directory/row['before_file']);after=q.regular_bytes(directory/row['after_file'])
            if q.digest(before)!=row['before_sha256'] or q.digest(after)!=row['after_sha256']:raise ValueError('transaction snapshot corrupted')
            actual=q.digest(q.regular_bytes(target))
            if actual not in (row['before_sha256'],row['after_sha256']):raise ValueError('unrecognized third table version')
            targets.append((target,row,after,actual))
        completion={'schema':'astra_navigation_n06_admin_completion_v1','stage':q.NAME,'plan':manifest_binding,'review':review_binding,
            'after':[{'path':str(target),'sha256':row['after_sha256']} for target,row,_,_ in targets],
            'three_original_paths_jointly_atomic':False,'recovery':'durable before/after journal, deterministic roll-forward'}
        done=directory/'completion.json'
        if done.exists():
            if q.regular_bytes(done)!=q.json_bytes(completion) or any(actual!=row['after_sha256'] for _,row,_,actual in targets):raise ValueError('completed transaction changed')
            recheck_inputs();return completion
        for target,row,after,actual in targets:
            # Recheck immediately before each replacement. The all-target
            # precheck above prevents overwriting after known foreign edits.
            recheck_inputs();actual=q.digest(q.regular_bytes(target))
            if actual not in (row['before_sha256'],row['after_sha256']):raise ValueError('concurrent administrative writer')
            if actual==row['before_sha256']:q.publish(target,after,replace=True)
            recheck_inputs()
            q.publish(directory/'progress.json',{'last_verified_target':str(target),'after_sha256':row['after_sha256']},replace=True)
        if any(q.digest(q.regular_bytes(target))!=row['after_sha256'] for target,row,_,_ in targets):raise ValueError('post-commit table changed')
        recheck_inputs()
        q.publish(done,completion);return completion
