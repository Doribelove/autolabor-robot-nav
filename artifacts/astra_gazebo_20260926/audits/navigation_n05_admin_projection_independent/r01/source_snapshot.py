"""Project terminal one-shot N05 ledger into durable administrative tables."""
from datetime import datetime, timezone
import csv
import hashlib
import json
import os
from pathlib import Path

S=Path(__file__).resolve().parents[1]
LEDGER=S/'queue/navigation_n05_ledger.json'
RELEASE=S/'contracts/navigation_n05_release.json'
DESIGN=S/'contracts/navigation_n05_preview_selection_design.json'
OUTPUT=S/'queue/navigation_n05_admin_closure.json'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def table(path):
    with path.open(newline='') as stream:
        reader=csv.DictReader(stream)
        return reader.fieldnames,list(reader)


def main():
    if OUTPUT.exists():
        raise FileExistsError('N05 administrative identity exists')
    release_sha=sha(RELEASE)
    ledger=json.loads(LEDGER.read_bytes())
    units=ledger['units']
    if ledger['status']!='terminal' or ledger['release_sha256']!=release_sha or len(units)!=24:
        raise ValueError('N05 terminal exact release/roster required')
    schedule={r['run_id']:r for r in json.loads(DESIGN.read_bytes())['schedule']}
    if set(schedule)!={u['run_id'] for u in units}:
        raise ValueError('N05 schedule differs from terminal roster')
    paths=[S/'seed_registry.csv',S/'run_manifest.csv',S/'EXPERIMENTS.csv']
    fields_rows=[table(path) for path in paths]
    (reg_fields,reg_rows),(run_fields,run_rows),(exp_fields,exp_rows)=fields_rows
    if any(r['run_id'] in schedule for r in run_rows+exp_rows):
        raise ValueError('N05 administrative rows already present')
    relevant=[r for r in reg_rows if r['run_id'] in schedule]
    if (len(relevant)!=24 or len({r['run_id'] for r in relevant})!=24 or
            any(r['status']!='reserved_exact_n05_release' or r['source_binding']!=release_sha for r in relevant)):
        raise ValueError('N05 exact reservations missing or already projected')
    by_id={r['run_id']:r for r in relevant}
    new_run=[];new_exp=[];valid=0;failed=0;retired=0
    for unit in units:
        name=unit['run_id']; root=S/'runs'/name; reg=by_id[name]
        if reg['seed']!=str(unit['seed']) or reg['split']!='development':
            raise ValueError('N05 seed/split differs')
        started=ended=exit_code=config=report=raw=code=''
        if unit['status']=='retired_unrun':
            retired+=1
            if root.exists() or unit.get('receipt'):
                raise ValueError('retired N05 identity has attempt root or receipt')
            reg['status']='retired_unrun_after_terminal_n05'
            status='retired_unrun_after_terminal_stop'
        elif unit['status']=='consumed':
            receipt_binding=unit.get('receipt')
            if not receipt_binding:
                raise ValueError('consumed N05 identity has no receipt binding')
            receipt_path=Path(receipt_binding['path'])
            if sha(receipt_path)!=receipt_binding['sha256']:
                raise ValueError('N05 receipt hash differs')
            receipt=json.loads(receipt_path.read_bytes())
            if (receipt['run_id'],receipt['seed'],receipt['method'],receipt['attempt'],receipt['release_sha256']) != (name,unit['seed'],unit['method'],1,release_sha):
                raise ValueError('N05 receipt identity differs')
            if receipt['integrity_pass'] is not unit.get('integrity_pass'):
                raise ValueError('N05 receipt integrity differs')
            allocated_path=root/'allocated.json'
            if allocated_path.is_file():
                started=json.loads(allocated_path.read_bytes())['started_utc']
            ended=datetime.fromtimestamp(receipt_path.stat().st_mtime,timezone.utc).isoformat()
            contract_path=root/'contract.json'
            config=sha(contract_path) if contract_path.is_file() else ''
            report=receipt['report']['path'] if receipt.get('report') else ''
            raw=str(root/'collector_raw.json') if (root/'collector_raw.json').is_file() else ''
            code=sha(S/'workspace/scripts/astra_gazebo/run_navigation_n05.py')
            exit_code=receipt.get('exit_code')
            process_path=root/'process.json'
            if process_path.is_file():
                process=json.loads(process_path.read_bytes())
                new_run.append(dict(run_id=name,pid=process['pid'],process_group=process['pgid'],
                                    command_json=json.dumps(['roslaunch',str(root/'fixture.launch')]),
                                    cwd=str(S/'workspace'),started_utc=started,ended_utc=ended,
                                    exit_code=exit_code,heartbeat_path=str(root/'heartbeat.json'),
                                    receipt_path=str(receipt_path),bindings_sha256=release_sha))
            if unit.get('integrity_pass') is True:
                if receipt['error'] is not None or receipt['task_outcome'] not in ('complete','timeout','fault','stopped'):
                    raise ValueError('N05 claimed valid task outcome differs')
                valid+=1;reg['status']='consumed_terminal_n05';status='terminal_'+receipt['task_outcome']
            else:
                failed+=1;reg['status']='consumed_integrity_failure_n05';status='terminal_integrity_failure'
        else:
            raise ValueError('N05 ledger status not terminal-consumed/retired')
        row=schedule[name]
        new_exp.append(dict(run_id=name,split='development',candidate_id=unit['method'],method=unit['method'],
                            scene_id=row['scene_id'],layout_id=row['layout_id'],seed=unit['seed'],training_seed='',
                            status=status,started_utc=started,ended_utc=ended,exit_code=exit_code,
                            code_sha256=code,config_sha256=config,model_sha256='',report_path=report,raw_path=raw))
    if valid+failed+retired!=24:
        raise ValueError('N05 ledger total differs')
    projections=[(paths[0],reg_fields,reg_rows),(paths[1],run_fields,run_rows+new_run),
                 (paths[2],exp_fields,exp_rows+new_exp)]
    before={str(path):sha(path) for path in paths}
    pending=[]
    for path,fields,rows in projections:
        target=path.with_name('.'+path.name+'.n05-pending')
        if target.exists():raise FileExistsError(target)
        with target.open('x',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields)
            writer.writeheader();writer.writerows(rows)
            stream.flush();os.fsync(stream.fileno())
        pending.append((path,target))
    for path,target in pending:
        os.replace(target,path)
    closure=dict(stage=ledger['stage'],status='administrative_terminal_projection',
                 release_sha256=release_sha,ledger_sha256=sha(LEDGER),
                 before=before,after={str(path):sha(path) for path in paths},
                 consumed=valid+failed,valid_assessment=valid,integrity_failed=failed,
                 retired_unrun=retired,run_manifest_rows=len(new_run),experiment_rows=len(new_exp),
                 interpretation='Projection only; algorithm failures retained. No N05 retry, resume or seed reuse.')
    with OUTPUT.open('x') as stream:
        json.dump(closure,stream,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
    print(json.dumps(dict(status=closure['status'],consumed=closure['consumed'],valid=valid,retired=retired,
                          closure_sha256=sha(OUTPUT))))


if __name__=='__main__':main()
