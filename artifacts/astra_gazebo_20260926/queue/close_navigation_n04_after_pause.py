"""Administrative terminal ledger projection; never reopens an N04 identity."""
from pathlib import Path
from datetime import datetime, timezone
import csv
import hashlib
import json
import os

S = Path(__file__).resolve().parents[1]
LEDGER = S/'queue/navigation_n04_ledger.json'
RELEASE = '3ad785e8f4b1242bb238b4714b2925a7bf280edb43f1fc1fcf2217a0a8a3efb9'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def table(path):
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames, list(reader)

def projection():
    ledger = json.loads(LEDGER.read_text())
    units = ledger['units']
    assert ledger['stage']=='astra_navigation_n04' and ledger['status']=='terminal' and len(units)==48
    assert ledger['release_sha256']==RELEASE
    assert sum(u['status']=='consumed' for u in units)==24
    assert sum(u['status']=='retired_unrun' for u in units)==24
    assert [u['seed'] for u in units]==[263001001+i//4 for i in range(48)]
    assert len({u['run_id'] for u in units})==48
    assert all(not (S/'runs'/u['run_id']).exists() for u in units if u['status']=='retired_unrun')
    design_path=S/'contracts/navigation_n04_design.json'
    assert sha(design_path)=='ce5afb9920eefb7a3c4d2bad2646405e977494a13cd61477bf248f3fa867a912'
    schedule={r['run_id']:r for r in json.loads(design_path.read_text())['schedule']}
    assert set(schedule)=={u['run_id'] for u in units}
    regf, regrows = table(S/'seed_registry.csv')
    runf, runrows = table(S/'run_manifest.csv')
    expf, exprows = table(S/'EXPERIMENTS.csv')
    assert not any(r['run_id'].startswith('astra_navigation_n04') for r in runrows+exprows)
    relevant = [r for r in regrows if r['run_id'].startswith('astra_navigation_n04')]
    assert len(relevant)==48 and {r['run_id'] for r in relevant}=={u['run_id'] for u in units}
    assert all(r['status']=='reserved_exact_n04_release' and r['source_binding']==RELEASE for r in relevant)
    by_id = {r['run_id']:r for r in relevant}
    produced_run = []; produced_exp = []
    for u in units:
        name = u['run_id']; root = S/'runs'/name
        row = by_id[name]
        assert row['seed']==str(u['seed']) and row['split']=='development'
        if u['status']=='retired_unrun':
            row['status']='retired_unrun_after_user_pause_n04'
            status='retired_unrun_after_user_pause'
            report=''; raw=''; started=''; ended=''; exit_code=''; code=''; config=''
        else:
            receipt_binding=u['receipt'];receipt_path=Path(receipt_binding['path'])
            assert receipt_path==root/'receipt.json' and sha(receipt_path)==receipt_binding['sha256']
            receipt=json.loads(receipt_path.read_text())
            assert (receipt['run_id'],receipt['seed'],receipt['method'],receipt['attempt'])==(name,u['seed'],u['method'],1)
            assert receipt['release_sha256']==RELEASE
            assert receipt['integrity_pass']==u['integrity_pass']
            config=sha(root/'contract.json')
            process=json.loads((root/'process.json').read_text())
            allocated=json.loads((root/'allocated.json').read_text())
            started=allocated['started_utc']
            # Receipt creation time is the available final administrative time,
            # not the earlier physical terminal or assessment completion time.
            ended=datetime.fromtimestamp(receipt_path.stat().st_mtime,timezone.utc).isoformat()
            report=receipt['report']['path'] if receipt.get('report') else ''
            raw=str(root/'collector_raw.json') if (root/'collector_raw.json').is_file() else ''
            code=sha(S/'workspace/scripts/astra_gazebo/run_navigation_n04.py')
            exit_code=receipt['exit_code']
            if u['integrity_pass'] is True:
                assert receipt['error'] is None and receipt['task_outcome'] in ('complete','timeout','fault','stopped')
                status='terminal_'+receipt['task_outcome']
                row['status']='consumed_terminal_n04'
            else:
                assert name==units[23]['run_id'] and 'KeyboardInterrupt' in (receipt['error'] or '')
                assert receipt['report'] is None and receipt['task_outcome'] is None
                status='assessment_interrupted_by_user_pause'
                row['status']='consumed_assessment_interrupted_n04'
            launch=root/'fixture.launch'
            produced_run.append(dict(run_id=name,pid=process['pid'],process_group=process['pgid'],
                command_json=json.dumps(['roslaunch',str(launch)]),cwd=str(S/'workspace'),
                started_utc=started,ended_utc=ended,exit_code=exit_code,
                heartbeat_path=str(root/'heartbeat.json'),receipt_path=str(receipt_path),bindings_sha256=RELEASE))
        produced_exp.append(dict(run_id=name,split='development',candidate_id=u['method'],method=u['method'],
            scene_id=schedule[name]['scene_id'],layout_id=schedule[name]['layout_id'],
            seed=u['seed'],training_seed='',status=status,started_utc=started,ended_utc=ended,
            exit_code=exit_code,code_sha256=code,config_sha256=config,model_sha256='',report_path=report,raw_path=raw))
    return ((S/'seed_registry.csv',regf,regrows),
            (S/'run_manifest.csv',runf,runrows+produced_run),
            (S/'EXPERIMENTS.csv',expf,exprows+produced_exp),ledger)

def main():
    output=S/'queue/navigation_n04_admin_closure.json'
    if output.exists():raise FileExistsError('N04 administrative closure already exists')
    projections=projection();before={str(p):sha(p) for p,_,_ in projections[:3]}
    pending=[]
    for path,fields,rows in projections[:3]:
        target=path.with_suffix(path.suffix+'.n04-pending')
        if target.exists():raise FileExistsError(target)
        with target.open('x',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields)
            writer.writeheader();writer.writerows(rows);stream.flush();os.fsync(stream.fileno())
        pending.append((path,target))
    for path,target in pending:os.replace(target,path)
    closure=dict(stage='astra_navigation_n04',status='administrative_terminal_closure',
        release_sha256=RELEASE,ledger_sha256=sha(LEDGER),before=before,
        after={str(p):sha(p) for p,_,_ in projections[:3]},
        consumed=24,complete_assessment=23,assessment_interrupted=1,retired_unrun=24,
        user_pause_binding=dict(path=str(S/'queue/user_pause_quiescence_20260926.json'),
                                sha256=sha(S/'queue/user_pause_quiescence_20260926.json')),
        interpretation='Valid N04 task outcomes only for 23 assessed rows; remaining matrix is incomplete. No retries or reused identities.')
    with output.open('x') as stream:
        json.dump(closure,stream,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
    print(json.dumps(dict(status=closure['status'],consumed=24,retired=24,closure_sha256=sha(output))))

if __name__=='__main__':main()
