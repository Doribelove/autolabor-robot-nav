"""Call only real inputs()+plan(); never close()/commit()/main() or mutate real tables."""
from pathlib import Path
import hashlib,json,time
A=Path(__file__).resolve().parent;S=A.parents[2];p=S/'queue/close_navigation_n05_r03.py';env={'__file__':str(p),'__name__':'offline_readonly_plan'};exec(compile(p.read_bytes(),str(p),'exec'),env)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
paths=[S/name for name in env['TABLES']];before={str(p):{'sha256':sha(p),'mtime_ns':p.stat().st_mtime_ns,'inode':p.stat().st_ino} for p in paths};markers=[S/'queue'/name for name in ['navigation_n05_admin_transaction_r03','navigation_n05_admin_closure_r03.json','navigation_n05_admin_r03.lock']];marker_before={str(p):p.exists() for p in markers}
start=time.monotonic();manager,validator,deps=env['load_dependencies'](S);ledger,schedule,receipts,metadata,resources=env['inputs'](S,manager,validator,deps);manifest,b,a=env['plan'](S,ledger,schedule,receipts,metadata,resources);elapsed=time.monotonic()-start
assert len(manifest['units'])==24 and manifest['consumed']==24;assert len(a)==3 and len(b)==3
assert all(env['digest'](data)==t['before_sha256'] for data,t in zip(b,manifest['tables']));assert all(env['digest'](data)==t['after_sha256'] for data,t in zip(a,manifest['tables']))
after={str(p):{'sha256':sha(p),'mtime_ns':p.stat().st_mtime_ns,'inode':p.stat().st_ino} for p in paths};assert before==after;assert marker_before=={str(p):p.exists() for p in markers}
# Save the plan solely inside the independent audit tree; it cannot act as a
# queue transaction, and its created_utc is intentionally not a future seal.
(A/'readonly_proposed_manifest.json').write_bytes(env['json_bytes'](manifest))
for name,old,new in zip(env['TABLES'],b,a):(A/('readonly_before_'+name)).write_bytes(old);(A/('readonly_proposed_after_'+name)).write_bytes(new)
r=dict(status='actual_full_release_inputs_and_in_memory_plan_pass_no_projection',source_sha256=sha(p),duration_wall_s=elapsed,validated_input_bindings=len(resources),release_sha256=ledger['release_sha256'],ledger_sha256=sha(S/'queue/navigation_n05_ledger.json'),counts={k:manifest[k] for k in ['consumed','valid_assessment','integrity_failed','retired_unrun','run_manifest_rows','experiment_rows']},real_table_hash_mtime_inode_unchanged=before,real_marker_existence_unchanged=marker_before,plan_snapshot=dict(path=str(A/'readonly_proposed_manifest.json'),sha256=sha(A/'readonly_proposed_manifest.json')),real_close_or_commit_or_main_called=False,real_csv_written=False,no_ROS=True)
(A/'readonly_actual_plan.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
