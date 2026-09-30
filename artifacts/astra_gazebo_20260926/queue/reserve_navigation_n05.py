"""Reserve N05's exact fresh paired identities after immutable release validation."""
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys

S = Path(__file__).resolve().parents[1]
W = S / 'workspace'
sys.path.insert(0, str(W / 'scripts/astra_gazebo'))
import run_navigation_n05 as manager

REGISTRY = S / 'seed_registry.csv'
RELEASE = S / 'contracts/navigation_n05_release.json'
SEAL = S / 'audits/navigation_n05_root_release/seal.json'
REPORT = S / 'audits/navigation_n05_root_reservation.json'
BACKUP = S / 'queue/seed_registry_pre_n05.csv'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if REPORT.exists() or BACKUP.exists():
        raise FileExistsError('N05 reservation identity already exists')
    seal = json.loads(SEAL.read_bytes())
    release_sha = sha(RELEASE)
    if (seal.get('status') != 'N05_full_prejournal_validation_pass_no_seed_allocation_or_ROS'
            or seal['release']['path'] != str(RELEASE)
            or seal['release']['sha256'] != release_sha):
        raise ValueError('N05 verified release seal is absent or changed')
    manager.verify_release(RELEASE, release_sha)
    if (S / 'queue/navigation_n05_ledger.json').exists():
        raise ValueError('N05 ledger already exists')
    ids = manager.identities()
    if len(ids) != 24 or len({name for name, _, _ in ids}) != 24:
        raise ValueError('exact N05 schedule missing')
    if {seed for _, seed, _ in ids} != set(range(263002001, 263002013)):
        raise ValueError('fresh N05 seed range differs')
    if any((S / 'runs' / name).exists() for name, _, _ in ids):
        raise ValueError('N05 attempt root already exists')
    with REGISTRY.open(newline='') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        rows = list(reader)
    if fields != ['namespace', 'split', 'purpose', 'seed', 'run_id', 'status', 'source_binding']:
        raise ValueError('seed registry schema differs')
    names = {name for name, _, _ in ids}
    seeds = {str(seed) for _, seed, _ in ids}
    if any(row['run_id'] in names or row['seed'] in seeds for row in rows):
        raise ValueError('N05 seed or identity already registered anywhere in stage')
    if any(sum(seed == other for _, other, _ in ids) != 2 for seed in range(263002001, 263002013)):
        raise ValueError('each paired seed requires exactly two methods')
    additions = [dict(namespace='astra_gazebo_20260926', split='development',
                      purpose='navigation_n05_preview_selection_paired_diagnosis',
                      seed=str(seed), run_id=name, status='reserved_exact_n05_release',
                      source_binding=release_sha) for name, seed, _ in ids]
    with BACKUP.open('xb') as stream:
        stream.write(REGISTRY.read_bytes())
        stream.flush(); os.fsync(stream.fileno())
    temporary = REGISTRY.with_name('.seed_registry_n05_tmp.csv')
    with temporary.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows + additions)
        stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, REGISTRY)
    with REGISTRY.open(newline='') as stream:
        result = list(csv.DictReader(stream))
    if result != rows + additions:
        raise ValueError('N05 registry postwrite verification failed')
    report = dict(status='exact_N05_fresh_reservation_after_verified_release',
                  stage=manager.NAME, release_sha256=release_sha,
                  seed_range=[263002001, 263002012], pairs=12, rows=additions,
                  original_registry_sha256=sha(BACKUP),
                  updated_registry_sha256=sha(REGISTRY),
                  registered_utc=datetime.now(timezone.utc).isoformat(),
                  simulation_started=False)
    with REPORT.open('x') as stream:
        json.dump(report, stream, indent=2); stream.write('\n')
    print(json.dumps(dict(status=report['status'], release_sha256=release_sha,
                          new_rows=len(additions))))


if __name__ == '__main__':
    main()
