#!/usr/bin/env python3
"""Inventory move_base's generated defaults against frozen B04 launch dumps.

The generated config and launch dumps are offline sources; this does not
observe the initialized parameter server or authorize baseline execution.
"""

import hashlib
import importlib
import json
from pathlib import Path


STAGE = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / 'inventory.json'
METHODS = ('actual_teb_rear_reference_b03', 'dwa_rear_reference_b03')


def bind(path):
    path = Path(path).resolve(strict=True)
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    if OUT.exists():
        raise FileExistsError('preserve prior inventory bytes')
    module = importlib.import_module('move_base.cfg.MoveBaseConfig')
    defaults = dict(module.defaults)
    if not defaults or not all(isinstance(k, str) for k in defaults):
        raise ValueError('unexpected generated move_base defaults')
    rows = []
    for method in METHODS:
        path = STAGE / 'audits/baseline_integration_b04/launch_preflight_final_r2' / method / 'effective_move_base.json'
        tree = json.loads(path.read_text())
        explicit = {key: tree[key] for key in defaults if key in tree}
        rows.append({
            'method': method,
            'offline_launch_dump': bind(path),
            'generated_default_keys': len(defaults),
            'root_generated_keys_explicit_in_launch_dump': sorted(explicit),
            'root_generated_keys_absent_from_launch_dump': sorted(set(defaults) - set(explicit)),
            'explicit_values_different_from_generated_defaults': sorted(
                key for key, value in explicit.items()
                if type(value) is not type(defaults[key]) or value != defaults[key]),
        })
    prior = STAGE / 'audits/baseline_static_defaults_b07/inventory.json'
    report = {
        'schema': 'astra_b08_move_base_generated_defaults_inventory_v1',
        'status': 'offline_gap_inventory_no_runtime_qualification',
        'generated_config': bind(module.__file__),
        'generated_defaults': defaults,
        'methods': rows,
        'b07_prior': bind(prior),
        'generator': bind(Path(__file__)),
        'authority': {'execution_authorized': False, 'budget': 0,
                      'seeds_registered': 0, 'ros_gazebo_started': False},
        'limits': [
            'Generated defaults are source candidates, not observed live parameters.',
            'A generated default may be changed by plugin initialization or may not appear as a persistent parameter.',
            'Full three-phase initialized /move_base tree and physical baseline qualification remain pending.',
        ],
    }
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
    print(json.dumps({'output': str(OUT), 'sha256': bind(OUT)['sha256'],
                      'missing_per_method': {row['method']: len(row['root_generated_keys_absent_from_launch_dump'])
                                             for row in rows}}, sort_keys=True))


if __name__ == '__main__':
    main()
