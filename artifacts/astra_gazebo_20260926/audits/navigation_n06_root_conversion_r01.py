#!/usr/bin/env python3
"""Prepare N06's eight offline URDF-to-SDF sidecars, with no ROS world."""
import datetime
import hashlib
import json
from pathlib import Path
import sys

S=Path(__file__).resolve().parents[1]
W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import dense_sweep_n05 as dense

PREPARED=S/'contracts/navigation_n06_prepared_r01/prepared.json'
OUT=S/'contracts/navigation_n06_conversion_r01'
EXPECTED_PREPARED_SHA='a2eee7ec93770e3a2bd6df755d587465814cfa7aaf44cfb9bac2a8361f8ac9ad'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def bind(path):return {'path':str(Path(path).resolve()),'sha256':sha(path)}

def main():
    if OUT.exists():raise FileExistsError('preserve existing conversion attempt')
    if sha(PREPARED)!=EXPECTED_PREPARED_SHA:raise ValueError('N06 prepared package changed')
    prepared=json.loads(PREPARED.read_text());units=prepared['units']
    if len(units)!=8 or prepared['execution_authorized'] is not False or prepared['seeds_registered'] is not False:
        raise ValueError('expected frozen zero-authority prepared package')
    if (S/'contracts/navigation_n06_manager_release_r01.json').exists():raise ValueError('unexpected N06 release')
    if (S/'queue/navigation_n06_ledger.json').exists():raise ValueError('unexpected N06 execution')
    OUT.mkdir()
    converted=[]
    for unit in units:
        inp=unit['robot_description'];path=Path(inp['path'])
        if bind(path)!=inp:raise ValueError('prepared URDF changed: '+unit['run_id'])
        manifest_path=dense.create_conversion(path,OUT/unit['run_id'])
        manifest=json.loads(manifest_path.read_text())
        if manifest['input_urdf']!=inp or manifest['output_bytes_reproduced'] is not True:
            raise ValueError('converter input/reproduction mismatch: '+unit['run_id'])
        resources=dense.conversion_resources(manifest_path)
        converted.append({'run_id':unit['run_id'],'input_urdf':inp,
                          'conversion_manifest':bind(manifest_path),'converted_sdf':manifest['converted_sdf'],
                          'conversion_resources':resources})
        print(unit['run_id'],manifest['converted_sdf']['sha256'],flush=True)
    bundle={'schema':'astra_navigation_n06_conversion_bundle_v1','stage':prepared['stage'],
            'scope':'offline converter process maps and byte-reproduced URDF-to-SDF; no ROS/Gazebo world',
            'execution_authorized':False,'seed_registered':False,'ROS_or_Gazebo_started':False,
            'prepared':bind(PREPARED),'generator':bind(Path(__file__)),
            'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'units':converted}
    out=OUT/'bundle.json';out.write_text(json.dumps(bundle,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print('bundle',bind(out),flush=True)

if __name__=='__main__':main()
