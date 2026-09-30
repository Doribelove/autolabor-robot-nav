import hashlib
import importlib.util
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

STAGE = Path(__file__).resolve().parents[2]
WS = STAGE / 'workspace'
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(WS / 'scripts/astra_gazebo'))
import run_m12_physics_p07 as m

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    return module

source=load('p07_source',m.SOURCE)
probe=load('p07_probe',m.PROBE)
old_source=load('p05_source',m.SOURCE.with_name('astra_actuation_command_source_p05.py'))
design, _, rows = m.load_design()
m.check_seed_freshness(rows)
results = []
for row in rows:
    folder = OUT / row['run_id']
    folder.mkdir()
    contract = yaml.safe_load(m.read_bound(STAGE / row['contract_path'], row['contract_sha256']))
    launch = folder / 'fixture.launch'
    launch.write_text(m.render_launch(row, STAGE / 'runs' / row['run_id'], contract))
    process = subprocess.run(['roslaunch', '--dump-params', str(launch)], capture_output=True, text=True, timeout=40)
    (folder / 'parameters.yaml').write_text(process.stdout)
    (folder / 'stderr.log').write_text(process.stderr)
    assert process.returncode == 0, process.stderr
    params = yaml.safe_load(process.stdout)
    model = ET.fromstring(params['/robot_description'])
    plugins = [node for node in model.iter('plugin') if node.get('filename') == 'libastra_physical_ackermann_p05_plugin.so']
    assert len(plugins) == 1
    plugin = plugins[0]
    for key, value in contract['effective_plugin_parameters'].items():
        assert float(plugin.find(key).text) == float(value), (row['run_id'], key)
    assert params['/actuation_command_source/contract_sha256'] == row['contract_sha256']
    assert params['/actuation_command_source/seed'] == row['seed']
    assert params['/actuation_command_source/run_id'] == row['run_id']
    assert params['/fixture_probe/recording_topics'] == m.TOPICS
    assert params['/fixture_probe/seed'] == row['seed']
    assert params['/fixture_probe/run_id'] == row['run_id']
    assert len(m.TOPICS) == 27
    loaded=source.load_contract(STAGE/row['contract_path'],row['contract_sha256'],row['seed'],row['run_id'])
    assert loaded==contract
    source.validate_runtime_environment('http://127.0.0.1:14820',True,True,params['/robot_description'],loaded,row['seed'])
    actual=probe.validate_effective_urdf(params['/robot_description'],loaded,row['seed'])
    probe.validate_world(loaded['world_fixture']['sdf'],loaded)
    assert actual['seed']==row['seed']
    try: old_source.validate_runtime_environment('http://127.0.0.1:14820',True,True,params['/robot_description'],loaded,row['seed'])
    except ValueError as exc: assert str(exc)=='unique P02 physical plugin required'
    else: raise AssertionError('historical startup defect not reproduced')
    for bad_xml in (params['/robot_description'].replace('libastra_physical_ackermann_p05_plugin.so','libastra_physical_ackermann_p02_plugin.so'),params['/robot_description'].replace('<dynamicsSeed>'+str(row['seed'])+'</dynamicsSeed>','<dynamicsSeed>1</dynamicsSeed>')):
        assert bad_xml!=params['/robot_description']
        try:source.validate_runtime_environment('http://127.0.0.1:14820',True,True,bad_xml,loaded,row['seed'])
        except ValueError:pass
        else:raise AssertionError('wrong driver/seed accepted')
    results.append(dict(run_id=row['run_id'], seed=row['seed'], kd=row['steering_kd'], contract_sha256=row['contract_sha256'], launch_sha256=sha(launch), parameters_sha256=sha(folder/'parameters.yaml'), exit_code=process.returncode))
report = dict(status='six_P07_actual_startup_validators_pass', ros_started=False, allocated_units=0, runner_sha256=sha(m.__file__), design_sha256=sha(m.DESIGN), checker_sha256=sha(__file__), runs=results)
with (OUT/'report.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(dict(status=report['status'], count=len(results), report_sha256=sha(OUT/'report.json'))))
