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
import run_m12_physics_p05 as m

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

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
    results.append(dict(run_id=row['run_id'], seed=row['seed'], kd=row['steering_kd'], contract_sha256=row['contract_sha256'], launch_sha256=sha(launch), parameters_sha256=sha(folder/'parameters.yaml'), exit_code=process.returncode))
report = dict(status='six_revised_P05_launch_expansions_pass', ros_started=False, allocated_units=0, runner_sha256=sha(m.__file__), design_sha256=sha(m.DESIGN), checker_sha256=sha(__file__), runs=results)
with (OUT/'report.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(dict(status=report['status'], count=len(results), report_sha256=sha(OUT/'report.json'))))
