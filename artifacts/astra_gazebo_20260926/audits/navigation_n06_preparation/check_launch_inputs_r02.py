#!/usr/bin/env python3
"""Expand exact launches and inspect their resolved configuration; never start."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent;S=A.parent.parent;W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import prepare_navigation_n06_static as prep
import roslaunch.config
import yaml

def main():
    folder=A/'launch_dump_attempt02';folder.mkdir()
    prepared=prep.read(prep.PREPARED/'prepared.json');rows=[]
    for unit in prepared['units']:
        d=folder/unit['run_id'];d.mkdir();launch=Path(unit['launch']['path'])
        command=['/opt/ros/noetic/bin/roslaunch','--dump-params',str(launch)]
        result=subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=30)
        (d/'params.yaml').write_text(result.stdout);(d/'stderr.log').write_text(result.stderr)
        prep.write_new(d/'command.json',{'argv':command,'returncode':result.returncode,'ROS_started':False})
        if result.returncode:raise RuntimeError('launch dump failed: '+unit['run_id'])
        params=yaml.safe_load(result.stdout)
        config=roslaunch.config.load_config_default([str(launch)],None,verbose=False)
        nodes={n.name:n for n in config.nodes}
        expected_method='astra_route_coordinated_runtime.py' if unit['method']==prep.METHODS[0] else 'astra_route_lookahead_runtime_n06.py'
        assert nodes['astra_route_runtime'].type==expected_method
        assert nodes['astra_navigation_coordinator'].type=='astra_navigation_coordinator_n06.py'
        assert nodes['astra_navigation_collector'].type=='astra_navigation_collector_n06.py'
        assert nodes['astra_navigation_collector'].required
        assert set(nodes)=={'gazebo','m2_robot_state_publisher','navigation_command_relay','astra_common_route_provider','astra_navigation_coordinator','astra_route_runtime','fixture_bag','astra_navigation_collector'}
        assert ('/cmd_vel_navigation','/astra/method_cmd') in [tuple(x) for x in nodes['astra_route_runtime'].remap_args]
        assert nodes['navigation_command_relay'].args=='/cmd_vel_navigation /cmd_vel'
        assert nodes['astra_route_runtime'].launch_prefix==str(W/'.venv/bin/python')
        assert params['/astra_route_runtime/method']==unit['method']
        assert params['/astra_route_runtime/seed']==unit['seed']
        assert params['/astra_navigation_coordinator/method']==unit['method']
        assert params['/astra_navigation_coordinator/seed']==unit['seed']
        assert params['/astra_route_runtime/maximum_episode_duration_s']==22.
        contract=prep.read(unit['contract']['path'])
        assert params['/astra_navigation_collector/contract_path']==unit['contract']['path']
        assert params['/astra_navigation_collector/contract_sha256']==unit['contract']['sha256']
        expected=Path(unit['robot_description']['path']).read_text()
        assert params['/robot_description']==expected
        driver=ET.fromstring(expected).find(".//plugin[@filename='libastra_physical_ackermann_p05_plugin.so']")
        assert int(driver.findtext('dynamicsSeed'))==unit['seed'] and float(driver.findtext('steeringKd'))==6.
        assert '--seed '+str(unit['seed']) in nodes['gazebo'].args
        assert '-u' in nodes['gazebo'].args
        assert all(topic in nodes['fixture_bag'].args.split() for topic in contract['record_topics'])
        assert 'preview' not in nodes['astra_route_runtime'].type
        rows.append({'run_id':unit['run_id'],'seed':unit['seed'],'method':unit['method'],'passed':True,
            'resolved_method_script':expected_method,'params':prep.binding(d/'params.yaml'),
            'nodes':[{'name':n.name,'package':n.package,'type':n.type,'args':n.args,'required':n.required,'remap_args':n.remap_args,'launch_prefix':n.launch_prefix} for n in config.nodes],
            'physics_seed':unit['seed'],'steering_kd':6.,'actual_source_world_geometry_bound':contract['world']['sha256']})
    report={'status':'all8_exact_offline_launch_dumps_pass','units':rows,'ROS_started':False,
        'scope':'roslaunch parse/parameter expansion only; no nodes, services, seed registration or simulation',
        'environment':{'THESIS_WS':os.environ.get('THESIS_WS'),'python_executable':sys.executable},'execution_authorized':False}
    prep.write_new(A/'launch_dump_report.json',report)
    print(json.dumps({'status':report['status'],'count':len(rows)}))

if __name__=='__main__':main()
