#!/usr/bin/env python3
"""Restart only a dead move_base inside its existing owned PID namespace."""
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import time
import xmlrpc.client
import rosgraph

socket.setdefaulttimeout(2)
root=Path('/tmp');session=json.loads((root/'session.json').read_text())
master=rosgraph.Master('/conbine_move_base_recovery')
assert master.getParam('/conbine/session')['session']==session['session']
state=json.loads((root/'status.json').read_text())
assert not state['latest']['/coverage/status']['active']
assert all(v==0 for v in state['latest']['/cmd_vel'])
try:
    uri=master.lookupNode('/move_base')
except rosgraph.masterapi.Error:
    uri=None
if uri:
    try:
        live=xmlrpc.client.ServerProxy(uri).getPid('/conbine_move_base_recovery')[0]==1
    except (OSError,xmlrpc.client.Error):
        live=False
    if live:raise RuntimeError('Refusing to replace a live move_base')
resolved=subprocess.run(['roslaunch','--args=/move_base','/tmp/navigation.launch'],
                        text=True,capture_output=True,check=True,timeout=15)
args=[a for a in shlex.split(resolved.stdout.strip()) if not a.startswith('__log:=')]
args.append('__log:=/tmp/logs/move_base_recovered_'+str(time.time_ns())+'.log')
child=subprocess.Popen(args,start_new_session=True)
(root/'move_base_recovery.json').write_text(json.dumps({'pid_in_namespace':child.pid,
    'session':session['session'],'args':args,'preserved_localizer':True,'started':time.time()},indent=2)+'\n')
def stop(*_):
    if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop)
raise SystemExit(child.wait())
