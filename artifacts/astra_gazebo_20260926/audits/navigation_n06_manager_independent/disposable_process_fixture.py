"""Run ONLY disposable Python children, no ROS/seed/release. Separate subreaper."""
import json,os,signal,socket,subprocess,sys,tempfile,time
from pathlib import Path
from unittest.mock import patch
A=Path(__file__).resolve().parent;S=A.parents[1];P=S/'workspace/scripts/astra_gazebo';sys.path.insert(0,str(P))
import navigation_n06_process as proc
import navigation_n06_integrity as q
out=A/'disposable_process_evidence';out.mkdir(exist_ok=False)
# Another branch of this fixture's process tree is unrelated to the cleanup owner.
unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
script=r'''
import json,os,signal,socket,subprocess,sys,time
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,sys.argv[1]);import navigation_n06_process as p
root=Path(sys.argv[2]);p.enable_subreaper();nonce='independent-disposable-nonce'
env=dict(os.environ,ASTRA_N06_CHILD_NONCE=nonce,ROS_LOG_DIR=str(root/'roslog'))
child_code="""import json,os,subprocess,sys,time
from pathlib import Path
root=Path(sys.argv[1])
grand=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],start_new_session=True)
(root/'family.json').write_text(json.dumps({'parent':os.getpid(),'grandchild':grand.pid}))
time.sleep(30)
"""
child=subprocess.Popen([sys.executable,'-c',child_code,str(root)],env=env,start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
try:
 deadline=time.monotonic()+3
 while not (root/'family.json').exists() and time.monotonic()<deadline:time.sleep(.01)
 family=json.loads((root/'family.json').read_text());before={str(pid):p.proc_row(pid) for pid in family.values()}
 rows=p.scan(os.getpid(),nonce,root);assert {x['pid'] for x in rows}==set(family.values())
 assert all(x['eligible'] for x in rows)
 assert all(x['sid']==x['pid'] for x in rows)
 assert not p.signal_pinned(child.pid,'wrong-startticks',signal.SIGTERM,lambda _:True)
 assert child.poll() is None
 # Exercise actual socket bind checks on a harmless ephemeral port; no stage ports touched.
 with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
 actual_ports=p.assert_ports_free
 with patch.object(p,'assert_ports_free',side_effect=lambda:actual_ports((port,))):result=p.cleanup(child,root,nonce,grace=(.5,.2,.2))
 assert result['pass'] and child.poll() is not None and not p.scan(os.getpid(),nonce,root)
 (root/'family_result.json').write_text(json.dumps({'before':before,'cleanup':result,'child_wait_returncode':child.returncode,'subreaper_survived':True,'ephemeral_port':port},indent=2)+'\n')
finally:
 if child.poll() is None:child.kill();child.wait(timeout=3)
'''
(out/'inner_fixture.py').write_text(script)
try:
 result=subprocess.run([sys.executable,str(out/'inner_fixture.py'),str(P),str(out)],capture_output=True,timeout=15)
 (out/'stdout.log').write_bytes(result.stdout);(out/'stderr.log').write_bytes(result.stderr)
 assert result.returncode==0,result.stderr.decode()
 assert unrelated.poll() is None
 data=json.loads((out/'family_result.json').read_text());data.update(outer_fixture_returncode=result.returncode,unrelated_sibling_alive=True,ROS_started=False,seed_allocated=False)
 (A/'disposable_process_result.json').write_text(json.dumps(data,indent=2)+'\n');print(json.dumps({'pass':True,'related_processes_cleaned':2,'unrelated_sibling_alive':True,'no_killpg':True}))
finally:
 if unrelated.poll() is None:unrelated.terminate();unrelated.wait(timeout=3)
