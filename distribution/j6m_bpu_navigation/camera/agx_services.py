"""Re-establish the private J6M data services after link/board recovery."""
import json
from pathlib import Path
import subprocess
import sys
import time


def ssh(c, action):
    args=['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=3','-o','ConnectionAttempts=1',
          '-o','StrictHostKeyChecking=yes','-o','UpdateHostKeys=no','-o','ControlMaster=no',
          '-o','ControlPath=none','-o','PermitLocalCommand=no','-o','RemoteCommand=none',
          '-o','BindAddress='+c['agx_ip'],'root@'+c['j6m_ip'],
          '/usr/bin/python3','-B',c['board_root']+'/board_services.py',action]
    return subprocess.run(args,capture_output=True,text=True,timeout=25)


def main():
    c=json.loads(Path(sys.argv[1]).read_text())
    if Path(c['agx_state']).resolve().parents[1].parent != Path('/home/slam/j6m-dev-b'):
        raise RuntimeError('Private B AGX deployment only')
    mode=sys.argv[2]
    if mode=='supervise':
        path=Path(c['agx_state'])/'board_supervisor.json'
        while True:
            try:
                p=ssh(c,'ensure')
                result=dict(at_ns=time.time_ns(),returncode=p.returncode,stdout=p.stdout[-3000:],stderr=p.stderr[-1500:])
            except Exception as e:
                result=dict(at_ns=time.time_ns(),error=repr(e))
            temp=path.with_suffix('.tmp');temp.write_text(json.dumps(result,indent=2));temp.replace(path)
            time.sleep(10)
    elif mode=='stop':
        subprocess.run(['systemctl','--user','stop','j6m-b-camera-link','j6m-b-camera-sender'],check=True)
        p=ssh(c,'stop')
        print(p.stdout,p.stderr)
        subprocess.run(['systemctl','--user','stop','j6m-b-clock'],check=True)
        raise SystemExit(p.returncode)
    elif mode=='start':
        subprocess.run(['systemctl','--user','start','j6m-b-clock','j6m-b-camera-link','j6m-b-camera-sender'],check=True)
    else:
        raise ValueError('Expected supervise/start/stop')


if __name__=='__main__':
    main()
