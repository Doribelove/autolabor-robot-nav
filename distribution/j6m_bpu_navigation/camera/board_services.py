"""Install only private B runtime units; preserve the board's boot image."""
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
UNITS = ['j6m-b-clock.service', 'j6m-b-camera-receiver.service']


def command(args, check=True, timeout=10):
    p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    if check and p.returncode:
        raise RuntimeError(str(args) + ': ' + p.stderr)
    return p


def ensure():
    if ROOT.parent != Path('/map/j6m-dev-b/camera/releases'):
        raise RuntimeError('Private board deployment only')
    config = json.loads((ROOT/'config.json').read_text())
    if config['board_root'] != str(ROOT):
        raise RuntimeError('Root/config mismatch')
    (ROOT/'clock').chmod(0o700)
    for name in UNITS:
        unit = ROOT/'units'/name
        fragment = command(['systemctl','show',name,'-p','FragmentPath','--value'],check=False).stdout.strip()
        if fragment and Path(fragment).resolve() != unit:
            raise RuntimeError('Foreign unit occupies ' + name + ': ' + fragment)
        if not fragment:
            command(['systemctl','link','--runtime',str(unit)])
    command(['systemctl','daemon-reload'])
    command(['systemctl','start','--no-block',*UNITS])
    return command(['systemctl','show',*UNITS,'-p','ActiveState,SubState,MainPID,FragmentPath']).stdout


def stop():
    for name in reversed(UNITS):
        fragment=command(['systemctl','show',name,'-p','FragmentPath','--value'],check=False).stdout.strip()
        if fragment and Path(fragment).resolve() != ROOT/'units'/name:
            raise RuntimeError('Refusing to stop foreign service '+name)
        command(['systemctl','stop',name],timeout=20)
    before=json.loads((ROOT/'predeploy_state.json').read_text())
    if before['sync_state']['out'].strip()=='active':
        command(['systemctl','start','--no-block','systemd-timesyncd.service'])
    return 'Private camera receiver/clock stopped; original timesync state requested.'


if __name__ == '__main__':
    print(json.dumps({'result': ensure() if sys.argv[1]=='ensure' else stop() if sys.argv[1]=='stop' else 'invalid action'}))
