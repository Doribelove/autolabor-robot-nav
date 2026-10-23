"""Stop/disable this camera deployment and remove only its added AGX address."""
import json
from pathlib import Path
import subprocess
import sys


def main():
    config=Path(sys.argv[1]).resolve()
    root=Path('/home/slam/j6m-dev-b')
    if root not in config.parents or config.name!='config.json':
        raise RuntimeError('Expected private B deployment config')
    run=config.parent
    source=Path(__file__).resolve().parent
    # Stop board services before removing the route needed to manage them.
    subprocess.run([sys.executable,'-B',str(source/'agx_services.py'),str(config),'stop'],check=True)
    subprocess.run(['systemctl','--user','disable','j6m-b-camera-link','j6m-b-camera-sender','j6m-b-clock'],check=True)
    before=json.loads((run/'network_before.json').read_text())
    if '192.168.10.50/24' in before[0]['output']:
        raise RuntimeError('This record did not own the secondary address')
    if Path('/sys/class/net/eth0/address').read_text().strip()!='4c:bb:47:4c:29:6a':
        raise RuntimeError('Network hardware identity changed; retain addresses')
    uuid='fbaf0051-f208-4c60-96a0-3649764e7aa2'
    live=subprocess.check_output(['nmcli','-g','GENERAL.CON-UUID','device','show','eth0'],text=True).strip()
    if live!=uuid:
        raise RuntimeError('Active profile changed; retain addresses')
    subprocess.run(['nmcli','connection','modify',uuid,'-ipv4.addresses','192.168.10.50/24'],check=True)
    subprocess.run(['nmcli','device','modify','eth0','-ipv4.addresses','192.168.10.50/24'],check=True)
    print('B camera services disabled; original timesync requested; owned secondary address removed. Releases and evidence retained.')


if __name__=='__main__':main()
