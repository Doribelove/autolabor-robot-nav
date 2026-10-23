"""Private AGX time reference. Standard NTP on an unprivileged dedicated port."""
import json
from pathlib import Path
import selectors
import signal
import socket
import struct
import subprocess
import sys
import time


def ntp_stamp(ns):
    sec, nano = divmod(ns, 1_000_000_000)
    return struct.pack('!II', (sec + 2208988800) & 0xffffffff, nano * (1 << 32) // 1_000_000_000)


def main():
    config = json.loads(Path(sys.argv[1]).read_text())
    select = selectors.DefaultSelector()
    for kind, port in [('probe', config['clock_port']), ('ntp', config['ntp_port'])]:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((config['agx_ip'], port))
        select.register(sock, selectors.EVENT_READ, kind)
    good, checked, requests = False, 0, 0
    while True:
        if time.monotonic() - checked > 10:
            status = subprocess.run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'],
                                    capture_output=True, text=True, timeout=3)
            good = status.returncode == 0 and status.stdout.strip() == 'yes'
            checked = time.monotonic()
            path = Path(config['agx_state']) / 'clock_server.json'
            temp = path.with_suffix('.tmp')
            temp.write_text(json.dumps(dict(at_ns=time.time_ns(), agx_ntp_synchronized=good, requests=requests)))
            temp.replace(path)
        for key, _ in select.select(timeout=1):
            data, peer = key.fileobj.recvfrom(4096)
            rx = time.time_ns()
            if peer[0] != config['j6m_ip']:
                continue
            if key.data == 'ntp':
                if len(data) < 48 or data[0] & 7 != 3:
                    continue
                reply = bytearray(48)
                reply[0] = (0 if good else 192) | 4 << 3 | 4
                reply[1], reply[2], reply[3] = (3 if good else 0), data[2], 236
                reply[8:12] = struct.pack('!I', 655)  # conservative 10 ms reference dispersion
                reply[12:16] = b'AGXB'
                reply[16:24] = ntp_stamp(rx)
                reply[24:32] = data[40:48]
                reply[32:40] = ntp_stamp(rx)
                reply[40:48] = ntp_stamp(time.time_ns())
                key.fileobj.sendto(reply, peer)
            else:
                try:
                    request = json.loads(data)
                    if request.get('token') != config['token']:
                        continue
                    reply = dict(t1=request['t1'], t2=rx, t3=time.time_ns(), synchronized=good)
                    key.fileobj.sendto(json.dumps(reply).encode(), peer)
                except (ValueError, KeyError, TypeError):
                    continue
            requests += 1


if __name__ == '__main__':
    main()
