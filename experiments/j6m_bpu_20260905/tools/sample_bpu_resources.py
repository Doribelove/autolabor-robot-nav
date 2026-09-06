#!/usr/bin/env python3
"""Read-only J6M CPU/BPU/memory/temperature samples, no sysfs writes."""
import argparse
import json
from pathlib import Path
import time


def cpu_ticks():
    fields = [int(x) for x in Path('/proc/stat').read_text().splitlines()[0].split()[1:9]]
    return sum(fields), fields[3] + fields[4]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=int, default=60)
    args = parser.parse_args()
    if not 2 <= args.seconds <= 300:
        raise ValueError('Sampling duration outside bounds')
    samples = []
    total, idle = cpu_ticks()
    for _ in range(args.seconds):
        time.sleep(1)
        current_total, current_idle = cpu_ticks()
        memory = dict((line.split(':')[0], int(line.split()[1]))
                      for line in Path('/proc/meminfo').read_text().splitlines())
        workers = {}
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():
                continue
            try:
                command = (process / 'cmdline').read_bytes()
                if b'/tools/bpu_preview_worker.py\x00' not in command:
                    continue
                status = (process / 'status').read_text().splitlines()
                workers[process.name] = {line.split(':')[0]: line.split(':')[1].strip()
                                         for line in status if line.startswith(('VmRSS:', 'Threads:'))}
            except (OSError, ProcessLookupError):
                continue
        samples.append(dict(monotonic=time.monotonic(),
                            cpu_percent=100 * (1 - (current_idle - idle) / max(1, current_total - total)),
                            bpu_ratio=Path('/sys/devices/system/bpu/bpu0/ratio').read_text().strip(),
                            mem_available_kib=memory['MemAvailable'], workers=workers,
                            temperature_sysfs_raw={str(p): p.read_text().strip()
                                                      for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')}))
        total, idle = current_total, current_idle
    print(json.dumps(samples, indent=2))


if __name__ == '__main__':
    main()
