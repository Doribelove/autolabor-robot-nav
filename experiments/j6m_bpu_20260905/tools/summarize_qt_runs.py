#!/usr/bin/env python3
"""Summarize recorded evidence; desktop samples are not a YOLO/BPU A/B test."""
import json
from pathlib import Path
import re
import statistics

LAB = Path(__file__).resolve().parents[1]


def describe(values):
    values = sorted(values)
    return dict(n=len(values), mean=statistics.mean(values), median=statistics.median(values),
                p95=values[min(len(values) - 1, int(len(values) * 0.95))],
                minimum=values[0], maximum=values[-1]) if values else {}


def main():
    report = {'not_yolo_bpu_ab': True, 'runs': {}, 'desktop_samples': {}}
    for name in ('qt_20260905_210513_769043', 'qt_20260905_211259_264022'):
        directory = LAB / 'live' / name
        rows = [json.loads(line) for line in (directory / 'results.jsonl').read_text().splitlines()]
        report['runs'][name] = {
            'summary': json.loads((directory / 'summary.json').read_text()),
            'roundtrip_ms': describe([row['roundtrip_ms'] for row in rows]),
            'source_age_ms': describe([row['source_age_ms'] for row in rows]),
            'vendor_infer_ms': describe([row['vendor_infer_ms'] for row in rows]),
            'all_display_only': all(row['demo_only'] and not row['motion_eligible'] for row in rows),
        }
    for name in ('host_idle_before_qt_2054', 'qt_active_tegrastats'):
        gpu, cpu, ram = [], [], []
        for line in (LAB / 'logs' / (name + '.log')).read_text().splitlines():
            gpu_match = re.search(r'GR3D_FREQ (\d+)%', line)
            cpu_match = re.search(r'CPU \[([^]]+)\]', line)
            ram_match = re.search(r'RAM (\d+)/', line)
            if gpu_match and cpu_match and ram_match:
                gpu.append(int(gpu_match[1]))
                cpu.append(statistics.mean([int(value) for value in re.findall(r'(\d+)%@', cpu_match[1])]))
                ram.append(int(ram_match[1]))
        report['desktop_samples'][name] = dict(gpu_percent=describe(gpu),
                                              cpu_whole_host_percent=describe(cpu), ram_mb=describe(ram))
    destination = LAB / 'logs/qt_measurements.json'
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
