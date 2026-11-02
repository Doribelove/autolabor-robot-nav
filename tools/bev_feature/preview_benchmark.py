#!/usr/bin/env python3
"""Measure the live pixel-BEV preview, not capture-to-map latency or accuracy."""
import argparse
import json
from pathlib import Path
import struct
import time
import urllib.request

import numpy as np


def percentiles(values):
    return {name: float(value) for name, value in zip(
        ('p50', 'p95', 'max'), np.percentile(values, (50, 95, 100)))} if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:18883/frame')
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 2 <= args.seconds <= 600:
        parser.error('--seconds must be 2..600')
    args.output.mkdir(parents=True, exist_ok=True)
    rows, failures = [], []
    start = time.monotonic()
    while time.monotonic()-start < args.seconds:
        begin = time.monotonic()
        try:
            with urllib.request.urlopen(args.url, timeout=2) as response:
                data = response.read()
            end = time.monotonic()
            magic, nj, ni = struct.unpack('!4sII', data[:12])
            if magic != b'BG01' or len(data) != 12+nj+ni:
                raise ValueError('Invalid ground preview packet')
            meta = json.loads(data[12:12+nj])
            rows.append(dict(sample_monotonic=begin, request_ms=(end-begin)*1000,
                             preview_age_ms=(time.time()-meta['produced_epoch_s'])*1000,
                             metadata=meta))
            if len(rows) == 1:
                (args.output/'frame.bg01').write_bytes(data)
                (args.output/'bev.jpg').write_bytes(data[12+nj:])
        except (OSError, ValueError, struct.error) as exc:
            failures.append(str(exc))
        time.sleep(max(0, .1-(time.monotonic()-begin)))
    unique = {(r['metadata']['fusion_session'], r['metadata']['fusion_sequence']):
              r['metadata'] for r in rows}
    sessions = {key[0] for key in unique}
    stamps = sorted(m['produced_epoch_s'] for m in unique.values())
    gaps = np.diff(stamps)*1000 if len(sessions) == 1 else []
    summary = dict(scope='pixel preview only; capture clocks remain unverified',
                   samples=len(rows), unique_frames=len(unique), failures=failures,
                   fps=(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(gaps) else None,
                   frame_interval_ms=percentiles(list(gaps)),
                   request_ms=percentiles([r['request_ms'] for r in rows]),
                   preview_age_ms=percentiles([r['preview_age_ms'] for r in rows]),
                   processing_ms=percentiles([m['processing_ms'] for m in unique.values()]),
                   source_receive_span_ms=percentiles([m['source_receive_span_ms'] for m in unique.values()]))
    (args.output/'samples.json').write_text(json.dumps(rows, indent=2)+'\n')
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    if not rows or failures or len(sessions) != 1:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
