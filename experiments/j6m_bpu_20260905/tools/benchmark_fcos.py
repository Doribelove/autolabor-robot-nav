#!/usr/bin/env python3
"""Bounded saved-image SSH benchmark with golden detection checks; no ROS."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import argparse
from datetime import datetime
import fcntl
import json
import subprocess
import time
import numpy as np
from live_preview import LAB, REMOTE
from live_protocol import frame_packet, receive_json, write_all
from preview_lifecycle import BoundedLog, drain_stderr
from qt_bpu_bridge import stop_worker


def summary(values):
    return dict(mean=float(np.mean(values)), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), p99=float(np.percentile(values, 99)),
                maximum=float(np.max(values)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--engine', choices=('resident', 'cli'), default='resident')
    parser.add_argument('--frames', type=int, default=200)
    parser.add_argument('--full-nv12', action='store_true')
    parser.add_argument('--max-fps', type=float, default=0,
                        help='0 = unpaced capacity test; >0 = rate-limited saved-image test')
    args = parser.parse_args()
    if os.environ.get('ROBOT_OPTIMIZED_SANDBOX') != '1':
        raise RuntimeError('Use optimized.sh run')
    if not 1 <= args.frames <= 1000 or not np.isfinite(args.max_fps) or not 0 <= args.max_fps <= 30:
        raise ValueError('Invalid benchmark bounds')
    with (LAB / 'preview_client.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run_id = 'bench_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        directory = LAB / 'live' / run_id
        directory.mkdir()
        cases = []
        for name in ('bus', 'zed_saved'):
            inputs = LAB / 'inputs' / name
            manifest = json.loads((inputs / 'input_manifest.json').read_text())
            cases.append(((inputs / 'y.bin').read_bytes() + (inputs / 'uv.bin').read_bytes(),
                          manifest, json.loads((LAB / 'results' / ('fcos_' + name) /
                                                'detections_demo.json').read_text())['detections']))
        rows, arrivals = [], []
        with BoundedLog(directory / 'remote.log') as remote_log:
            worker = subprocess.Popen(['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                                       '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2',
                                       'root@192.168.10.100', 'exec nice -n 10 python3 -u ' + REMOTE +
                                       '/tools/bpu_preview_worker.py ' + run_id + ' --engine ' + args.engine],
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, bufsize=0, start_new_session=True)
            drain_stderr(worker, remote_log)
            try:
                ready = receive_json(worker.stdout)
                if ready.get('engine') != args.engine or ready.get('ready') is not True:
                    raise ValueError('Wrong worker handshake')
                started = time.monotonic()
                next_send = started
                for sequence in range(1, args.frames + 1):
                    if time.monotonic() - started > 600:
                        raise TimeoutError('Benchmark exceeded ten minutes')
                    payload, manifest, expected = cases[(sequence - 1) % len(cases)]
                    if args.max_fps:
                        time.sleep(max(0, next_send - time.monotonic()))
                    stamp = time.time_ns()
                    sent = time.monotonic()
                    next_send = sent + (1 / args.max_fps if args.max_fps else 0)
                    packet = frame_packet(sequence, stamp, manifest['original_width'],
                                          manifest['original_height'], payload, not args.full_nv12)
                    write_all(worker.stdin, packet)
                    actual = receive_json(worker.stdout)
                    done = time.monotonic()
                    if (actual['sequence'] != sequence or actual['stamp_ns'] != stamp or
                            actual['motion_eligible'] is not False or actual['demo_only'] is not True):
                        raise ValueError('Frame identity/eligibility mismatch')
                    detections = actual['detections']
                    if len(detections) != len(expected):
                        raise ValueError('Golden detection count changed')
                    for detection, reference in zip(detections, expected):
                        if detection['class_id'] != reference['class_id']:
                            raise ValueError('Golden class changed')
                        np.testing.assert_allclose(detection['confidence'], reference['confidence'], rtol=0, atol=1e-6)
                        np.testing.assert_allclose(detection['bbox_model_px'], reference['bbox_model_px'], rtol=0, atol=1e-5)
                    actual.update(roundtrip_ms=(done - sent) * 1000, completed_monotonic=done,
                                  wire_bytes=len(packet))
                    rows.append(actual)
                    arrivals.append(done)
                    if sequence == 1 or sequence % 100 == 0:
                        print('FRAME %d roundtrip_ms=%.2f worker_ms=%.2f' %
                              (sequence, actual['roundtrip_ms'], actual['worker_ms']), flush=True)
                worker.stdin.close()
                if worker.wait(timeout=5) != 0:
                    raise RuntimeError('Worker did not cleanly exit on EOF')
            finally:
                stop_worker(worker)
        measured = rows[1:] or rows  # Report cold first frame separately.
        report = dict(run_id=run_id, engine=args.engine, frames=len(rows), max_fps=args.max_fps,
                      omit_constant_padding=not args.full_nv12,
                      handshake=ready, golden_detections_passed=True, camera_live=False,
                      throughput_fps=(len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 else 0,
                      first_frame=rows[0], timing={key: summary([r[key] for r in measured]) for key in
                          ('roundtrip_ms', 'worker_ms', 'vendor_infer_ms', 'decode_ms')})
        (directory / 'results.json').write_text(json.dumps(rows, indent=2) + '\n')
        (directory / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
