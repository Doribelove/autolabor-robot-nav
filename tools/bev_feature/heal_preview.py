#!/usr/bin/env python3
"""Run the official six-slot HBM with four real views, two excluded views.

Independent BPU experiment. Does not publish ROS observations or cleaning goals.
Native model retains its trained nuScenes geometry/classes, including 0.8 m grid.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import signal
import struct
import subprocess
import sys
import time
import urllib.request

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'distribution/j6m_bpu_navigation'),
               str(ROOT/'distribution/j6m_bpu_navigation/tools')]
from surround.ground_protocol import decode_view_frame
from surround.detection_views import DetectionMasks
from tensor_wire import send, receive, pack_arrays, unpack_arrays
from heal_reference import HealCPU, ORDER, CLASSES, MODEL_SHA, MODEL_NAME, prepare_images, quantize_grids

SSH = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'root@192.168.10.100']
STOP = False


def write_json(path, value):
    tmp = path.with_suffix('.json.tmp')
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    tmp.replace(path)


def remote(args, **kwargs):
    return subprocess.run(SSH+[shlex.join(args)], check=True, timeout=60, **kwargs)


def deploy(output):
    old = json.loads((ROOT/'validation/oe391_20261101/board_runtime_matched/runtime_manifest.json').read_text())
    sdk = Path(old['sdk'])
    lib = output/'libj6m_tensor.so'
    deps = sdk/'samples/ucp_tutorial/deps_aarch64/ucp'
    build = ['g++', '-std=c++14', '-O3', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
             '-DJ6M_DNN_EXPECTED_VERSION="3.15.8_(4.11.11 HBRT)"', '-I'+str(deps/'include'),
             str(ROOT/'distribution/j6m_bpu_navigation/native/tensor_executor.cpp'),
             '-L'+str(deps/'lib'), '-ldnn', '-lhbucp', '-Wl,--allow-shlib-undefined', '-o', str(lib)]
    subprocess.run(build, check=True, timeout=60)
    target = remote(['mktemp', '-d', '/tmp/conbine-heal-XXXXXXXX'], capture_output=True, text=True).stdout.strip()
    if not target.startswith('/tmp/conbine-heal-'):
        raise ValueError('Unexpected board directory')
    runtime = target
    remote(['mkdir', target+'/lib'])
    model = ROOT/'runtime/bev_feature_20261101/official/nash_m/compile/model.hbm'
    if hashlib.sha256(model.read_bytes()).hexdigest() != MODEL_SHA:
        raise ValueError('Official model file changed')
    subprocess.run(['scp', '-q', str(model), 'root@192.168.10.100:'+target+'/model.hbm'],
                   check=True, timeout=60)
    subprocess.run(['scp', '-q', *[str(deps/'lib'/n) for n in old['libraries']],
                    'root@192.168.10.100:'+target+'/lib/'], check=True, timeout=60)
    files = [lib, Path(__file__).with_name('heal_board_worker.py'),
             ROOT/'distribution/j6m_bpu_navigation/tools/native_tensor.py',
             ROOT/'distribution/j6m_bpu_navigation/tools/tensor_wire.py']
    subprocess.run(['scp', '-q', *map(str, files), 'root@192.168.10.100:'+target+'/'], check=True, timeout=60)
    hashes = remote(['sha256sum', *[target+'/'+p.name for p in files], runtime+'/model.hbm',
                     *[runtime+'/lib/'+n for n in old['libraries']]], capture_output=True, text=True).stdout
    actual = {Path(line.split()[1]).name: line.split()[0] for line in hashes.splitlines()}
    expected = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    expected.update(old['libraries']); expected['model.hbm'] = MODEL_SHA
    if actual != expected:
        raise ValueError('Board model/runtime/executor digest mismatch')
    manifest = dict(board=target, runtime=runtime, hashes=actual, build=build)
    write_json(output/'deployment.json', manifest)
    return manifest


class Board:
    def __init__(self, deployment, output):
        self.sequence = 0
        self.log = (output/'board.log').open('a')
        cmd = ['python3', '-B', '-u', deployment['board']+'/heal_board_worker.py',
               '--runtime', deployment['runtime'], '--sha256', MODEL_SHA]
        self.process = subprocess.Popen(SSH+[shlex.join(cmd)], stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=self.log, bufsize=0)
        try:
            ready, payload = receive(self.process.stdout, timeout=20)
            if payload or not ready['ready'] or ready['model_sha256'] != MODEL_SHA:
                raise ValueError('Wrong board model')
            self.metadata = ready['metadata']
            if self.metadata['model_name'] != MODEL_NAME:
                raise ValueError('Wrong board model name')
        except Exception:
            self.close()
            raise

    def infer(self, tensors):
        self.sequence += 1
        desc, raw = pack_arrays(tensors)
        unpack_arrays(desc, raw, self.metadata['inputs'])
        start = time.monotonic()
        send(self.process.stdin, dict(sequence=self.sequence, tensors=desc), raw, timeout=10)
        head, raw = receive(self.process.stdout, timeout=10)
        if head['sequence'] != self.sequence:
            raise ValueError('Wrong inference sequence')
        arrays = unpack_arrays(head['tensors'], raw, self.metadata['outputs'], outputs=True)
        return [a.copy() for a in arrays], dict(head['timing'], transport_infer_ms=(time.monotonic()-start)*1000)

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=3)
        self.log.close()


def render(output, meta, images, bev, heat, rows, timing):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))
    for ax, name in zip(axes.ravel()[:4], ORDER):
        ax.imshow(cv2.cvtColor(images[name], cv2.COLOR_BGR2RGB))
        # Mark the retained bottom crop in the original upright view.
        ax.axhline(images[name].shape[0]*140/396, color='yellow', linewidth=1)
        ax.set_title(name+' | source #'+str(meta['sources'][name]['sequence']))
        ax.axis('off')
    xmin, xmax = meta['x_range_m']; ymin, ymax = meta['y_range_m']
    ax = axes.ravel()[4]
    ax.imshow(cv2.cvtColor(bev, cv2.COLOR_BGR2RGB), extent=[ymax, ymin, xmin, xmax])
    ax.set_title('Existing RGB ground projection (CPU)')
    ax.set_xlabel('Y left (m)'); ax.set_ylabel('X forward (m)')
    ax = axes.ravel()[5]
    probability = heat.max(axis=0)
    # Heatmap head axes are [Y, X]; transpose into forward-up / left-left display.
    grid = ax.imshow(probability.T[::-1, ::-1], extent=[50.8, -51.6, -51.6, 50.8],
                     vmin=0, vmax=1, interpolation='nearest', cmap='magma')
    ax.set_xlim(ymax, ymin); ax.set_ylim(xmin, xmax)
    ax.set_title('BPU traffic-class heatmap | 0.8 m grid')
    ax.set_xlabel('Y left (m)'); ax.set_ylabel('X forward (m)'); ax.grid(alpha=.3)
    local = [r for r in rows if xmin <= r['x_ground'] <= xmax and ymin <= r['y_ground'] <= ymax]
    for row in local[:12]:
        ax.plot(row['y_ground'], row['x_ground'], 'c+')
        ax.annotate('%s %.2f'%(row['class_name'], row['score']),
                    (row['y_ground'], row['x_ground']), color='cyan', fontsize=6)
    ax.plot(0, 0, 'go'); ax.annotate('ego', (0,0), color='lime', fontsize=8)
    fig.colorbar(grid, ax=ax, fraction=.04)
    fig.suptitle('BEV-VTv2 on J6M | 4 real views + 2 excluded slots | traffic model, NOT trash\n'
                 'Pre-NMS local candidates >=0.1: %d | BPU call %.1f ms | source group #%s'%(
                     len(local), timing['vendor_infer_ms'], meta['fusion_sequence']))
    fig.tight_layout()
    temporary = output/'comparison.tmp.png'; fig.savefig(temporary, dpi=110); plt.close(fig)
    temporary.replace(output/'comparison.png')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, default=ROOT/'runtime/heal_source_20261101/Heal-main')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--deployment', type=Path)
    p.add_argument('--deploy', action='store_true')
    p.add_argument('--capture', type=Path, help='Replay one saved MV01 packet')
    p.add_argument('--frames', type=int, default=10, help='0 means continuous live preview')
    p.add_argument('--ablation', action='store_true')
    p.add_argument('--display', action='store_true')
    a = p.parse_args()
    if a.frames < 0 or (a.capture and a.frames == 0):
        p.error('Invalid frame count for this source')
    a.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(2); cv2.setNumThreads(1)
    heal = HealCPU(a.source)
    masks = DetectionMasks(ROOT/'config/bev_detection_masks.json')
    deployment = deploy(a.output) if a.deploy else json.loads(a.deployment.read_text())
    board = Board(deployment, a.output)
    write_json(a.output/'model_metadata.json', board.metadata)
    write_json(a.output/'source_methods.json', dict(source=str(a.source), hashes=heal.hashes,
        cpu_torch_version=torch.__version__, network_runtime='J6M HBM; no PyTorch network emulation'))
    viewer = None; records = []; calibration = None; last_group = None; total = 0
    try:
        if a.display:
            viewer = subprocess.Popen(['/usr/bin/python3', str(Path(__file__).with_name('heal_viewer.py')), str(a.output)])
        while not STOP and (not a.frames or total < a.frames):
            if viewer and viewer.poll() is not None:
                break
            start = time.monotonic()
            packet = (a.capture.read_bytes() if a.capture else
                      urllib.request.urlopen('http://127.0.0.1:18883/views', timeout=5).read())
            replay_now = None
            if a.capture:
                # Validate against the saved packet's own time for offline replay.
                # Never rewrite acquisition timestamps or report replay as live latency.
                _, header_length, _ = struct.unpack('!4sII', packet[:12])
                body = packet[12+header_length:]
                _, meta_length, _ = struct.unpack('!4sII', body[:12])
                replay_now = json.loads(body[12:12+meta_length])['produced_epoch_s']
            meta, bev, _, images = decode_view_frame(packet, now=replay_now)
            group = (meta['fusion_session'], meta['fusion_sequence'])
            if not a.capture and group == last_group:
                time.sleep(.05); continue
            last_group = group
            geometry_key = json.dumps([meta[k] for k in ('camera_image_sizes', 'projection_intrinsics',
                                                         'T_ground_from_upright_optical')], sort_keys=True)
            if geometry_key != calibration:
                generated = time.monotonic()
                grids, stats, support = heal.grids(meta)
                quantized = quantize_grids(grids, board.metadata['inputs'][-2:])
                write_json(a.output/'geometry.json', dict(stats, source_metadata=meta,
                    preparation_ms=(time.monotonic()-generated)*1000,
                    frame_id=meta['frame_id'], camera_order=list(ORDER)+['excluded', 'excluded'],
                    source_grid='HEAL linspace(-51.2,51.2,128); decoder cell pitch 0.8 m unchanged'))
                np.savez(a.output/'sampling.npz', points0=quantized[0], points1=quantized[1], support=support)
                calibration = geometry_key
            inputs, crops = prepare_images(masks.apply(images))
            inputs.extend(quantized)
            if not records:
                (a.output/'capture.mv01').write_bytes(packet)
                for i, value in enumerate(inputs):
                    value.tofile(str(a.output/('input_%02d.bin'%i)))
                for name, image in images.items():
                    cv2.imwrite(str(a.output/(name+'.png')), image)
                    cv2.imwrite(str(a.output/(name+'_model_crop.png')), cv2.cvtColor(crops[name], cv2.COLOR_RGB2BGR))
                for _ in range(2):
                    board.infer(inputs)
            raw, timing = board.infer(inputs)
            rows, heat = heal.decode(raw)
            timing['request_to_decode_ms'] = (time.monotonic()-start)*1000
            # This is NOT exposure-to-map latency; the mixed source clocks are unverified.
            timing['fusion_publish_to_decode_ms'] = None if a.capture else (time.time_ns()-meta['at_epoch_ns'])/1e6
            if not records:
                np.savez(a.output/'first_outputs.npz', **{'head%d'%i:v for i,v in enumerate(raw)})
            if not records and a.ablation:
                cases = []
                for camera in (*ORDER, 'excluded_slots'):
                    variant = [v.copy() for v in inputs]
                    indices = [ORDER.index(camera)] if camera in ORDER else [4, 5]
                    for slot in indices:
                        variant[slot*2].fill(32 if camera == 'excluded_slots' else 128)
                        variant[slot*2+1].fill(16 if camera == 'excluded_slots' else 128)
                    changed, _ = board.infer(variant)
                    differences = [float(np.max(np.abs(x-y))) for x,y in zip(raw, changed)]
                    cases.append(dict(camera=camera, max_difference_per_head=differences))
                write_json(a.output/'ablation.json', cases)
                if any(cases[-1]['max_difference_per_head']):
                    raise ValueError('Excluded camera images affect the official HBM outputs')
            record = dict(sequence=meta['fusion_sequence'], sources=meta['sources'], timing=timing,
                candidates_pre_nms=rows, class_max_probabilities=dict(zip(CLASSES, map(float, heat.max(axis=(1,2))))),
                source_exposure_synchronized=meta['source_exposure_synchronized'],
                detection_mask_sha256=masks.sha256,
                frame_id=meta['frame_id'], model_sha256=MODEL_SHA, updated_epoch_s=time.time(),
                stage='official_traffic_model_four_real_views_six_input_slots', trash_model=False,
                input_mode='saved_packet_replay' if a.capture else 'live', startup_frame=total == 0)
            records.append(record)
            total += 1
            if len(records) > 100:
                records.pop(0)
            write_json(a.output/'latest.json', record)
            if not a.frames or total == 1 or total == a.frames:
                previews = masks.annotate(images, {n: [] for n in images})
                render(a.output, meta, previews, bev, heat, rows, timing)
            if not a.frames:
                time.sleep(max(0, 1.-(time.monotonic()-start)))
        write_json(a.output/'measurements.json', records)
        print(json.dumps(dict(frames=len(records), output=str(a.output),
            vendor_call_median_ms=float(np.median([r['timing']['vendor_infer_ms'] for r in records])),
            transport_infer_median_ms=float(np.median([r['timing']['transport_infer_ms'] for r in records])))))
    finally:
        board.close()
        if viewer and viewer.poll() is None:
            viewer.terminate(); viewer.wait(timeout=3)


def stop(*_):
    global STOP
    STOP = True


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    main()
