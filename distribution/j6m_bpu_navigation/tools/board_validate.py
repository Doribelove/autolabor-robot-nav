#!/usr/bin/env python3
"""Isolated offline checks for the pinned two-stage BPU candidate; no ROS."""
import argparse
import contextlib
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import resource
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = Path('/map/robot_j6m_navigation_20260907/bpu_lab_20260905')
RUNTIME = str(REFERENCE / 'vendor/runtime_4_9_2/lib') + ':/usr/hobot/lib'


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def local_file(relative):
    path = ROOT / relative
    if (Path(relative).is_absolute() or '..' in Path(relative).parts or path.is_symlink()
            or not path.is_file() or ROOT not in path.resolve().parents):
        raise ValueError('Expected a regular candidate file')
    return path


def snapshot():
    result = {'monotonic': time.monotonic()}
    for name in ['ratio', 'users', 'bpu0/task_running']:
        try:
            result[name] = (Path('/sys/devices/system/bpu') / name).read_text()
        except OSError as error:
            result[name] = {'error': str(error)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['preprocess', 'infer'], required=True)
    args = parser.parse_args()
    if ROOT.parent != Path('/map/j6m-dev-b/vision-bpu/conversion_runs'):
        raise RuntimeError('Use the dedicated vision-bpu conversion validation directory')
    if os.environ.get('LD_LIBRARY_PATH') != RUNTIME:
        os.execve(sys.executable, [sys.executable, '-B', *sys.argv],
                  dict(os.environ, LD_LIBRARY_PATH=RUNTIME, PYTHONDONTWRITEBYTECODE='1',
                       OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
    sys.path[:0] = [str(ROOT / 'deps'), str(REFERENCE / 'vendor/python311')]
    import cv2
    import numpy as np
    import PIL
    from board_preprocess import detector_input, classifier_input
    from native_tensor import TensorEngine
    from production_io import detector_output, classifier_output, validate_production_interface
    from vision_two_stage import context_crop

    cv2.setNumThreads(1)
    token = datetime.datetime.now().astimezone().strftime('%Y%m%dT%H%M%S') + '_' + str(os.getpid())
    output = ROOT / 'results' / (args.mode + '_' + token)
    output.mkdir(parents=True)
    report = dict(mode=args.mode, token=token, operator='B/Codex', pid=os.getpid(),
                  started=datetime.datetime.now().astimezone().isoformat(),
                  validator_sha256=digest(Path(__file__)),
                  production_ready=False, motion_eligible=False, int8_accuracy_accepted=False,
                  scope='offline calibration inputs; no camera, ROS, tracking, depth or vehicle control',
                  crop_context_fraction=0.20,
                  versions=dict(numpy=np.__version__, opencv=cv2.__version__, pillow=PIL.__version__),
                  passed=False)
    try:
        samples = json.loads(local_file('samples.json').read_text())
        contract = json.loads(local_file('production_models.json').read_text())
        checked = []
        counts = {}
        for sample in samples:
            path = local_file(sample['path'])
            if digest(path) != sample['source_sha256']:
                raise ValueError('Changed source image')
            frame = cv2.imread(str(path))
            kind = sample['kind']
            value = detector_input(frame)[0] if kind == 'detector_zed' else classifier_input(frame)
            if (list(value.shape) != sample['input_shape']
                    or hashlib.sha256(value.tobytes()).hexdigest() != sample['tensor_sha256']):
                raise ValueError('Board preprocessing differs from calibration: ' + sample['path'])
            counts[kind] = counts.get(kind, 0) + 1
            checked.append(sample)
        report['preprocessing_exact_matches'] = counts
        if args.mode == 'infer':
            dependencies = json.loads(local_file('DEPENDENCIES.json').read_text())
            for item in dependencies['files']:
                if digest(Path(item['path'])) != item['sha256']:
                    raise ValueError('Changed private runtime dependency: ' + item['path'])
            artifacts = json.loads(local_file('MODEL_ARTIFACTS.json').read_text())
            references = json.loads(local_file('REFERENCE.json').read_text())
            report['artifact_manifest'] = artifacts
            report['before'] = snapshot()
            with (REFERENCE / 'preview_worker.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with contextlib.ExitStack() as stack:
                    engines = {}
                    for kind in ['detector_zed', 'classifier']:
                        model = artifacts[kind]
                        if model['onnx_sha256'] != contract['models'][kind]['onnx_sha256']:
                            raise ValueError('HBM source model identity mismatch')
                        engines[kind] = stack.enter_context(TensorEngine(
                            dependencies['executor'], local_file(model['path']),
                            model['name'], model['sha256']))
                        validate_production_interface(engines[kind].metadata, kind)
                    report['engines'] = {kind: dict(metadata=engine.metadata, load_ms=engine.load_ms)
                                         for kind, engine in engines.items()}
                    telemetry = []
                    stop = threading.Event()

                    def monitor():
                        while not stop.is_set():
                            telemetry.append(snapshot())
                            stop.wait(.05)

                    thread = threading.Thread(target=monitor, daemon=True)
                    thread.start()
                    try:
                        report['comparisons'] = []
                        source_frames = []
                        last_inputs = {}
                        for kind in ['detector_zed', 'classifier']:
                            ref_by_source = {r['sample_path']: r for r in references['models'][kind]['samples']}
                            for index, sample in enumerate(s for s in checked if s['kind'] == kind):
                                frame = cv2.imread(str(local_file(sample['path'])))
                                if kind == 'detector_zed':
                                    tensor, geometry = detector_input(frame)
                                else:
                                    tensor = classifier_input(frame)
                                last_inputs[kind] = tensor
                                outputs, timing = engines[kind].infer([tensor], timeout_ms=8000)
                                actual = outputs[0]
                                expected_record = ref_by_source[sample['path']]
                                ref_path = local_file(expected_record['path'])
                                if digest(ref_path) != expected_record['sha256']:
                                    raise ValueError('Changed CPU reference')
                                expected = np.load(ref_path, allow_pickle=False)
                                if actual.shape != expected.shape or not np.isfinite(actual).all():
                                    raise ValueError('Invalid model output')
                                np.save(output / ('%s_%03d.npy' % (kind, index)), actual, allow_pickle=False)
                                a, b = actual.astype(np.float64).ravel(), expected.astype(np.float64).ravel()
                                denominator = float(np.linalg.norm(a) * np.linalg.norm(b))
                                item = dict(kind=kind, source=sample['path'], source_sha256=sample['source_sha256'],
                                            timing=timing, max_abs_error=float(np.max(np.abs(a - b))),
                                            mean_abs_error=float(np.mean(np.abs(a - b))),
                                            cosine=float(np.dot(a, b) / denominator) if denominator else None)
                                if kind == 'detector_zed':
                                    boxes = detector_output(actual, geometry)
                                    item.update(bpu_detections=boxes.tolist(), fp32_detections=expected_record['detections'])
                                    if len(source_frames) < 3 and len(boxes):
                                        source_frames.append((sample, frame))
                                else:
                                    probabilities = classifier_output(actual)
                                    item.update(probabilities=probabilities.tolist(), label=sample['label'],
                                                prediction=contract['models'][kind]['classes'][int(probabilities.argmax())],
                                                fp32_prediction=expected_record['prediction'])
                                report['comparisons'].append(item)

                        def run_frame(frame, crop_count=None):
                            started = time.perf_counter()
                            tensor, geometry = detector_input(frame)
                            raw, detection_timing = engines['detector_zed'].infer([tensor], timeout_ms=8000)
                            boxes = detector_output(raw[0], geometry)
                            boxes = boxes[:8] if crop_count is None else [boxes[i % len(boxes)] for i in range(crop_count)]
                            results = []
                            for box in boxes:
                                crop, bounds = context_crop(frame, tuple(box[:4]), .20)
                                raw, timing = engines['classifier'].infer([classifier_input(crop)], timeout_ms=8000)
                                probabilities = classifier_output(raw[0])
                                results.append(dict(box=np.asarray(box).tolist(), crop_bounds=list(bounds), probabilities=probabilities.tolist(),
                                                    material=contract['models']['classifier']['classes'][int(probabilities.argmax())],
                                                    timing=timing))
                            return dict(frame_ms=(time.perf_counter() - started) * 1000,
                                        detector_timing=detection_timing, detections=results)

                        report['two_stage'] = [dict(source=s['path'], source_sha256=s['source_sha256'],
                                                    result=run_frame(frame)) for s, frame in source_frames]
                        if not source_frames or not any(r['result']['detections'] for r in report['two_stage']):
                            raise ValueError('No actual detection-to-classification result')
                        report['performance'] = []
                        for crops in [0, 1, 4, 8]:
                            frame = source_frames[0][1]
                            for _ in range(3):
                                run_frame(frame, crops)
                            times = [run_frame(frame, crops)['frame_ms'] for _ in range(20)]
                            report['performance'].append(dict(crops=crops, warmup=3, iterations=20,
                                latencies_ms=times,
                                workload='fixed image; repeat detected crops when more crops are requested',
                                excludes='image acquisition, decode, network, depth pairing and tracking',
                                p50_ms=float(np.percentile(times, 50)), p95_ms=float(np.percentile(times, 95)),
                                p99_ms=float(np.percentile(times, 99)), mean_ms=float(np.mean(times))))
                        report['rejected_invalid_requests'] = []
                        report['recovery_after_invalid_requests'] = []
                        for kind, engine in engines.items():
                            valid = last_inputs[kind]
                            nan = valid.copy()
                            nan.flat[0] = np.nan
                            tests = [('wrong_shape', [valid[:, :, :-1, :]], 2000),
                                     ('wrong_dtype', [valid.astype(np.uint8)], 2000),
                                     ('nonfinite', [nan], 2000), ('wrong_count', [], 2000),
                                     ('invalid_timeout', [valid], 0)]
                            for case, tensors, timeout in tests:
                                try:
                                    engine.infer(tensors, timeout_ms=timeout)
                                except ValueError:
                                    report['rejected_invalid_requests'].append(kind + ':' + case)
                                else:
                                    raise AssertionError('Invalid request was not rejected: ' + case)
                            recovered, recovery_timing = engine.infer([valid], timeout_ms=8000)
                            last_index = counts[kind] - 1
                            previous = np.load(output / ('%s_%03d.npy' % (kind, last_index)), allow_pickle=False)
                            if not np.array_equal(recovered[0], previous):
                                raise AssertionError('Valid inference changed after invalid requests: ' + kind)
                            report['recovery_after_invalid_requests'].append(dict(
                                kind=kind, output_exact_match=True, timing=recovery_timing))
                    finally:
                        stop.set()
                        thread.join(timeout=2)
                        report['bpu_telemetry'] = telemetry
            report['after'] = snapshot()
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
        report['traceback'] = traceback.format_exc()
    finally:
        report['finished'] = datetime.datetime.now().astimezone().isoformat()
        report['max_rss_kib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps(dict(report=str(output / 'report.json'), passed=report['passed'],
                              error=report.get('error'), production_ready=False, motion_eligible=False)), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
