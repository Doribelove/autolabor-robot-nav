#!/usr/bin/env python3
"""CPU boundary verification with actual pinned FP32 ONNX and training tensors."""
import argparse
import json
import os
from pathlib import Path

os.environ['CUDA_VISIBLE_DEVICES'] = ''
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
os.environ['YOLO_AUTOINSTALL'] = 'false'

from production_ptq import verify_workspace, digest, write_json
from production_io import preprocessors, detector_input, classifier_input, detector_output, classifier_output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--ultralytics-root', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if args.report.exists():
        raise FileExistsError('Preserve earlier verification reports')
    registry = verify_workspace(args.workspace, require_calibration=True)
    letterbox, transform = preprocessors(args.ultralytics_root)
    import cv2
    import numpy as np
    import onnxruntime as ort
    import torch
    from ultralytics.utils.nms import non_max_suppression
    from ultralytics.utils.ops import scale_boxes
    cv2.setNumThreads(1)
    torch.set_num_threads(1)
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    counts, checks = {}, []
    for kind, spec in registry['models'].items():
        calibration = args.workspace / 'calibration' / kind
        manifest = json.loads((calibration / 'manifest.json').read_text())
        if kind == 'classifier' and repr(transform) != manifest['classifier_transform']:
            raise ValueError('Classifier runtime transform differs from calibration')
        session = ort.InferenceSession(str(args.workspace / 'models' / (kind + '.onnx')),
                                       sess_options=options, providers=['CPUExecutionProvider'])
        if session.get_providers() != ['CPUExecutionProvider']:
            raise ValueError('Only CPU FP32 is allowed in this boundary check')
        selected = set()
        counts[kind] = 0
        for index, sample in enumerate(manifest['samples']):
            source = Path(sample['source'])
            if digest(source) != sample['source_sha256']:
                raise ValueError('Training source changed')
            frame = cv2.imread(str(source))
            if kind == 'detector_zed':
                tensor, geometry = detector_input(frame, letterbox)
            else:
                tensor = classifier_input(frame, transform)
            saved = np.load(calibration / sample['path'], allow_pickle=False)
            if not np.array_equal(tensor, saved):
                raise ValueError('CPU preprocessing differs from the calibrated tensor')
            counts[kind] += 1
            should_check = index < 3 if kind == 'detector_zed' else sample['class_name'] not in selected
            if not should_check:
                continue
            selected.add(sample['class_name'])
            raw = session.run([spec['output_name']], {spec['input_name']: tensor})[0]
            if kind == 'detector_zed':
                actual = detector_output(raw, geometry)
                original = non_max_suppression(torch.from_numpy(raw.copy()), conf_thres=.20,
                                               iou_thres=.60, max_det=50, nc=1)[0]
                ratio, left, top, h, w = geometry
                original[:, :4] = scale_boxes((576, 1024), original[:, :4], (h, w),
                                              ratio_pad=((ratio, ratio), (left, top)))
                expected = original[:, :5].numpy()
                if actual.shape != expected.shape or not np.allclose(actual, expected, atol=1e-4, rtol=1e-6):
                    raise ValueError('YOLO decoding/NMS differs from the original implementation')
                check = {'kind': kind, 'sample': sample['path'], 'detections': len(actual),
                         'max_abs_error': float(np.abs(actual - expected).max()) if actual.size else 0}
            else:
                actual = classifier_output(raw)
                if not np.array_equal(actual, raw[0]):
                    raise ValueError('Classifier probabilities were changed')
                check = {'kind': kind, 'sample': sample['path'], 'source_class': sample['class_name'],
                         'probability_sum': float(actual.sum())}
            checks.append(dict(check, passed=True))
        del session
    if torch.cuda.is_initialized():
        raise RuntimeError('Boundary check unexpectedly initialized CUDA')
    report = {'preprocessing_exact_matches': counts, 'fp32_output_checks': checks,
              'onnxruntime': ort.__version__, 'cuda_initialized': False,
              'scope': 'training-input preprocessing and CPU FP32 output boundary only',
              'int8_accuracy_accepted': False, 'hbm_board_accepted': False, 'production_ready': False}
    write_json(args.report, report)
    print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
