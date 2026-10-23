"""CPU input/output boundary for the pinned YOLO11-GAM and five-class ONNX.

These functions do not create a ROS backend, replace BoT-SORT, or grant motion.
The transforms reuse the custom implementation verified during calibration.
"""
import importlib
from pathlib import Path
import sys

import numpy as np

from production_ptq import contract, digest

AUGMENT_SHA = '5d1e76fc1126c053ab74cc44551ca99bf933e913a44c1815a5e774e51db1656b'


def preprocessors(ultralytics_root):
    root = Path(ultralytics_root).resolve(strict=True)
    if digest(root / 'ultralytics/data/augment.py') != AUGMENT_SHA:
        raise ValueError('Custom preprocessing source changed; repeat calibration/equivalence')
    sys.path.insert(0, str(root))
    package = importlib.import_module('ultralytics')
    if root not in Path(package.__file__).resolve().parents:
        raise ValueError('A different Ultralytics package is already imported')
    from ultralytics.data.augment import LetterBox, classify_transforms
    return LetterBox(new_shape=(576, 1024), auto=False, stride=32), classify_transforms(size=224)


def detector_input(image_bgr, letterbox):
    if (not isinstance(image_bgr, np.ndarray) or image_bgr.dtype != np.uint8
            or image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or min(image_bgr.shape[:2]) <= 0):
        raise ValueError('Expected a nonempty BGR U8 image')
    h, w = image_bgr.shape[:2]
    ratio = min(576 / h, 1024 / w)
    left = round((1024 - round(w * ratio)) / 2 - 0.1)
    top = round((576 - round(h * ratio)) / 2 - 0.1)
    frame = letterbox(image=image_bgr)
    tensor = np.ascontiguousarray(frame[:, :, ::-1].transpose(2, 0, 1)[None],
                                  dtype=np.float32) / np.float32(255)
    if list(tensor.shape) != contract()['models']['detector_zed']['input_shape']:
        raise ValueError('Detector preprocessing shape mismatch')
    return tensor, (ratio, left, top, h, w)


def classifier_input(crop_bgr, transform):
    import cv2
    from PIL import Image
    if (not isinstance(crop_bgr, np.ndarray) or crop_bgr.dtype != np.uint8
            or crop_bgr.ndim != 3 or crop_bgr.shape[2] != 3 or min(crop_bgr.shape[:2]) <= 0):
        raise ValueError('Expected a nonempty BGR U8 crop')
    value = transform(Image.fromarray(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)))
    tensor = value.unsqueeze(0).numpy().astype(np.float32)
    if list(tensor.shape) != contract()['models']['classifier']['input_shape']:
        raise ValueError('Classifier preprocessing shape mismatch')
    return tensor


def validate_production_interface(metadata, kind):
    spec = contract()['models'][kind]
    if len(metadata['inputs']) != 1 or len(metadata['outputs']) != 1:
        raise ValueError('Compiled model changed the single input/output interface')
    for side, name, shape in [('inputs', spec['input_name'], spec['input_shape']),
                              ('outputs', spec['output_name'], spec['output_shape'])]:
        tensor = metadata[side][0]
        if tensor['name'] != name or tensor['shape'] != shape:
            raise ValueError('Compiled tensor name/shape differs from pinned ONNX')
    if metadata['inputs'][0]['type'] != 7 or metadata['inputs'][0]['quant'] != 0:
        raise ValueError('Expected unquantized normalized F32 runtime featuremap')


def detector_output(raw, geometry, confidence=0.20, iou=0.60, max_detections=50):
    if (not isinstance(raw, np.ndarray) or raw.dtype != np.float32
            or list(raw.shape) != contract()['models']['detector_zed']['output_shape']
            or not np.isfinite(raw).all()):
        raise ValueError('Expected YOLO11 single-class [1,5,12096] F32 output')
    if (not 0 <= confidence <= 1 or not 0 <= iou <= 1
            or type(max_detections) is not int or not 1 <= max_detections <= 300):
        raise ValueError('Invalid detection thresholds/budget')
    rows = raw[0].T
    # CPU FP32 ONNX sigmoid can produce -5.96e-8 close to zero. Accept only
    # floating roundoff, then restore the mathematical probability interval.
    if np.any(rows[:, 4] < -1e-6) or np.any(rows[:, 4] > 1 + 1e-6) or np.any(rows[:, 2:4] < 0):
        raise ValueError('Invalid detector confidence or box dimensions')
    rows = rows.copy()
    rows[:, 4] = np.clip(rows[:, 4], 0, 1)
    rows = rows[rows[:, 4] > confidence]
    boxes = np.column_stack((rows[:, :2] - rows[:, 2:4] / 2,
                             rows[:, :2] + rows[:, 2:4] / 2))
    scores = rows[:, 4]
    order = np.argsort(-scores, kind='stable')
    keep = []
    areas = np.prod(np.maximum(0, boxes[:, 2:] - boxes[:, :2]), axis=1)
    while len(order) and len(keep) < max_detections:
        chosen = int(order[0]); keep.append(chosen)
        other = order[1:]
        intersection = np.prod(np.maximum(0, np.minimum(boxes[chosen, 2:], boxes[other, 2:])
                                         - np.maximum(boxes[chosen, :2], boxes[other, :2])), axis=1)
        union = areas[chosen] + areas[other] - intersection
        overlap = np.divide(intersection, union, out=np.zeros_like(intersection), where=union > 0)
        order = other[overlap <= iou]
    result = np.column_stack((boxes[keep], scores[keep])).astype(np.float32)
    ratio, left, top, height, width = geometry
    if not np.isfinite(ratio) or ratio <= 0 or height <= 0 or width <= 0:
        raise ValueError('Invalid source image geometry')
    result[:, [0, 2]] = np.clip((result[:, [0, 2]] - left) / ratio, 0, width)
    result[:, [1, 3]] = np.clip((result[:, [1, 3]] - top) / ratio, 0, height)
    return result


def classifier_output(raw):
    if (not isinstance(raw, np.ndarray) or raw.dtype != np.float32 or raw.shape != (1, 5)
            or not np.isfinite(raw).all() or np.any(raw < 0) or np.any(raw > 1)
            or not np.allclose(raw.sum(axis=1), 1.0, atol=1e-4, rtol=0)):
        raise ValueError('Expected five probabilities in the pinned material order')
    # The exported classify head already applies softmax; do not apply it twice.
    return raw[0].copy()
