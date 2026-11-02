"""RGB-only adapter for the already installed FCOS 896 NV12 HBM.

Keep the existing lab resize/pad and decoder conventions. Their numerical
equivalence to the HEAL training pipeline has not been independently verified.
"""
import cv2
import numpy as np

CLASS_NAMES = (
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush')


def prepare_rgb(image):
    """Aspect-preserving top-left RGB padding; return NV12 and exact XY scales."""
    height, width = image.shape[:2]
    scale = min(896 / width, 896 / height)
    tw, th = max(1, round(width*scale)), max(1, round(height*scale))
    padded = np.zeros((896, 896, 3), np.uint8)
    padded[:th, :tw] = cv2.resize(image, (tw, th), interpolation=cv2.INTER_LINEAR)
    i420 = cv2.cvtColor(padded, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    count = 896*896
    nv12 = np.empty(count*3//2, np.uint8)
    nv12[:count] = i420[:count]
    nv12[count::2] = i420[count:count*5//4]
    nv12[count+1::2] = i420[count*5//4:]
    return nv12.tobytes(), dict(width=width, height=height, scale_x=tw/width, scale_y=th/height)


def source_boxes(detections, geometry):
    """Undo actual rounded resize; discard padding-only or invalid detections."""
    rows = []
    w, h = geometry['width'], geometry['height']
    for item in detections:
        box = np.asarray(item['bbox_model_px'], dtype=float).copy()
        if box.shape != (4,) or not np.isfinite(box).all():
            continue
        box[[0, 2]] = np.clip(box[[0, 2]] / geometry['scale_x'], 0, w)
        box[[1, 3]] = np.clip(box[[1, 3]] / geometry['scale_y'], 0, h)
        label, score = int(item['class_id']), float(item['confidence'])
        if box[2] <= box[0] or box[3] <= box[1] or not 0 <= label < 80 or not 0 <= score <= 1:
            continue
        rows.append([*box.tolist(), score, label])
    return rows
