#!/usr/bin/env python3
"""Offline FCOS decoder for inspecting dequantized, unpadded vendor dumps.

The stride-normalized LTRB and score-product convention is an explicit demo
assumption, not a verified reproduction of HEAL's exact FCOSDecoder. No ROS.
"""
import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
import argparse
import json
from pathlib import Path
import numpy as np

SIZES = (112, 56, 28, 14, 7)
STRIDES = (8, 16, 32, 64, 128)


def sigmoid(values):
    return 1.0 / (1.0 + np.exp(-np.clip(values, -60.0, 60.0)))


def score_candidates(logits, centerness, threshold):
    # Each sigmoid factor must reach the product threshold. Reject background
    # logits before exponentiation, using threshold/2 as a conservative bound
    # with ample float32 rounding margin. The final original score gate remains.
    # np.nonzero preserves the dense decoder's class/y/x stable tie order.
    centers = sigmoid(centerness)
    half = threshold * 0.5
    cutoff = np.log(half / (1.0 - half)) if half > 0 else -np.inf
    possible = logits >= cutoff if cutoff > -60 else np.ones(logits.shape, dtype=bool)
    class_ids, ys, xs = np.nonzero(possible & (centers >= threshold))
    scores = sigmoid(logits[class_ids, ys, xs]) * centers[0, ys, xs]
    keep = scores >= threshold
    return class_ids[keep], ys[keep], xs[keep], scores[keep]


def decode(outputs, threshold=0.2, nms_iou=0.6, maximum=100):
    if len(outputs) != 15 or not 0 < threshold < 1 or not 0 < nms_iou < 1:
        raise ValueError("Invalid FCOS output/threshold contract")
    if not isinstance(maximum, int) or maximum < 1:
        raise ValueError("Invalid result bound")
    for index, output in enumerate(outputs):
        size = SIZES[index % 5]
        channels = 80 if index < 5 else 4 if index < 10 else 1
        if output.shape != (channels, size, size) or not np.isfinite(output).all():
            raise ValueError("Malformed/nonfinite FCOS output " + str(index))
    candidates = []
    for level, (size, stride) in enumerate(zip(SIZES, STRIDES)):
        class_ids, ys, xs, confidence = score_candidates(outputs[level], outputs[10 + level], threshold)
        if not len(class_ids):
            continue
        order = np.argsort(-confidence, kind="stable")[:1000]
        distances = outputs[5 + level][:, ys[order], xs[order]].astype(np.float64) * stride
        if (distances < 0).any():
            raise ValueError("Negative LTRB distance: decoder convention does not match model")
        cx, cy = (xs[order] + 0.5) * stride, (ys[order] + 0.5) * stride
        boxes = np.clip(np.column_stack((cx - distances[0], cy - distances[1],
                                         cx + distances[2], cy + distances[3])), 0, 896)
        for candidate, box in zip(order, boxes):
            if box[2] > box[0] and box[3] > box[1]:
                candidates.append((float(confidence[candidate]), int(class_ids[candidate]), box))
    return suppress_candidates(candidates, nms_iou, maximum)


def suppress_candidates(candidates, nms_iou, maximum):
    """Same stable greedy per-class NMS, vectorized across remaining boxes."""
    candidates.sort(key=lambda item: -item[0])
    if not candidates:
        return []
    boxes = np.array([item[2] for item in candidates], dtype=np.float64)
    labels = np.array([item[1] for item in candidates])
    areas = np.prod(boxes[:, 2:] - boxes[:, :2], axis=1)
    suppressed = np.zeros(len(candidates), dtype=bool)
    kept = []
    for index, (confidence, label, box) in enumerate(candidates):
        if suppressed[index]:
            continue
        kept.append({"class_id": label, "confidence": confidence, "bbox_model_px": box.tolist()})
        if len(kept) == maximum:
            break
        rest = np.flatnonzero((labels[index + 1:] == label) & ~suppressed[index + 1:]) + index + 1
        if rest.size:
            intersection = np.prod(np.maximum(0, np.minimum(box[2:], boxes[rest, 2:]) -
                                                 np.maximum(box[:2], boxes[rest, :2])), axis=1)
            iou = intersection / np.maximum(areas[index] + areas[rest] - intersection, 1e-12)
            suppressed[rest[iou > nms_iou]] = True
    return kept


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name")
    args = parser.parse_args()
    if not args.name.isidentifier() or os.environ.get("ROBOT_OPTIMIZED_SANDBOX") != "1":
        raise ValueError("Use a simple case name through optimized.sh run")
    import cv2
    import yaml
    cv2.setNumThreads(1)
    lab = Path(__file__).resolve().parents[1]
    directory = lab / "results" / ("fcos_" + args.name)
    manifest = json.loads((lab / "inputs" / args.name / "input_manifest.json").read_text())
    outputs = []
    for index in range(15):
        path = directory / "dump" / ("model_infer_output_%d__output_%d.bin" % (index, index))
        data = np.fromfile(str(path), dtype="<f4")
        channels = 80 if index < 5 else 4 if index < 10 else 1
        size = SIZES[index % 5]
        outputs.append(data.reshape(channels, size, size))
    detections = decode(outputs)
    with Path("/home/slam/yolo11/yolo11_GAM/ultralytics/cfg/datasets/coco.yaml").open() as stream:
        classes = yaml.safe_load(stream)["names"]
    image = cv2.imread(manifest["source"])
    for detection in detections:
        box = np.array(detection["bbox_model_px"])
        box[[0, 2]] = np.clip((box[[0, 2]] - manifest["pad_left"]) / manifest["scale_x"],
                             0, manifest["original_width"])
        box[[1, 3]] = np.clip((box[[1, 3]] - manifest["pad_top"]) / manifest["scale_y"],
                             0, manifest["original_height"])
        detection["bbox_source_px"] = box.tolist()
        detection["class_name"] = classes[detection["class_id"]]
        x1, y1, x2, y2 = np.rint(box).astype(int)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 210, 0), 2)
        label = "{} {:.2f}".format(detection["class_name"], detection["confidence"])
        cv2.putText(image, label, (x1, max(20, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 220), 2)
    report = {"detections": detections, "demo_only": True, "motion_eligible": False,
              "decoder": "sigmoid(class)*sigmoid(centerness); LTRB*stride; classwise NMS",
              "threshold": 0.2, "nms_iou": 0.6, "exact_HEAL_decoder_verified": False,
              "input": manifest}
    with (directory / "detections_demo.json").open("x") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    cv2.imwrite(str(directory / "detections_demo.png"), image)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
