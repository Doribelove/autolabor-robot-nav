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
        scores = sigmoid(outputs[level]) * sigmoid(outputs[10 + level])
        class_ids, ys, xs = np.nonzero(scores >= threshold)
        if not len(class_ids):
            continue
        confidence = scores[class_ids, ys, xs]
        order = np.argsort(-confidence, kind="stable")[:1000]
        for candidate in order:
            label, y, x = int(class_ids[candidate]), int(ys[candidate]), int(xs[candidate])
            distances = outputs[5 + level][:, y, x].astype(np.float64) * stride
            if (distances < 0).any():
                raise ValueError("Negative LTRB distance: decoder convention does not match model")
            cx, cy = (x + 0.5) * stride, (y + 0.5) * stride
            box = np.clip([cx - distances[0], cy - distances[1],
                           cx + distances[2], cy + distances[3]], 0, 896)
            if box[2] > box[0] and box[3] > box[1]:
                candidates.append((float(confidence[candidate]), label, box))
    candidates.sort(key=lambda item: -item[0])
    kept = []
    for confidence, label, box in candidates:
        suppressed = False
        for previous in kept:
            if label != previous["class_id"]:
                continue
            other = np.array(previous["bbox_model_px"])
            area = (box[2] - box[0]) * (box[3] - box[1])
            other_area = (other[2] - other[0]) * (other[3] - other[1])
            intersection = np.prod(np.maximum(0, np.minimum(box[2:], other[2:]) - np.maximum(box[:2], other[:2])))
            iou = intersection / max(area + other_area - intersection, 1e-12)
            if iou > nms_iou:
                suppressed = True
                break
        if not suppressed:
            kept.append({"class_id": label, "confidence": confidence, "bbox_model_px": box.tolist()})
            if len(kept) == maximum:
                break
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
