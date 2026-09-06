#!/usr/bin/env python3
"""Prepare bounded, packed NV12 Y/UV planes for an offline FCOS smoke test.

This explicitly records a proposed resize/padding/color contract. It is not
claimed to reproduce the HEAL training preprocessor until numerical validation.
"""
import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--name", required=True)
    args = parser.parse_args()
    if not args.name.isidentifier():
        raise ValueError("Name must be a simple identifier")
    if os.environ.get("ROBOT_OPTIMIZED_SANDBOX") != "1":
        raise RuntimeError("Use scripts/optimized.sh run")
    cv2.setNumThreads(1)
    source = Path(args.image).resolve(strict=True)
    image = cv2.imread(str(source), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("Cannot decode source image")
    height, width = image.shape[:2]
    size = 896
    scale = min(size / width, size / height)
    target_width, target_height = round(width * scale), round(height * scale)
    resized = cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
    padded = np.zeros((size, size, 3), dtype=np.uint8)
    padded[:target_height, :target_width] = resized
    i420 = cv2.cvtColor(padded, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    pixel_count = size * size
    y = i420[:pixel_count]
    uv = np.stack((i420[pixel_count:pixel_count * 5 // 4],
                   i420[pixel_count * 5 // 4:]), axis=-1).reshape(-1)
    destination = Path(__file__).resolve().parents[1] / "inputs" / args.name
    destination.mkdir(parents=True, exist_ok=False)
    y.tofile(str(destination / "y.bin"))
    uv.tofile(str(destination / "uv.bin"))
    cv2.imwrite(str(destination / "input_rgb.png"), padded)
    report = {"source": str(source), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
              "original_width": width, "original_height": height,
              "resized_width": target_width, "resized_height": target_height,
              "scale_x": target_width / width, "scale_y": target_height / height,
              "pad_left": 0, "pad_top": 0, "pad_bgr": [0, 0, 0],
              "conversion": "OpenCV BGR2YUV_I420 to packed NV12 (BT.601 limited range)",
              "input_shapes": [[1, 896, 896, 1], [1, 448, 448, 2]],
              "input_stride": "802816,896,1,1;401408,896,2,1",
              "input_bytes": [y.size, uv.size], "training_preprocess_equivalence_verified": False}
    with (destination / "input_manifest.json").open("x") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
