#!/usr/bin/env python3
"""Run trusted YOLO weights against one image, a video, or a local camera."""

import argparse
import json
from collections import Counter
from pathlib import Path

import cv2

from autolabor_fod_vision.detector import UltralyticsDetector, annotate_image


IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}
DEFAULT_WEIGHTS = (
    Path(__file__).resolve().parents[3] / "yolo" / "yolo11n.pt"
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="YOLO/CUDA smoke test; current COCO weights are not FOD acceptance data."
    )
    parser.add_argument(
        "--weights",
        default=str(DEFAULT_WEIGHTS),
        help="Trusted .pt checkpoint",
    )
    parser.add_argument("--source", required=True, help="Image, video, camera path, or index")
    parser.add_argument("--device", default="auto", help="auto, cpu, or CUDA index")
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--max-frames", type=int, default=300)
    parser.add_argument("--output", default="", help="Annotated image/video path")
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def summarize(frame_index, result):
    counts = Counter(item.class_name for item in result.detections)
    return {
        "frame": frame_index,
        "inference_ms": round(result.inference_ms, 2),
        "detections": len(result.detections),
        "classes": dict(sorted(counts.items())),
    }


def run_image(detector, source, args):
    image = cv2.imread(source, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot read image: {}".format(source))
    result = detector.predict(image)
    annotated = annotate_image(
        image, result.detections, result.inference_ms, "SMOKE ONLY"
    )
    output_path = args.output or "/tmp/fod_yolo_smoke.jpg"
    if not cv2.imwrite(output_path, annotated):
        raise RuntimeError("cannot write annotated image: {}".format(output_path))
    print(json.dumps(summarize(0, result), ensure_ascii=False))
    print("annotated_output={}".format(output_path))
    if args.show:
        cv2.imshow("FOD YOLO smoke test", annotated)
        cv2.waitKey(0)


def _capture_source(source):
    return int(source) if source.isdigit() else source


def run_stream(detector, source, args):
    capture_source = _capture_source(source)
    backend = cv2.CAP_V4L2 if source.isdigit() or source.startswith("/dev/") else cv2.CAP_ANY
    capture = cv2.VideoCapture(capture_source, backend)
    if not capture.isOpened() and backend != cv2.CAP_ANY:
        capture.release()
        capture = cv2.VideoCapture(capture_source)
    if not capture.isOpened():
        raise RuntimeError("cannot open stream: {}".format(source))

    writer = None
    frame_index = 0
    aggregate = Counter()
    inference_total = 0.0
    try:
        while args.max_frames <= 0 or frame_index < args.max_frames:
            ok, image = capture.read()
            if not ok or image is None:
                break
            result = detector.predict(image)
            annotated = annotate_image(
                image, result.detections, result.inference_ms, "SMOKE ONLY"
            )
            inference_total += result.inference_ms
            aggregate.update(item.class_name for item in result.detections)
            if args.output:
                if writer is None:
                    fps = capture.get(cv2.CAP_PROP_FPS)
                    if not fps or fps <= 0.0 or fps > 240.0:
                        fps = 20.0
                    writer = cv2.VideoWriter(
                        args.output,
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        fps,
                        (annotated.shape[1], annotated.shape[0]),
                    )
                    if not writer.isOpened():
                        raise RuntimeError(
                            "cannot open video writer: {}".format(args.output)
                        )
                writer.write(annotated)
            if args.show:
                cv2.imshow("FOD YOLO smoke test", annotated)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    frame_index += 1
                    break
            print(json.dumps(summarize(frame_index, result), ensure_ascii=False))
            frame_index += 1
    finally:
        capture.release()
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()
    if frame_index == 0:
        raise RuntimeError("stream produced no frames")
    print(
        json.dumps(
            {
                "summary": {
                    "frames": frame_index,
                    "average_inference_ms": round(inference_total / frame_index, 2),
                    "classes": dict(sorted(aggregate.items())),
                    "output": args.output,
                }
            },
            ensure_ascii=False,
        )
    )


def main():
    args = parse_args()
    detector = UltralyticsDetector(
        weights=args.weights,
        device=args.device,
        image_size=args.image_size,
        confidence=args.confidence,
        iou=args.iou,
        warmup=True,
    )
    print(
        "model={} task={} device={} sha256={}".format(
            detector.model_name,
            detector.task,
            detector.device,
            detector.model_sha256,
        )
    )
    source_path = Path(args.source).expanduser()
    if source_path.suffix.lower() in IMAGE_SUFFIXES and source_path.is_file():
        run_image(detector, str(source_path), args)
    else:
        run_stream(detector, args.source, args)


if __name__ == "__main__":
    main()
