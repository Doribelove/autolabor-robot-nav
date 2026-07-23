"""Ultralytics adapter and OpenCV-only debug rendering."""

from dataclasses import dataclass, field
from hashlib import sha256
from pathlib import Path
from time import perf_counter
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class Detection:
    class_id: int
    class_name: str
    confidence: float
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    anchor_u: float
    anchor_v: float
    mask: List[Tuple[float, float]] = field(default_factory=list)


@dataclass
class InferenceResult:
    detections: List[Detection]
    inference_ms: float


def file_sha256(path: str) -> str:
    digest = sha256()
    with Path(path).open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _name_for_class(names, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return str(names[class_id])
    return str(class_id)


class UltralyticsDetector:
    def __init__(
        self,
        weights: str,
        device: str = "auto",
        image_size: int = 640,
        confidence: float = 0.25,
        iou: float = 0.45,
        max_detections: int = 100,
        classes: Optional[Sequence[int]] = None,
        half: bool = True,
        warmup: bool = True,
        expected_sha256: str = "",
    ):
        weights_path = Path(weights).expanduser().resolve()
        if not weights_path.is_file():
            raise FileNotFoundError("YOLO weights do not exist: {}".format(weights_path))

        self.model_sha256 = file_sha256(str(weights_path))
        expected_sha256 = str(expected_sha256).strip().lower()
        if expected_sha256:
            if len(expected_sha256) != 64 or any(
                character not in "0123456789abcdef"
                for character in expected_sha256
            ):
                raise ValueError("expected model SHA256 is not a valid digest")
            if self.model_sha256.lower() != expected_sha256:
                raise RuntimeError(
                    "YOLO weights SHA256 mismatch: expected {}, got {}".format(
                        expected_sha256, self.model_sha256
                    )
                )

        import torch
        from ultralytics import YOLO

        self.weights = str(weights_path)
        self.model_name = weights_path.name
        self.device = (
            "0" if device == "auto" and torch.cuda.is_available() else
            "cpu" if device == "auto" else str(device)
        )
        self.half = bool(half and self.device != "cpu" and torch.cuda.is_available())
        self.image_size = int(image_size)
        self.confidence = float(confidence)
        self.iou = float(iou)
        self.max_detections = int(max_detections)
        self.classes = list(classes) if classes else None
        self.model = YOLO(self.weights)
        self.task = str(getattr(self.model, "task", "unknown"))
        self.names: Dict[int, str] = {
            int(key): str(value)
            for key, value in (
                self.model.names.items()
                if isinstance(self.model.names, dict)
                else enumerate(self.model.names)
            )
        }
        if self.task not in ("detect", "segment"):
            raise ValueError(
                "first implementation supports detect/segment models, got {}".format(
                    self.task
                )
            )
        if warmup:
            blank = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
            self.predict(blank)

    def predict(self, image_bgr: np.ndarray) -> InferenceResult:
        start = perf_counter()
        result = self.model.predict(
            source=image_bgr,
            imgsz=self.image_size,
            conf=self.confidence,
            iou=self.iou,
            max_det=self.max_detections,
            classes=self.classes,
            device=self.device,
            half=self.half,
            verbose=False,
        )[0]
        elapsed_ms = (perf_counter() - start) * 1000.0
        detections: List[Detection] = []
        boxes = result.boxes
        masks_xy = result.masks.xy if result.masks is not None else []
        if boxes is None:
            return InferenceResult(detections=detections, inference_ms=elapsed_ms)

        xyxy = boxes.xyxy.detach().cpu().numpy()
        confidences = boxes.conf.detach().cpu().numpy()
        class_ids = boxes.cls.detach().cpu().numpy().astype(np.int32)
        for index, (bounds, score, class_id) in enumerate(
            zip(xyxy, confidences, class_ids)
        ):
            xmin, ymin, xmax, ymax = [float(value) for value in bounds]
            mask: List[Tuple[float, float]] = []
            if index < len(masks_xy):
                mask = [
                    (float(point[0]), float(point[1]))
                    for point in np.asarray(masks_xy[index])
                ]
            if mask:
                max_y = max(point[1] for point in mask)
                bottom = [point for point in mask if point[1] >= max_y - 2.0]
                anchor_u = sum(point[0] for point in bottom) / len(bottom)
                anchor_v = max_y
            else:
                anchor_u = 0.5 * (xmin + xmax)
                anchor_v = ymax
            detections.append(
                Detection(
                    class_id=int(class_id),
                    class_name=_name_for_class(self.names, int(class_id)),
                    confidence=float(score),
                    xmin=xmin,
                    ymin=ymin,
                    xmax=xmax,
                    ymax=ymax,
                    anchor_u=anchor_u,
                    anchor_v=anchor_v,
                    mask=mask,
                )
            )
        return InferenceResult(detections=detections, inference_ms=elapsed_ms)


def annotate_image(
    image_bgr: np.ndarray,
    detections: Sequence[Detection],
    inference_ms: float,
    banner: str,
) -> np.ndarray:
    output = image_bgr.copy()
    for detection in detections:
        x1 = max(0, int(round(detection.xmin)))
        y1 = max(0, int(round(detection.ymin)))
        x2 = min(output.shape[1] - 1, int(round(detection.xmax)))
        y2 = min(output.shape[0] - 1, int(round(detection.ymax)))
        color = (50, 220, 50)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        cv2.circle(
            output,
            (int(round(detection.anchor_u)), int(round(detection.anchor_v))),
            4,
            (0, 0, 255),
            -1,
        )
        if detection.mask:
            polygon = np.asarray(detection.mask, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(output, [polygon], True, (255, 180, 0), 2)
        label = "{} {:.2f}".format(detection.class_name, detection.confidence)
        cv2.putText(
            output,
            label,
            (x1, max(16, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )
    text = "{} | {:.1f} ms | {} objects".format(
        banner, inference_ms, len(detections)
    )
    cv2.rectangle(output, (0, 0), (min(output.shape[1], 700), 28), (0, 0, 0), -1)
    cv2.putText(
        output,
        text,
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return output
