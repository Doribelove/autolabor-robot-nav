"""Crop inference to the ground band, retaining full-camera pixel coordinates."""

from dataclasses import replace
import math

import cv2
import numpy as np

from .detector import InferenceResult


def mark_recovery_region(image, state):
    """Draw the controller's actual pickup area without changing detections."""
    if not isinstance(state, dict):
        return image
    region = state.get("recovery_region_px")
    if (not isinstance(region, (list, tuple)) or len(region) != 4
            or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in region)):
        return image
    height, width = image.shape[:2]
    left, top, right, bottom = [int(round(v)) for v in region]
    left, right = max(0, left), min(width - 1, right)
    top, bottom = max(0, top), min(height - 1, bottom)
    if left >= right or top >= bottom:
        return image
    armed = bool(state.get("recovery_region_entered", False))
    color = (60, 230, 80) if armed else (0, 200, 255)
    distance = float(state.get("blind_target_m", .80))
    if not math.isfinite(distance):
        distance = .80
    label = "RECOVERY {} {:.2f}m".format("READY" if armed else "AREA", distance)
    if state.get("state") in ("BLIND_ADVANCE", "FINAL_STOP", "COMPLETE"):
        label = "RECOVERING {:.2f}/{:.2f}m".format(float(state.get("blind_forward_m", 0)), distance)
    cv2.rectangle(image, (left, top), (right, bottom), color, 3)
    cv2.putText(image, label, (left + 8, top + 25),
                cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
    return image


class DetectionRoi:
    def __init__(self, top=0.0, bottom=1.0):
        self.top, self.bottom = float(top), float(bottom)
        if not (math.isfinite(self.top) and math.isfinite(self.bottom)
                and 0.0 <= self.top < self.bottom <= 1.0):
            raise ValueError("detection ROI must satisfy 0 <= top < bottom <= 1")

    def rows(self, height):
        first, end = int(round(height * self.top)), int(round(height * self.bottom))
        if end <= first:
            raise ValueError("detection ROI has no image rows")
        return first, end

    def predict(self, detector, image):
        first, end = self.rows(image.shape[0])
        result = detector.predict(np.ascontiguousarray(image[first:end]))
        if first == 0 and end == image.shape[0]:
            return result
        def full_v(v):
            return first + min(end - first - 1.0, max(0.0, v))
        return InferenceResult([
            replace(box, ymin=full_v(box.ymin), ymax=full_v(box.ymax),
                    anchor_v=full_v(box.anchor_v),
                    mask=[(u, full_v(v)) for u, v in box.mask])
            for box in result.detections
        ], result.inference_ms)

    def mark_preview(self, image):
        first, end = self.rows(image.shape[0])
        if first == 0 and end == image.shape[0]:
            return image
        image[:first] = (image[:first].astype(np.float32) * 0.35).astype(np.uint8)
        image[end:] = (image[end:].astype(np.float32) * 0.35).astype(np.uint8)
        cv2.rectangle(image, (0, first), (image.shape[1] - 1, end - 1),
                      (255, 220, 0), 2)
        cv2.putText(image, "YOLO ground ROI", (12, first + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 220, 0), 2)
        return image
