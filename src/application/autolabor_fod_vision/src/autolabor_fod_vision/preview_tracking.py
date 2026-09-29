"""Short image-space tracks for preview only; never manufacture control hits."""

from dataclasses import dataclass, field, replace
from math import hypot, isfinite
from typing import Dict, List, Optional, Sequence, Tuple

from .detector import Detection


@dataclass
class PreviewBox:
    detection: Detection
    track_id: int
    held_for_sec: float = 0.0


@dataclass
class _Track:
    track_id: int
    observed: Detection
    display: Detection
    last_seen: float
    class_scores: Dict[int, float] = field(default_factory=dict)
    class_names: Dict[int, str] = field(default_factory=dict)


def _geometry(a: Detection, b: Detection) -> Tuple[float, float, float]:
    aw, ah = a.xmax - a.xmin, a.ymax - a.ymin
    bw, bh = b.xmax - b.xmin, b.ymax - b.ymin
    intersection = max(0.0, min(a.xmax, b.xmax) - max(a.xmin, b.xmin)) * max(
        0.0, min(a.ymax, b.ymax) - max(a.ymin, b.ymin)
    )
    iou = intersection / max(1e-6, aw * ah + bw * bh - intersection)
    distance = hypot(
        (a.xmin + a.xmax - b.xmin - b.xmax) * 0.5,
        (a.ymin + a.ymax - b.ymin - b.ymax) * 0.5,
    ) / max(1.0, hypot(aw, ah), hypot(bw, bh))
    area_ratio = bw * bh / max(1e-6, aw * ah)
    return iou, distance, area_ratio


class PreviewStabilizer:
    """Smooth current boxes, vote on labels, and briefly show missed boxes.

    The caller must continue publishing the detector's original observations
    to /fod/detections. Returned boxes are presentation data, including holds.
    """

    def __init__(self, alpha: float = 0.65, hold_sec: float = 0.30):
        if not 0.0 < alpha <= 1.0 or not isfinite(hold_sec) or hold_sec < 0.0:
            raise ValueError("invalid preview smoothing or hold time")
        self.alpha = float(alpha)
        self.hold_sec = float(hold_sec)
        self._tracks: List[_Track] = []
        self._last_stamp: Optional[float] = None
        self._image_size: Optional[Tuple[int, int]] = None
        self._next_id = 1

    def _observe(self, track: _Track, detection: Detection, stamp: float) -> None:
        for class_id in track.class_scores:
            track.class_scores[class_id] *= 0.85
        track.class_scores[detection.class_id] = (
            track.class_scores.get(detection.class_id, 0.0) + detection.confidence
        )
        track.class_names[detection.class_id] = detection.class_name
        class_id = max(track.class_scores, key=track.class_scores.get)
        # Follow large movements immediately; smooth small frame-to-frame jitter.
        _, distance, _ = _geometry(track.observed, detection)
        alpha = 1.0 if distance > 0.35 else self.alpha
        bounds = {
            key: alpha * getattr(detection, key) + (1.0 - alpha) * getattr(track.display, key)
            for key in ("xmin", "ymin", "xmax", "ymax")
        }
        scale_x = (bounds["xmax"] - bounds["xmin"]) / max(1e-6, detection.xmax - detection.xmin)
        scale_y = (bounds["ymax"] - bounds["ymin"]) / max(1e-6, detection.ymax - detection.ymin)

        def transform(x, y):
            return (
                bounds["xmin"] + (x - detection.xmin) * scale_x,
                bounds["ymin"] + (y - detection.ymin) * scale_y,
            )

        anchor_u, anchor_v = transform(detection.anchor_u, detection.anchor_v)
        track.display = replace(
            detection, **bounds, class_id=class_id, class_name=track.class_names[class_id],
            anchor_u=anchor_u, anchor_v=anchor_v,
            mask=[transform(x, y) for x, y in detection.mask],
        )
        track.observed = detection
        track.last_seen = stamp

    def update(
        self, detections: Sequence[Detection], stamp: float, image_size: Tuple[int, int]
    ) -> List[PreviewBox]:
        stamp = float(stamp)
        if not isfinite(stamp):
            self._tracks.clear()
            self._last_stamp = None
            return []
        if image_size != self._image_size or (
            self._last_stamp is not None and stamp <= self._last_stamp
        ):
            self._tracks.clear()
        self._image_size = image_size
        self._last_stamp = stamp
        # Expire BEFORE association: a stale track must not relabel a new object.
        self._tracks = [t for t in self._tracks
                        if stamp - t.last_seen <= max(self.hold_sec, 0.30)]
        pairs = []
        for track_index, track in enumerate(self._tracks):
            for detection_index, detection in enumerate(detections):
                iou, distance, area_ratio = _geometry(track.observed, detection)
                if 0.35 <= area_ratio <= 2.85 and (iou >= 0.15 or distance <= 0.25):
                    pairs.append((iou - 0.2 * distance, track_index, detection_index))
        used_tracks, used_detections = set(), set()
        for _, track_index, detection_index in sorted(pairs, reverse=True):
            if track_index in used_tracks or detection_index in used_detections:
                continue
            self._observe(self._tracks[track_index], detections[detection_index], stamp)
            used_tracks.add(track_index)
            used_detections.add(detection_index)
        for index, detection in enumerate(detections):
            if index in used_detections:
                continue
            self._tracks.append(_Track(
                track_id=self._next_id, observed=detection, display=replace(detection),
                last_seen=stamp, class_scores={detection.class_id: detection.confidence},
                class_names={detection.class_id: detection.class_name},
            ))
            self._next_id += 1
        # Suppress stale preview duplicates when a new, differently sized box
        # already covers that location. Keep every genuine current detection.
        self._tracks = [t for t in self._tracks if t.last_seen == stamp or not any(
            _geometry(t.observed, detection)[0] > 0.5 for detection in detections
        )]
        return [PreviewBox(t.display, t.track_id, max(0.0, stamp - t.last_seen))
                for t in self._tracks if stamp - t.last_seen <= self.hold_sec]
