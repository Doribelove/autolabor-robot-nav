"""Single-process custom-Ultralytics runtime for trash detect + material classify."""

from dataclasses import dataclass
from hashlib import sha256
import gc
import importlib
from pathlib import Path
import sys
import threading
from time import perf_counter
from typing import List, Sequence, Tuple

import numpy as np

from .confidence_control import validate_detection_confidence
from .two_stage import BBox, ImageTrackFallback, MATERIAL_CLASSES


def file_sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(4 * 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _validated_file(path: str, expected_sha256: str, label: str) -> Path:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError("{} does not exist: {}".format(label, resolved))
    actual = file_sha256(resolved)
    expected = str(expected_sha256).strip().lower()
    if len(expected) != 64 or any(character not in "0123456789abcdef" for character in expected):
        raise ValueError("{} expected SHA256 is invalid".format(label))
    if actual != expected:
        raise RuntimeError(
            "{} SHA256 mismatch: expected {}, got {}".format(label, expected, actual)
        )
    return resolved


def _import_custom_ultralytics(root: str):
    configured = Path(root).expanduser().resolve()
    if configured.name == "ultralytics" and (configured / "__init__.py").is_file():
        package_root = configured
        import_root = configured.parent
    else:
        package_root = configured / "ultralytics"
        import_root = configured
    if not (package_root / "__init__.py").is_file():
        raise FileNotFoundError(
            "custom Ultralytics package is missing: {}".format(package_root)
        )
    existing = sys.modules.get("ultralytics")
    if existing is not None:
        actual = Path(existing.__file__).resolve()
        try:
            actual.relative_to(package_root)
        except ValueError as error:
            raise RuntimeError(
                "Ultralytics was already imported from {}, outside {}".format(
                    actual, package_root
                )
            ) from error
    import_root_text = str(import_root)
    if import_root_text not in sys.path:
        sys.path.insert(0, import_root_text)
    ultralytics = importlib.import_module("ultralytics")
    actual = Path(ultralytics.__file__).resolve()
    try:
        actual.relative_to(package_root)
    except ValueError as error:
        raise RuntimeError(
            "Ultralytics import conflict: {} is outside {}".format(
                actual, package_root
            )
        ) from error
    return ultralytics, import_root, actual


@dataclass(frozen=True)
class TrackedTrash:
    track_id: int
    bbox: BBox
    confidence: float
    class_id: int = 0


class TwoStageUltralyticsRuntime:
    """Loads both checkpoints once, warms them, and keeps both GPU models alive."""

    backend = "detect_and_classify"
    motion_eligible = False
    confidence_calibrated = False

    def __init__(
        self,
        ultralytics_root: str,
        detector_weights: str,
        detector_sha256: str,
        classifier_weights: str,
        classifier_sha256: str,
        tracker_config: str,
        device: str = "cuda:0",
        detector_imgsz: int = 768,
        detector_confidence: float = 0.25,
        detector_iou: float = 0.60,
        detector_max_detections: int = 50,
        classifier_imgsz: int = 224,
        classifier_max_batch: int = 8,
        half: bool = True,
        warmup_frames: int = 3,
        track_buffer_frames: int = 45,
    ):
        self.detector_path = _validated_file(
            detector_weights, detector_sha256, "trash detector"
        )
        self.classifier_path = _validated_file(
            classifier_weights, classifier_sha256, "material classifier"
        )
        self.detector_model_sha256 = file_sha256(self.detector_path)
        self.classifier_model_sha256 = file_sha256(self.classifier_path)
        self.ultralytics, import_root, import_path = _import_custom_ultralytics(
            ultralytics_root
        )
        from ultralytics import YOLO
        import torch

        if not torch.cuda.is_available():
            raise RuntimeError("detect_and_classify requires CUDA")
        self.torch = torch
        self.device = str(device)
        self.half = bool(half)
        self.detector_imgsz = int(detector_imgsz)
        self.detector_confidence = validate_detection_confidence(
            detector_confidence
        )
        self._confidence_lock = threading.Lock()
        self.detector_iou = float(detector_iou)
        self.detector_max_detections = int(detector_max_detections)
        self.classifier_imgsz = int(classifier_imgsz)
        self.classifier_max_batch = max(1, int(classifier_max_batch))
        self.tracker_config = str(Path(tracker_config).expanduser().resolve())
        if not Path(self.tracker_config).is_file():
            raise FileNotFoundError(
                "BoT-SORT config does not exist: {}".format(self.tracker_config)
            )
        self.runtime_path = str(import_root)
        self.ultralytics_path = str(import_path)
        self.runtime_version = str(self.ultralytics.__version__)
        self.ultralytics_version = self.runtime_version
        self.model_name = "{} + {}".format(
            self.detector_path.name, self.classifier_path.name
        )
        self.task = "detect_and_classify"
        self.names = {index: name for index, name in enumerate(MATERIAL_CLASSES)}
        self.load_counts = {"detector": 0, "classifier": 0}
        self._fallback_tracker = ImageTrackFallback(
            buffer_frames=track_buffer_frames
        )
        self._closed = False

        # The classification smoke test intentionally runs first.  This proves
        # the unmodified classify task works in the same custom import tree.
        self.classifier = YOLO(str(self.classifier_path))
        self.load_counts["classifier"] += 1
        classifier_names = [
            str(self.classifier.names[index])
            for index in range(len(self.classifier.names))
        ]
        if str(self.classifier.task) != "classify":
            raise RuntimeError(
                "material checkpoint task is {}, expected classify".format(
                    self.classifier.task
                )
            )
        if classifier_names != list(MATERIAL_CLASSES):
            raise RuntimeError(
                "material class order is {}, expected {}".format(
                    classifier_names, list(MATERIAL_CLASSES)
                )
            )
        blank_classification = np.zeros(
            (self.classifier_imgsz, self.classifier_imgsz, 3), dtype=np.uint8
        )
        warmups = max(1, int(warmup_frames))
        for _ in range(warmups):
            smoke = self.classifier.predict(
                source=blank_classification,
                imgsz=self.classifier_imgsz,
                device=self.device,
                half=self.half,
                verbose=False,
            )[0]
            if smoke.probs is None or tuple(smoke.probs.data.shape) != (
                len(MATERIAL_CLASSES),
            ):
                shape = None if smoke.probs is None else tuple(smoke.probs.data.shape)
                raise RuntimeError(
                    "classification smoke test returned probability shape {}".format(
                        shape
                    )
                )

        self.detector = YOLO(str(self.detector_path))
        self.load_counts["detector"] += 1
        detector_names = {
            int(index): str(name)
            for index, name in (
                self.detector.names.items()
                if isinstance(self.detector.names, dict)
                else enumerate(self.detector.names)
            )
        }
        if str(self.detector.task) != "detect":
            raise RuntimeError(
                "trash checkpoint task is {}, expected detect".format(
                    self.detector.task
                )
            )
        if detector_names != {0: "trash"}:
            raise RuntimeError(
                "trash detector class map is {}, expected {{0: 'trash'}}".format(
                    detector_names
                )
            )
        self.gam_layer_count = sum(
            "GAM" in module.__class__.__name__.upper()
            for module in self.detector.model.modules()
        )
        if self.gam_layer_count < 1:
            raise RuntimeError("trash detector checkpoint contains no GAM layer")
        blank_detection = np.zeros(
            (self.detector_imgsz, self.detector_imgsz, 3), dtype=np.uint8
        )
        for _ in range(warmups):
            self.detector.predict(
                source=blank_detection,
                imgsz=self.detector_imgsz,
                conf=self.detector_confidence,
                iou=self.detector_iou,
                max_det=self.detector_max_detections,
                device=self.device,
                half=self.half,
                verbose=False,
            )

    def detect(self, image_bgr: np.ndarray, frame_index: int) -> Tuple[List[TrackedTrash], float]:
        if self._closed:
            raise RuntimeError("two-stage runtime is closed")
        with self._confidence_lock:
            detector_confidence = self.detector_confidence
        started = perf_counter()
        result = self.detector.track(
            source=image_bgr,
            persist=True,
            tracker=self.tracker_config,
            imgsz=self.detector_imgsz,
            conf=detector_confidence,
            iou=self.detector_iou,
            max_det=self.detector_max_detections,
            classes=[0],
            device=self.device,
            half=self.half,
            verbose=False,
        )[0]
        elapsed_ms = (perf_counter() - started) * 1000.0
        if result.boxes is None or len(result.boxes) == 0:
            self._fallback_tracker.update([], int(frame_index))
            return [], elapsed_ms
        xyxy = result.boxes.xyxy.detach().cpu().numpy()
        confidences = result.boxes.conf.detach().cpu().numpy()
        class_ids = result.boxes.cls.detach().cpu().numpy().astype(np.int32)
        boxes = [tuple(float(value) for value in bounds) for bounds in xyxy]
        if result.boxes.id is None:
            ids = [
                1000000 + value
                for value in self._fallback_tracker.update(boxes, int(frame_index))
            ]
        else:
            ids = result.boxes.id.detach().cpu().numpy().astype(np.int64).tolist()
        output: List[TrackedTrash] = []
        for track_id, box, confidence, class_id in zip(
            ids, boxes, confidences, class_ids
        ):
            if int(class_id) != 0:
                raise RuntimeError(
                    "single-class detector returned unexpected class {}".format(
                        class_id
                    )
                )
            output.append(
                TrackedTrash(
                    track_id=max(1, int(track_id)),
                    bbox=box,
                    confidence=float(confidence),
                )
            )
        return output, elapsed_ms

    def set_detector_confidence(self, confidence: float) -> None:
        value = validate_detection_confidence(confidence)
        with self._confidence_lock:
            self.detector_confidence = value

    def get_detector_confidence(self) -> float:
        with self._confidence_lock:
            return float(self.detector_confidence)

    def classify(self, crops_bgr: Sequence[np.ndarray]) -> Tuple[np.ndarray, float]:
        if self._closed:
            raise RuntimeError("two-stage runtime is closed")
        if not crops_bgr:
            return np.empty((0, len(MATERIAL_CLASSES)), dtype=np.float32), 0.0
        if len(crops_bgr) > self.classifier_max_batch:
            raise ValueError(
                "live classification batch {} exceeds configured limit {}".format(
                    len(crops_bgr), self.classifier_max_batch
                )
            )
        started = perf_counter()
        results = self.classifier.predict(
            source=list(crops_bgr),
            imgsz=self.classifier_imgsz,
            device=self.device,
            half=self.half,
            batch=len(crops_bgr),
            verbose=False,
        )
        elapsed_ms = (perf_counter() - started) * 1000.0
        probabilities: List[np.ndarray] = []
        for result in results:
            if result.probs is None:
                raise RuntimeError("classifier returned no probabilities")
            vector = result.probs.data.detach().float().cpu().numpy().reshape(-1)
            if vector.shape != (len(MATERIAL_CLASSES),):
                raise RuntimeError(
                    "classifier returned probability shape {}".format(vector.shape)
                )
            probabilities.append(vector.astype(np.float32))
        if len(probabilities) != len(crops_bgr):
            raise RuntimeError(
                "classifier returned {} results for {} crops".format(
                    len(probabilities), len(crops_bgr)
                )
            )
        return np.stack(probabilities, axis=0), elapsed_ms

    def cuda_memory_stats(self):
        """Return process-local PyTorch CUDA allocator counters in bytes."""
        if self._closed or not self.torch.cuda.is_available():
            return {
                "allocated": 0,
                "reserved": 0,
                "max_allocated": 0,
                "max_reserved": 0,
            }
        return {
            "allocated": int(self.torch.cuda.memory_allocated(self.device)),
            "reserved": int(self.torch.cuda.memory_reserved(self.device)),
            "max_allocated": int(
                self.torch.cuda.max_memory_allocated(self.device)
            ),
            "max_reserved": int(
                self.torch.cuda.max_memory_reserved(self.device)
            ),
        }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._fallback_tracker.reset()
        if getattr(self, "detector", None) is not None:
            self.detector.predictor = None
        if getattr(self, "classifier", None) is not None:
            self.classifier.predictor = None
        self.detector = None
        self.classifier = None
        gc.collect()
        if self.torch.cuda.is_available():
            self.torch.cuda.empty_cache()
