"""One locked, resident pair of the existing YOLO11-GAM / cls5 HBM models."""
from contextlib import ExitStack
import fcntl
from pathlib import Path
import sys
import time

import numpy as np

from . import MATERIALS
from .contracts import sha256

TOOLS = Path(__file__).resolve().parents[1] / "tools"
ROOT = Path(__file__).resolve().parents[3]


class BpuEngine:
    def __init__(self, config, root=ROOT):
        # These imports load numerical helpers only, never a ZED service.
        sys.path.insert(0, str(TOOLS))
        sys.path.insert(0, str(root / "src/application/autolabor_fod_vision/src"))
        from native_tensor import TensorEngine
        from production_io import validate_production_interface
        self.config = config
        self.stack = ExitStack()
        self.engines = {}
        self.identity = {}
        try:
            library = Path(config["library"])
            if sha256(library) != config["library_sha256"]:
                raise ValueError("Native executor hash differs from verified runtime")
            # The same lock as the existing BPU service: no parallel owner.
            lock = Path(config["exclusive_lock"]).open("a")
            self.stack.enter_context(lock)
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            for kind in ("detector_zed", "classifier"):
                spec = config[kind]
                path = Path(spec["path"])
                if not path.is_absolute():
                    path = root / path
                engine = self.stack.enter_context(TensorEngine(
                    library, path, spec["model_name"], spec["sha256"]))
                validate_production_interface(engine.metadata, kind)
                self.engines[kind] = engine
                self.identity[kind] = dict(sha256=spec["sha256"], model_name=spec["model_name"])
        except Exception:
            self.stack.close()
            raise

    def infer(self, image, source_deadline_ns):
        from board_preprocess import detector_input, classifier_input
        from production_io import detector_output, classifier_output
        from autolabor_fod_vision.two_stage import context_crop
        begin = time.monotonic_ns()
        if time.time_ns() >= source_deadline_ns:
            raise ValueError("Source expired before detector")
        tensor, geometry = detector_input(image)
        outputs, detector_timing = self.engines["detector_zed"].infer([tensor], timeout_ms=200)
        boxes = detector_output(outputs[0], geometry, confidence=self.config["confidence"])
        objects = []
        classify_begin = time.monotonic_ns()
        classified = 0
        for box in boxes:
            obj = dict(bbox=box[:4].tolist(), class_id=0, detect_confidence=float(box[4]),
                       material="unknown", classify_confidence=0., probabilities=None)
            if (classified < self.config["max_crops"]
                    and time.monotonic_ns() - classify_begin < self.config["classify_budget_ms"] * 1e6
                    and time.time_ns() < source_deadline_ns):
                crop, _ = context_crop(image, box[:4], context_fraction=.20)
                raw, _ = self.engines["classifier"].infer([classifier_input(crop)], timeout_ms=200)
                p = classifier_output(raw[0])
                obj.update(material=MATERIALS[int(p.argmax())], classify_confidence=float(p.max()),
                           probabilities=p.tolist())
                classified += 1
            objects.append(obj)
        if time.time_ns() >= source_deadline_ns:
            raise ValueError("Source expired during BPU processing")
        return objects, dict(processing_ms=(time.monotonic_ns() - begin) / 1e6,
                             classified=classified, unclassified=len(objects) - classified,
                             detector_timing=detector_timing, models=self.identity,
                             inference_backend="j6m_bpu")

    def close(self):
        self.stack.close()


class FixtureEngine:
    """Only used by the explicit offline replay command; never a live fallback."""
    def __init__(self):
        self.objects = []

    def infer(self, image, source_deadline_ns):
        import copy
        objects = copy.deepcopy(self.objects)
        for obj in objects:
            p = obj.get("probabilities")
            if p is not None:
                p = np.asarray(p, dtype=float)
                if p.shape != (5,) or not np.isfinite(p).all() or (p < 0).any() or not np.isclose(p.sum(), 1):
                    raise ValueError("Bad fixture probabilities")
                obj["material"] = MATERIALS[int(p.argmax())]
                obj["classify_confidence"] = float(p.max())
            else:
                obj.update(material="unknown", classify_confidence=0., probabilities=None)
            if obj.get("class_id", 0) != 0 or not 0 <= obj["detect_confidence"] <= 1:
                raise ValueError("Bad fixture detector output")
            obj["class_id"] = 0
        return objects, dict(inference_backend="fixture_not_inference", processing_ms=0.)

    def close(self):
        pass
