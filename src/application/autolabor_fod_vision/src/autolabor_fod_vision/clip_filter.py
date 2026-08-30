"""Official OpenAI CLIP post-filter for YOLO detections.

The YOLO adapter remains responsible only for object detection.  This module
receives the completed YOLO detections, applies the requested confidence gate,
and batches only the ambiguous crops through a frozen CLIP model.
"""

from dataclasses import dataclass
from hashlib import sha256
import importlib
import json
import math
import os
from pathlib import Path
import sys
from time import perf_counter
from typing import List, Sequence, Tuple

import numpy as np

from autolabor_fod_vision.detector import Detection


@dataclass(frozen=True)
class ClipFilterStats:
    input_count: int = 0
    high_confidence_kept: int = 0
    low_confidence_dropped: int = 0
    clip_candidates: int = 0
    clip_kept: int = 0
    clip_dropped: int = 0
    invalid_crop_dropped: int = 0
    output_count: int = 0
    clip_inference_ms: float = 0.0


@dataclass(frozen=True)
class ClipFilterResult:
    detections: List[Detection]
    stats: ClipFilterStats


def file_sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def resolve_clip_layout(root: str) -> Tuple[Path, Path]:
    """Return ``(import_root, package_root)`` for an isolated CLIP install."""
    value = str(root).strip()
    if not value:
        raise ValueError("the official OpenAI CLIP Python root is empty")
    requested = Path(value).expanduser()
    if not requested.is_absolute():
        workspace = str(os.environ.get("DUAL_HOST_WS", "")).strip()
        requested = (Path(workspace) / requested) if workspace else requested
    requested = requested.resolve()
    package_root = requested / "clip"
    if not (package_root / "__init__.py").is_file():
        raise FileNotFoundError(
            "CLIP Python root must contain clip/__init__.py: {}".format(
                requested
            )
        )
    return requested, package_root


def import_official_clip(clip_python_root: str):
    """Import CLIP only from the configured project-local official install."""
    import_root, package_root = resolve_clip_layout(clip_python_root)
    import_root_text = str(import_root)
    if not sys.path or sys.path[0] != import_root_text:
        try:
            sys.path.remove(import_root_text)
        except ValueError:
            pass
        sys.path.insert(0, import_root_text)

    module = importlib.import_module("clip")
    module_file = getattr(module, "__file__", "")
    if not module_file:
        raise ImportError("the imported clip module has no filesystem path")
    module_path = Path(module_file).resolve()
    try:
        module_path.relative_to(package_root)
    except ValueError as error:
        raise ImportError(
            "CLIP import conflict: {} is outside the configured official "
            "package {}".format(module_path, package_root)
        ) from error
    if not callable(getattr(module, "load", None)) or not callable(
        getattr(module, "tokenize", None)
    ):
        raise ImportError("the configured clip package has no load/tokenize API")
    return module, module_path


def validate_clip_provenance(clip_python_root: str, expected_commit: str) -> str:
    """Require pip VCS metadata proving the configured source is OpenAI CLIP."""
    import_root, _ = resolve_clip_layout(clip_python_root)
    commit = str(expected_commit).strip().lower()
    if len(commit) != 40 or any(
        character not in "0123456789abcdef" for character in commit
    ):
        raise ValueError("expected OpenAI CLIP source commit must be 40 hex digits")
    metadata_path = import_root / "clip-1.0.dist-info" / "direct_url.json"
    if not metadata_path.is_file():
        raise FileNotFoundError(
            "OpenAI CLIP VCS provenance metadata is missing: {}".format(
                metadata_path
            )
        )
    document = json.loads(metadata_path.read_text(encoding="utf-8"))
    source_url = str(document.get("url", "")).rstrip("/")
    actual_commit = str(
        document.get("vcs_info", {}).get("commit_id", "")
    ).lower()
    if source_url != "https://github.com/openai/CLIP.git":
        raise RuntimeError(
            "CLIP source is not the official OpenAI repository: {}".format(
                source_url or "<empty>"
            )
        )
    if actual_commit != commit:
        raise RuntimeError(
            "OpenAI CLIP source commit mismatch: expected {}, got {}".format(
                commit, actual_commit or "<empty>"
            )
        )
    return source_url


class OfficialClipRuntime:
    """Frozen official OpenAI CLIP runtime with cached text prototypes."""

    def __init__(
        self,
        weights: str,
        expected_sha256: str,
        clip_python_root: str,
        positive_prompts: Sequence[str],
        negative_prompts: Sequence[str],
        model_name: str = "ViT-B/32",
        source_commit: str = "",
        device: str = "auto",
        warmup: bool = True,
    ):
        import torch

        weights_path = Path(weights).expanduser().resolve()
        if not weights_path.is_file():
            raise FileNotFoundError(
                "official CLIP weights do not exist: {}".format(weights_path)
            )
        expected = str(expected_sha256).strip().lower()
        if len(expected) != 64 or any(
            character not in "0123456789abcdef" for character in expected
        ):
            raise ValueError("expected CLIP SHA256 is not a valid digest")
        actual = file_sha256(weights_path)
        if actual != expected:
            raise RuntimeError(
                "CLIP weights SHA256 mismatch: expected {}, got {}".format(
                    expected, actual
                )
            )

        self.positive_prompts = self._clean_prompts(
            positive_prompts, "positive"
        )
        self.negative_prompts = self._clean_prompts(
            negative_prompts, "negative"
        )
        self.model_name = str(model_name).strip() or "ViT-B/32"
        self.source_commit = str(source_commit).strip().lower()
        self.source_url = validate_clip_provenance(
            clip_python_root, self.source_commit
        )
        self.weights = str(weights_path)
        self.weights_sha256 = actual
        self.device = (
            "cuda:0"
            if device == "auto" and torch.cuda.is_available()
            else "cpu" if device == "auto" else str(device)
        )
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CLIP CUDA device requested but CUDA is unavailable")

        clip_module, module_path = import_official_clip(clip_python_root)
        self.clip_import_path = str(module_path)
        self.model, self.preprocess = clip_module.load(
            self.weights,
            device=self.device,
            jit=False,
        )
        self.model.eval()
        self.model.requires_grad_(False)
        self._torch = torch

        prompt_texts = self.positive_prompts + self.negative_prompts
        try:
            tokens = clip_module.tokenize(prompt_texts, truncate=False)
        except RuntimeError as error:
            raise ValueError(
                "a configured CLIP prompt exceeds the official 77-token limit"
            ) from error
        tokens = tokens.to(self.device)
        with torch.inference_mode():
            prompt_features = self.model.encode_text(tokens).float()
            prompt_features = self._normalize(prompt_features)
            positive_count = len(self.positive_prompts)
            positive = self._normalize(
                prompt_features[:positive_count].mean(dim=0, keepdim=True)
            )[0]
            negative = self._normalize(
                prompt_features[positive_count:].mean(dim=0, keepdim=True)
            )[0]
            self.class_features = torch.stack((negative, positive), dim=0)
            self.logit_scale = float(
                self.model.logit_scale.exp().detach().float().clamp(max=100.0)
            )

        if warmup:
            blank = np.zeros((224, 224, 3), dtype=np.uint8)
            self.score_positive([blank])

    @staticmethod
    def _clean_prompts(prompts: Sequence[str], label: str) -> List[str]:
        cleaned = [str(value).strip() for value in prompts if str(value).strip()]
        if not cleaned:
            raise ValueError("CLIP {} prompts must not be empty".format(label))
        return cleaned

    def _normalize(self, features):
        return features / features.norm(dim=-1, keepdim=True).clamp_min(1e-12)

    def _synchronize(self) -> None:
        if self.device.startswith("cuda"):
            self._torch.cuda.synchronize(self.device)

    def score_positive(
        self, crops_bgr: Sequence[np.ndarray]
    ) -> Tuple[List[float], float]:
        """Return positive-class probabilities for one batched forward pass."""
        # On aarch64, Pillow may consume static TLS before NVIDIA PyTorch can
        # load libgomp. OfficialClipRuntime imports torch first, so keep this
        # image-only dependency lazy and preserve that required load order.
        from PIL import Image

        if not crops_bgr:
            return [], 0.0
        tensors = []
        for crop in crops_bgr:
            if crop.ndim != 3 or crop.shape[2] != 3 or crop.size == 0:
                raise ValueError("CLIP crop must be a non-empty BGR image")
            image_rgb = np.ascontiguousarray(crop[:, :, ::-1])
            tensors.append(self.preprocess(Image.fromarray(image_rgb)))
        batch = self._torch.stack(tensors, dim=0).to(self.device)
        self._synchronize()
        start = perf_counter()
        with self._torch.inference_mode():
            image_features = self.model.encode_image(batch).float()
            image_features = self._normalize(image_features)
            logits = self.logit_scale * image_features @ self.class_features.T
            probabilities = logits.softmax(dim=-1)[:, 1]
        self._synchronize()
        elapsed_ms = (perf_counter() - start) * 1000.0
        return [float(value) for value in probabilities.cpu()], elapsed_ms


class ClipDetectionFilter:
    """Confidence gate plus batched CLIP validation for ambiguous boxes."""

    def __init__(
        self,
        runtime,
        low_confidence: float = 0.20,
        high_confidence: float = 0.60,
        positive_probability: float = 0.50,
        crop_padding_fraction: float = 0.10,
    ):
        self.runtime = runtime
        self.low_confidence = float(low_confidence)
        self.high_confidence = float(high_confidence)
        self.positive_probability = float(positive_probability)
        self.crop_padding_fraction = float(crop_padding_fraction)
        if not 0.0 <= self.low_confidence <= self.high_confidence <= 1.0:
            raise ValueError("CLIP confidence gates must satisfy 0 <= low <= high <= 1")
        if not 0.0 <= self.positive_probability <= 1.0:
            raise ValueError("CLIP positive probability must be in [0, 1]")
        if not 0.0 <= self.crop_padding_fraction <= 1.0:
            raise ValueError("CLIP crop padding fraction must be in [0, 1]")

    def _crop(self, image_bgr: np.ndarray, detection: Detection):
        height, width = image_bgr.shape[:2]
        bounds = (
            detection.xmin,
            detection.ymin,
            detection.xmax,
            detection.ymax,
        )
        if not all(math.isfinite(float(value)) for value in bounds):
            return None
        box_width = max(0.0, float(detection.xmax) - float(detection.xmin))
        box_height = max(0.0, float(detection.ymax) - float(detection.ymin))
        if box_width <= 0.0 or box_height <= 0.0:
            return None
        pad_x = box_width * self.crop_padding_fraction
        pad_y = box_height * self.crop_padding_fraction
        x1 = max(0, int(math.floor(float(detection.xmin) - pad_x)))
        y1 = max(0, int(math.floor(float(detection.ymin) - pad_y)))
        x2 = min(width, int(math.ceil(float(detection.xmax) + pad_x)))
        y2 = min(height, int(math.ceil(float(detection.ymax) + pad_y)))
        if x2 <= x1 or y2 <= y1:
            return None
        crop = image_bgr[y1:y2, x1:x2]
        return crop if crop.size else None

    def filter(
        self,
        image_bgr: np.ndarray,
        detections: Sequence[Detection],
    ) -> ClipFilterResult:
        keep = [False] * len(detections)
        candidate_indices = []
        candidate_crops = []
        high_kept = 0
        low_dropped = 0
        invalid_dropped = 0

        for index, detection in enumerate(detections):
            confidence = float(detection.confidence)
            if not math.isfinite(confidence):
                invalid_dropped += 1
            elif confidence > self.high_confidence:
                keep[index] = True
                high_kept += 1
            elif confidence < self.low_confidence:
                low_dropped += 1
            else:
                crop = self._crop(image_bgr, detection)
                if crop is None:
                    invalid_dropped += 1
                else:
                    candidate_indices.append(index)
                    candidate_crops.append(crop)

        if candidate_crops:
            probabilities, elapsed_ms = self.runtime.score_positive(
                candidate_crops
            )
        else:
            probabilities, elapsed_ms = [], 0.0
        if len(probabilities) != len(candidate_indices):
            raise RuntimeError(
                "CLIP returned {} scores for {} crops".format(
                    len(probabilities), len(candidate_indices)
                )
            )
        clip_kept = 0
        for index, probability in zip(candidate_indices, probabilities):
            if math.isfinite(float(probability)) and float(
                probability
            ) >= self.positive_probability:
                keep[index] = True
                clip_kept += 1

        filtered = [item for index, item in enumerate(detections) if keep[index]]
        clip_candidates = len(candidate_indices)
        return ClipFilterResult(
            detections=filtered,
            stats=ClipFilterStats(
                input_count=len(detections),
                high_confidence_kept=high_kept,
                low_confidence_dropped=low_dropped,
                clip_candidates=clip_candidates,
                clip_kept=clip_kept,
                clip_dropped=clip_candidates - clip_kept,
                invalid_crop_dropped=invalid_dropped,
                output_count=len(filtered),
                clip_inference_ms=float(elapsed_ms),
            ),
        )
