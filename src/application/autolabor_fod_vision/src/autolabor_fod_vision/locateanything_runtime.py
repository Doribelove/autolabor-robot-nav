"""LocateAnything worker protocol, model identity, and output parsing.

The ROS process deliberately does not import the large vision-language model.
It keeps ROS Noetic/cv_bridge on Python 3.8 and delegates model loading to an
isolated JSON-lines worker.  All worker caches and logs live below the external
model directory.
"""

from dataclasses import dataclass
import base64
from hashlib import sha256
import json
import os
from pathlib import Path
import selectors
import subprocess
import threading
import time
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np

from .detector import Detection, InferenceResult


_HEX_DIGITS = frozenset("0123456789abcdef")


@dataclass(frozen=True)
class LocateAnythingCategory:
    class_id: int
    class_name: str
    prompt: str
    aliases: Tuple[str, ...] = ()
    query_prompts: Tuple[str, ...] = ()

    @property
    def grounding_prompts(self) -> Tuple[str, ...]:
        """Return individual positive queries, preserving legacy configs."""
        return self.query_prompts or (self.prompt,)


@dataclass(frozen=True)
class VerifiedManifest:
    digest: str
    repo_id: str
    revision: str
    files: Tuple[Mapping[str, object], ...]


def _valid_sha256(value: str) -> bool:
    text = str(value).strip().lower()
    return len(text) == 64 and all(character in _HEX_DIGITS for character in text)


def _file_sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(4 * 1024 * 1024)
            if not chunk:
                return digest.hexdigest()
            digest.update(chunk)


def validate_max_image_side(value: object) -> int:
    """Validate the optional pre-inference downscale limit.

    Zero explicitly disables adapter-level resizing, so the worker hands the
    source frame at its native dimensions to the model image processor.
    """
    maximum = int(value)
    if maximum != 0 and not 224 <= maximum <= 1536:
        raise ValueError(
            "LocateAnything max_image_side must be 0 (native input) or in "
            "[224, 1536]"
        )
    return maximum


def verify_model_manifest(
    model_root: str,
    manifest_path: str,
    expected_digest: str = "",
    verify_files: bool = True,
) -> VerifiedManifest:
    """Validate a deployment manifest and, optionally, every declared file."""
    root = Path(model_root).expanduser().resolve()
    manifest = Path(manifest_path).expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError("LocateAnything model root is missing: {}".format(root))
    if not manifest.is_file():
        raise FileNotFoundError(
            "LocateAnything deployment manifest is missing: {}".format(manifest)
        )
    try:
        manifest.relative_to(root)
    except ValueError as error:
        raise ValueError(
            "LocateAnything deployment manifest must be inside the model root"
        ) from error

    payload = manifest.read_bytes()
    digest = sha256(payload).hexdigest()
    expected = str(expected_digest).strip().lower()
    if expected and (not _valid_sha256(expected) or digest != expected):
        raise RuntimeError(
            "LocateAnything manifest SHA256 mismatch: expected {}, got {}".format(
                expected, digest
            )
        )
    try:
        data = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("LocateAnything deployment manifest is invalid JSON") from error
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("LocateAnything deployment manifest schema must be 1")
    repo_id = str(data.get("repo_id", "")).strip()
    revision = str(data.get("revision", "")).strip().lower()
    if repo_id != "nvidia/LocateAnything-3B":
        raise ValueError("unexpected LocateAnything repo_id: {}".format(repo_id))
    if len(revision) != 40 or any(character not in _HEX_DIGITS for character in revision):
        raise ValueError("LocateAnything revision must be a 40-character Git commit")
    files = data.get("files")
    if not isinstance(files, list) or not files:
        raise ValueError("LocateAnything deployment manifest has no files")

    seen = set()
    verified = []
    for index, entry in enumerate(files):
        if not isinstance(entry, dict):
            raise ValueError("manifest file entry {} is not an object".format(index))
        relative = str(entry.get("path", "")).strip()
        expected_sha = str(entry.get("sha256", "")).strip().lower()
        expected_size = entry.get("size")
        if not relative or relative in seen:
            raise ValueError("manifest contains an empty or duplicate file path")
        if Path(relative).is_absolute() or ".." in Path(relative).parts:
            raise ValueError("manifest file path escapes the model root: {}".format(relative))
        if not _valid_sha256(expected_sha):
            raise ValueError("manifest has an invalid SHA256 for {}".format(relative))
        if not isinstance(expected_size, int) or expected_size < 1:
            raise ValueError("manifest has an invalid size for {}".format(relative))
        path = (root / relative).resolve()
        try:
            path.relative_to(root)
        except ValueError as error:
            raise ValueError("manifest file resolves outside model root: {}".format(relative)) from error
        if not path.is_file():
            raise FileNotFoundError("LocateAnything model file is missing: {}".format(path))
        actual_size = path.stat().st_size
        if actual_size != expected_size:
            raise RuntimeError(
                "LocateAnything file size mismatch for {}: expected {}, got {}".format(
                    relative, expected_size, actual_size
                )
            )
        if verify_files:
            actual_sha = _file_sha256(path)
            if actual_sha != expected_sha:
                raise RuntimeError(
                    "LocateAnything file SHA256 mismatch for {}: expected {}, got {}".format(
                        relative, expected_sha, actual_sha
                    )
                )
        seen.add(relative)
        verified.append(dict(entry))

    required = {
        "config.json",
        "model.safetensors.index.json",
        "model-00001-of-00002.safetensors",
        "model-00002-of-00002.safetensors",
        "tokenizer_config.json",
        "preprocessor_config.json",
        "processor_config.json",
        "modeling_locateanything.py",
        "processing_locateanything.py",
    }
    missing = sorted(required - seen)
    if missing:
        raise ValueError(
            "LocateAnything deployment manifest omits required files: {}".format(
                ", ".join(missing)
            )
        )
    return VerifiedManifest(
        digest=digest,
        repo_id=repo_id,
        revision=revision,
        files=tuple(verified),
    )


def parse_categories(values: Iterable[Mapping[str, object]]) -> Tuple[LocateAnythingCategory, ...]:
    categories = []
    ids = set()
    names = set()
    for raw in values:
        if not isinstance(raw, Mapping):
            raise ValueError("each LocateAnything category must be an object")
        class_id = int(raw.get("class_id", -1))
        class_name = str(raw.get("class_name", "")).strip()
        prompt = str(raw.get("prompt", "")).strip()
        query_prompts_raw = raw.get("query_prompts", [])
        if isinstance(query_prompts_raw, str):
            query_prompts = tuple(
                item.strip()
                for item in query_prompts_raw.split("</c>")
                if item.strip()
            )
        elif isinstance(query_prompts_raw, (list, tuple)):
            query_prompts = tuple(
                str(item).strip()
                for item in query_prompts_raw
                if str(item).strip()
            )
        else:
            raise ValueError(
                "LocateAnything category query_prompts must be a list or "
                "</c>-separated string"
            )
        if not prompt and query_prompts:
            prompt = query_prompts[0]
        if not query_prompts and prompt:
            query_prompts = (prompt,)
        aliases_raw = raw.get("aliases", [])
        if isinstance(aliases_raw, str):
            aliases = tuple(
                item.strip() for item in aliases_raw.split(",") if item.strip()
            )
        elif isinstance(aliases_raw, (list, tuple)):
            aliases = tuple(str(item).strip() for item in aliases_raw if str(item).strip())
        else:
            raise ValueError("LocateAnything category aliases must be a list or CSV string")
        if class_id < 0 or not class_name or not prompt or not query_prompts:
            raise ValueError("LocateAnything categories require class_id, class_name, and prompt")
        if len({_normalise_label(item) for item in query_prompts}) != len(
            query_prompts
        ):
            raise ValueError("LocateAnything query_prompts must be unique")
        if class_id in ids or class_name in names:
            raise ValueError("LocateAnything category IDs and names must be unique")
        ids.add(class_id)
        names.add(class_name)
        categories.append(
            LocateAnythingCategory(
                class_id,
                class_name,
                prompt,
                aliases,
                query_prompts,
            )
        )
    if not categories:
        raise ValueError("at least one LocateAnything category is required")
    return tuple(categories)


def _normalise_label(value: str) -> str:
    text = str(value).strip().lower().replace("_", " ").replace("-", " ")
    return " ".join("".join(character if character.isalnum() else " " for character in text).split())


def _category_for_label(
    label: str, categories: Sequence[LocateAnythingCategory]
) -> Optional[LocateAnythingCategory]:
    # With one requested semantic category, every box emitted for that query
    # belongs to the same output contract even if the VLM chooses a more
    # specific <ref> label such as "plastic bag" or "food wrapper".
    if len(categories) == 1:
        return categories[0]
    normalised = _normalise_label(label)
    if not normalised:
        return None
    exact = {}
    for category in categories:
        for candidate in (
            (category.class_name,)
            + category.grounding_prompts
            + category.aliases
        ):
            candidate_normalised = _normalise_label(candidate)
            if candidate_normalised:
                exact[candidate_normalised] = category
    if normalised in exact:
        return exact[normalised]
    matches = {
        category
        for candidate, category in exact.items()
        if len(candidate) >= 4
        and (candidate in normalised or normalised in candidate)
    }
    return next(iter(matches)) if len(matches) == 1 else None


def _box_iou(left: Mapping[str, float], right: Mapping[str, float]) -> float:
    ix1 = max(left["xmin"], right["xmin"])
    iy1 = max(left["ymin"], right["ymin"])
    ix2 = min(left["xmax"], right["xmax"])
    iy2 = min(left["ymax"], right["ymax"])
    intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    left_area = max(0.0, left["xmax"] - left["xmin"]) * max(
        0.0, left["ymax"] - left["ymin"]
    )
    right_area = max(0.0, right["xmax"] - right["xmin"]) * max(
        0.0, right["ymax"] - right["ymin"]
    )
    union = left_area + right_area - intersection
    return intersection / union if union > 0.0 else 0.0


def parse_locateanything_answer(
    answer: str,
    categories: Sequence[LocateAnythingCategory],
    image_width: int,
    image_height: int,
    max_detections: int = 100,
    min_box_area_fraction: float = 0.00005,
    max_box_area_fraction: float = 0.75,
) -> Tuple[List[Dict[str, object]], int]:
    """Parse labelled normalized boxes; return boxes and ignored-box count."""
    import re

    if image_width < 1 or image_height < 1:
        raise ValueError("image dimensions must be positive")
    if not 0.0 <= min_box_area_fraction < max_box_area_fraction <= 1.0:
        raise ValueError("LocateAnything box-area fractions are invalid")
    token_pattern = re.compile(
        r"<ref>(.*?)</ref>|<box><(-?\d+)><(-?\d+)><(-?\d+)><(-?\d+)></box>",
        re.IGNORECASE | re.DOTALL,
    )
    current_label = ""
    parsed: List[Dict[str, object]] = []
    ignored = 0
    for match in token_pattern.finditer(str(answer)):
        if match.group(1) is not None:
            current_label = match.group(1)
            continue
        category = _category_for_label(current_label, categories)
        coordinates = [int(match.group(index)) for index in range(2, 6)]
        if category is None or any(value < 0 or value > 1000 for value in coordinates):
            ignored += 1
            continue
        x1, y1, x2, y2 = coordinates
        xmin = min(x1, x2) / 1000.0 * image_width
        ymin = min(y1, y2) / 1000.0 * image_height
        xmax = max(x1, x2) / 1000.0 * image_width
        ymax = max(y1, y2) / 1000.0 * image_height
        if xmax - xmin < 1.0 or ymax - ymin < 1.0:
            ignored += 1
            continue
        area_fraction = (
            (xmax - xmin) * (ymax - ymin) / float(image_width * image_height)
        )
        if not min_box_area_fraction <= area_fraction <= max_box_area_fraction:
            ignored += 1
            continue
        candidate: Dict[str, object] = {
            "class_id": category.class_id,
            "class_name": category.class_name,
            "xmin": xmin,
            "ymin": ymin,
            "xmax": xmax,
            "ymax": ymax,
        }
        duplicate = any(
            item["class_id"] == category.class_id
            and _box_iou(item, candidate) >= 0.98
            for item in parsed
        )
        if duplicate:
            ignored += 1
            continue
        parsed.append(candidate)
        if len(parsed) >= max(1, int(max_detections)):
            break
    return parsed, ignored


class LocateAnythingDetector:
    """Synchronous detector facade backed by an isolated model worker."""

    backend = "locateanything"
    task = "detect"
    motion_eligible = False
    confidence_calibrated = False
    gam_layer_count = 0
    ultralytics_path = ""
    ultralytics_version = "not-used"

    def __init__(
        self,
        model_root: str,
        manifest_path: str,
        expected_sha256: str,
        worker_python: str,
        categories: Sequence[LocateAnythingCategory],
        generation_mode: str = "hybrid",
        max_new_tokens: int = 512,
        temperature: float = 0.0,
        max_image_side: int = 896,
        max_detections: int = 100,
        jpeg_quality: int = 95,
        min_box_area_fraction: float = 0.00005,
        max_box_area_fraction: float = 0.75,
        startup_timeout_sec: float = 420.0,
        inference_timeout_sec: float = 180.0,
    ):
        self.model_root = str(Path(model_root).expanduser().resolve())
        self.manifest_path = str(Path(manifest_path).expanduser().resolve())
        manifest = verify_model_manifest(
            self.model_root,
            self.manifest_path,
            expected_sha256,
            verify_files=False,
        )
        self.model_sha256 = manifest.digest
        self.model_name = "LocateAnything-3B@{}".format(manifest.revision[:12])
        self.categories = tuple(categories)
        self.names: Dict[int, str] = {
            category.class_id: category.class_name for category in self.categories
        }
        self.device = "cuda"
        self.runtime_path = self.model_root
        self.runtime_version = "loading"
        self.generation_mode = str(generation_mode).strip().lower()
        if self.generation_mode not in ("fast", "slow", "hybrid"):
            raise ValueError("LocateAnything generation_mode must be fast, slow, or hybrid")
        self.max_new_tokens = int(max_new_tokens)
        self.temperature = float(temperature)
        self.max_image_side = validate_max_image_side(max_image_side)
        self.max_detections = int(max_detections)
        self.jpeg_quality = int(jpeg_quality)
        self.min_box_area_fraction = float(min_box_area_fraction)
        self.max_box_area_fraction = float(max_box_area_fraction)
        self.startup_timeout_sec = float(startup_timeout_sec)
        self.inference_timeout_sec = float(inference_timeout_sec)
        if not 32 <= self.max_new_tokens <= 4096:
            raise ValueError("LocateAnything max_new_tokens must be in [32, 4096]")
        if not 1 <= self.max_detections <= 300:
            raise ValueError("LocateAnything max_detections must be in [1, 300]")
        if not 80 <= self.jpeg_quality <= 100:
            raise ValueError("LocateAnything JPEG quality must be in [80, 100]")
        if not 0.0 <= self.min_box_area_fraction < self.max_box_area_fraction <= 1.0:
            raise ValueError("LocateAnything box-area fractions are invalid")
        # Do not resolve the venv's python symlink: CPython uses the invoked
        # path to discover pyvenv.cfg.  Resolving it would silently select the
        # system interpreter and its CPU-only user-site PyTorch.
        python = Path(worker_python).expanduser()
        if not python.is_absolute():
            python = (Path.cwd() / python).absolute()
        if not python.is_file() or not os.access(str(python), os.X_OK):
            raise FileNotFoundError("LocateAnything worker Python is not executable: {}".format(python))
        self.worker_python = str(python)
        self._lock = threading.Lock()
        self._sequence = 0
        self._process = None
        self._stderr_stream = None
        self.worker_log = ""
        self.last_answer = ""
        self.last_ignored_boxes = 0
        self.semantic_prompt_preloaded = False
        self.semantic_prompt_sha256 = ""
        self.semantic_prompt_token_count = 0
        self.semantic_query_count = 0
        self.last_prompt_tensor_cache_hit = False
        self.prompt_tensor_cache_entries = 0
        self._start_worker(manifest)

    def _worker_environment(self) -> Dict[str, str]:
        environment = dict(os.environ)
        cache = Path(self.model_root) / ".cache"
        runtime = Path(self.model_root) / ".runtime"
        directories = {
            "HF_HOME": cache / "huggingface",
            "HUGGINGFACE_HUB_CACHE": cache / "huggingface" / "hub",
            "TRANSFORMERS_CACHE": cache / "huggingface" / "transformers",
            "HF_ASSETS_CACHE": cache / "huggingface" / "assets",
            "HF_MODULES_CACHE": cache / "huggingface" / "modules",
            "TORCH_HOME": cache / "torch",
            "XDG_CACHE_HOME": cache / "xdg",
            "CUDA_CACHE_PATH": runtime / "cuda_cache",
            "TRITON_CACHE_DIR": runtime / "triton",
            "NUMBA_CACHE_DIR": runtime / "numba",
            "TMPDIR": runtime / "tmp",
            "PYTHONPYCACHEPREFIX": runtime / "pycache",
        }
        for path in directories.values():
            path.mkdir(parents=True, exist_ok=True)
        environment.update({key: str(value) for key, value in directories.items()})
        environment["TRANSFORMERS_OFFLINE"] = "1"
        environment["HF_HUB_OFFLINE"] = "1"
        environment["TOKENIZERS_PARALLELISM"] = "false"
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        return environment

    def _start_worker(self, manifest: VerifiedManifest) -> None:
        runtime = Path(self.model_root) / ".runtime"
        log_dir = runtime / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / "locateanything_worker_{}_{}.log".format(stamp, os.getpid())
        self._stderr_stream = log_path.open("ab", buffering=0)
        self.worker_log = str(log_path)
        command = [
            self.worker_python,
            "-m",
            "autolabor_fod_vision.locateanything_worker",
            "--model-root",
            self.model_root,
            "--manifest",
            self.manifest_path,
            "--expected-sha256",
            self.model_sha256,
        ]
        self._process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=self._stderr_stream,
            env=self._worker_environment(),
            bufsize=0,
            close_fds=True,
        )
        try:
            ready = self._read_response(self.startup_timeout_sec)
            if ready.get("event") != "ready" or not ready.get("ok"):
                raise RuntimeError("LocateAnything worker did not report readiness: {}".format(ready))
            if ready.get("model_sha256") != manifest.digest:
                raise RuntimeError("LocateAnything worker reported the wrong model identity")
            if ready.get("motion_eligible") is not False:
                raise RuntimeError("LocateAnything worker must remain motion-ineligible")
            assert self._process.stdin is not None
            self._process.stdin.write(
                json.dumps(
                    self._configuration_request(), separators=(",", ":")
                ).encode("utf-8")
                + b"\n"
            )
            self._process.stdin.flush()
            configured = self._read_response(self.startup_timeout_sec)
            if (
                configured.get("event") != "configured"
                or not configured.get("ok")
                or configured.get("semantic_prompt_preloaded") is not True
            ):
                raise RuntimeError(
                    "LocateAnything worker did not preload the semantic prompt: {}".format(
                        configured
                    )
                )
            if int(configured.get("category_count", -1)) != len(self.categories):
                raise RuntimeError(
                    "LocateAnything worker preloaded the wrong category count"
                )
            expected_query_count = sum(
                len(category.grounding_prompts)
                for category in self.categories
            )
            if int(configured.get("query_count", -1)) != expected_query_count:
                raise RuntimeError(
                    "LocateAnything worker preloaded the wrong query count"
                )
            semantic_sha256 = str(
                configured.get("semantic_prompt_sha256", "")
            ).strip().lower()
            semantic_token_count = int(
                configured.get("semantic_prompt_token_count", 0)
            )
            if not _valid_sha256(semantic_sha256) or semantic_token_count < 1:
                raise RuntimeError(
                    "LocateAnything worker reported invalid semantic prompt metadata"
                )
            self.semantic_prompt_preloaded = True
            self.semantic_prompt_sha256 = semantic_sha256
            self.semantic_prompt_token_count = semantic_token_count
            self.semantic_query_count = expected_query_count
            self.runtime_version = "transformers={} torch={}".format(
                ready.get("transformers_version", "unknown"),
                ready.get("torch_version", "unknown"),
            )
            self.device = str(ready.get("device", "cuda"))
        except Exception:
            self.shutdown()
            raise

    def _configuration_request(self) -> Dict[str, object]:
        """Build the single immutable semantic/configuration worker request."""
        return {
            "op": "configure",
            "categories": [
                {
                    "class_id": category.class_id,
                    "class_name": category.class_name,
                    "prompt": category.prompt,
                    "aliases": list(category.aliases),
                    "query_prompts": list(category.grounding_prompts),
                }
                for category in self.categories
            ],
            "generation_mode": self.generation_mode,
            "max_new_tokens": self.max_new_tokens,
            "temperature": self.temperature,
            "max_image_side": self.max_image_side,
            "max_detections": self.max_detections,
            "min_box_area_fraction": self.min_box_area_fraction,
            "max_box_area_fraction": self.max_box_area_fraction,
        }

    @staticmethod
    def _prediction_request(request_id: int, encoded_jpeg: bytes) -> Dict[str, object]:
        """Build an image-only request; semantic text is preloaded once."""
        return {
            "op": "predict",
            "id": int(request_id),
            "image_jpeg_b64": base64.b64encode(encoded_jpeg).decode("ascii"),
        }

    def _read_response(self, timeout_sec: float) -> Dict[str, object]:
        if self._process is None or self._process.stdout is None:
            raise RuntimeError("LocateAnything worker is not running")
        selector = selectors.DefaultSelector()
        selector.register(self._process.stdout, selectors.EVENT_READ)
        try:
            events = selector.select(max(0.0, timeout_sec))
        finally:
            selector.close()
        if not events:
            raise TimeoutError(
                "LocateAnything worker timed out after {:.1f}s; log={}".format(
                    timeout_sec, self.worker_log
                )
            )
        line = self._process.stdout.readline()
        if not line:
            code = self._process.poll()
            raise RuntimeError(
                "LocateAnything worker exited unexpectedly (code {}); log={}".format(
                    code, self.worker_log
                )
            )
        try:
            response = json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise RuntimeError("LocateAnything worker returned invalid JSON") from error
        if not isinstance(response, dict):
            raise RuntimeError("LocateAnything worker response is not an object")
        return response

    def predict(self, image_bgr: np.ndarray) -> InferenceResult:
        if not isinstance(image_bgr, np.ndarray) or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("LocateAnything expects a BGR HxWx3 image")
        with self._lock:
            if self._process is None or self._process.poll() is not None:
                raise RuntimeError("LocateAnything worker is not available; log={}".format(self.worker_log))
            encoded_ok, encoded = cv2.imencode(
                ".jpg",
                image_bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not encoded_ok:
                raise RuntimeError("failed to encode a frame for LocateAnything")
            self._sequence += 1
            request_id = self._sequence
            request = self._prediction_request(request_id, encoded.tobytes())
            assert self._process.stdin is not None
            try:
                self._process.stdin.write(
                    json.dumps(request, separators=(",", ":")).encode("utf-8") + b"\n"
                )
                self._process.stdin.flush()
                response = self._read_response(self.inference_timeout_sec)
            except Exception:
                self.shutdown()
                raise
            if response.get("id") != request_id:
                self.shutdown()
                raise RuntimeError("LocateAnything worker response ID mismatch")
            if not response.get("ok"):
                raise RuntimeError(
                    "LocateAnything inference failed: {}".format(
                        response.get("error", "unknown worker error")
                    )
                )
            if response.get("semantic_prompt_preloaded") is not True:
                self.shutdown()
                raise RuntimeError(
                    "LocateAnything worker lost its preloaded semantic prompt"
                )
            self.last_answer = str(response.get("answer", ""))
            self.last_ignored_boxes = int(response.get("ignored_boxes", 0))
            self.last_prompt_tensor_cache_hit = bool(
                response.get("prompt_tensor_cache_hit", False)
            )
            self.prompt_tensor_cache_entries = int(
                response.get("prompt_tensor_cache_entries", 0)
            )
            detections = []
            for raw in response.get("detections", []):
                if not isinstance(raw, dict):
                    continue
                xmin = float(raw["xmin"])
                ymin = float(raw["ymin"])
                xmax = float(raw["xmax"])
                ymax = float(raw["ymax"])
                detections.append(
                    Detection(
                        class_id=int(raw["class_id"]),
                        class_name=str(raw["class_name"]),
                        # LocateAnything does not expose calibrated per-box scores.
                        # Zero keeps these recognition boxes outside all motion gates.
                        confidence=0.0,
                        xmin=xmin,
                        ymin=ymin,
                        xmax=xmax,
                        ymax=ymax,
                        anchor_u=0.5 * (xmin + xmax),
                        anchor_v=ymax,
                    )
                )
            return InferenceResult(
                detections=detections,
                inference_ms=float(response.get("inference_ms", 0.0)),
            )

    def shutdown(self) -> None:
        process = self._process
        self._process = None
        if process is not None:
            try:
                if process.stdin is not None and process.poll() is None:
                    process.stdin.write(b'{"op":"shutdown"}\n')
                    process.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.0)
            for stream in (process.stdin, process.stdout):
                if stream is not None:
                    stream.close()
        if self._stderr_stream is not None:
            self._stderr_stream.close()
            self._stderr_stream = None
