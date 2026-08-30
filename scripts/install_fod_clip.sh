#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DETECTOR_PYTHON="${FOD_YOLO_PYTHON:-/home/slam/robot_ws/.venv/fod_yolo/bin/python3}"
CLIP_RUNTIME_ROOT="${FOD_CLIP_RUNTIME_ROOT:-$WORKSPACE_ROOT/runtime/fod_clip}"
CLIP_PYTHON_ROOT="${FOD_CLIP_PYTHON_ROOT:-$CLIP_RUNTIME_ROOT/python}"
CLIP_WEIGHT_DIR="${FOD_CLIP_WEIGHT_DIR:-$WORKSPACE_ROOT/src/application/autolabor_fod_vision/models/clip}"
CLIP_WEIGHT_FILE="$CLIP_WEIGHT_DIR/ViT-B-32.pt"
CLIP_CACHE_FILE="${FOD_CLIP_CACHE_FILE:-/home/slam/.cache/clip/ViT-B-32.pt}"
CLIP_SOURCE_COMMIT=d05afc436d78f1c48dc0dbf8e5980a9d471f35f6
CLIP_WEIGHT_SHA256=40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af
CLIP_WEIGHT_URL="https://openaipublic.azureedge.net/clip/models/$CLIP_WEIGHT_SHA256/ViT-B-32.pt"

[[ -x "$DETECTOR_PYTHON" ]] || {
  echo "FOD detector Python is missing: $DETECTOR_PYTHON" >&2
  exit 2
}
mkdir -p "$CLIP_PYTHON_ROOT" "$CLIP_WEIGHT_DIR"

clip_install_valid() {
  CLIP_PYTHON_ROOT="$CLIP_PYTHON_ROOT" \
  CLIP_SOURCE_COMMIT="$CLIP_SOURCE_COMMIT" \
  PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH="$CLIP_PYTHON_ROOT" \
    "$DETECTOR_PYTHON" - <<'PY' >/dev/null 2>&1
import importlib.metadata
import json
import os
from pathlib import Path

root = Path(os.environ["CLIP_PYTHON_ROOT"]).resolve()
distribution = importlib.metadata.distribution("clip")
direct_url = Path(distribution.locate_file("clip-1.0.dist-info/direct_url.json"))
document = json.loads(direct_url.read_text(encoding="utf-8"))
commit = document.get("vcs_info", {}).get("commit_id", "")
if commit != os.environ["CLIP_SOURCE_COMMIT"]:
    raise SystemExit(1)
module_path = Path(distribution.locate_file("clip/__init__.py")).resolve()
module_path.relative_to(root)
PY
}

if clip_install_valid; then
  echo "Official OpenAI CLIP Python source is already pinned at $CLIP_SOURCE_COMMIT."
elif [[ -e "$CLIP_PYTHON_ROOT/clip" || -e "$CLIP_PYTHON_ROOT/clip-1.0.dist-info" ]]; then
  echo "Existing CLIP runtime is incomplete or not pinned to $CLIP_SOURCE_COMMIT:" >&2
  echo "  $CLIP_PYTHON_ROOT" >&2
  echo "Preserving it unchanged; choose an empty FOD_CLIP_RUNTIME_ROOT and retry." >&2
  exit 3
else
  "$DETECTOR_PYTHON" -m pip install \
    --no-deps \
    --target "$CLIP_PYTHON_ROOT" \
    "git+https://github.com/openai/CLIP.git@$CLIP_SOURCE_COMMIT"
  clip_install_valid || {
    echo "Installed CLIP source failed the pinned-commit validation." >&2
    exit 3
  }
fi

weight_hash=""
if [[ -f "$CLIP_WEIGHT_FILE" ]]; then
  weight_hash="$(sha256sum -- "$CLIP_WEIGHT_FILE" | awk '{print $1}')"
  [[ "$weight_hash" == "$CLIP_WEIGHT_SHA256" ]] || {
    echo "Existing CLIP weight has an unexpected SHA256; preserving it unchanged:" >&2
    echo "  $CLIP_WEIGHT_FILE" >&2
    exit 4
  }
else
  cache_hash=""
  if [[ -f "$CLIP_CACHE_FILE" ]]; then
    cache_hash="$(sha256sum -- "$CLIP_CACHE_FILE" | awk '{print $1}')"
  fi
  if [[ "$cache_hash" == "$CLIP_WEIGHT_SHA256" ]]; then
    cp --reflink=auto --no-clobber -- "$CLIP_CACHE_FILE" "$CLIP_WEIGHT_FILE"
  else
    partial="$CLIP_WEIGHT_FILE.partial"
    curl --fail --location --output "$partial" "$CLIP_WEIGHT_URL"
    partial_hash="$(sha256sum -- "$partial" | awk '{print $1}')"
    [[ "$partial_hash" == "$CLIP_WEIGHT_SHA256" ]] || {
      echo "Downloaded CLIP weight SHA256 mismatch: $partial_hash" >&2
      exit 4
    }
    [[ ! -e "$CLIP_WEIGHT_FILE" ]] || {
      echo "CLIP weight appeared concurrently; preserving both files." >&2
      exit 4
    }
    mv -- "$partial" "$CLIP_WEIGHT_FILE"
  fi
  chmod 0644 "$CLIP_WEIGHT_FILE"
fi

CLIP_PYTHON_ROOT="$CLIP_PYTHON_ROOT" \
CLIP_WEIGHT_FILE="$CLIP_WEIGHT_FILE" \
CLIP_WEIGHT_SHA256="$CLIP_WEIGHT_SHA256" \
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$CLIP_PYTHON_ROOT" \
  "$DETECTOR_PYTHON" - <<'PY'
import hashlib
import os
from pathlib import Path

import clip
import ftfy
import regex

root = Path(os.environ["CLIP_PYTHON_ROOT"]).resolve() / "clip"
module = Path(clip.__file__).resolve()
module.relative_to(root)
weight = Path(os.environ["CLIP_WEIGHT_FILE"]).resolve()
hasher = hashlib.sha256()
with weight.open("rb") as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        hasher.update(chunk)
digest = hasher.hexdigest()
if digest != os.environ["CLIP_WEIGHT_SHA256"]:
    raise SystemExit("CLIP weight validation changed during preflight")
tokens = clip.tokenize(["地面上的垃圾", "地面反光、纹理或阴影"])
if tuple(tokens.shape) != (2, 77):
    raise SystemExit("unexpected official CLIP token shape: {}".format(tokens.shape))
print("clip_import={}".format(module))
print("clip_weight={}".format(weight))
print("clip_weight_sha256={}".format(digest))
print("clip_tokenizer_shape={}".format(tuple(tokens.shape)))
PY

echo "FOD CLIP runtime is ready."
