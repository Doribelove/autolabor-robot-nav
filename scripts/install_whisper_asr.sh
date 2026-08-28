#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_PARENT="$(cd "$WORKSPACE_ROOT/.." && pwd)"

ASR_RUNTIME_ROOT="${SWEEPER_ASR_RUNTIME_ROOT:-$WORKSPACE_ROOT/runtime/asr}"
ASR_VENV="${SWEEPER_ASR_VENV:-$ASR_RUNTIME_ROOT/venv}"
ASR_MODEL_DIR="${SWEEPER_ASR_MODEL_DIR:-$ASR_RUNTIME_ROOT/models}"
ASR_WHEEL_DIR="$ASR_RUNTIME_ROOT/wheels"
ASR_PYTHON="${SWEEPER_ASR_BOOTSTRAP_PYTHON:-/usr/bin/python3}"
ASR_PYPI_INDEX="${SWEEPER_ASR_PYPI_INDEX:-https://pypi.org/simple}"

TORCH_WHEEL_NAME="torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"
TORCH_WHEEL_SOURCE="${SWEEPER_ASR_TORCH_SEED:-$WORKSPACE_PARENT/robot_ws/.deps/wheels/$TORCH_WHEEL_NAME}"
TORCH_WHEEL_COPY="$ASR_WHEEL_DIR/$TORCH_WHEEL_NAME"
TORCH_WHEEL_SHA256="39eeb9894ef8c7b84249ab917f212a91703f30255d591b956ab12cc10e836532"

WHISPER_COMMIT="5f86d1d86363843179951550570367b37c5d6f78"
WHISPER_SMALL_SHA256="9ecf779972d90ba49c06d968637d720dd632c55bbf19d441fb42bf17a411e794"
WHISPER_MEDIUM_SHA256="345ae4da62f9b3d59415adc60127b97c714f32e89e936602e85993674d08dcb1"
WHISPER_LARGE_SHA256="e5b1a55b89c1367dacf97e3e19bfd829a01529dbfdeefa8caeb59b3f1b81dadb"
WHISPER_COMMIT_MARKER="$ASR_RUNTIME_ROOT/whisper.commit"

usage() {
  cat <<EOF
Usage: $0

Create an isolated CUDA Whisper ASR environment under:
  $ASR_RUNTIME_ROOT

The installer copies and verifies the known Jetson CUDA PyTorch wheel before
installation, pins OpenAI Whisper to a Git commit, and downloads the fixed
small, medium, and large-v3 checkpoints through .partial files followed by
SHA-256 verification and atomic renames. Runtime ASR code cannot download.

Optional environment overrides:
  SWEEPER_ASR_RUNTIME_ROOT
  SWEEPER_ASR_VENV
  SWEEPER_ASR_MODEL_DIR
  SWEEPER_ASR_TORCH_SEED
  SWEEPER_ASR_BOOTSTRAP_PYTHON
  SWEEPER_ASR_PYPI_INDEX
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  usage >&2
  exit 2
fi

hash_matches() {
  local path="$1" expected="$2" actual
  [[ -f "$path" ]] || return 1
  actual="$(sha256sum -- "$path" | awk '{print $1}')"
  [[ "${actual,,}" == "${expected,,}" ]]
}

copy_torch_seed() {
  local partial="$TORCH_WHEEL_COPY.partial" actual
  [[ -f "$TORCH_WHEEL_SOURCE" ]] || {
    echo "Missing NVIDIA CUDA PyTorch seed wheel: $TORCH_WHEEL_SOURCE" >&2
    exit 3
  }
  actual="$(sha256sum -- "$TORCH_WHEEL_SOURCE" | awk '{print $1}')"
  [[ "${actual,,}" == "$TORCH_WHEEL_SHA256" ]] || {
    echo "NVIDIA CUDA PyTorch seed SHA-256 mismatch: $actual" >&2
    exit 3
  }
  if hash_matches "$TORCH_WHEEL_COPY" "$TORCH_WHEEL_SHA256"; then
    return 0
  fi
  cp -- "$TORCH_WHEEL_SOURCE" "$partial"
  hash_matches "$partial" "$TORCH_WHEEL_SHA256" || {
    echo "Copied NVIDIA CUDA PyTorch wheel failed SHA-256 verification." >&2
    exit 3
  }
  chmod 0644 "$partial"
  mv -f -- "$partial" "$TORCH_WHEEL_COPY"
}

download_checkpoint() {
  local model="$1" filename="$2" expected="$3"
  local model_path="$ASR_MODEL_DIR/$filename"
  local partial="$model_path.partial"
  local url="https://openaipublic.azureedge.net/main/whisper/models/$expected/$filename"
  if hash_matches "$model_path" "$expected"; then
    echo "Whisper $model checkpoint already verified: $model_path"
    return 0
  fi

  echo "Downloading fixed Whisper $model checkpoint..."
  if command -v curl >/dev/null 2>&1; then
    curl --fail --location --retry 3 --connect-timeout 20 \
      --proto '=https' --tlsv1.2 \
      --continue-at - \
      --output "$partial" "$url"
  elif command -v wget >/dev/null 2>&1; then
    wget --continue --tries=3 --timeout=20 --output-document="$partial" \
      "$url"
  else
    echo "curl or wget is required to download the Whisper checkpoint." >&2
    exit 4
  fi

  if ! hash_matches "$partial" "$expected"; then
    local actual="unreadable"
    if [[ -f "$partial" ]]; then
      actual="$(sha256sum -- "$partial" | awk '{print $1}')"
    fi
    echo "Whisper checkpoint SHA-256 mismatch: $actual" >&2
    echo "Expected: $expected" >&2
    exit 4
  fi
  chmod 0644 "$partial"
  mv -f -- "$partial" "$model_path"
}

[[ "$(uname -m)" == "aarch64" ]] || {
  echo "This installer is pinned to the NVIDIA Jetson aarch64 wheel." >&2
  exit 2
}
[[ -x "$ASR_PYTHON" ]] || {
  echo "Python executable not found: $ASR_PYTHON" >&2
  exit 2
}
[[ "$($ASR_PYTHON -c 'import sys; print("%d.%d" % sys.version_info[:2])')" == "3.8" ]] || {
  echo "JetPack 5 / ROS Noetic ASR requires Python 3.8." >&2
  exit 2
}
command -v git >/dev/null 2>&1 || {
  echo "git is required to install the pinned OpenAI Whisper commit." >&2
  exit 2
}
command -v arecord >/dev/null 2>&1 || {
  echo "alsa-utils/arecord is required for microphone capture." >&2
  exit 2
}

mkdir -p "$ASR_RUNTIME_ROOT" "$ASR_WHEEL_DIR" "$ASR_MODEL_DIR"
copy_torch_seed

if [[ ! -x "$ASR_VENV/bin/python3" ]] ||
   ! "$ASR_VENV/bin/python3" -m pip --version >/dev/null 2>&1; then
  if "$ASR_PYTHON" -m virtualenv --version >/dev/null 2>&1; then
    # JetPack images commonly omit python3.8-venv/ensurepip.  Reuse the
    # user-installed virtualenv bootstrap without changing system packages.
    "$ASR_PYTHON" -m virtualenv --clear --python "$ASR_PYTHON" "$ASR_VENV"
  else
    "$ASR_PYTHON" -m venv --clear "$ASR_VENV"
  fi
fi
VENV_PYTHON="$ASR_VENV/bin/python3"
[[ "$($VENV_PYTHON -c 'import sys; print("%d.%d" % sys.version_info[:2])')" == "3.8" ]] || {
  echo "Existing ASR environment is not Python 3.8: $ASR_VENV" >&2
  exit 2
}

export PIP_DISABLE_PIP_VERSION_CHECK=1
"$VENV_PYTHON" -m pip install --index-url "$ASR_PYPI_INDEX" \
  pip==25.0.1 setuptools==75.3.4 wheel==0.45.1

if ! "$VENV_PYTHON" - <<'PY' >/dev/null 2>&1
import torch

assert torch.__version__.startswith("2.0.0+nv23.05")
assert torch.version.cuda == "11.4"
PY
then
  "$VENV_PYTHON" -m pip install --no-deps --force-reinstall \
    "$TORCH_WHEEL_COPY"
fi

# Versions are fixed for Python 3.8 and the JetPack 5 NVIDIA PyTorch wheel.
"$VENV_PYTHON" -m pip install --index-url "$ASR_PYPI_INDEX" \
  filelock==3.16.1 \
  jinja2==3.1.4 \
  llvmlite==0.41.1 \
  more-itertools==10.5.0 \
  networkx==3.1 \
  numba==0.58.1 \
  numpy==1.24.4 \
  opencc-python-reimplemented==0.1.7 \
  sympy==1.13.3 \
  tiktoken==0.7.0 \
  tqdm==4.66.5 \
  typing-extensions==4.12.2

whisper_commit_matches() {
  SWEEPER_ASR_EXPECTED_COMMIT="$WHISPER_COMMIT" \
    "$VENV_PYTHON" - >/dev/null 2>&1 <<'PY'
import importlib.metadata
import json
import os

import whisper

distribution = importlib.metadata.distribution("openai-whisper")
direct_url = json.loads(distribution.read_text("direct_url.json"))
commit = direct_url.get("vcs_info", {}).get("commit_id", "")
assert commit == os.environ["SWEEPER_ASR_EXPECTED_COMMIT"]
PY
}

if ! whisper_commit_matches; then
  "$VENV_PYTHON" -m pip install --no-deps --force-reinstall \
    "git+https://github.com/openai/whisper.git@$WHISPER_COMMIT"
  printf '%s\n' "$WHISPER_COMMIT" >"$WHISPER_COMMIT_MARKER.partial"
  mv -f -- "$WHISPER_COMMIT_MARKER.partial" "$WHISPER_COMMIT_MARKER"
fi
whisper_commit_matches || {
  echo "Installed OpenAI Whisper commit does not match $WHISPER_COMMIT" >&2
  exit 5
}

download_checkpoint "small" "small.pt" "$WHISPER_SMALL_SHA256"
download_checkpoint "medium" "medium.pt" "$WHISPER_MEDIUM_SHA256"
download_checkpoint "large" "large-v3.pt" "$WHISPER_LARGE_SHA256"
"$VENV_PYTHON" -m pip check

"$VENV_PYTHON" - <<'PY'
import torch
import whisper

assert torch.__version__.startswith("2.0.0+nv23.05")
assert torch.version.cuda == "11.4"
assert torch.cuda.is_available(), "NVIDIA CUDA is unavailable to the ASR venv"
print("torch={}".format(torch.__version__))
print("cuda_device={}".format(torch.cuda.get_device_name(0)))
print("whisper_module={}".format(whisper.__file__))
PY

cat <<EOF
Whisper ASR environment is ready.
  python:     $VENV_PYTHON
  default:    medium
  small:      $ASR_MODEL_DIR/small.pt
  medium:     $ASR_MODEL_DIR/medium.pt
  large:      $ASR_MODEL_DIR/large-v3.pt
  commit:     $WHISPER_COMMIT

Runtime downloads remain disabled. Configure a real ALSA capture device before
enabling voice input; this installer does not open the microphone.
EOF
