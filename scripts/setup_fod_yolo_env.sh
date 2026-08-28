#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE_ROOT="${WORKSPACE_ROOT}/src/application/autolabor_fod_vision"
PRIVATE_SETUP="${PRIVATE_SETUP:-${WORKSPACE_ROOT}/.deps/setup.bash}"
ENV_DIR="${FOD_YOLO_ENV:-${WORKSPACE_ROOT}/.venv/fod_yolo}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
PYPI_INDEX_URL="${FOD_PYPI_INDEX_URL:-https://mirrors.ustc.edu.cn/pypi/simple}"
TORCH_WHEEL="${FOD_TORCH_WHEEL:-${WORKSPACE_ROOT}/.deps/wheels/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl}"
TORCHVISION_WHEEL="${FOD_TORCHVISION_WHEEL:-${WORKSPACE_ROOT}/.deps/wheels/torchvision-0.15.1-cp38-cp38-linux_aarch64.whl}"
ULTRALYTICS_CONFIG="${NVIDIA_FOD_ULTRALYTICS_ROOT:-${WORKSPACE_ROOT}/ultralytics_yolo11_custom}"

if [[ -r "$ULTRALYTICS_CONFIG/ultralytics/__init__.py" ]]; then
  ULTRALYTICS_ROOT="$(readlink -f -- "$ULTRALYTICS_CONFIG")"
elif [[ "${ULTRALYTICS_CONFIG##*/}" == ultralytics &&
        -r "$ULTRALYTICS_CONFIG/__init__.py" ]]; then
  ULTRALYTICS_PACKAGE="$(readlink -f -- "$ULTRALYTICS_CONFIG")"
  ULTRALYTICS_ROOT="${ULTRALYTICS_PACKAGE%/ultralytics}"
else
  echo "Project-local Ultralytics source is missing: $ULTRALYTICS_CONFIG" >&2
  exit 2
fi

if [[ -f "$PRIVATE_SETUP" ]]; then
  source "$PRIVATE_SETUP"
else
  source /opt/ros/noetic/setup.bash
fi

if [[ "$("${PYTHON_BIN}" -c 'import sys; print("%d.%d" % sys.version_info[:2])')" != "3.8" ]]; then
  echo "ROS Noetic on this computer expects Python 3.8." >&2
  exit 2
fi

if [[ ! -x "${ENV_DIR}/bin/python3" ]] ||
   ! "${ENV_DIR}/bin/python3" -m pip --version >/dev/null 2>&1 ||
   ! grep -q '^include-system-site-packages = true$' "${ENV_DIR}/pyvenv.cfg" 2>/dev/null; then
  # ROS Noetic Python modules remain visible while FOD application packages
  # stay isolated from the navigation environment.
  "${PYTHON_BIN}" -m venv --clear --system-site-packages "${ENV_DIR}"
fi

"${ENV_DIR}/bin/python3" -m pip install --upgrade pip==25.0.1 setuptools wheel
if ! "${ENV_DIR}/bin/python3" - <<'PY' >/dev/null 2>&1
import torch
import torchvision

assert torch.__version__.startswith("2.0.0+nv23.05")
assert torchvision.__version__.startswith("0.15.1")
PY
then
  for wheel in "$TORCH_WHEEL" "$TORCHVISION_WHEEL"; do
    if [[ ! -f "$wheel" ]]; then
      echo "Missing Jetson ARM64 wheel: $wheel" >&2
      exit 3
    fi
  done
  "${ENV_DIR}/bin/python3" -m pip install \
    --no-deps --force-reinstall \
    "$TORCH_WHEEL" "$TORCHVISION_WHEEL"
fi
"${ENV_DIR}/bin/python3" -m pip install \
  --index-url "${PYPI_INDEX_URL}" \
  --requirement "${PACKAGE_ROOT}/requirements-yolo.txt"
"${ENV_DIR}/bin/python3" -m pip check

if [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/devel/setup.bash"
fi

export AUTOLABOR_FOD_ULTRALYTICS_ROOT="$ULTRALYTICS_ROOT"
export PYTHONPATH="$ULTRALYTICS_ROOT${PYTHONPATH:+:$PYTHONPATH}"

"${ENV_DIR}/bin/python3" - <<'PY'
import os
from pathlib import Path

import cv2
import rospy
import torch
import torchvision
from cv_bridge import CvBridge
import ultralytics
from ultralytics import YOLO
from ultralytics.nn.modules import GAM_Attention

expected = Path(os.environ["AUTOLABOR_FOD_ULTRALYTICS_ROOT"]).resolve() / "ultralytics"
actual = Path(ultralytics.__file__).resolve()
try:
    actual.relative_to(expected)
except ValueError as error:
    raise SystemExit(
        "Ultralytics import conflict: {} is outside {}".format(actual, expected)
    ) from error

print("torch={}".format(torch.__version__))
print("torchvision={}".format(torchvision.__version__))
print("opencv={}".format(cv2.__version__))
print("ultralytics={} ({})".format(ultralytics.__version__, actual))
print("custom_gam={}".format(GAM_Attention.__name__))
print("cuda_available={}".format(torch.cuda.is_available()))
if torch.cuda.is_available():
    print("cuda_device={}".format(torch.cuda.get_device_name(0)))
elif os.environ.get("FOD_ALLOW_CPU", "0") != "1":
    raise SystemExit(
        "CUDA is unavailable. Set FOD_ALLOW_CPU=1 only for an intentional CPU setup."
    )
print("ROS/cv_bridge/project-local-ultralytics imports: OK")
PY

echo "FOD YOLO environment is ready: ${ENV_DIR}"
