#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE_ROOT="${WORKSPACE_ROOT}/src/application/autolabor_fod_vision"
ENV_DIR="${FOD_YOLO_ENV:-/home/robot/python_env/fod_yolo}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
PYPI_INDEX_URL="${FOD_PYPI_INDEX_URL:-https://mirrors.ustc.edu.cn/pypi/simple}"

if [[ "$("${PYTHON_BIN}" -c 'import sys; print("%d.%d" % sys.version_info[:2])')" != "3.8" ]]; then
  echo "ROS Noetic on this computer expects Python 3.8." >&2
  exit 2
fi

if [[ ! -x "${ENV_DIR}/bin/python3" ]] ||
   ! "${ENV_DIR}/bin/python3" -m pip --version >/dev/null 2>&1 ||
   ! grep -q '^include-system-site-packages = true$' "${ENV_DIR}/pyvenv.cfg" 2>/dev/null; then
  # Reuse the computer's already-tested CUDA PyTorch while isolating all FOD
  # application dependencies from the navigation environment.
  "${PYTHON_BIN}" -m venv --clear --system-site-packages "${ENV_DIR}"
fi

"${ENV_DIR}/bin/python3" -m pip install --upgrade pip==25.0.1 setuptools wheel
"${ENV_DIR}/bin/python3" -c \
  'import torch; assert torch.__version__.startswith("2.4.1")'
"${ENV_DIR}/bin/python3" -m pip install \
  --no-deps torchvision==0.19.1 \
  --index-url https://download.pytorch.org/whl/cu121
"${ENV_DIR}/bin/python3" -m pip install \
  --index-url "${PYPI_INDEX_URL}" \
  --requirement "${PACKAGE_ROOT}/requirements-yolo.txt"
"${ENV_DIR}/bin/python3" -m pip check

source /opt/ros/noetic/setup.bash
if [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/devel/setup.bash"
fi

"${ENV_DIR}/bin/python3" - <<'PY'
import os

import cv2
import rospy
import torch
import torchvision
from cv_bridge import CvBridge
from ultralytics import YOLO

print("torch={}".format(torch.__version__))
print("torchvision={}".format(torchvision.__version__))
print("opencv={}".format(cv2.__version__))
print("cuda_available={}".format(torch.cuda.is_available()))
if torch.cuda.is_available():
    print("cuda_device={}".format(torch.cuda.get_device_name(0)))
elif os.environ.get("FOD_ALLOW_CPU", "0") != "1":
    raise SystemExit(
        "CUDA is unavailable. Set FOD_ALLOW_CPU=1 only for an intentional CPU setup."
    )
print("ROS/cv_bridge/ultralytics imports: OK")
PY

echo "FOD YOLO environment is ready: ${ENV_DIR}"
