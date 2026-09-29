#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE_ROOT="${WORKSPACE_ROOT}/src/application/autolabor_fod_vision"
ENV_DIR="${FOD_YOLO_ENV:-$HOME/python_env/fod_yolo}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
PYPI_INDEX_URL="${FOD_PYPI_INDEX_URL:-https://pypi.org/simple}"
# Ignore unrelated ~/.local packages, including possible CPU-only torch wheels.
export PYTHONNOUSERSITE=1

if [[ "$("${PYTHON_BIN}" -c 'import sys; print("%d.%d" % sys.version_info[:2])')" != "3.8" ]]; then
  echo "ROS Noetic on this computer expects Python 3.8." >&2
  exit 2
fi
if [[ ! -e "${ENV_DIR}" ]]; then
  "${PYTHON_BIN}" -m venv --system-site-packages "${ENV_DIR}"
elif [[ ! -x "${ENV_DIR}/bin/python3" ]] ||
     ! grep -q '^include-system-site-packages = true$' "${ENV_DIR}/pyvenv.cfg"; then
  echo "Existing environment is not a system-site-packages venv: ${ENV_DIR}" >&2
  echo "Select a new FOD_YOLO_ENV; existing files have been preserved." >&2
  exit 2
fi
PYTHON="${ENV_DIR}/bin/python3"
"${PYTHON}" -m pip install --index-url "${PYPI_INDEX_URL}" pip==25.0.1 setuptools==75.3.4 wheel==0.45.1

if ! "${PYTHON}" - <<'PYTORCH'
import torch, torchvision
assert torch.cuda.is_available()
boxes = torch.tensor([[0., 0., 10., 10.]], device="cuda")
assert torchvision.ops.nms(boxes, torch.ones(1, device="cuda"), .5).is_cuda
print("Reusing CUDA torch", torch.__version__, "torchvision", torchvision.__version__)
PYTORCH
then
  if [[ "$(uname -m)" == aarch64 ]]; then
    # NVIDIA's 23.05 wheel targets JetPack 5.1.x, Python 3.8 and CUDA 11.4.
    if ! grep -q '^# R35 ' /etc/nv_tegra_release; then
      echo "This ARM bootstrap targets JetPack 5.1.x / L4T 35. Select wheels for this host's JetPack." >&2
      exit 2
    fi
    TORCH_WHEEL="${FOD_TORCH_WHEEL:-https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl}"
    "${PYTHON}" -m pip install --no-deps "${TORCH_WHEEL}"
    if [[ -n "${FOD_TORCHVISION_WHEEL:-}" ]]; then
      "${PYTHON}" -m pip install --no-deps "${FOD_TORCHVISION_WHEEL}"
    else
      TV_SOURCE="${XDG_CACHE_HOME:-$HOME/.cache}/autolabor-torchvision-0.15.1"
      if [[ ! -d "${TV_SOURCE}" ]]; then
        git clone --depth 1 --branch v0.15.1 https://github.com/pytorch/vision.git "${TV_SOURCE}"
      fi
      "${PYTHON}" -m pip install --index-url "${PYPI_INDEX_URL}" numpy==1.24.4 pillow typing-extensions sympy networkx jinja2 filelock
      CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}" FORCE_CUDA=1 \
        TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.7}" MAX_JOBS="${BUILD_JOBS:-4}" \
        BUILD_VERSION=0.15.1 "${PYTHON}" -m pip install --no-deps --no-build-isolation "${TV_SOURCE}"
    fi
  else
    "${PYTHON}" -m pip install torch==2.4.1 torchvision==0.19.1 \
      --index-url https://download.pytorch.org/whl/cu121
  fi
fi

# Pin the actual CUDA builds so ultralytics cannot replace them with CPU wheels.
"${PYTHON}" - <<'PYCONSTRAINTS' > "${ENV_DIR}/cuda-constraints.txt"
from importlib.metadata import version
for package in ("torch", "torchvision"):
    print("{}=={}".format(package, version(package)))
PYCONSTRAINTS
"${PYTHON}" -m pip install --index-url "${PYPI_INDEX_URL}" \
  --constraint "${ENV_DIR}/cuda-constraints.txt" \
  --requirement "${PACKAGE_ROOT}/requirements-yolo.txt"
# JetPack exposes graphsurgeon through system-site-packages but may omit ONNX.
if "${PYTHON}" -c 'from importlib.metadata import version; version("onnx-graphsurgeon")' 2>/dev/null; then
  "${PYTHON}" -m pip install --index-url "${PYPI_INDEX_URL}" \
    --constraint "${ENV_DIR}/cuda-constraints.txt" onnx==1.15.0 protobuf==3.20.3
fi
"${PYTHON}" -m pip check

source /opt/ros/noetic/setup.bash
if [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/devel/setup.bash"
fi
"${PYTHON}" - <<'PYVERIFY'
import cv2, rospy, torch, torchvision, ultralytics
from cv_bridge import CvBridge
print("torch={} torchvision={} opencv={} ultralytics={}".format(
    torch.__version__, torchvision.__version__, cv2.__version__, ultralytics.__version__))
assert torch.cuda.is_available(), "CUDA is required for this environment"
boxes = torch.tensor([[0., 0., 10., 10.]], device="cuda")
assert torchvision.ops.nms(boxes, torch.ones(1, device="cuda"), .5).is_cuda
print("CUDA device={}, CUDA NMS and ROS/cv_bridge imports: OK".format(torch.cuda.get_device_name(0)))
PYVERIFY

echo "FOD YOLO environment is ready: ${ENV_DIR}"
