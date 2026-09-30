#!/usr/bin/env bash
# Source the isolated ROS workspace, then select its separate CUDA Python env.
# This performs no package installation, ROS startup or learning.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this file: source /path/to/workspace/scripts/astra_train_env.sh" >&2
  exit 2
fi

_astra_train_activate() {
  local astra_train_ws
  astra_train_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)" || return
  if [[ ! -f "$astra_train_ws/.venv_train_cuda/bin/activate" ]]; then
    echo "The isolated .venv_train_cuda environment is not prepared." >&2
    return 2
  fi
  source "$astra_train_ws/scripts/astra_env.sh" || return
  source "$astra_train_ws/.venv_train_cuda/bin/activate" || return
  export ASTRA_TRAIN_PYTHON="$astra_train_ws/.venv_train_cuda/bin/python"
  export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
  export CUBLAS_WORKSPACE_CONFIG=:4096:8
  export PYTHONHASHSEED="${ASTRA_TRAIN_SEED:-260926}"
  export CUDA_DEVICE_ORDER=PCI_BUS_ID
  # Training entrypoints must additionally seed Python/NumPy/Torch and explicitly
  # enable deterministic algorithms, disable cuDNN benchmarking/TF32 as reviewed.
}
_astra_train_activate
_astra_train_status=$?
unset -f _astra_train_activate
return "$_astra_train_status"
