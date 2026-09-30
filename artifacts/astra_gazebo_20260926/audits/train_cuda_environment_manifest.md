The separate CUDA environment is prepared and verified, with zero training steps, environment steps or optimizer updates. No ROS node or simulation was started. The original CPU .venv RECORD, frozen requirements and activation/build scripts retain their captured SHA256 values.

Activate from the independent workspace with:

```bash
source scripts/astra_train_env.sh
```

This sources the existing independent ROS environment and then activates .venv_train_cuda. It uses Python 3.8.10, Torch 2.4.1+cu121, CUDA runtime 12.1 and cuDNN 9.1. The actual device is NVIDIA GeForce RTX 4060 Laptop GPU, compute capability 8.9, with driver 580.82.09. Torch reports 8,158,576,640 device-memory bytes. The CUDA install matches the [official PyTorch 2.4.1 instructions](https://pytorch.org/get-started/previous-versions/#v241).

The runtime check imports the RL stack and ROS bindings, performs a small CUDA matrix/autograd calculation and cuDNN convolution, and compares against CPU algebra. Maximum gradient error was 7.11e-15. Seeded output and gradient hashes matched across repeated calculations and two separate processes. Peak allocated tensor memory was 67,116,544 bytes. Actual loaded accelerator library paths point to the new environment, apart from the host NVIDIA driver. pip check exits 0.

Initial import verification exposed an existing host compatibility issue: SciPy 1.3.3 and NetworkX 2.4 reference aliases removed from the retained NumPy 1.24.4. That failed check remains saved. Only the new environment received SciPy 1.10.1 and NetworkX 3.1; the final imports pass. Their Python compatibility is documented by [SciPy](https://docs.scipy.org/doc/scipy-1.10.1/release.1.10.0.html) and [NetworkX](https://networkx.org/documentation/networkx-3.1/install.html).

Reproduction on the same ROS host, in a fresh destination rather than over the prepared environment:

```bash
/usr/bin/python3 -m venv --system-site-packages .venv_train_cuda
PYTHONPATH= PYTHONNOUSERSITE=1 PIP_CONFIG_FILE=/dev/null \
  PIP_INDEX_URL=https://pypi.org/simple PIP_EXTRA_INDEX_URL= \
  .venv_train_cuda/bin/python -m pip install pip==24.3.1
PYTHONPATH= PYTHONNOUSERSITE=1 PIP_CONFIG_FILE=/dev/null \
  PIP_INDEX_URL=https://download.pytorch.org/whl/cu121 PIP_EXTRA_INDEX_URL= \
  .venv_train_cuda/bin/python -m pip install --no-deps torch==2.4.1+cu121
PYTHONPATH= PYTHONNOUSERSITE=1 PIP_CONFIG_FILE=/dev/null \
  PIP_INDEX_URL=https://pypi.org/simple PIP_EXTRA_INDEX_URL= \
  .venv_train_cuda/bin/python -m pip install -r requirements/astra-train-cuda-lock.txt
source scripts/astra_train_env.sh
python ../audits/train_cuda_verify.py --output /path/to/new_verification.json
```

The lock captures 48 packages and their installed metadata/RECORD hashes. This environment intentionally inherits host site packages for ROS; ten locked Python dependencies also resolve from the host, with their exact origins and versions recorded. It is separate from the CPU environment but is not fully hermetic. The official Torch index's published wheel SHA256 is cb4f502f910b47e1e366ccf7b231dac2967d2efb47d4b8cb33fc63b4bc5eeed8; this is clearly labelled as the published artifact hash, not a separately re-downloaded wheel. PyPI install reports retain artifact URLs and hashes.

The activation script sets PYTHONHASHSEED=260926 by default (override through ASTRA_TRAIN_SEED) and CUBLAS_WORKSPACE_CONFIG=:4096:8. Python entrypoints still must seed Python/NumPy/Torch, call torch.use_deterministic_algorithms(True), disable cuDNN benchmarking and TF32, and handle any worker RNGs. These flags were applied in verification; shell activation cannot apply them to future Python processes. The [versioned PyTorch reproducibility guide](https://raw.githubusercontent.com/pytorch/pytorch/v2.4.1/docs/source/notes/randomness.rst) explains the distinction and does not guarantee identical results across platforms or releases.

Artifacts:

- workspace/scripts/astra_train_env.sh
- workspace/requirements/astra-train-cuda.txt
- workspace/requirements/astra-train-cuda-lock.txt and .json
- audits/train_cuda_environment_manifest.json
- audits/train_cuda_verification_01.json (retained import failure)
- audits/train_cuda_verification_final_r1.json and train_cuda_verify.py
- audits/train_cuda_torch_provenance.json, train_cuda_rl_install_report.json and train_cuda_compat_install_report.json
- logs/train_cuda_* installation and verification logs

Environment preparation does not grant training authorization or establish learning performance.
