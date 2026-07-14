"""Reproducible CPU SAC construction and checkpoint/resume for T09."""

from dataclasses import asdict, dataclass
import hashlib
import os
from pathlib import Path
import platform
import random
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import gymnasium
import numpy as np
import stable_baselines3
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import VecNormalize
import torch
import yaml


class SacTrainingError(RuntimeError):
    pass


@dataclass(frozen=True)
class SacTrainingConfig:
    seed: int = 42
    learning_rate: float = 3.0e-4
    buffer_size: int = 100000
    learning_starts: int = 1000
    batch_size: int = 256
    tau: float = 0.005
    gamma: float = 0.99
    train_freq: int = 1
    gradient_steps: int = 1
    net_arch: Tuple[int, ...] = (256, 256)
    device: str = "cpu"

    def __post_init__(self) -> None:
        if self.device != "cpu":
            raise SacTrainingError("T09 frozen training device must be cpu")
        if self.seed < 0 or self.buffer_size <= 0 or self.learning_starts < 0:
            raise SacTrainingError("invalid SAC seed/buffer settings")
        if self.batch_size <= 0 or self.train_freq <= 0 or self.gradient_steps < 0:
            raise SacTrainingError("invalid SAC update settings")
        if not self.net_arch or any(value <= 0 for value in self.net_arch):
            raise SacTrainingError("net_arch must contain positive layers")


def set_global_seeds(seed: int) -> None:
    os.environ["PYTHONHASHSEED"] = str(seed)
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.use_deterministic_algorithms(True, warn_only=True)
    torch.set_num_threads(1)


def build_sac(env: Any, config: SacTrainingConfig, verbose: int = 0) -> SAC:
    set_global_seeds(config.seed)
    return SAC(
        "MlpPolicy", env,
        learning_rate=config.learning_rate,
        buffer_size=config.buffer_size,
        learning_starts=config.learning_starts,
        batch_size=config.batch_size,
        tau=config.tau,
        gamma=config.gamma,
        train_freq=(config.train_freq, "step"),
        gradient_steps=config.gradient_steps,
        policy_kwargs={"net_arch": list(config.net_arch)},
        seed=config.seed,
        device=config.device,
        verbose=verbose,
    )


def sha256_file(path: Any) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def runtime_versions() -> Dict[str, Any]:
    return {
        "python": platform.python_version(), "torch": str(torch.__version__),
        "torch_cuda_available": bool(torch.cuda.is_available()),
        "gymnasium": str(gymnasium.__version__),
        "stable_baselines3": str(stable_baselines3.__version__),
        "numpy": str(np.__version__), "device": "cpu",
    }


class SacCheckpointManager:
    """Save model, replay buffer, normalization statistics and audited hashes."""

    FILES = ("model.zip", "replay_buffer.pkl", "vecnormalize.pkl")

    ALGORITHMS = ("RL-TEB-Semantic-Eta", "RL-TEB-Direct-Theta")

    def __init__(
        self, directory: Any, algorithm: str = "RL-TEB-Semantic-Eta"
    ) -> None:
        self.directory = Path(directory)
        if algorithm not in self.ALGORITHMS:
            raise SacTrainingError("unsupported SAC checkpoint algorithm")
        self.algorithm = algorithm

    def save(
        self,
        model: SAC,
        vecnormalize: VecNormalize,
        config: SacTrainingConfig,
        metadata: Optional[Mapping[str, Any]] = None,
    ) -> Path:
        self.directory.mkdir(parents=True, exist_ok=True)
        model_tmp = self.directory / "model.tmp"
        model.save(str(model_tmp))
        model_tmp.replace(self.directory / "model.zip")
        replay_tmp = self.directory / "replay_buffer.pkl.tmp"
        model.save_replay_buffer(str(replay_tmp))
        replay_tmp.replace(self.directory / "replay_buffer.pkl")
        vec_tmp = self.directory / "vecnormalize.pkl.tmp"
        vecnormalize.save(str(vec_tmp))
        vec_tmp.replace(self.directory / "vecnormalize.pkl")
        hashes = {name: sha256_file(self.directory / name) for name in self.FILES}
        state = {
            "schema_version": "1.0", "status": "complete",
            "algorithm": self.algorithm, "num_timesteps": int(model.num_timesteps),
            "config": asdict(config), "runtime": runtime_versions(),
            "files": hashes, "metadata": dict(metadata or {}),
        }
        destination = self.directory / "checkpoint_manifest.yaml"
        temporary = destination.with_suffix(".yaml.tmp")
        temporary.write_text(yaml.safe_dump(state, sort_keys=False), encoding="utf-8")
        temporary.replace(destination)
        return destination

    def validate(self) -> Mapping[str, Any]:
        manifest = self.directory / "checkpoint_manifest.yaml"
        try:
            state = yaml.safe_load(manifest.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise SacTrainingError("cannot read checkpoint manifest: {}".format(exc))
        if not isinstance(state, dict) or state.get("status") != "complete":
            raise SacTrainingError("checkpoint manifest is incomplete")
        if state.get("algorithm") != self.algorithm:
            raise SacTrainingError("checkpoint algorithm mismatch")
        for name in self.FILES:
            path = self.directory / name
            if not path.is_file() or sha256_file(path) != state.get("files", {}).get(name):
                raise SacTrainingError("checkpoint hash mismatch: {}".format(name))
        runtime = state.get("runtime", {})
        current = runtime_versions()
        for key in ("python", "torch", "gymnasium", "stable_baselines3", "numpy", "device"):
            if str(runtime.get(key)) != str(current[key]):
                raise SacTrainingError("checkpoint runtime mismatch: {}".format(key))
        if runtime.get("torch_cuda_available") is not False:
            raise SacTrainingError("checkpoint unexpectedly depends on CUDA")
        return state

    def load(self, env: Any) -> Tuple[SAC, VecNormalize, Mapping[str, Any]]:
        state = self.validate()
        vec = VecNormalize.load(str(self.directory / "vecnormalize.pkl"), env)
        vec.training = True
        model = SAC.load(str(self.directory / "model.zip"), env=vec, device="cpu")
        model.load_replay_buffer(str(self.directory / "replay_buffer.pkl"))
        return model, vec, state
