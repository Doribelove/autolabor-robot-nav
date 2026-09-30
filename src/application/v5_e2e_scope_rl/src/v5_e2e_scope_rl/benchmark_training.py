"""Development-only PCT training aligned with the Gazebo benchmark contract."""

from __future__ import annotations

import json

import numpy as np

from .benchmark_runtime import BenchmarkSensorCandidateEnv
from .privileged_distillation import PrivilegedRankPPOTrainer


class BenchmarkPrivilegedRankSensorEnv(BenchmarkSensorCandidateEnv):
    """Benchmark backend with a candidate-conditioned privileged rank label.

    The label retains every observable analytic-prior term and replaces only
    its observable full-clearance contribution with the simulator-truth
    candidate clearance.  It therefore teaches a residual correction rather
    than a hard-coded scene-side maneuver.  In particular, once an actor has
    passed, the label can return to the faster straight candidate.
    """

    def privileged_rank_scores(self):
        scores = np.full(8, -np.inf, dtype=np.float64)
        for action in range(8):
            if not bool(self._action_mask[action]):
                continue
            observed_clearance = float(self._candidate_features[action, 3]) * 3.0
            oracle_clearance = float(self._oracle_clearance(action))
            scores[action] = (
                float(self._analytic_prior[action])
                - 1.50 * np.clip(observed_clearance, -1.0, 2.0)
                + 1.50 * np.clip(oracle_clearance, -1.0, 2.0)
            )
        return scores

    def privileged_rank_target(self):
        scores = self.privileged_rank_scores()
        if not np.any(np.isfinite(scores)):
            raise RuntimeError("PRIVILEGED_RANK_TARGET_EMPTY_SAFE_SET")
        return int(np.argmax(scores))


class BenchmarkPrivilegedRankPPOTrainer(PrivilegedRankPPOTrainer):
    """Use the benchmark-aligned environment with the existing audited PPO."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.environments = [
            BenchmarkPrivilegedRankSensorEnv(self.environment_config)
            for _ in range(self.config.vector_envs)
        ]
        self.episode_counts[:] = 0
        self.observations = []
        for index, environment in enumerate(self.environments):
            observation, _ = environment.reset(seed=self._episode_seed(index))
            self.observations.append(observation)

    def train(self):
        summary = super().train()
        summary.update(
            {
                "benchmark_distribution_alignment": "pct_teb_dwa_gazebo_development_v2_evaluator_footprint_aligned",
                "privileged_rank_label": "observable_prior_with_oracle_candidate_clearance_substitution",
                "head_on_label": "generic_oracle_clearance_substitution_no_scene_specific_bonus",
                "maximum_episode_decisions": 88,
            }
        )
        path = self.output_directory / "training_summary.json"
        path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return summary
