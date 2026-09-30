"""Learning-value audit for the V4 sequential candidate environment."""

import numpy as np

from .fast_env import ACTION_NAMES, CounterfactualCrossingEnv


def _finish_analytic(env):
    while True:
        _, _, terminated, truncated, info = env.step(env.analytic_action())
        if terminated or truncated:
            return info


def audit_response_value(seeds, env_config):
    blocks = []
    state_count = 0
    multiaction_state_count = 0
    for seed in seeds:
        env = CounterfactualCrossingEnv(env_config)
        env.reset(seed=int(seed))
        while not (
            env.actor_coordinate >= -1.2
            or env.action_masks()[4]
            or env.action_masks()[5]
        ):
            state_count += 1
            multiaction_state_count += int(np.sum(env.action_masks()) > 1)
            _, _, terminated, truncated, _ = env.step(env.analytic_action())
            if terminated or truncated:
                break
        state_count += 1
        multiaction_state_count += int(np.sum(env.action_masks()) > 1)
        boundary = env.clone()
        analytic_action = boundary.analytic_action()
        alternatives = []
        for action in range(len(ACTION_NAMES)):
            trial = boundary.clone()
            if not bool(trial.action_masks()[action]):
                alternatives.append(
                    {
                        "action_id": action,
                        "action_name": ACTION_NAMES[action],
                        "accepted": False,
                        "restricted_elapsed_s": None,
                    }
                )
                continue
            _, _, terminated, truncated, info = trial.step(action)
            if not (terminated or truncated):
                info = _finish_analytic(trial)
            alternatives.append(
                {
                    "action_id": action,
                    "action_name": ACTION_NAMES[action],
                    "accepted": True,
                    "restricted_elapsed_s": float(info["restricted_elapsed_s"]),
                }
            )
        analytic_time = alternatives[analytic_action]["restricted_elapsed_s"]
        best_time = min(
            item["restricted_elapsed_s"]
            for item in alternatives
            if item["accepted"]
        )
        reduction = (analytic_time - best_time) / analytic_time
        blocks.append(
            {
                "seed": int(seed),
                "actor_policy_evaluator_only": boundary.actor_policy,
                "actor_angle_deg": float(boundary.actor_angle_deg),
                "boundary_decision": int(boundary.step_count),
                "boundary_actor_coordinate_m": float(boundary.actor_coordinate),
                "analytic_action_id": int(analytic_action),
                "analytic_restricted_elapsed_s": float(analytic_time),
                "best_counterfactual_restricted_elapsed_s": float(best_time),
                "relative_time_regret": float(reduction),
                "alternatives": alternatives,
            }
        )
    regrets = np.asarray([item["relative_time_regret"] for item in blocks])
    summary = {
        "block_count": len(blocks),
        "counterfactual_candidate_episode_count": len(blocks) * len(ACTION_NAMES),
        "analytic_prefix_state_count": state_count,
        "safe_multiaction_state_fraction": multiaction_state_count / float(state_count),
        "positive_sequential_regret_block_fraction": float(np.mean(regrets > 1.0e-12)),
        "mean_relative_time_regret": float(np.mean(regrets)),
        "maximum_relative_time_regret": float(np.max(regrets)),
    }
    summary["safe_multiaction_gate_pass"] = summary["safe_multiaction_state_fraction"] >= 0.20
    summary["sequential_regret_gate_pass"] = summary["positive_sequential_regret_block_fraction"] >= 0.05
    summary["audit_gate_pass"] = bool(
        summary["safe_multiaction_gate_pass"]
        and summary["sequential_regret_gate_pass"]
    )
    return {"summary": summary, "blocks": blocks}
