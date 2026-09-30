"""Deterministic paired fast-backend evaluation utilities."""

import hashlib
import json

import numpy as np

from .fast_env import ACTION_NAMES, CounterfactualCrossingEnv


def digest(value):
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def rollout(policy_name, seed, model=None, options=None, record_trace=False, env_config=None):
    env = CounterfactualCrossingEnv(env_config)
    observation, _ = env.reset(seed=int(seed), options=options)
    trace = []
    total_return = 0.0
    while True:
        mask = env.action_masks()
        if policy_name == "analytic":
            action = env.analytic_action()
        elif policy_name == "masked_ppo":
            if model is None:
                raise ValueError("MASKED_PPO_MODEL_REQUIRED")
            action = model.predict(observation, mask, deterministic=True)
        elif policy_name == "stop_then_analytic":
            action = 1 if env.step_count < 4 and mask[1] else env.analytic_action()
        else:
            raise ValueError("UNKNOWN_POLICY:" + policy_name)
        next_observation, reward, terminated, truncated, info = env.step(action)
        total_return += reward
        if record_trace:
            trace.append(
                {
                    "decision": env.step_count,
                    "action_id": int(action),
                    "action_name": ACTION_NAMES[action],
                    "safe_action_ids": np.flatnonzero(mask).astype(int).tolist(),
                    "reward": float(reward),
                    "ego_x_m": float(info["ego_x_m"]),
                    "ego_y_m": float(info["ego_y_m"]),
                    "actor_coordinate_m": float(info["actor_coordinate_m"]),
                    "minimum_clearance_m": float(info["minimum_clearance_m"]),
                }
            )
        observation = next_observation
        if terminated or truncated:
            break
    result = {
        "policy": policy_name,
        "seed": int(seed),
        "actor_policy": info["actor_policy_evaluator_only"],
        "actor_angle_deg": float(info["actor_angle_deg"]),
        "goal_reached": bool(info["goal_reached"]),
        "contact": bool(info["contact"]),
        "mask_violation_count": int(info["mask_violation_count"]),
        "minimum_clearance_m": float(info["minimum_clearance_m"]),
        "restricted_elapsed_s": float(info["restricted_elapsed_s"]),
        "return": float(total_return),
        "decision_count": int(info["step_count"]),
    }
    if record_trace:
        result["trace"] = trace
        result["trace_sha256"] = digest(trace)
    return result


def evaluate_paired(seeds, model, include_traces=False, env_config=None):
    episodes = []
    for seed in seeds:
        episodes.append(
            rollout("analytic", seed, record_trace=include_traces, env_config=env_config)
        )
        episodes.append(
            rollout(
                "masked_ppo",
                seed,
                model=model,
                record_trace=include_traces,
                env_config=env_config,
            )
        )
    analytic = [item for item in episodes if item["policy"] == "analytic"]
    learned = [item for item in episodes if item["policy"] == "masked_ppo"]
    analytic_time = np.asarray([item["restricted_elapsed_s"] for item in analytic])
    learned_time = np.asarray([item["restricted_elapsed_s"] for item in learned])
    reductions = (analytic_time - learned_time) / analytic_time
    summary = {
        "block_count": len(analytic),
        "episode_count": len(episodes),
        "analytic_goal_rate": float(np.mean([item["goal_reached"] for item in analytic])),
        "learned_goal_rate": float(np.mean([item["goal_reached"] for item in learned])),
        "analytic_mean_restricted_elapsed_s": float(np.mean(analytic_time)),
        "learned_mean_restricted_elapsed_s": float(np.mean(learned_time)),
        "analytic_mean_return": float(np.mean([item["return"] for item in analytic])),
        "learned_mean_return": float(np.mean([item["return"] for item in learned])),
        "paired_relative_time_reduction_mean": float(np.mean(reductions)),
        "positive_block_fraction": float(np.mean(reductions > 0.0)),
        "contact_count": int(sum(item["contact"] for item in episodes)),
        "mask_violation_count": int(sum(item["mask_violation_count"] for item in episodes)),
    }
    return {"summary": summary, "episodes": episodes, "evidence_sha256": digest(episodes)}
