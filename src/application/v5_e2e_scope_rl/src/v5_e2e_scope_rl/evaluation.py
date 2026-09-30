"""Deterministic, failure-inclusive evaluation utilities for V5."""

import time
from typing import Dict, Iterable, Mapping, Optional

import numpy as np
import torch

from .asymmetric_ppo import SensorActor, observations_to_torch, stack_observations
from .sensor_env import FAMILY_NAMES, SensorCandidateEnv


def run_episode(
    seed: int,
    family: str,
    method: str,
    actor: Optional[SensorActor] = None,
    advantage_gate_margin: Optional[float] = None,
    no_scan_history: bool = False,
    device: str = "cpu",
) -> Dict[str, object]:
    environment = SensorCandidateEnv({"family": family, "no_scan_history": no_scan_history})
    observation, _ = environment.reset(seed=int(seed))
    intervention_count = 0
    learned_difference_count = 0
    inference_latencies_ms = []
    while True:
        if method == "sensor_analytic_prior":
            action = environment.analytic_action()
        else:
            if actor is None:
                raise ValueError("LEARNED_METHOD_REQUIRES_ACTOR")
            batch = observations_to_torch(
                stack_observations([observation]), torch.device(device)
            )
            started = time.perf_counter_ns()
            with torch.no_grad():
                actions, metadata = actor.choose_actions(
                    batch,
                    deterministic=True,
                    advantage_gate_margin=advantage_gate_margin,
                )
            inference_latencies_ms.append((time.perf_counter_ns() - started) / 1.0e6)
            action = int(actions[0].cpu())
            intervention_count += int(metadata["intervened"][0].cpu())
            learned_difference_count += int(
                metadata["learned_action"][0].cpu()
                != metadata["analytic_action"][0].cpu()
            )
        observation, _, terminated, truncated, info = environment.step(action)
        if terminated or truncated:
            break
    maximum_time_s = environment.max_steps * environment.dt
    return {
        "seed": int(seed),
        "family": family,
        "method": method,
        "goal_reached": bool(info["goal_reached"]),
        "contact_count": int(info["contact_count"]),
        "mask_violation_count": int(info["mask_violation_count"]),
        "activation_revalidation_failure_count": int(
            info["activation_revalidation_failure_count"]
        ),
        "elapsed_s": float(info["restricted_elapsed_s"]),
        "failure_inclusive_restricted_time_s": float(
            info["restricted_elapsed_s"] if info["goal_reached"] else maximum_time_s
        ),
        "minimum_physical_clearance_m": float(info["minimum_physical_clearance_m"]),
        "decision_count": int(info["step_count"]),
        "learned_difference_count": int(learned_difference_count),
        "advantage_gate_intervention_count": int(intervention_count),
        "mean_inference_latency_ms": float(
            np.mean(inference_latencies_ms) if inference_latencies_ms else 0.0
        ),
        "p99_inference_latency_ms": float(
            np.percentile(inference_latencies_ms, 99.0) if inference_latencies_ms else 0.0
        ),
        "actor_used_privileged_observation": False,
        "no_scan_history": bool(no_scan_history),
    }


def evaluate_method(
    seeds: Iterable[int],
    method: str,
    actor: Optional[SensorActor] = None,
    advantage_gate_margin: Optional[float] = None,
    no_scan_history: bool = False,
    device: str = "cpu",
):
    rows = []
    for block_index, seed in enumerate(seeds):
        family = FAMILY_NAMES[block_index % len(FAMILY_NAMES)]
        row = run_episode(
            seed=seed,
            family=family,
            method=method,
            actor=actor,
            advantage_gate_margin=advantage_gate_margin,
            no_scan_history=no_scan_history,
            device=device,
        )
        row["block_index"] = int(block_index)
        rows.append(row)
    return rows


def summarize_rows(rows: Iterable[Mapping[str, object]]) -> Dict[str, object]:
    rows = list(rows)
    return {
        "episodes": len(rows),
        "goal_count": int(sum(row["goal_reached"] for row in rows)),
        "contact_count": int(sum(row["contact_count"] for row in rows)),
        "mask_violation_count": int(sum(row["mask_violation_count"] for row in rows)),
        "mean_failure_inclusive_restricted_time_s": float(
            np.mean([row["failure_inclusive_restricted_time_s"] for row in rows])
        ),
        "minimum_physical_clearance_m": float(
            min(row["minimum_physical_clearance_m"] for row in rows)
        ),
        "mean_inference_latency_ms": float(
            np.mean([row["mean_inference_latency_ms"] for row in rows])
        ),
        "advantage_gate_intervention_count": int(
            sum(row["advantage_gate_intervention_count"] for row in rows)
        ),
    }

