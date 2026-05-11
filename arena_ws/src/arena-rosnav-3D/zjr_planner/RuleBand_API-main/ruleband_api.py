"""
rule_api.py
===========
Rule-based sub-goal generator – no neural net, no checkpoint.
Steps
-----
1.  Load raw scenario (JSON) or pre-built tensors via utils.data_loader.
2.  Compute **τ = dist_goal / MAX_DIST**  (auto-normalised per batch).
3.  Convert τ → probability vector over 10 bands with `rule_probs_temp`.
4.  Sample a band, pick a random cell inside, map→world, done.
5.  Optional: `debug=True` shows heat-map + chosen cell.
"""

from __future__ import annotations
from pathlib import Path
import random, warnings
from typing import Tuple, List

import torch, numpy as np

from utils.dataloading import (load_raw_json, scenario_from_json, preprocess_for_rule)
from utils.environment_functions import visualize_band_and_subgoal, map_to_world_coords

# ------------------------------------------------------------
N_BANDS = 10
MAX_GOAL_DIST = 15.0          # metres – cap for τ normalisation
# ------------------------------------------------------------

# ─────────────────────────  RULE  ⟶  P(band)  ──────────────────────────
def rule_probs_temp(tau: torch.Tensor,
                    N: int = N_BANDS,
                    T_min: float = 0.05,
                    T_max: float = 6.0) -> torch.Tensor:
    """
    tau may be shape (B,) or (B,1) – we squeeze so output is (B,N).
    """
    tau = tau.reshape(-1)                    # <-- squeeze any trailing dims
    idx = torch.arange(N, device=tau.device) # (N,)
    T   = T_min + tau.unsqueeze(-1) * (T_max - T_min)  # (B,1)
    logits = -idx / T                                   # (B,N)
    return torch.softmax(logits, dim=-1)                # (B,N)


# ─────────────────────────  cell sampler  ──────────────────────────────
def _sample_cell_from_band(band_map: np.ndarray, band_idx: int) -> Tuple[int, int]:
    cand = np.argwhere(band_map == band_idx)          # (K,2)
    if cand.size == 0:                                # graceful fallback
        for delta in range(1, N_BANDS):
            for alt in (band_idx-delta, band_idx+delta):
                if 0 <= alt < N_BANDS:
                    cand = np.argwhere(band_map == alt)
                    if cand.size:
                        warnings.warn(f"Band {band_idx} empty; using {alt}", RuntimeWarning)
                        band_idx = alt
                        break
            if cand.size:
                break
    if cand.size == 0:
        raise RuntimeError("No valid cells in any band.")
    mx, my = cand[random.randrange(len(cand))]
    return int(mx), int(my), int(band_idx)            # return final band too

# ─────────────────────────  API CLASS  ─────────────────────────────────
class RuleBandAPI:
    """
    Drop-in replacement for neural BandNetAPI but 100 % rule-based.
    """

    def __init__(self, device: str = "cpu"):
        self.device = torch.device(device)

    # ---------- top-level helpers ----------
    def predict_from_file(self, json_path: str | Path,
                          debug: bool=False) -> Tuple[float, float]:
        raw = load_raw_json(json_path)
        scenario = scenario_from_json(raw)
        return self.predict_from_scenario(scenario, debug=debug)

    def predict_from_scenario(self, scenario: dict,
                              debug: bool=False) -> Tuple[float, float]:
        band_map, paths_pos, start_pt, tau = preprocess_for_rule(scenario)
        return self.predict(band_map, paths_pos, start_pt, tau, debug=debug)


    # ---------- core ----------
    @torch.inference_mode()
    def predict(self,
                band_map : np.ndarray,
                paths_pos: List,
                start_pt : dict,
                tau      : torch.Tensor,
                debug: bool=False
            ) -> Tuple[float, float]:
        """
        dist_goal : (1,1)  torch float
        band_map  : (100,100) int (–1 or 0..9)
        """
        # 1. softmax over bands
        probs = rule_probs_temp(tau)                # (1,10)
        # 2. sample band
        band_idx = int(torch.multinomial(probs, 1)[0].item())
        # 3. sample cell & map→world
        mx, my, band_idx_final = _sample_cell_from_band(band_map, band_idx)
        wx, wy = map_to_world_coords(mx, my, start_pt["x"], start_pt["y"])
        # 4. debug viz
        # if debug:
        #     visualize_band_and_subgoal(
        #         band_map,
        #         band_idx_final,
        #         mx,
        #         my,
        #         title=f"Rule • τ={tau.item():.2f} • chosen band={band_idx_final}")
            
        return float(wx), float(wy)

# ─────────────────────────  CLI DEMO  ──────────────────────────────────
if __name__ == "__main__":
    api = RuleBandAPI()
    x, y = api.predict_from_file("data/example_data.json", debug=True)
    print(f"Rule sub-goal: ({x:.3f}, {y:.3f})")
