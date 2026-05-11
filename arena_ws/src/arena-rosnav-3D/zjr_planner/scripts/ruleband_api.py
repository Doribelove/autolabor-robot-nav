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
from torch.distributions import Beta

import torch, numpy as np
from utils.dataloading import (load_raw_json, scenario_from_json, preprocess_for_rule)
from utils.environment_functions import visualize_band_and_subgoal, map_to_world_coords

# ------------------------------------------------------------
N_BANDS = 10
MAX_GOAL_DIST = 15.0          # metres – cap for τ normalisation
# ------------------------------------------------------------

import os, time, torch

import warnings
warnings.filterwarnings(
    "ignore",
    message="CUDA initialization: The NVIDIA driver on your system is too old.*",
    category=UserWarning
)


_eri_nn = None
_eri_nn_mtime = None
_eri_nn_path = None
_last_check_t = 0.0

def _ros_get_param(name, default=None):
    try:
        import rospy
        if rospy.core.is_initialized():
            return rospy.get_param(name, default)
    except Exception:
        pass
    return default

def resolve_save_dir():
    # 1) node private
    p = _ros_get_param("~save_dir", None)
    if p: return os.path.abspath(p)

    # 2) global param (optional but handy)
    p = _ros_get_param("/save_dir", None)
    if p: return os.path.abspath(p)

    # 3) env fallback (optional)
    p = os.environ.get("SDERI_SAVE_DIR", None)
    if p: return os.path.abspath(p)

    # 4) last resort
    return os.path.join(os.path.dirname(__file__), "imilearning_model")

def resolve_runtime_tag():
    tag = _ros_get_param("~runtime_tag", None)
    if tag is None:
        tag = _ros_get_param("/runtime_tag", None)
    if tag is None:
        tag = os.environ.get("SDERI_RUNTIME_TAG", None)
    return str(tag).lower() if tag else "latest"

def resolve_model_path():
    root = resolve_save_dir()
    tag = resolve_runtime_tag()
    if tag == "best":
        return os.path.join(root, "best", "exports", "eri_net_ts.pt")
    return os.path.join(root, "latest", "exports", "eri_net_ts.pt")

def get_eri_nn():
    global _eri_nn, _eri_nn_mtime, _eri_nn_path, _last_check_t

    reload_every = _ros_get_param("~reload_every_sec", None)
    if reload_every is None:
        reload_every = _ros_get_param("/reload_every_sec", 2.0)
    reload_every = float(reload_every)
    now = time.time()
    if now - _last_check_t < reload_every:
        return _eri_nn
    _last_check_t = now

    path = resolve_model_path()
    if not os.path.exists(path):
        return None

    mtime = os.path.getmtime(path)
    if (_eri_nn is None) or (path != _eri_nn_path) or (mtime != _eri_nn_mtime):
        _eri_nn = torch.jit.load(path, map_location="cpu")
        _eri_nn.eval()
        _eri_nn_path = path
        _eri_nn_mtime = mtime
    return _eri_nn


@torch.no_grad()
def student_eri_unified(features_vec, eri_min=0.0, eri_max=10.0):
    """
    Accepts BOTH:
      - old ERINet: forward(x) -> scalar in [0,1]  (sigmoid)
      - new Beta actor: forward(x) -> (alpha>0, beta>0)  OR  Tensor shape (1,2)
    Returns: (eri_value, api_type, aux)
      api_type in {"scalar","beta",None}
      aux = {"alpha":..., "beta":...} if beta, else {}
    """
    m = get_eri_nn()
    if m is None:
        return None, None, {}

    # x: (1,2) = [tau, rho]
    x = torch.tensor([[float(features_vec[0]), float(features_vec[1])]], dtype=torch.float32)

    out = m(x)

    # --- Case A: Beta actor returns a tuple/list of two tensors
    if isinstance(out, (tuple, list)) and len(out) == 2:
        alpha = out[0].reshape(-1)[0].clamp_min(1e-3)
        beta  = out[1].reshape(-1)[0].clamp_min(1e-3)
        e = Beta(alpha, beta).rsample().clamp(1e-6, 1 - 1e-6)  # e in (0,1)
        eri = float(eri_min + (eri_max - eri_min) * e.item())
        return eri, "beta", {"alpha": float(alpha.item()), "beta": float(beta.item())}

    # --- Case B: Beta actor returns a single tensor with two params (e.g., shape (1,2))
    if torch.is_tensor(out) and out.numel() == 2:
        # assume out[...,0] -> alpha_raw, out[...,1] -> beta_raw; apply softplus+1 in TS model ideally
        alpha = out.reshape(-1)[0].clamp_min(1e-3)
        beta  = out.reshape(-1)[1].clamp_min(1e-3)
        e = Beta(alpha, beta).rsample().clamp(1e-6, 1 - 1e-6)
        eri = float(eri_min + (eri_max - eri_min) * e.item())
        return eri, "beta", {"alpha": float(alpha.item()), "beta": float(beta.item())}

    # --- Case C: Old scalar imitation model: out in [0,1] (via Sigmoid)
    if torch.is_tensor(out):
        val = float(out.reshape(-1)[0].item())
        val = max(0.0, min(1.0, val))
        eri = float(eri_min + (eri_max - eri_min) * val)
        return eri, "scalar", {}

    # Unknown output type
    return None, None, {}

def as_xy(pt):
    """Return (x, y) from either {'x':..,'y':..} or [x,y]/(x,y)."""
    if isinstance(pt, dict):
        return float(pt["x"]), float(pt["y"])
    # tuple/list/np.array
    return float(pt[0]), float(pt[1])

def _get_float(priv, glob, default):
    v = _ros_get_param(priv, None)
    if v is None:
        v = _ros_get_param(glob, None)
    if v is None:
        v = default
    return float(v)

def _get_bool(priv, glob, default):
    v = _ros_get_param(priv, None)
    if v is None:
        v = _ros_get_param(glob, None)
    if v is None:
        v = default
    # robust parse
    if isinstance(v, str):
        return v.strip().lower() in ("1","true","yes","y","t")
    return bool(v)


from collections import deque

# rolling timing buffers
_feat_ms_buf  = deque(maxlen=200)
_total_ms_buf = deque(maxlen=200)

def _get_str(priv, glob, default):
    v = _ros_get_param(priv, None)
    if v is None:
        v = _ros_get_param(glob, None)
    if v is None:
        v = default
    return str(v)

def _maybe_ros_log_throttle(msg: str, throttle_sec: float = 2.0):
    try:
        import rospy
        if rospy.core.is_initialized():
            rospy.loginfo_throttle(throttle_sec, msg)
            return
    except Exception:
        pass
    # fallback when not in ROS
    print(msg)

def _pctl(arr, q: float):
    if not arr:
        return None
    a = np.asarray(arr, dtype=np.float32)
    return float(np.percentile(a, q))


def _norm_rho_feature(rho_raw: float) -> float:
    """
    Normalize rho to [0,1] if enabled.
    Uses ROS params:
      ~normalize_rho or /normalize_rho (default: False for backward compatibility)
      ~rho_max or /rho_max (default: 0.03)
    """
    normalize = _get_bool("~normalize_rho", "/normalize_rho", True)
    if not normalize:
        return float(rho_raw)

    rho_max = _get_float("~rho_max", "/rho_max", 0.03)
    denom = max(1e-8, float(rho_max))
    v = float(rho_raw) / denom
    # clamp to [0,1]
    return max(0.0, min(1.0, v))



# ─────────────────────────  RULE  ⟶  P(band)  ──────────────────────────
def rule_probs_temp(eri_rule,
                    N: int = N_BANDS,
                    T_min: float = 0.05,
                    T_max: float = 6.0) -> torch.Tensor:
    """
    tau may be shape (B,) or (B,1) – we squeeze so output is (B,N).
    """
    if not isinstance(eri_rule, torch.Tensor):
        eri_rule = torch.tensor([eri_rule], device='cuda' if torch.cuda.is_available() else 'cpu')
    eri_rule = eri_rule.reshape(-1)                    # <-- squeeze any trailing dims
    idx = torch.arange(N, device=eri_rule.device) # (N,)
    T   = T_min + eri_rule.unsqueeze(-1) * (T_max - T_min)  # (B,1)
    logits = -idx / T                                   # (B,N)
    return torch.softmax(logits, dim=-1)                # (B,N)


# ─────────────────────────  cell sampler (robust)  ──────────────────────────────
from typing import Optional, Tuple
import numpy as np, warnings, random

def _sample_cell_from_band(
    band_map: np.ndarray,
    band_idx: int,
    start_pt: Optional[Tuple[float, float]] = None,
    goal_pt:  Optional[Tuple[float, float]] = None,
) -> Tuple[int, int, int]:
    """
    Stable sampler that never hard-crashes:
      1) sanitize band_map (nan/inf -> invalid)
      2) try target band
      3) widen to neighboring bands
      4) fallback to ANY valid cell
      5) ultimate fallback to goal/start (if given)
    Returns (mx, my, final_band_idx)
    """
    # 0) sanitize
    bm = np.asarray(band_map)
    if bm.ndim != 2:
        raise ValueError("band_map must be HxW")
    # Treat non-finite as invalid
    valid_mask = np.isfinite(bm)
    # Convention: valid cells are those with band index in [0, N_BANDS)
    # If your pipeline uses -1 for invalid, that’s fine; valid_mask will zero them out later.
    try:
        H, W = bm.shape
        N = int(N_BANDS)  # from your module
    except Exception:
        # if N_BANDS isn’t imported here, infer a safe upper bound from data
        N = int(np.nanmax(bm[bm >= 0])) + 1 if np.any(bm >= 0) else 0

    # Helper: pick a random (x,y) from a boolean mask
    def _pick(mask: np.ndarray) -> Optional[Tuple[int, int]]:
        idx = np.argwhere(mask)
        if idx.size == 0:
            return None
        m = idx[random.randrange(len(idx))]
        return int(m[0]), int(m[1])

    # 1) preferred band
    target_mask = (bm == band_idx) & valid_mask
    pick = _pick(target_mask)
    if pick is not None:
        return pick[0], pick[1], int(band_idx)

    # 2) widen to neighboring bands (1,2,3,...)
    for delta in range(1, max(1, N)):
        cand = []
        for alt in (band_idx - delta, band_idx + delta):
            if 0 <= alt < N:
                cand.append((bm == alt))
        if cand:
            widened = np.logical_or.reduce(cand) & valid_mask
            pick = _pick(widened)
            if pick is not None:
                warnings.warn(f"Band {band_idx} empty; used neighbor (±{delta})", RuntimeWarning)
                # choose an alt idx to report (first that matches)
                # (cosmetic: we can keep band_idx to avoid confusing downstream code)
                return pick[0], pick[1], int(band_idx)

    # 3) fallback to ANY valid cell (any non-negative / finite band)
    any_valid = (bm >= 0) & valid_mask
    pick = _pick(any_valid)
    if pick is not None:
        warnings.warn("All bands empty; sampled from any valid cell.", RuntimeWarning)
        return pick[0], pick[1], int(max(0, min(band_idx, N-1))) if N else 0

    # 4) ultimate fallback: goal, then start, then center of map
    if goal_pt is not None:
        gx, gy = as_xy(goal_pt)
        return int(round(gx)), int(round(gy)), int(max(0, min(band_idx, N-1))) if N else 0
    if start_pt is not None:
        sx, sy = as_xy(start_pt)
        return int(round(sx)), int(round(sy)), int(max(0, min(band_idx, N-1))) if N else 0

    # 5) last resort: center
    return H // 2, W // 2, int(max(0, min(band_idx, N-1))) if N else 0


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

    def predict_from_scenario(self, scenario: dict, debug: bool=False) -> Tuple[float, float]:
        # teacher-only experiment controls
        eri_mode    = _get_str("~eri_mode", "/eri_mode", "base").lower()
        teacher_only = _get_bool("~teacher_only", "/teacher_only", True)
        log_timing   = _get_bool("~log_timing", "/log_timing", True)

        t0 = time.perf_counter()

        # Get teacher ERI computed inside preprocess_for_rule (base/front/corr/front_corr)
        out = preprocess_for_rule(scenario, eri_mode=eri_mode, return_extras=True)

        # Backward-safe unpack
        if len(out) == 8:
            band_map, paths_pos, start_pt, tau, goal_pt, rho, eri_rule, extras = out
        else:
            band_map, paths_pos, start_pt, tau, goal_pt, rho, eri_rule = out
            extras = {}

        t1 = time.perf_counter()
        feat_ms = (t1 - t0) * 1000.0

        # Call core predict (teacher_only flag passed down)
        ret = self.predict(
            band_map, paths_pos, start_pt, tau, goal_pt, rho, eri_rule,
            extras=extras, teacher_only=teacher_only, debug=debug
        )

        t2 = time.perf_counter()
        total_ms = (t2 - t0) * 1000.0

        # Timing stats
        if log_timing:
            _feat_ms_buf.append(feat_ms)
            _total_ms_buf.append(total_ms)
            if len(_feat_ms_buf) >= 50:  # don't log too early
                msg = (
                    f"[RuleBandAPI] eri_mode={eri_mode} teacher_only={teacher_only} | "
                    f"feat_ms mean={np.mean(_feat_ms_buf):.3f} p95={_pctl(_feat_ms_buf,95):.3f} | "
                    f"total_ms mean={np.mean(_total_ms_buf):.3f} p95={_pctl(_total_ms_buf,95):.3f} | "
                    f"rho_front={extras.get('rho_front', None)} b_corr={extras.get('b_corr', None)}"
                )
                _maybe_ros_log_throttle(msg, throttle_sec=2.0)

        return ret

    # ---------- core ----------
    @torch.inference_mode()
    def predict(self,
                band_map: np.ndarray,
                paths_pos: List,
                start_pt: dict,
                tau: torch.Tensor,
                goal_pt,
                rho,
                eri_rule,
                extras: dict = None,
                teacher_only: bool = True,
                debug: bool = False):
        """
        dist_goal : (1,1)  torch float
        band_map  : (100,100) int (–1 or 0..9)
        """
        # ---- features for ERI-NN ----
        tau_scalar = float(tau.squeeze().clamp(0, 1).item())
        rho_raw = float(rho)
        rho_feat = _norm_rho_feature(rho_raw)
        features_vec = [tau_scalar, rho_feat]

        # ---- teacher-only control ----
        # For experiments: teacher_only=True will force teacher policy
        if teacher_only:
            use_nn = False
            teacher_p = 1.0
        else:
            teacher_p = _get_float("~teacher_p", "/teacher_p", 0.8)
            use_nn    = _get_bool("~use_nn", "/use_nn", True)

        eri_min = _get_float("~eri_min", "/eri_min", 0.0)
        eri_max = _get_float("~eri_max", "/eri_max", 10.0)

        eri_nn, api_type, aux = (None, None, {})
        if use_nn:
            eri_nn, api_type, aux = student_eri_unified(features_vec, eri_min, eri_max)

        acted_by = "teacher"
        eri_act = float(eri_rule)

        # If not teacher-only, allow student mixing as before
        if (not teacher_only) and (eri_nn is not None) and (random.random() >= teacher_p):
            eri_act = float(eri_nn)
            acted_by = "student"

        # ---- 1) ERI -> band distribution (unchanged helper) ----
        # IMPORTANT: rule_probs_temp expects ERI in [0,1]
        eri_act = float(max(0.0, min(1.0, eri_act)))
        probs = rule_probs_temp(eri_act)
        # probs = rule_probs_temp(eri_act)   # (1,N)
        band_idx = int(torch.multinomial(probs, 1)[0].item())

        # 2. sample band
        band_idx = int(torch.multinomial(probs, 1)[0].item())
        # 3. sample cell & map→world
        # near goal safety net
        NEAR_GOAL_M = 0.5  # tweak: 0.3–1.0 m
        # print(f"goal_pt: {goal_pt}")
        if start_pt is not None and goal_pt is not None:
            gx, gy = as_xy(goal_pt)
            sx, sy = as_xy(start_pt)
            dx = float(gx) - float(sx)
            dy = float(gy) - float(sy)
            if (dx*dx + dy*dy) ** 0.5 < NEAR_GOAL_M:
                # Go straight to goal; bands become degenerate when start≈goal
                wx, wy = float(gx), float(gy)
                band_idx_final = int(max(0, min(band_idx, N_BANDS-1))) if 'band_idx' in locals() else 0
                return wx, wy, band_idx_final

        mx, my, band_idx_final = _sample_cell_from_band(band_map, band_idx)
        sx, sy = as_xy(start_pt)
        wx, wy = map_to_world_coords(mx, my, sx, sy)

        # Return extras for debug: (x, y, eri_like, band_idx)
        if extras is None:
            extras = {}
        return (
            float(wx), float(wy),
            float(eri_rule), int(band_idx_final),
            (float(eri_nn) if eri_nn is not None else None),
            features_vec, acted_by, float(eri_act),
            extras
        )
        # return float(wx), float(wy), float(eri_rule), int(band_idx_final), (float(eri_nn) if eri_nn is not None else None), features_vec, acted_by, float(eri_act)


# ─────────────────────────  CLI DEMO  ──────────────────────────────────
if __name__ == "__main__":
    api = RuleBandAPI()
    x, y, eri_rule, band_index_fianl, features_vec = api.predict_from_file("/home/robot/catkin_arena/src/zjr_planner/scripts/data_json/example_data.json", debug=True)
    print(f"Rule sub-goal: ({x:.3f}, {y:.3f}), band_index_final: {band_index_fianl}, eri_rule: {eri_rule}, features_vec: {features_vec}")
