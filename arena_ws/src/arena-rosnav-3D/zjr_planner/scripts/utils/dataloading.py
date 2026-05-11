from __future__ import annotations

import json
from pathlib import Path
from typing   import Dict, List, Tuple

import cv2
import numpy as np
import torch

# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #
MAP_SIZE          = 100       # map is 100 × 100 cells
MAP_RES           = 0.05      # 1 cell = 5 cm
N_BANDS           = 10
SAFE_DIST_M       = 0.30      # metres – obstacle inflation
BOUNDS_WIDTH_M    = 1.00      # inner dead-zone radius around robot
NEAR_PATH_RADIUS  = 25        # pixels – corridor half-width for valid cells
DEVICE_DEFAULT    = torch.device("cpu")
import numpy as np
import torch
from typing import Tuple, Dict, List

# --- Rho config (tune later if needed) ---
RHO_RADIUS_M   = 2.0     # meters (window around robot)
MAP_RES_M      = 0.05    # meters per cell (100x100 @ 5m span is common)
OCC_VALUE      = 100     # your map marks occupied as 100
INCLUDE_UNKNOWN_AS_OCC = False  # set True if you want unknowns to count as obstacles

def compute_rho_from_occ_local_center(occ_map: np.ndarray,
                                      radius_m: float = RHO_RADIUS_M,
                                      res_m: float = MAP_RES_M) -> float:
    """
    Compute crowding rho as the fraction of occupied cells within a circular
    window centered at the robot (assumed map center).
    """
    H, W = occ_map.shape
    cx, cy = H // 2, W // 2                 # robot at grid center (common for local maps)
    r_cells = max(1, int(round(radius_m / res_m)))

    yy, xx = np.ogrid[:H, :W]
    circ = (yy - cx)**2 + (xx - cy)**2 <= r_cells**2

    if INCLUDE_UNKNOWN_AS_OCC:
        # consider both 100 (occupied) and -1 (unknown) as "occupied-like"
        occ_mask = (occ_map == OCC_VALUE) | (occ_map < 0)
    else:
        occ_mask = (occ_map == OCC_VALUE)

    area = int(circ.sum())
    if area <= 0:
        return 0.0

    occ_count = int((occ_mask & circ).sum())
    rho = float(occ_count) / float(area)
    # Clip to [0,1] to be safe
    return float(np.clip(rho, 0.0, 1.0))


def _angle_wrap(a: np.ndarray) -> np.ndarray:
    """Wrap angle to [-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


def compute_rho_front_sector(
    occ_map: np.ndarray,
    start_pt: Dict,
    goal_pt: Dict,
    radius_m: float = RHO_RADIUS_M,
    res_m: float = MAP_RES_M,
    half_angle_deg: float = 60.0,
    occ_value: int = OCC_VALUE,
) -> float:
    """
    Directional crowding: occupied fraction in a circular sector centered at robot,
    pointing to goal direction.

    - robot assumed at map center
    - goal direction approximated by (goal - start) in world coords
    """
    H, W = occ_map.shape
    cx, cy = H // 2, W // 2
    r_cells = max(1, int(round(radius_m / res_m)))

    # goal direction (world) -> use as grid direction
    gx = float(goal_pt["x"]) - float(start_pt["x"])
    gy = float(goal_pt["y"]) - float(start_pt["y"])
    # if start≈goal, fall back to isotropic rho
    if gx * gx + gy * gy < 1e-8:
        return compute_rho_from_occ_local_center(occ_map, radius_m, res_m)

    theta0 = float(np.arctan2(gy, gx))  # goal direction angle

    yy, xx = np.ogrid[:H, :W]
    dy = yy - cx
    dx = xx - cy

    in_circle = (dx * dx + dy * dy) <= (r_cells * r_cells)
    ang = np.arctan2(dy, dx)
    dtheta = _angle_wrap(ang - theta0)
    in_sector = np.abs(dtheta) <= np.deg2rad(half_angle_deg)

    mask = in_circle & in_sector
    area = int(mask.sum())
    if area <= 0:
        return 0.0

    occ_mask = (occ_map == occ_value)
    occ_count = int((occ_mask & mask).sum())
    return float(np.clip(occ_count / area, 0.0, 1.0))


def _rho_squash(rho: float, k: float = 23.0) -> float:
    """Same squashing you already use for rho."""
    rho = float(np.clip(rho, 0.0, 1.0))
    return float((1.0 - np.exp(-k * rho)) / (1.0 - np.exp(-k)))


def compute_teacher_eri(
    tau_scalar: float,
    rho: float,
    rho_front: float = 0.0,
    b_corr: float = 0.0,
    eri_mode: str = "base",
    rho_k: float = 23.0,
) -> float:
    """
    Teacher ERI in [0,1]. Modes:
      - base:      uses tau + rho
      - front:     tau + rho + rho_front
      - corr:      tau + rho + b_corr
      - front_corr:tau + rho + rho_front + b_corr
    """
    tau_scalar = float(np.clip(tau_scalar, 0.0, 1.0))
    rho_s = _rho_squash(rho, k=rho_k)
    rf_s  = _rho_squash(rho_front, k=rho_k)  # same squash for consistency
    bc    = float(np.clip(b_corr, 0.0, 1.0))

    # Keep tau weight constant; split the remaining budget among crowd-related terms.
    w_tau = 0.60
    if eri_mode == "base":
        w_rho, w_rf, w_bc = 0.40, 0.00, 0.00
    elif eri_mode == "front":
        w_rho, w_rf, w_bc = 0.25, 0.15, 0.00
    elif eri_mode == "corr":
        w_rho, w_rf, w_bc = 0.25, 0.00, 0.15
    elif eri_mode == "front_corr":
        w_rho, w_rf, w_bc = 0.20, 0.10, 0.10
    else:
        # fallback to base
        w_rho, w_rf, w_bc = 0.40, 0.00, 0.00

    eri = w_tau * tau_scalar + w_rho * rho_s + w_rf * rf_s + w_bc * bc
    return float(np.clip(eri, 0.0, 1.0))


# --------------------------------------------------------------------------- #
# Rule-based slim pre-processing
# --------------------------------------------------------------------------- #
def preprocess_for_rule(scenario: Dict, device=DEVICE_DEFAULT, eri_mode: str = "base", return_extras: bool = False):
# def preprocess_for_rule(scenario: Dict, device = DEVICE_DEFAULT) -> Tuple[np.ndarray, List[List[List[float]]], Dict, torch.Tensor]:
    """
    Returns the four items the rule API expects:

        band_map         (100,100)  numpy int
        paths_positions  list[3][T][2]  original waypoints
        start_pt         dict {"x":..,"y":..}
        tau              (1,1) torch float   distance_hat ∈ [0,1]
    """
    occ_map   = scenario["occupancy_map"]
    start_pt  = scenario["start_point"]
    goal_pt   = scenario["goal_point"]
    paths     = preprocess_paths(scenario["path_dicts"])     # ensure 3 paths

    # 1) build band map  (reuse helper)
    # _, band_idx, _ = build_valid_mask_and_bands(occ_map, paths, start_pt)
    out = build_valid_mask_and_bands(occ_map, paths, start_pt, return_corridor=True)
    valid_mask_t, band_idx, band_mean_dist, path_corridor = out
    band_map = band_idx.cpu().numpy()        # (100,100) int

    corridor = (path_corridor > 0)
    corridor_area = int(corridor.sum())
    if corridor_area <= 0:
        b_corr = 0.0
    else:
        occ_mask = (occ_map == OCC_VALUE)
        b_corr = float(np.clip((occ_mask & corridor).sum() / corridor_area, 0.0, 1.0))

    rho_front = compute_rho_front_sector(
    occ_map, start_pt, goal_pt,
    radius_m=RHO_RADIUS_M, res_m=MAP_RES_M,
    half_angle_deg=60.0
    )

    extras = {
    "rho_front": float(rho_front),
    "b_corr": float(b_corr),
    }

    # 2) τ  (scaled 0-1, clipped)
    dist = np.linalg.norm([(goal_pt["x"] - start_pt["x"]),
                           (goal_pt["y"] - start_pt["y"])])       # metres
    d_hat = np.clip(dist / 25.0, 0.0, 1.0)
    tau   = torch.tensor([[d_hat]], dtype=torch.float32, device=device)  # (1,1)

    # 3) ρ  (crowding in a local circular window)
    rho = compute_rho_from_occ_local_center(
        occ_map,
        radius_m=RHO_RADIUS_M,
        res_m=MAP_RES_M
    )

    # 4) ERI from [tau, rho]
    #    Start simple: convex combination (monotonic in both inputs), then clip to [0,1].
    ERI_W_TAU = 0.6   # weight on tau  (progress / task difficulty)
    ERI_W_RHO = 0.4   # weight on rho  (local crowding)
    RHO_K = 23.0
    rho_scaled = (1.0 - np.exp(-RHO_K * float(rho))) / (1.0 - np.exp(-RHO_K))
    # tau_scalar = float(tau.reshape(-1)[0].clamp(0, 1).item())
    # eri_rule = float(np.clip(ERI_W_TAU * tau_scalar + ERI_W_RHO * rho_scaled, 0.0, 1.0))
    tau_scalar = float(tau.reshape(-1)[0].clamp(0, 1).item())

    eri_rule = compute_teacher_eri(
        tau_scalar=tau_scalar,
        rho=rho,
        rho_front=rho_front,
        b_corr=b_corr,
        eri_mode=eri_mode,
        rho_k=23.0
    )

    # 5) raw path positions (for overlay)
    paths_pos = [[pt["position"] for pt in p["path"]] for p in paths]

    # Return:
    # - Keep original items in the same order,
    # - plus rho and eri_rule at the end so callers can start using them.
    # return band_map, paths_pos, start_pt, tau, goal_pt, rho, eri_rule
    if return_extras:
        return band_map, paths_pos, start_pt, tau, goal_pt, rho, eri_rule, extras
    return band_map, paths_pos, start_pt, tau, goal_pt, rho, eri_rule



# --------------------------------------------------------------------------- #
# Raw-JSON helpers
# --------------------------------------------------------------------------- #
def load_raw_json(path: str | Path) -> Dict:
    """Load the raw scenario JSON exactly as on disk."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open("r") as f:
        return json.load(f)


def scenario_from_json(raw: Dict) -> Dict:
    """
    Re-shape the odd key pattern of the recording script into a cleaner dict.

    Assumes one start, one goal, and exactly three paths:
        start_point1
        goal_points_1_1
        path_1_1_1 / _2 / _3
        pointcloud1
    """
    occ_map = np.array(raw["pointcloud1"]["grid_map"], dtype=np.uint8)
    start   = raw["start_point1"]
    goal    = raw["goal_points_1_1"]
    paths   = [raw[f"path_1_1_{i}"] for i in (1, 2, 3)]

    return dict(
        occupancy_map = occ_map,
        start_point   = start,
        goal_point    = goal,
        path_dicts    = paths,          # list of 3 path dicts
    )


# --------------------------------------------------------------------------- #
# Path, mask & band helpers
# --------------------------------------------------------------------------- #
def preprocess_paths(paths: List[Dict], target_num: int = 3) -> List[Dict]:
    """
    Ensure exactly `target_num` paths by duplicating or truncating.

    This keeps shapes fixed downstream.
    """
    if len(paths) < target_num:
        paths += [paths[-1]] * (target_num - len(paths))
    elif len(paths) > target_num:
        paths = paths[:target_num]
    return paths


def draw_path_lines(paths   : List[Dict],
                    start_pt: Dict) -> np.ndarray:
    """
    Rasterise all paths into a binary mask - path pixels = 1, else 0.
    """
    mask = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)

    for p_dict in paths:
        pts_map: List[Tuple[int, int]] = [
            world_to_map_coords(
                pt["position"][0], pt["position"][1],
                start_pt["x"],      start_pt["y"],
                MAP_RES)
            for pt in p_dict["path"]
        ]
        for i in range(len(pts_map) - 1):
            cv2.line(mask,
                     pts_map[i], pts_map[i + 1],
                     color=1, thickness=1)

    return mask


def compute_path_dist_map_fast(occupancy_map: np.ndarray,
                               paths        : List[Dict],
                               start_pt     : Dict) -> np.ndarray:
    """
    Return a float32 (100, 100) array - for each free cell, the Euclidean
    distance (metres) to the nearest rasterised global-path pixel.
    """
    # 1) build path pixels = 0 mask
    mask = np.ones_like(occupancy_map, dtype=np.uint8)
    path_pix = draw_path_lines(paths, start_pt)
    mask[path_pix == 1] = 0          # 0 where path

    # 2) distance transform in pixels
    dist_pix = cv2.distanceTransform(mask, cv2.DIST_L2, 5)

    # 3) convert to metres
    return dist_pix.astype(np.float32) * MAP_RES


def build_valid_mask_and_bands(occ_map    : np.ndarray,
                               paths      : List[Dict],
                               start_pt   : Dict,
                               safe_dist_m: float       = SAFE_DIST_M,
                               bounds_m   : float       = BOUNDS_WIDTH_M,
                               device     = DEVICE_DEFAULT,
                               return_corridor: bool = False
                               ) -> Tuple[torch.Tensor, torch.Tensor, np.ndarray]:
    """
    Core logic reused from training:

        • valid_mask      : (100,100) bool
        • band_idx_map    : (100,100) long  (-1 = invalid)
        • band_mean_dist  : (10,)     float np.ndarray
    """
    # --------- 1. path distance map ------------------------------
    dist_map = compute_path_dist_map_fast(occ_map, paths, start_pt)  # metres

    # --------- 2. rule-based masks -------------------------------
    valid_mask = np.ones_like(occ_map, dtype=bool)  # start with all True

    # 2a. boundary dead-zone
    centre_px = MAP_SIZE // 2
    bounds_px = int((MAP_SIZE * MAP_RES / 2 - bounds_m) / MAP_RES)
    lo, hi    = centre_px - bounds_px, centre_px + bounds_px
    valid_mask[lo:hi, lo:hi] = False

    # 2b. obstacle inflation
    if safe_dist_m > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (int(2 * safe_dist_m / MAP_RES + 1),
             int(2 * safe_dist_m / MAP_RES + 1)))
        dilated = cv2.dilate(occ_map.astype(np.uint8), kernel)
        valid_mask[dilated > 0] = False

    # 2c. near-path corridor
    path_corridor = cv2.dilate(
        draw_path_lines(paths, start_pt),
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (2 * NEAR_PATH_RADIUS + 1,
                                   2 * NEAR_PATH_RADIUS + 1)))
    valid_mask[path_corridor == 0] = False

    # --------- 3. band index map & means -------------------------
    band_idx_map = np.full_like(occ_map, fill_value=-1, dtype=np.int64)

    # ignore invalid cells when computing max distance
    dist_valid = dist_map[valid_mask]
    if dist_valid.size == 0:
        max_d = 1e-3  # avoid div-by-zero
    else:
        max_d = dist_valid.max()

    band_width = max_d / N_BANDS if N_BANDS > 0 else max_d

    for b in range(N_BANDS):
        lo, hi = b * band_width, (b + 1) * band_width
        band_cells = (dist_map >= lo) & (dist_map < hi) & valid_mask
        band_idx_map[band_cells] = b

    # mean distance per band
    band_mean_dist = np.zeros(N_BANDS, dtype=np.float32)
    for b in range(N_BANDS):
        d = dist_map[band_idx_map == b]
        band_mean_dist[b] = d.mean() if d.size else 0.0

    # convert to torch
    valid_mask_t   = torch.from_numpy(valid_mask)
    band_idx_map_t = torch.from_numpy(band_idx_map)

    if return_corridor:
        return valid_mask_t, band_idx_map_t, band_mean_dist, path_corridor
    return valid_mask_t, band_idx_map_t, band_mean_dist
    # return valid_mask_t, band_idx_map_t, band_mean_dist


# --------------------------------------------------------------------------- #
# Coords transformation
# --------------------------------------------------------------------------- #
def world_to_map_coords(g_x: float, g_y: float, start_x: float, start_y: float, map_resolution: float = 0.05):
    map_size = 100
    # 计算地图左下角原点（世界坐标）
    origin_x = start_x - (map_size // 2) * map_resolution  # start_x - 2.5 meters
    origin_y = start_y - (map_size // 2) * map_resolution  # start_y - 2.5 meters
    
    # 转换为地图索引（左下角为原点）
    mx = int((g_x - origin_x) / map_resolution)
    my = int((g_y - origin_y) / map_resolution)
    
    return mx, my


def map_to_world_coords(mx: int, my: int, start_x: float, start_y: float, map_resolution: float = 0.05) :
    map_size = 100
    half_size = map_size // 2
    # 计算地图左下角原点（世界坐标）
    origin_x = start_x - half_size * map_resolution  # start_x - 2.5 meters
    origin_y = start_y - half_size * map_resolution  # start_y - 2.5 meters
    
    # 计算实际世界坐标
    g_x = origin_x + mx * map_resolution
    g_y = origin_y + my * map_resolution
    
    return g_x, g_y

# --------------------------------------------------------------------------- #
# 🌟  Quick CLI check (optional)
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    # Rudimentary sanity-run
    json_path = Path(__file__).parent / "data" / "example_data.json"
    raw       = load_raw_json(json_path)
    scenario  = scenario_from_json(raw)
    m4, mean10, dgoal, _, _ = preprocess_for_rule(scenario)
    print("dist_goal:", dgoal.item())
    print("✅ dataloading.py loaded. Has preprocess_for_rule =", 'preprocess_for_rule' in dir())

