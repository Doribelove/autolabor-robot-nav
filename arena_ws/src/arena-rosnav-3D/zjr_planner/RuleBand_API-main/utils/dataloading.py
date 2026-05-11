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


# --------------------------------------------------------------------------- #
# Rule-based slim pre-processing
# --------------------------------------------------------------------------- #
def preprocess_for_rule(scenario: Dict, device = DEVICE_DEFAULT) -> Tuple[np.ndarray, List[List[List[float]]], Dict, torch.Tensor]:
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
    _, band_idx, _ = build_valid_mask_and_bands(occ_map, paths, start_pt)
    band_map = band_idx.cpu().numpy()        # (100,100) int

    # 2) τ  (scaled 0-1, clipped)
    dist = np.linalg.norm([(goal_pt["x"] - start_pt["x"]),
                           (goal_pt["y"] - start_pt["y"])])       # metres
    d_hat = np.clip(dist / 25.0, 0.0, 1.0)
    tau   = torch.tensor([[d_hat]], dtype=torch.float32, device=device)  # (1,1)

    # 3) raw path positions (for overlay)
    paths_pos = [[pt["position"] for pt in p["path"]] for p in paths]

    return band_map, paths_pos, start_pt, tau


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
                               device     = DEVICE_DEFAULT
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

    return valid_mask_t, band_idx_map_t, band_mean_dist


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

