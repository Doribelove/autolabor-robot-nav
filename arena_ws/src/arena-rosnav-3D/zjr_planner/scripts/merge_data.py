# merge_data.py
import json
import os
from typing import Optional, Tuple, Dict, Any

def _extract_xy_from_position(pos) -> Optional[Tuple[float, float]]:
    """
    辅助：支持从不同格式的 position 中提取 (x,y)
    - pos 可能是 [x, y] 或 {"x": x, "y": y}
    返回 (x,y) 或 None
    """
    if pos is None:
        return None
    try:
        # list / tuple 风格
        if isinstance(pos, (list, tuple)) and len(pos) >= 2:
            return float(pos[0]), float(pos[1])
        # dict 风格
        if isinstance(pos, dict):
            if "x" in pos and "y" in pos:
                return float(pos["x"]), float(pos["y"])
    except Exception:
        return None
    return None

def merge_start_and_paths(start: Tuple[float, float],
                          goal: Optional[Tuple[float, float]] = None,
                          out_path: Optional[str] = None) -> str:
    """
    Read pointcloud + paths from the correct data_json folder, insert start & goal,
    and write merged JSON to out_path. Cleans up the source JSONs afterward.

    Robust to:
    - data_json being under RuleBand_API-main/data_json OR scripts/data_json
    - data_start_* index changing per run (picks the newest)
    """
    import glob
    import time

    # 0) Choose output path
    if out_path is None:
        out_path = "example_data.json"

    # 1) Locate the data_json directory (prefer RuleBand_API-main/data_json)
    here = os.path.dirname(os.path.abspath(__file__))  # .../zjr_planner/scripts
    candidates = [
        os.path.abspath(os.path.join(here, "..", "RuleBand_API-main", "data_json")),
        os.path.abspath(os.path.join(here, "data_json")),
    ]
    data_dir = None
    for c in candidates:
        if os.path.isdir(c):
            # must contain paths.json or at least one data_start_*.json
            has_paths = os.path.isfile(os.path.join(c, "paths.json"))
            has_start = bool(glob.glob(os.path.join(c, "data_start_*.json")))
            if has_paths or has_start:
                data_dir = c
                break
    if data_dir is None:
        raise FileNotFoundError(
            f"Cannot locate data_json folder. Tried: {candidates}"
        )

    # 2) Resolve files:
    #    - latest data_start_*.json (sorted by mtime)
    #    - paths.json (must exist)
    start_candidates = glob.glob(os.path.join(data_dir, "data_start_*.json"))
    if not start_candidates:
        raise FileNotFoundError(
            f"No data_start_*.json found in {data_dir}. "
            f"Ensure the pointcloud step ran and wrote there."
        )
    # pick newest by mtime
    data_start_path = max(start_candidates, key=os.path.getmtime)

    paths_path = os.path.join(data_dir, "paths.json")
    if not os.path.isfile(paths_path):
        raise FileNotFoundError(
            f"paths.json not found in {data_dir}. Ensure the RRT step wrote it."
        )

    # 3) Load start (pointcloud) JSON
    with open(data_start_path, "r", encoding="utf-8") as f:
        data_start = json.load(f)

    # find first key that starts with 'pointcloud'
    pc_key = next((k for k in data_start.keys() if k.lower().startswith("pointcloud")), None)
    if pc_key is None:
        raise KeyError(f"No key starting with 'pointcloud' in {data_start_path}")
    pointcloud_obj = data_start[pc_key]

    # 4) Build base output with start + grid
    start_x, start_y = start
    output: Dict[str, Any] = {
        "start_point1": {"x": round(float(start_x), 3), "y": round(float(start_y), 3)},
        "pointcloud1": {"grid_map": pointcloud_obj.get("grid_map", [])},
    }

    # 5) Load paths.json
    with open(paths_path, "r", encoding="utf-8") as f:
        paths = json.load(f)

    # collect path_* keys or fallback to all keys
    path_keys = [k for k in paths.keys() if k.startswith("path_")]
    if not path_keys:
        path_keys = list(paths.keys())

    # 6) Determine goal (explicit arg has priority; else last point of preferred path)
    goal_x = goal_y = None
    if goal is not None:
        goal_x, goal_y = float(goal[0]), float(goal[1])
    else:
        preferred = "path_1_1_1" if "path_1_1_1" in paths else (path_keys[0] if path_keys else None)
        candidates_for_goal = ([preferred] if preferred else []) + [k for k in path_keys if k != preferred]
        for k in candidates_for_goal:
            try:
                last_pt_raw = paths[k]["path"][-1]["position"]
                xy = _extract_xy_from_position(last_pt_raw)
                if xy:
                    goal_x, goal_y = xy
                    break
            except Exception:
                continue
    if goal_x is None or goal_y is None:
        raise ValueError("Unable to infer goal from paths.json and no goal was provided.")

    output["goal_points_1_1"] = {"x": round(float(goal_x), 3), "y": round(float(goal_y), 3)}

    # 7) Normalize & copy paths, then emit exactly path_1_1_1..3 (pad if needed)
    # collect path_* keys only
    raw_keys = sorted([k for k in paths.keys() if k.startswith("path_")])

    # normalize each existing path into our canonical list of dicts
    normalized = []
    for k in raw_keys:
        entry = paths.get(k, {})
        new_path = []
        for pt in entry.get("path", []):
            pos_raw = pt.get("position") if isinstance(pt, dict) else None
            xy = _extract_xy_from_position(pos_raw) or _extract_xy_from_position(pt)
            if xy:
                x, y = float(xy[0]), float(xy[1])
                new_path.append({"position": [round(x, 3), round(y, 3)]})
        # carry over length if present
        length = entry.get("length")
        try:
            length = round(float(length), 2) if length is not None else None
        except Exception:
            length = None
        out_entry: Dict[str, Any] = {"path": new_path}
        if length is not None:
            out_entry["length"] = length
        normalized.append(out_entry)

    # if RRT produced fewer than 3 paths, pad by duplicating the last available
    if not normalized:
        # no paths at all -> create a tiny placeholder from start->goal so the loader won't crash
        sx, sy = output["start_point1"]["x"], output["start_point1"]["y"]
        gx, gy = output["goal_points_1_1"]["x"], output["goal_points_1_1"]["y"]
        placeholder = {"path": [{"position": [sx, sy]}, {"position": [gx, gy]}]}
        normalized = [placeholder]
    while len(normalized) < 3:
        normalized.append(normalized[-1])

    # rename first three into the exact keys dataloading expects
    output["path_1_1_1"] = normalized[0]
    output["path_1_1_2"] = normalized[1]
    output["path_1_1_3"] = normalized[2]


    # 8) Save merged result
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=4, ensure_ascii=False)
    # print(f"[merge_start_and_paths] Saved merged file → {out_path}")

    # 9) Cleanup temp files (best-effort)
    for file_path in (data_start_path, paths_path):
        try:
            if os.path.exists(file_path):
                os.remove(file_path)
                # print(f"[merge_start_and_paths] Deleted temporary file: {file_path}")
        except Exception as e:
            print(f"[merge_start_and_paths] Warning: failed to delete {file_path}: {e}")

    return out_path
