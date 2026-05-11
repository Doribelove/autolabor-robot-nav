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
    从 data/pointcloud/data_start_5.json 和 data/ros_data/paths.json 读取数据，
    插入起始点和 goal（如果传入），并保存为 example_data.json（或 out_path）。

    - start: (start_x, start_y) 起始点坐标 (浮点)
    - goal: 可选，(goal_x, goal_y)。若为 None，则从 paths 文件中提取每个 path 的最后一点作为 goal（优先 path_1_1_1）。
    - out_path: 输出文件路径，None 则默认保存为 ./example_data.json
    返回：写入的文件路径
    """
    if out_path is None:
        out_path = "example_data.json"

    # 获取当前脚本所在目录
    base_dir = os.path.dirname(os.path.abspath(__file__))
    data_start_path = os.path.join(base_dir, "data_json", "data_start_5.json")
    paths_path = os.path.join(base_dir, "data_json", "paths.json")

    # 1) 读取 data_start 文件
    with open(data_start_path, "r", encoding="utf-8") as f:
        data_start = json.load(f)

    # 找到第一个 pointcloud 键
    pc_key = None
    for k in data_start.keys():
        if k.lower().startswith("pointcloud"):
            pc_key = k
            break
    if pc_key is None:
        raise KeyError(f"No key starting with 'pointcloud' found in {data_start_path}")

    pointcloud_obj = data_start[pc_key]

    # 输出结构
    output: Dict[str, Any] = {}
    start_x, start_y = start
    output["start_point1"] = {"x": round(float(start_x), 3), "y": round(float(start_y), 3)}
    output["pointcloud1"] = {"grid_map": pointcloud_obj.get("grid_map", [])}

    # 2) 读取 paths 文件
    with open(paths_path, "r", encoding="utf-8") as f:
        paths = json.load(f)

    # 找 path_* 键
    path_keys = [k for k in paths.keys() if k.startswith("path_")]
    if not path_keys:
        path_keys = list(paths.keys())

    # 处理 goal
    goal_x = goal_y = None
    if goal is not None:
        goal_x, goal_y = float(goal[0]), float(goal[1])
    else:
        preferred = "path_1_1_1" if "path_1_1_1" in paths else (path_keys[0] if path_keys else None)
        if preferred:
            try:
                last_pt_raw = paths[preferred]["path"][-1]["position"]
                xy = _extract_xy_from_position(last_pt_raw)
                if xy:
                    goal_x, goal_y = xy
            except Exception:
                pass
        if goal_x is None or goal_y is None:
            for k in path_keys:
                try:
                    last_pt_raw = paths[k]["path"][-1]["position"]
                    xy = _extract_xy_from_position(last_pt_raw)
                    if xy:
                        goal_x, goal_y = xy
                        break
                except Exception:
                    continue

    if goal_x is None or goal_y is None:
        raise ValueError("无法从 paths 文件中提取 goal 点（最后一点），且调用时未提供 goal 参数。")

    output["goal_points_1_1"] = {"x": round(goal_x, 3), "y": round(goal_y, 3)}

    # 3) 复制路径数据并规范化为 [x, y] 格式
    for key in path_keys:
        entry = paths.get(key, {})
        new_path = []
        for pt in entry.get("path", []):
            pos_raw = pt.get("position") if isinstance(pt, dict) else None
            xy = _extract_xy_from_position(pos_raw)
            if xy is None:
                xy = _extract_xy_from_position(pt)
            if xy:
                x, y = xy
                x = round(float(x), 3)
                y = round(float(y), 3)
                # ⚠️ 保持为 list 格式，兼容 RuleBand_API
                new_path.append({"position": [x, y]})
            else:
                continue

        length = entry.get("length")
        if length is not None:
            try:
                length = round(float(length), 2)
            except Exception:
                length = None

        out_entry: Dict[str, Any] = {"path": new_path}
        if length is not None:
            out_entry["length"] = length
        output[key] = out_entry

    # 4) 保存
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=4, ensure_ascii=False)

    return out_path


if __name__ == "__main__":
    start = (0.921, -5.607)  # 示例起点
    goal = (4.632, -4.474)   # 示例 goal（可以设为 None 自动提取）
    saved = merge_start_and_paths(start, goal, out_path="example_data.json")
    print("Saved merged file to:", saved)
