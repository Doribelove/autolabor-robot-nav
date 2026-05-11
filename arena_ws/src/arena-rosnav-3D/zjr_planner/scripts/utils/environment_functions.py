import matplotlib.pyplot as plt
import numpy as np

N_BANDS = 10

def visualize_band_and_subgoal(
    band_map: np.ndarray,
    band_idx: int,
    mx: int,
    my: int,
    title: str = "Band selection debug",
):
    """
    Super-light plot: entire band map coloured by index, with the sampled
    (mx,my) pixel marked by a bright red ✕.

    """

    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111)

    # 0-9 bands → viridis,  -1 (=invalid) → white
    base_cmap = plt.colormaps.get_cmap("viridis")
    cmap = base_cmap.with_extremes(over="white", under="white")

    cmap.set_bad(color="white")
    show_map = np.ma.masked_where(band_map == -1, band_map)

    im = ax.imshow(
        show_map,
        cmap=cmap,
        origin="lower",           # (0,0) bottom-left
        vmin=0,
        vmax=N_BANDS,
    )
    cbar = fig.colorbar(im, ax=ax, ticks=range(N_BANDS))
    cbar.set_label("Band index")

    # Mark the sampled cell
    ax.scatter(my, mx, marker="x", s=120, linewidths=2, color="red", label="Sub-goal")
    ax.set_title(f"{title}  •  chosen band = {band_idx}")
    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


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