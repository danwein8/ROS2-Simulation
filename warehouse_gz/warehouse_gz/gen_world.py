import argparse
from pathlib import Path
from typing import List, Set, Tuple

import yaml

try:
    from warehouse_gz.map_utils import (
        grid_cell_to_world_center,
        load_blocked_cells,
        parse_octile_map,
    )
except ImportError:
    from map_utils import grid_cell_to_world_center, load_blocked_cells, parse_octile_map


def merge_blocked_cells(
    blocked_cells: Set[Tuple[int, int]], map_height: int, map_width: int
) -> List[Tuple[int, int, int, int]]:
    """Merge blocked cells into maximal rectangles to reduce Gazebo model count.
    Scans unvisited obstacles horizontally then vertically to greedily make rectangles.

    Returns a list of (row_start, col_start, row_end, col_end) inclusive rectangles.
    """
    grid = [[False] * map_width for _ in range(map_height)]
    for r, c in blocked_cells:
        grid[r][c] = True

    visited = [[False] * map_width for _ in range(map_height)]
    rectangles: List[Tuple[int, int, int, int]] = []

    for row in range(map_height):
        col = 0
        while col < map_width:
            if grid[row][col] and not visited[row][col]:
                # find the end of this horizontal run
                col_end = col
                while col_end + 1 < map_width and grid[row][col_end + 1] and not visited[row][col_end + 1]:
                    col_end += 1

                # extend downward while the full column range is blocked and unvisited
                row_end = row
                while row_end + 1 < map_height:
                    can_extend = True
                    for c in range(col, col_end + 1):
                        if not grid[row_end + 1][c] or visited[row_end + 1][c]:
                            can_extend = False
                            break
                    if not can_extend:
                        break
                    row_end += 1

                # mark all cells in this rectangle as visited
                for r in range(row, row_end + 1):
                    for c in range(col, col_end + 1):
                        visited[r][c] = True

                rectangles.append((row, col, row_end, col_end))
                col = col_end + 1
            else:
                col += 1

    return rectangles

SDF_HEADER = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="warehouse_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
          <!-- simple gray material; we can get fancier if desired -->
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

SDF_FOOTER = """
  </world>
</sdf>
"""

def box_model(name: str, x: float, y: float, z: float, sx: float, sy: float, sz: float) -> str:
    # XML model for the obstacle boxes with collision
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <!-- simple brown material -->
          <material>
            <ambient>0.6 0.3 0.1 1</ambient>
            <diffuse>0.6 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

def task_marker(name: str, x: float, y: float) -> str:
    # visual-only marker: cylinder on the floor
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.05 0 0 0</pose>
      <link name="link">
        <visual name="marker">
          <geometry><cylinder><radius>0.35</radius><length>0.1</length></cylinder></geometry>
          <!-- simple red material for tasks -->
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    with open(args.yaml, "r", encoding="utf-8") as cfg_stream:
        cfg = yaml.safe_load(cfg_stream)
    if cfg is None:
        cfg = {}

    resolution = float(cfg["resolution"])
    if resolution <= 0.0:
        raise ValueError(f"resolution must be > 0 in {args.yaml}, got {resolution}")

    origin = cfg.get("origin")
    if not isinstance(origin, (list, tuple)) or len(origin) != 2:
        raise ValueError(
            f"origin must be a two-element list in {args.yaml}, got {origin!r}"
        )
    origin_xy = (float(origin[0]), float(origin[1]))

    obstacle_height = float(cfg.get("map_obstacle_height", 1.5))
    if obstacle_height <= 0.0:
        raise ValueError(
            f"map_obstacle_height must be > 0 in {args.yaml}, got {obstacle_height}"
        )

    map_path = Path(args.map)
    map_width, map_height, _rows = parse_octile_map(map_path)
    blocked_cells = load_blocked_cells(map_path)
    rectangles = merge_blocked_cells(blocked_cells, map_height, map_width)

    parts = [SDF_HEADER]

    for index, (row_start, col_start, row_end, col_end) in enumerate(rectangles, start=1):
        # compute center of the merged rectangle
        mid_row = (row_start + row_end) / 2.0
        mid_col = (col_start + col_end) / 2.0
        origin_x, origin_y = origin_xy
        cx = origin_x + (mid_col + 0.5) * resolution
        cy = origin_y + (map_height - mid_row - 0.5) * resolution

        n_cols = col_end - col_start + 1
        n_rows = row_end - row_start + 1
        parts.append(
            box_model(
                name=f"map_obs_{index}",
                x=cx,
                y=cy,
                z=0.5 * obstacle_height,
                sx=n_cols * resolution,
                sy=n_rows * resolution,
                sz=obstacle_height,
            )
        )

    # tasks as markers
    for t in cfg.get("tasks", []):
        tid = str(t["id"])
        parts.append(task_marker(f"task_{tid}", float(t["x"]), float(t["y"])))

    parts.append(SDF_FOOTER)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("".join(parts))
    print(
        f"Wrote world: {out_path} "
        f"(map={map_path}, blocked_cells={len(blocked_cells)}, "
        f"merged_models={len(rectangles)})"
    )

if __name__ == "__main__":
    main()
