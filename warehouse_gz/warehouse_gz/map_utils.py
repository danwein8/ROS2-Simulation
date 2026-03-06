"""Helpers for reading octile map files and converting map/world coordinates."""

import math
from pathlib import Path
from typing import Iterable, Optional, Set, Tuple

FREE_TERRAIN_CHARS = frozenset({".", "G", "S"})


def _require_positive(value: float, label: str) -> None:
    if value <= 0.0:
        raise ValueError(f"{label} must be > 0, got {value}")


def parse_octile_map(path: Path) -> Tuple[int, int, Tuple[str, ...]]:
    """Parse an octile .map file and return (width, height, map_rows)."""
    # Get the map file path and check it exists before reading
    map_path = Path(path)
    if not map_path.is_file():
        raise FileNotFoundError(f"Map file not found: {map_path}")

    # Read in lines and make sure there is more than just the 4 line header
    lines = map_path.read_text(encoding="utf-8").splitlines()
    if len(lines) < 4:
        raise ValueError(f"Malformed map file {map_path}: missing required header")

    # Make sure map is type octile
    if lines[0].strip() != "type octile":
        raise ValueError(f"Malformed map file {map_path}: first line must be 'type octile'")

    # Make sure map matches header format
    try:
        height_key, height_value = lines[1].split(maxsplit=1)
        width_key, width_value = lines[2].split(maxsplit=1)
    except ValueError as exc:
        raise ValueError(
            f"Malformed map file {map_path}: expected 'height <n>' and 'width <n>'"
        ) from exc

    # Make sure map matches header format
    if height_key != "height" or width_key != "width":
        raise ValueError(
            f"Malformed map file {map_path}: expected 'height <n>' then 'width <n>'"
        )

    # Height and Width values must be positive ints
    try:
        height = int(height_value)
        width = int(width_value)
    except ValueError as exc:
        raise ValueError(
            f"Malformed map file {map_path}: width and height must be integers"
        ) from exc

    if height <= 0 or width <= 0:
        raise ValueError(
            f"Malformed map file {map_path}: width and height must be positive"
        )

    if lines[3].strip() != "map":
        raise ValueError(f"Malformed map file {map_path}: fourth line must be 'map'")

    rows = tuple(lines[4:])
    if len(rows) != height:
        raise ValueError(
            f"Malformed map file {map_path}: expected {height} map rows, found {len(rows)}"
        )

    for index, row in enumerate(rows):
        if len(row) != width:
            raise ValueError(
                f"Malformed map file {map_path}: row {index} has width {len(row)}, "
                f"expected {width}"
            )

    return width, height, rows


def is_blocked_char(cell: str) -> bool:
    """Return True if a map cell should be treated as an obstacle."""
    if len(cell) != 1:
        raise ValueError(f"Expected single map character, got {cell!r}")
    return cell not in FREE_TERRAIN_CHARS


def load_blocked_cells(path: Path) -> Set[Tuple[int, int]]:
    """Load blocked cells as (row, col) from an octile map file."""
    _width, _height, rows = parse_octile_map(path)
    blocked: Set[Tuple[int, int]] = set()
    for row_index, row in enumerate(rows):
        for col_index, value in enumerate(row):
            if is_blocked_char(value):
                blocked.add((row_index, col_index))
    return blocked


def grid_cell_to_world_center(
    row: int,
    col: int,
    origin_xy: Iterable[float],
    resolution: float,
    map_height: int,
) -> Tuple[float, float]:
    """Convert a grid cell index (row, col) to world coordinates (x, y)."""
    _require_positive(resolution, "resolution")
    origin_x, origin_y = tuple(origin_xy)
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (map_height - row - 0.5) * resolution
    return x, y


def world_to_grid_cell(
    x: float,
    y: float,
    origin_xy: Iterable[float],
    resolution: float,
    map_height: int,
    map_width: int,
) -> Optional[Tuple[int, int]]:
    """Convert world coordinates to grid indices. Return None when out of bounds."""
    _require_positive(resolution, "resolution")
    if map_height <= 0 or map_width <= 0:
        raise ValueError(
            f"map_height and map_width must be > 0, got ({map_height}, {map_width})"
        )

    origin_x, origin_y = tuple(origin_xy)

    col = int(math.floor((x - origin_x) / resolution))
    row_from_bottom = int(math.floor((y - origin_y) / resolution))
    row = map_height - 1 - row_from_bottom

    if row < 0 or row >= map_height or col < 0 or col >= map_width:
        return None
    return row, col


def is_pose_clear_of_blocked_cells(
    x: float,
    y: float,
    clearance: float,
    blocked_cells: Set[Tuple[int, int]],
    origin_xy: Iterable[float],
    resolution: float,
    map_height: int,
    map_width: int,
) -> bool:
    """Return True when pose has requested clearance from all blocked cells."""
    _require_positive(resolution, "resolution")
    if clearance < 0.0:
        raise ValueError(f"clearance must be >= 0, got {clearance}")
    if map_height <= 0 or map_width <= 0:
        raise ValueError(
            f"map_height and map_width must be > 0, got ({map_height}, {map_width})"
        )

    origin_x, origin_y = tuple(origin_xy)
    map_max_x = origin_x + map_width * resolution
    map_max_y = origin_y + map_height * resolution

    if (
        x - clearance < origin_x
        or x + clearance > map_max_x
        or y - clearance < origin_y
        or y + clearance > map_max_y
    ):
        return False

    center_cell = world_to_grid_cell(
        x=x,
        y=y,
        origin_xy=(origin_x, origin_y),
        resolution=resolution,
        map_height=map_height,
        map_width=map_width,
    )
    if center_cell is None:
        return False

    col_min = int(math.floor((x - clearance - origin_x) / resolution))
    col_max = int(math.floor((x + clearance - origin_x) / resolution))
    row_bottom_min = int(math.floor((y - clearance - origin_y) / resolution))
    row_bottom_max = int(math.floor((y + clearance - origin_y) / resolution))

    row_min = map_height - 1 - row_bottom_max
    row_max = map_height - 1 - row_bottom_min
    row_min = max(0, row_min)
    row_max = min(map_height - 1, row_max)
    col_min = max(0, col_min)
    col_max = min(map_width - 1, col_max)

    half = 0.5 * resolution
    clearance_sq = clearance * clearance
    for row_index in range(row_min, row_max + 1):
        for col_index in range(col_min, col_max + 1):
            if (row_index, col_index) not in blocked_cells:
                continue
            cell_x, cell_y = grid_cell_to_world_center(
                row=row_index,
                col=col_index,
                origin_xy=(origin_x, origin_y),
                resolution=resolution,
                map_height=map_height,
            )
            dx = max(abs(x - cell_x) - half, 0.0)
            dy = max(abs(y - cell_y) - half, 0.0)
            if dx * dx + dy * dy <= clearance_sq:
                return False

    return True
