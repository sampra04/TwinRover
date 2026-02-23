import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

GridCell = Tuple[int, int]


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]


def _grid_index(width: int, x: int, y: int) -> int:
    return y * width + x


def _in_bounds(width: int, height: int, x: int, y: int) -> bool:
    return 0 <= x < width and 0 <= y < height


def _is_frontier_cell(grid_map: GridMap, x: int, y: int) -> bool:
    if grid_map.data[_grid_index(grid_map.width, x, y)] != 0:
        return False

    neighbors = [
        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1),
        (x + 1, y + 1),
        (x - 1, y - 1),
        (x + 1, y - 1),
        (x - 1, y + 1),
    ]
    for nx, ny in neighbors:
        if _in_bounds(grid_map.width, grid_map.height, nx, ny):
            if grid_map.data[_grid_index(grid_map.width, nx, ny)] == -1:
                return True
    return False


def cell_is_safe(grid_map: GridMap, cell: GridCell, safety_clearance_m: float) -> bool:
    clearance_cells = max(1, int(safety_clearance_m / max(1e-3, grid_map.resolution)))
    cx, cy = cell

    for dy in range(-clearance_cells, clearance_cells + 1):
        for dx in range(-clearance_cells, clearance_cells + 1):
            x = cx + dx
            y = cy + dy
            if not _in_bounds(grid_map.width, grid_map.height, x, y):
                return False
            cost = grid_map.data[_grid_index(grid_map.width, x, y)]
            if cost > 50:
                return False
    return True


def detect_frontiers(
    grid_map: GridMap,
    min_frontier_cells: int,
    safety_clearance_m: float,
) -> List[List[GridCell]]:
    frontier_cells = set()
    for y in range(1, grid_map.height - 1):
        for x in range(1, grid_map.width - 1):
            if _is_frontier_cell(grid_map, x, y):
                frontier_cells.add((x, y))

    clusters: List[List[GridCell]] = []
    visited = set()
    for cell in frontier_cells:
        if cell in visited:
            continue

        cluster = []
        queue = deque([cell])
        visited.add(cell)
        while queue:
            cx, cy = queue.popleft()
            cluster.append((cx, cy))
            for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                ncell = (nx, ny)
                if ncell in frontier_cells and ncell not in visited:
                    visited.add(ncell)
                    queue.append(ncell)

        if len(cluster) >= min_frontier_cells:
            safe_cells = [c for c in cluster if cell_is_safe(grid_map, c, safety_clearance_m)]
            if len(safe_cells) >= min_frontier_cells:
                clusters.append(safe_cells)

    return clusters


def cluster_centroid(cluster: List[GridCell]) -> GridCell:
    sx = sum(c[0] for c in cluster)
    sy = sum(c[1] for c in cluster)
    return (int(sx / len(cluster)), int(sy / len(cluster)))


def cell_to_xy(grid_map: GridMap, cell: GridCell) -> Tuple[float, float]:
    x = grid_map.origin_x + (cell[0] + 0.5) * grid_map.resolution
    y = grid_map.origin_y + (cell[1] + 0.5) * grid_map.resolution
    return (x, y)


def euclidean(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def estimate_obstacle_proximity_penalty(grid_map: GridMap, cell: GridCell) -> float:
    cx, cy = cell
    max_radius = 6
    penalty = 0.0

    for radius in range(1, max_radius + 1):
        hits = 0
        samples = 0
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                x = cx + dx
                y = cy + dy
                if not _in_bounds(grid_map.width, grid_map.height, x, y):
                    continue
                if abs(dx) != radius and abs(dy) != radius:
                    continue
                samples += 1
                if grid_map.data[_grid_index(grid_map.width, x, y)] > 50:
                    hits += 1
        if samples > 0:
            penalty += float(hits) / float(samples)

    return penalty


def select_best_frontier(
    grid_map: GridMap,
    frontiers: List[List[GridCell]],
    robot_xy: Tuple[float, float],
    goal_blacklist: Dict[GridCell, object],
    w_info: float,
    w_dist: float,
    w_risk: float,
) -> Optional[GridCell]:
    best_cell = None
    best_score = -float('inf')

    for cluster in frontiers:
        centroid = cluster_centroid(cluster)
        if centroid in goal_blacklist:
            continue

        info_gain = float(len(cluster))
        dist = euclidean(robot_xy, cell_to_xy(grid_map, centroid))
        risk = estimate_obstacle_proximity_penalty(grid_map, centroid)
        score = w_info * info_gain - w_dist * dist - w_risk * risk

        if score > best_score:
            best_score = score
            best_cell = centroid

    return best_cell


def cleanup_blacklist_entries(goal_blacklist: Dict[GridCell, float], now_s: float) -> List[GridCell]:
    return [cell for cell, expiry_s in goal_blacklist.items() if now_s >= expiry_s]
