from frontier_explorer.frontier_logic import (
    GridMap,
    cleanup_blacklist_entries,
    detect_frontiers,
    select_best_frontier,
)


def build_grid(width, height, fill=0):
    return [fill for _ in range(width * height)]


def set_cell(data, width, x, y, value):
    data[y * width + x] = value


def test_frontier_detection_finds_clusters():
    width, height = 12, 12
    data = build_grid(width, height, fill=0)

    for y in range(4, 8):
        for x in range(4, 8):
            set_cell(data, width, x, y, -1)

    grid = GridMap(width=width, height=height, resolution=0.2, origin_x=0.0, origin_y=0.0, data=data)
    clusters = detect_frontiers(grid, min_frontier_cells=6, safety_clearance_m=0.2)

    assert len(clusters) >= 1
    assert max(len(c) for c in clusters) >= 6


def test_cluster_size_filter_rejects_small_clusters():
    width, height = 12, 12
    data = build_grid(width, height, fill=0)
    set_cell(data, width, 6, 6, -1)

    grid = GridMap(width=width, height=height, resolution=0.2, origin_x=0.0, origin_y=0.0, data=data)
    clusters = detect_frontiers(grid, min_frontier_cells=20, safety_clearance_m=0.2)

    assert clusters == []


def test_safety_clearance_rejects_obstacle_near_frontier():
    width, height = 14, 14
    data = build_grid(width, height, fill=0)

    for y in range(5, 9):
        for x in range(5, 9):
            set_cell(data, width, x, y, -1)

    for y in range(3, 11):
        set_cell(data, width, 3, y, 100)
        set_cell(data, width, 10, y, 100)
    for x in range(3, 11):
        set_cell(data, width, x, 3, 100)
        set_cell(data, width, x, 10, 100)

    grid = GridMap(width=width, height=height, resolution=0.2, origin_x=0.0, origin_y=0.0, data=data)
    clusters = detect_frontiers(grid, min_frontier_cells=6, safety_clearance_m=0.4)

    assert clusters == []


def test_score_prefers_larger_frontier_when_distance_equal():
    width, height = 20, 20
    data = build_grid(width, height, fill=0)
    grid = GridMap(width=width, height=height, resolution=0.2, origin_x=0.0, origin_y=0.0, data=data)

    big_cluster = [(8, 10), (9, 10), (10, 10), (11, 10), (9, 9), (10, 9)]
    small_cluster = [(8, 12), (9, 12), (10, 12)]

    best = select_best_frontier(
        grid,
        [small_cluster, big_cluster],
        robot_xy=(2.0, 2.0),
        goal_blacklist={},
        w_info=1.0,
        w_dist=0.0,
        w_risk=0.0,
    )

    assert best == (9, 9)


def test_blacklist_skip_and_expiry_behavior():
    width, height = 20, 20
    data = build_grid(width, height, fill=0)
    grid = GridMap(width=width, height=height, resolution=0.2, origin_x=0.0, origin_y=0.0, data=data)

    preferred = [(8, 10), (9, 10), (10, 10), (11, 10)]
    fallback = [(2, 2), (2, 3), (3, 2), (3, 3)]

    blacklist = {(9, 10): 100.0}
    best = select_best_frontier(
        grid,
        [preferred, fallback],
        robot_xy=(0.0, 0.0),
        goal_blacklist=blacklist,
        w_info=1.0,
        w_dist=0.0,
        w_risk=0.0,
    )
    assert best == (2, 2)

    expired = cleanup_blacklist_entries(
        {(9, 10): 10.0, (2, 2): 25.0},
        now_s=20.0,
    )
    assert expired == [(9, 10)]
