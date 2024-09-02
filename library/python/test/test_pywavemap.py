# pylint: disable=import-outside-toplevel
def load_test_map():
    import pywavemap
    return pywavemap.Map.load(
        "/home/victor/data/wavemaps/newer_college_mine_10cm.wvmp")


def test_import():
    import pywavemap

    assert pywavemap is not None


def test_batched_fixed_resolution_queries():
    import numpy as np

    test_map = load_test_map()

    cell_indices = np.random.randint(-100, 100, size=(64 * 64 * 32, 3))
    cell_values = test_map.getCellValues(cell_indices)
    for cell_idx in range(cell_indices.shape[0]):
        cell_index = cell_indices[cell_idx, :]
        cell_value = test_map.getCellValue(cell_index)
        assert cell_values[cell_idx] == cell_value


def test_batched_multi_resolution_queries():
    import numpy as np
    import pywavemap

    test_map = load_test_map()

    cell_positions = np.random.randint(-100, 100, size=(64 * 64 * 32, 3))
    cell_heights = np.random.randint(0, 6, size=(64 * 64 * 32, 1))
    cell_indices = np.concatenate((cell_heights, cell_positions), axis=1)
    cell_values = test_map.getCellValues(cell_indices)
    for cell_idx in range(cell_positions.shape[0]):
        cell_index = pywavemap.OctreeIndex(cell_heights[cell_idx],
                                           cell_positions[cell_idx, :])
        cell_value = test_map.getCellValue(cell_index)
        assert cell_values[cell_idx] == cell_value


def test_batched_nearest_neighbor_interpolation():
    import numpy as np

    test_map = load_test_map()

    points = np.random.random(size=(64 * 64 * 32, 3))
    points_log_odds = test_map.interpolateNearest(points)
    for point_idx in range(points.shape[0]):
        point = points[point_idx, :]
        point_log_odds = test_map.interpolateNearest(point)
        assert points_log_odds[point_idx] == point_log_odds


def test_batched_trilinear_interpolation():
    import numpy as np

    test_map = load_test_map()

    points = np.random.random(size=(64 * 64 * 32, 3))
    points_log_odds = test_map.interpolateTrilinear(points)
    for point_idx in range(points.shape[0]):
        point = points[point_idx, :]
        point_log_odds = test_map.interpolateTrilinear(point)
        assert points_log_odds[point_idx] == point_log_odds
