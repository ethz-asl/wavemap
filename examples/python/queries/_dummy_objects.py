import pywavemap as pw


def example_occupancy_log_odds():
    return 0.0


def example_map():
    return pw.Map.create({
        "type": "hashed_chunked_wavelet_octree",
        "min_cell_width": {
            "meters": 0.1
        }
    })
