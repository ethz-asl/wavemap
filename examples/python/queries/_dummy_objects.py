import pywavemap as wave


def example_occupancy_log_odds():
    """Function that returns a dummy occupancy value to be used in examples."""
    return 0.0


def example_map():
    """Function that returns a dummy map to be used in examples."""
    return wave.Map.create({
        "type": "hashed_wavelet_octree",
        "min_cell_width": {
            "meters": 0.1
        }
    })
