import numpy as np
from pywavemap import InterpolationMode
import _dummy_objects

# Load a map
your_map = _dummy_objects.example_map()

# Declare the point to query [in map frame]
query_point = np.array([0.4, 0.5, 0.6])

# Query a single point
occupancy_log_odds = your_map.interpolate(query_point,
                                          InterpolationMode.NEAREST)
print(occupancy_log_odds)

# Vectorized query for a list of points
points = np.random.random(size=(64 * 64 * 32, 3))
points_log_odds = your_map.interpolate(points, InterpolationMode.NEAREST)
print(points_log_odds)
