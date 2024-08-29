import numpy as np
import _dummy_objects

# Load a map
your_map = _dummy_objects.example_map()

# Declare the point to query [in map frame]
query_point = np.array([0.4, .5, 0.6])

# Query a single point
occupancy_log_odds = your_map.interpolateTrilinear(query_point)
print(occupancy_log_odds)

# Vectorized query for a list of points
points = np.random.random(size=(64 * 64 * 32, 3))
points_log_odds = your_map.interpolateTrilinear(points)
print(points_log_odds)
