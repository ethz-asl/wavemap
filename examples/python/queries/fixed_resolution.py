import numpy as np
import _dummy_objects

# Load a map
your_map = _dummy_objects.example_map()

# Declare the index to query
query_index = np.array([0, 0, 0])

# Query the map's value at the given index
occupancy_log_odds = your_map.getCellValue(query_index)
print(occupancy_log_odds)
