import numpy as np

# Load a map
from io_load_map_from_file import your_map

# Declare the index to query
query_index = np.array([0, 0, 0])

# Query the map's value at the given index
occupancy_log_odds = your_map.getCellValue(query_index)
print(occupancy_log_odds)

# TODO(victorr): Extend bindings with vectorized version
