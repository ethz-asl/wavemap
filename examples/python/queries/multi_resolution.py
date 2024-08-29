import numpy as np
import pywavemap as wave
import _dummy_objects

# Load a map
your_map = _dummy_objects.example_map()

# Define the center point and the minimum width of your region of interest
query_point = np.array([0.4, 0.5, 0.6])
query_min_cell_width = 0.5  # in meters

# Compute the index of the smallest node that covers it completely
query_height = wave.convert.cell_width_to_height(query_min_cell_width,
                                                 your_map.min_cell_width)
query_index = wave.convert.point_to_node_index(query_point,
                                               your_map.min_cell_width,
                                               query_height)

# Query the node's average occupancy
occupancy_log_odds = your_map.getCellValue(query_index)
print(occupancy_log_odds)
