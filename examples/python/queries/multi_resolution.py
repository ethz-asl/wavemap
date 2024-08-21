import numpy as np
import pywavemap as pw
import _dummy_objects

# Load a map
your_map = _dummy_objects.example_map()

# Define the center point and the minimum width of the octree cell to query
query_point = np.array([0.4, 0.5, 0.6])
query_min_cell_width = 0.5  # in meters

# Convert it to an octree node index
map_min_cell_width = your_map.min_cell_width
query_height = pw.convert.cell_width_to_height(query_min_cell_width,
                                               map_min_cell_width)
query_index = pw.convert.point_to_node_index(query_point, map_min_cell_width,
                                             query_height)

# Query the map
occupancy_log_odds = your_map.getCellValue(query_index)
print(occupancy_log_odds)
