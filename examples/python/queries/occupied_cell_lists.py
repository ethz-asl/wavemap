import _dummy_objects

# Load a map
your_map = _dummy_objects.example_map()

# Log odds threshold above which to consider a cell occupied
log_odds_occupancy_threshold = 1e-3

# Get the node indices of all occupied leaf nodes
occupied_nodes = your_map.get_occupied_node_indices(
    log_odds_occupancy_threshold)
print(occupied_nodes)

# Get the center points of all occupied cells at the highest resolution
occupied_points = your_map.get_occupied_pointcloud(
    log_odds_occupancy_threshold)
print(occupied_points)
