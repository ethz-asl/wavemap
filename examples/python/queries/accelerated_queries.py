import numpy as np

import _dummy_objects

# Load a map
your_map = _dummy_objects.example_map()

# Vectorized query for a list of fixed-resolution indices
indices = np.random.randint(-100, 100, size=(64 * 64 * 32, 3))
values = your_map.getCellValues(indices)

# Vectorized query for a list of node indices
node_heights = np.random.randint(0, 6, size=(64 * 64 * 32, 1))
node_indices = np.concatenate((node_heights, indices), axis=1)
node_values = your_map.getCellValues(node_indices)
