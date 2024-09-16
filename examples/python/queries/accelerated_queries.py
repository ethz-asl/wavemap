import numpy as np

import _dummy_objects

# Load a map
your_map = _dummy_objects.example_map()

# Vectorized query for a list of indices at the highest resolution (height 0)
indices = np.random.randint(-100, 100, size=(64 * 64 * 32, 3))
values = your_map.get_cell_values(indices)
print(values)

# Vectorized query for a list of multi-resolution indices (at random heights)
node_heights = np.random.randint(0, 6, size=(64 * 64 * 32, 1))
node_indices = np.concatenate((node_heights, indices), axis=1)
node_values = your_map.get_cell_values(node_indices)
print(node_values)
