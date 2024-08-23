import numpy as np
import pywavemap as wave

import _dummy_objects

# Load a map
your_map = _dummy_objects.example_map()

# Create a query accelerator
accelerator = wave.HashedWaveletOctreeQueryAccelerator(your_map)

# Query fixed resolution indices one by one
value_1 = accelerator.getCellValue(np.array([1, 2, 3]))
value_2 = accelerator.getCellValue(np.array([4, 5, 6]))
value_3 = accelerator.getCellValue(np.array([7, 8, 9]))

# Query node indices one by one
value_4 = accelerator.getCellValue(wave.OctreeIndex(0, np.array([1, 2, 3])))
value_5 = accelerator.getCellValue(wave.OctreeIndex(1, np.array([4, 5, 6])))
value_6 = accelerator.getCellValue(wave.OctreeIndex(2, np.array([7, 8, 9])))

# Vectorized query for a list of fixed-resolution indices
indices = np.random.randint(-100, 100, size=(64 * 64 * 32, 3))
values = accelerator.getCellValues(indices)

# Vectorized query for a list of node indices
node_heights = np.random.randint(0, 6, size=(64 * 64 * 32, 1))
node_indices = np.concatenate((node_heights, indices), axis=1)
node_values = accelerator.getCellValues(node_indices)
