import os
import pywavemap as wave

# Create an empty map for illustration purposes
your_map = wave.Map.create({
    "type": "hashed_chunked_wavelet_octree",
    "min_cell_width": {
        "meters": 0.1
    }
})

# Save the map
user_home = os.path.expanduser('~')
map_path = os.path.join(user_home, "your_map.wvmp")
your_map.store(map_path)
