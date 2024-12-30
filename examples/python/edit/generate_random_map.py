import os
import numpy as np
import pywavemap as wave

# Create an empty map
user_home = os.path.expanduser('~')
your_map = wave.Map.create({
    "type": "hashed_wavelet_octree",
    "min_cell_width": {
        "meters": 0.1
    }
})

# Create a large box of free space
map_aabb = wave.AABB(min=np.array([-50.0, -50.0, -50.0]),
                     max=np.array([50.0, 50.0, 50.0]))
wave.edit.sum(your_map, map_aabb, -0.05)

# Add random obstacles and cutouts
num_obstacles = 500
for idx in range(num_obstacles):
    random_center = 2.0 * (np.random.rand(3) - 0.5)
    random_center *= np.array([50.0, 50.0, 50.0])
    random_width = 10.0 * np.random.rand(3)
    update = float(np.random.rand(1)[0] * 0.2)
    if np.random.rand(1)[0] < 0.7:
        random_radius = float(random_width[0])
        sphere = wave.Sphere(center=random_center, radius=random_radius)
        wave.edit.sum(your_map, sphere, update)
    else:
        box = wave.AABB(min=random_center - random_width,
                        max=random_center + random_width)
        wave.edit.sum(your_map, box, update)
    if idx % 20:
        your_map.threshold()

# Trim off any excess parts
wave.edit.crop(your_map, map_aabb)
your_map.prune()

# Save the map
output_map_path = os.path.join(user_home, "random_map.wvmp")
your_map.store(output_map_path)
