import os
import numpy as np
import pywavemap as wave

# Load the map
user_home = os.path.expanduser('~')
input_map_path = os.path.join(user_home, "your_map.wvmp")
your_map = wave.Map.load(input_map_path)

# Crop the map
center = np.array([-2.2, -1.4, 0.0])
radius = 3.0
wave.edit.crop_to_sphere(your_map, center, radius)

# Create a translated copy
translation = np.array([5.0, 5.0, 0.0])
rotation = wave.Rotation(w=1.0, x=0.0, y=0.0, z=0.0)
transformation = wave.Pose(rotation, translation)
your_map_translated = wave.edit.transform(your_map, transformation)

# Merge them together
wave.edit.sum(your_map, your_map_translated)

# Set a box in the map to free
box = wave.AABB(min=np.array([6.0, 6.0, -2.0]),
                max=np.array([10.0, 10.0, 2.0]))
wave.edit.sum(your_map, box, -1.0)

# Set a sphere in the map to occupied
sphere = wave.Sphere(center=np.array([8.0, 8.0, 0.0]), radius=1.5)
wave.edit.sum(your_map, sphere, 2.0)

# Save the map
output_map_path = os.path.join(user_home, "your_map_merged.wvmp")
your_map.store(output_map_path)
