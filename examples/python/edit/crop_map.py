import os
import numpy as np
import pywavemap as wave

# Load the map
user_home = os.path.expanduser('~')
input_map_path = os.path.join(user_home, "your_map.wvmp")
your_map = wave.Map.load(input_map_path)

# Crop the map
cropping_sphere = wave.Sphere(center=np.array([-2.2, -1.4, 0.0]), radius=3.0)
wave.edit.crop(your_map, cropping_sphere)

# Save the map
output_map_path = os.path.join(user_home, "your_map_cropped.wvmp")
your_map.store(output_map_path)
