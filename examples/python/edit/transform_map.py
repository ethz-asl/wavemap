import os
import numpy as np
import pywavemap as wave

# Load the map
user_home = os.path.expanduser('~')
input_map_path = os.path.join(user_home, "your_map.wvmp")
your_map = wave.Map.load(input_map_path)

# Define a transformation that flips the map upside down, for illustration
translation = np.array([0.0, 0.0, 0.0])
rotation = wave.Rotation(w=0.0, x=1.0, y=0.0, z=0.0)
transformation = wave.Pose(rotation, translation)

# Transform the map
your_map = wave.edit.transform(your_map, transformation)

# Save the map
output_map_path = os.path.join(user_home, "your_map_transformed.wvmp")
your_map.store(output_map_path)
