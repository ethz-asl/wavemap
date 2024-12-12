import os
import pywavemap as wave

# Load the map
user_home = os.path.expanduser('~')
input_map_path = os.path.join(user_home, "your_map.wvmp")
your_map = wave.Map.load(input_map_path)

# Use the multiply method to implement exponential forgetting
decay_factor = 0.9
wave.edit.multiply(your_map, decay_factor)

# Save the map
output_map_path = os.path.join(user_home, "your_map_decayed.wvmp")
your_map.store(output_map_path)
