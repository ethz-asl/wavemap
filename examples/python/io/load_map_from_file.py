import os
import pywavemap as wave

# Load the map
user_home = os.path.expanduser('~')
map_path = os.path.join(user_home, "your_map.wvmp")
your_map = wave.Map.load(map_path)
