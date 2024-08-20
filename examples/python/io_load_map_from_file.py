import os
import pywavemap as pw

# Load the map
user_home = os.path.expanduser('~')
map_path = os.path.join(user_home, "your_map.wvmp")
your_map = pw.Map.load(map_path)
