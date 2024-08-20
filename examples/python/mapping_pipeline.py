# !/usr/bin/env python3

import os
import csv
from PIL import Image as PilImage
import numpy as np
import pywavemap as pw

# Parameters
home_dir = os.path.expanduser('~')
measurement_dir = os.path.join(home_dir,
                               "data/panoptic_mapping/flat_dataset/run2")
output_map_path = os.path.join(home_dir, "your_map.wvmp")

# Create a map
your_map = pw.Map.create({
    "type": "hashed_chunked_wavelet_octree",
    "min_cell_width": {
        "meters": 0.05
    }
})

# Create a measurement integration pipeline
pipeline = pw.Pipeline(your_map)
# Add map operations
pipeline.addOperation({
    "type": "threshold_map",
    "once_every": {
        "seconds": 5.0
    }
})
pipeline.addOperation({"type": "prune_map", "once_every": {"seconds": 10.0}})
# Add a measurement integrator
pipeline.addIntegrator(
    "my_integrator", {
        "projection_model": {
            "type": "pinhole_camera_projector",
            "width": 640,
            "height": 480,
            "fx": 320.0,
            "fy": 320.0,
            "cx": 320.0,
            "cy": 240.0
        },
        "measurement_model": {
            "type": "continuous_ray",
            "range_sigma": {
                "meters": 0.01
            },
            "scaling_free": 0.2,
            "scaling_occupied": 0.4
        },
        "integration_method": {
            "type": "hashed_chunked_wavelet_integrator",
            "min_range": {
                "meters": 0.1
            },
            "max_range": {
                "meters": 5.0
            }
        },
    })

# Index the input data
ids = []
times = []
stamps_file = os.path.join(measurement_dir, 'timestamps.csv')
if not os.path.isfile(stamps_file):
    print(f"Could not find timestamp file '{stamps_file}'.")
with open(stamps_file, 'r') as read_obj:
    csv_reader = csv.reader(read_obj)
    for row in csv_reader:
        if row[0] == "ImageID":
            continue
        ids.append(str(row[0]))
        times.append(float(row[1]) / 1e9)
ids = [x for _, x in sorted(zip(times, ids))]

# Integrate all the measurements
current_index = 0
while True:
    # Check we're not done
    if current_index >= len(ids):
        break

    # Load depth image
    file_path_prefix = os.path.join(measurement_dir, ids[current_index])
    depth_file = file_path_prefix + "_depth.tiff"
    if not os.path.isfile(depth_file):
        print(f"Could not find depth image file '{depth_file}'")
        current_index += 1
        raise SystemExit
    cv_img = PilImage.open(depth_file)
    image = pw.Image(np.array(cv_img).transpose())

    # Load transform
    pose_file = file_path_prefix + "_pose.txt"
    if not os.path.isfile(pose_file):
        print(f"Could not find pose file '{pose_file}'")
        current_index += 1
        raise SystemExit
    if os.path.isfile(pose_file):
        with open(pose_file, 'r') as f:
            pose_data = [float(x) for x in f.read().split()]
            transform = np.eye(4)
            for row in range(4):
                for col in range(4):
                    transform[row, col] = pose_data[row * 4 + col]
    pose = pw.Pose(transform)

    # Integrate the depth image
    print(f"Integrating measurement {ids[current_index]}")
    pipeline.runPipeline(["my_integrator"], pw.PosedImage(pose, image))

    current_index += 1

# Save the map
print(f"Saving map of size {your_map.memory_usage}")
your_map.store(output_map_path)

# Avoids leak warnings on old Python versions with lazy garbage collectors
del pipeline, your_map
