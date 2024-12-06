"""
Extract depth maps from wavemap maps at a given sensor pose and intrinsics
"""

import time
from pathlib import Path

import numpy as np
from PIL import Image
import pywavemap as wm


def save_depth_as_png(depth_map: np.ndarray, output_path: Path):
    depth_min = np.min(depth_map)
    depth_max = np.max(depth_map)

    # Avoid division by zero in case all values are the same
    if depth_max - depth_min > 0:
        depth_map_normalized = (depth_map - depth_min) / (depth_max -
                                                          depth_min)
    else:
        depth_map_normalized = np.zeros_like(depth_map)

    # Convert floats (meters) to uint8 and save to png
    depth_map_8bit = (depth_map_normalized * 255).astype(np.uint8)
    image = Image.fromarray(depth_map_8bit)
    image.save(output_path)


if __name__ == "__main__":
    map_path = Path.home() \
               / "data/panoptic_mapping/flat_dataset/run2/your_map.wvmp"
    out_path = Path(__file__).parent / "depth.png"

    # Configure the virtual sensor's projection model
    # NOTE: The projection model can be configured through a Python dictionary.
    #       Diagnostics for invalid configurations are printed to the terminal.
    #       For an overview of all the available models and their params, see:
    #       pylint: disable=line-too-long
    #       https://ethz-asl.github.io/wavemap/pages/parameters/measurement_integrators.html#projection-models
    #       Examples for a broad selection of common sensors can be found in the
    #       `interfaces/ros1/wavemap_ros/config` directory.
    #       The intrinsics below match the Zed 2i depth camera.
    projection_model = wm.Projector.create({
        "type": "pinhole_camera_projector",
        "width": 1280,
        "height": 720,
        "fx": 526.21539307,
        "fy": 526.21539307,
        "cx": 642.309021,
        "cy": 368.69949341
    })

    # Load map from file
    your_map = wm.Map.load(map_path)

    # Create the depth image renderer
    log_odds_occupancy_threshold = 0.1
    max_range = 6.0
    default_depth_value = -1.0
    renderer = wm.RaycastingRenderer(your_map, projection_model,
                                     log_odds_occupancy_threshold, max_range,
                                     default_depth_value)

    # Create pose
    rotation = wm.Rotation(np.eye(3))
    translation = np.zeros(3)
    pose = wm.Pose(rotation, translation)

    # Extract depth
    t1 = time.perf_counter()
    depth_image = renderer.render(pose)
    t2 = time.perf_counter()
    print(f"Depth image of size {depth_image.width}x{depth_image.height} "
          f"created in {t2 - t1:.02f} seconds")
    save_depth_as_png(depth_image.data, out_path)

    # Avoids leak warnings on old Python versions with lazy garbage collectors
    del renderer, depth_image
