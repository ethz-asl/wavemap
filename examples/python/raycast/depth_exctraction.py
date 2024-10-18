"""
Depth map extraction from HashedWaveletOctree map at given camera pose and intrinsics
"""

import time
from pathlib import Path

import numpy as np
from PIL import Image
import pywavemap as wm

def save_depth_as_png(depth_map: np.ndarray, out_path: Path):
    depth_min = np.min(depth_map)
    depth_max = np.max(depth_map)

    # Avoid division by zero in case all values are the same
    if depth_max - depth_min > 0:
        depth_map_normalized = (depth_map - depth_min) / (depth_max - depth_min)
    else:
        depth_map_normalized = np.zeros_like(depth_map)

    # Convert floats (meters) to uint8 and save to png
    depth_map_8bit = (depth_map_normalized * 255).astype(np.uint8)
    image = Image.fromarray(depth_map_8bit)
    image.save(out_path)

if __name__ == "__main__":
    map_path = Path.home() / "data/panoptic_mapping/flat_dataset/run2/your_map.wvmp"
    out_path = Path(__file__).parent / "depth.png"
    camera_cfg = wm.PinholeCameraProjectorConfig(
        width=1280,
        height=720,
        fx=526.21539307,
        fy=526.21539307,
        cx=642.309021,
        cy=368.69949341,
    ) # Note: these are intrinsics for Zed 2i

    # Load map from file
    map = wm.Map.load(map_path)

    # Create pose
    rotation = wm.Rotation(np.eye(3))
    translation = np.zeros(3)
    pose = wm.Pose(rotation, translation)

    # Extract depth
    t1 = time.perf_counter()
    depth = wm.get_depth(map, pose, camera_cfg, 0.1, 10)
    t2 = time.perf_counter()
    print(f"Depth map of size {camera_cfg.width}x{camera_cfg.height} created in {t2-t1:.02f} seconds")
    save_depth_as_png(depth, out_path)