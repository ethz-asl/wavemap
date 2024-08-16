# !/usr/bin/env python3

import os
import csv
from PIL import Image as PilImage
import numpy as np
import pywavemap as pw


class DataLoader():
    # pylint: disable=R0902
    def __init__(self, data_path):
        self.data_path = data_path

        self.map = pw.Map.create({
            "type": "hashed_chunked_wavelet_octree",
            "min_cell_width": {
                "meters": 0.05
            }
        })

        self.pipeline = pw.Pipeline(self.map)

        self.pipeline.addOperation({
            "type": "threshold_map",
            "once_every": {
                "seconds": 5.0
            }
        })
        self.pipeline.addOperation({
            "type": "prune_map",
            "once_every": {
                "seconds": 10.0
            }
        })

        self.pipeline.addIntegrator(
            "dummy_integrator", {
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

        # setup
        stamps_file = os.path.join(self.data_path, 'timestamps.csv')
        self.times = []
        self.ids = []
        self.current_index = 0  # Used to iterate through
        if not os.path.isfile(stamps_file):
            print(f"No timestamp file '{stamps_file}' found.")
        with open(stamps_file, 'r') as read_obj:
            csv_reader = csv.reader(read_obj)
            for row in csv_reader:
                if row[0] == "ImageID":
                    continue
                self.ids.append(str(row[0]))
                self.times.append(float(row[1]) / 1e9)

        self.ids = [x for _, x in sorted(zip(self.times, self.ids))]
        self.times = sorted(self.times)

    def run(self):
        while self.integrate_frame():
            pass

    def integrate_frame(self):
        # Check we're not done.
        if self.current_index >= len(self.times):
            return False
        print(f"Integrating frame {self.current_index} of {len(self.times)}")

        # Get all data and publish.
        file_id = os.path.join(self.data_path, self.ids[self.current_index])

        # Read the image and pose
        depth_file = file_id + "_depth.tiff"
        pose_file = file_id + "_pose.txt"
        files = [depth_file, pose_file]
        for f in files:
            if not os.path.isfile(f):
                print(f"Could not find file '{f}', skipping frame.")
                self.current_index += 1
                return False

        # Load depth image
        cv_img = PilImage.open(depth_file)
        image = pw.Image(np.array(cv_img).transpose())

        # Load transform
        if os.path.isfile(pose_file):
            with open(pose_file, 'r') as f:
                pose_data = [float(x) for x in f.read().split()]
                transform = np.eye(4)
                for row in range(4):
                    for col in range(4):
                        transform[row, col] = pose_data[row * 4 + col]
        pose = pw.Pose(transform)

        self.pipeline.runPipeline(["dummy_integrator"],
                                  pw.PosedImage(pose, image))

        self.current_index += 1

        return True

    def save_map(self, path):
        print(f"Saving map of size {self.map.memory_usage}")
        self.map.store(path)


if __name__ == '__main__':
    user_home = os.path.expanduser('~')
    panoptic_mapping_dir = os.path.join(user_home,
                                        "data/panoptic_mapping/flat_dataset")
    panoptic_mapping_seq = "run2"
    output_map_path = os.path.join(
        user_home, f"panoptic_mapping_{panoptic_mapping_seq}.wvmp")
    data_loader = DataLoader(
        os.path.join(panoptic_mapping_dir, panoptic_mapping_seq))
    data_loader.run()
    data_loader.save_map(output_map_path)
    del data_loader  # To avoid mem leak warnings on older Python versions
