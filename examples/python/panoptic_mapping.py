# !/usr/bin/env python3

import os
import csv
from PIL import Image as PilImage
import numpy as np
import yaml
import pywavemap as wave
from tqdm import tqdm


class DataLoader():

    def __init__(self, params, data_path):
        self.data_path = data_path

        self.map = wave.Map.create(params["map"])

        self.pipeline = wave.Pipeline(self.map)

        for operation in params["map_operations"]:
            self.pipeline.add_operation(operation)

        measurement_integrators = params["measurement_integrators"]
        if len(measurement_integrators) != 1:
            print("Expected 1 integrator to be specified. "
                  f"Got {len(measurement_integrators)}.")
            raise SystemExit
        self.integrator_name, integrator_params = \
            measurement_integrators.popitem()

        self.pipeline.add_integrator(self.integrator_name, integrator_params)

        # Load list of measurements
        stamps_file = os.path.join(self.data_path, 'timestamps.csv')
        self.times = []
        self.ids = []
        self.current_index = 0  # Used to iterate through
        if not os.path.isfile(stamps_file):
            print(f"No timestamp file '{stamps_file}' found.")
        with open(stamps_file) as read_obj:
            csv_reader = csv.reader(read_obj)
            for row in csv_reader:
                if row[0] == "ImageID":
                    continue
                self.ids.append(str(row[0]))
                self.times.append(float(row[1]) / 1e9)

        self.ids = [x for _, x in sorted(zip(self.times, self.ids))]
        self.times = sorted(self.times)

    def run(self):
        for _ in tqdm(range(len(self.times)), desc="Integrating..."):
            if not self.integrate_frame():
                break

    def integrate_frame(self):
        # Check we're not done.
        if self.current_index >= len(self.times):
            return False

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
        image = wave.Image(np.array(cv_img).transpose())

        # Load transform
        if os.path.isfile(pose_file):
            with open(pose_file) as f:
                pose_data = [float(x) for x in f.read().split()]
                transform = np.eye(4)
                for row in range(4):
                    for col in range(4):
                        transform[row, col] = pose_data[row * 4 + col]
        pose = wave.Pose(transform)

        self.pipeline.run_pipeline([self.integrator_name],
                                   wave.PosedImage(pose, image))

        self.current_index += 1

        return True

    def save_map(self, path):
        print(f"Saving map of size {self.map.memory_usage}")
        self.map.store(path)


if __name__ == '__main__':
    config_dir = os.path.abspath(
        os.path.join(__file__, "../../../interfaces/ros1/wavemap_ros/config"))
    config_file = os.path.join(config_dir,
                               "wavemap_panoptic_mapping_rgbd.yaml")
    with open(config_file) as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    user_home = os.path.expanduser('~')
    panoptic_mapping_dir = os.path.join(user_home,
                                        "data/panoptic_mapping/flat_dataset")
    panoptic_mapping_seq = "run2"
    output_map_path = os.path.join(
        user_home, f"panoptic_mapping_{panoptic_mapping_seq}.wvmp")

    data_loader = DataLoader(
        config, os.path.join(panoptic_mapping_dir, panoptic_mapping_seq))
    data_loader.run()
    data_loader.save_map(output_map_path)
    del data_loader  # To avoid mem leak warnings on older Python versions
