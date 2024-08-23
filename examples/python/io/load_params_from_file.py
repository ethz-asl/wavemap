import os
import yaml
import pywavemap as wave


def create_map_from_config(config_file_path):
    """
    Example function that creates a map based on parameters in a YAML file.
    """
    with open(config_file_path) as file:
        try:
            config = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print(exc)
            return None

    if isinstance(config, dict) and "map" in config.keys():
        return wave.Map.create(config["map"])

    return None


# Provide the path to your config
config_dir = os.path.abspath(
    os.path.join(__file__, "../../../../interfaces/ros1/wavemap_ros/config"))
config_file = os.path.join(config_dir, "wavemap_panoptic_mapping_rgbd.yaml")

# Create the map
your_map = create_map_from_config(config_file)
