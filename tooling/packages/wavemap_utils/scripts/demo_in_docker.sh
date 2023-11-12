#!/usr/bin/env bash

# Calling this script with no arguments will start a wavemap container running
# an interactive bash session.
# You can also call this script with arguments, which will then be executed as a
# command in the container, for example:
# `run_in_docker.sh roslaunch wavemap_ros newer_college_os0_cloister.launch`

# The following named volume will be used to persist wavemap's configuration
# files when the container is restarted. This allows users to set the sensor
# extrinsics (e.g. depth camera and LiDAR to IMU transformations) directly in
# the container and have the changes persist across Docker runs.
# To reset the configuration files, run
# docker volume rm wavemap_demo_config
config_volume=wavemap_demo_config

# Allow the docker user to connect to the X window display, to open GUIs
xhost + local:docker

# Detect whether the Nvidia container toolkit is available
# NOTE: With this toolkit, Nvidia GPUs on the host can be shared with processes
#       running in the container. This makes graphics intensive applications
#       such as Rviz much more responsive.
#       If you have an Nvidia GPU, you can install it with these instructions:
#       https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#docker
if docker info | grep "Runtimes.*nvidia.*" &>/dev/null; then
# Run the Docker container
# NOTE: We use the --rm flag to create an ephemeral container, meaning that the
#       container will be erased once it stops running. In case you want changes
#       that you make and data that you load into the container to keep existing
#       between sessions, you can omit the --rm flag and give your container a
#       name with --name <your_name>. When the container exits, you will then be
#       able to restart it with `docker start -i <your_name>`. If you do this,
#       don't forget to manually delete the container with
#       `docker container rm <your_name>` once you no longer need it, to free up
#       disk space.
#       In case you want to let ROS nodes running in the container
#       (e.g. wavemap_ros) talk to ROS nodes (e.g. rosbag play) on the host,
#       you can add the --network="host" flag or configure custom networks.
#       The full documentation for docker run is available at:
#       https://docs.docker.com/engine/reference/run/
  docker run --rm -it --privileged --net=host \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --runtime=nvidia --gpus all \
    -v "$config_volume":/home/ci/catkin_ws/src/wavemap/ros/wavemap_ros:rw \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics \
    wavemap_demo "$@"
else
# If the Nvidia container toolkit is not available, we launch the container with
# access to the host display but without GPU acceleration.
# NOTE: This mode works well for regular processing. However, displaying large
#       maps in Rviz might be slow as it will only be able to use software
#       rendering.
  docker run --rm -it --privileged --net=host --device /dev/dri \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$config_volume":/home/ci/catkin_ws/src/wavemap/ros/wavemap_ros:rw \
    wavemap_demo "$@"
fi
