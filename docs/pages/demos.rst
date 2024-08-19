Demos
#####
.. highlight:: bash
.. include:: <isonum.txt>
.. rstcheck: ignore-roles=gh_file

All of the demos currently use wavemap's ROS1 interface. In case they interest you but you do not use ROS1, we recommend trying them through a :ref:`temporary Docker container <installation-ros1-docker>`.

Quick start
***********
To get an impression of the maps wavemap can generate, you can download the maps of our `RSS paper <https://www.roboticsproceedings.org/rss19/p065.pdf>`__ and directly view them in Rviz. To do so,

* Choose and download one of the maps provided `here <https://drive.google.com/drive/folders/1sTmDBUt97wwE220gVFwCq88JT5IOQlk5>`__
* Open Rviz, for example by running :code:`roscore & rviz`
* Load wavemap's rviz plugin by clicking: `Add` |rarr| `By display type` |rarr| `wavemap_rviz_plugin` |rarr| `WavemapMap`
* In the plugin settings, under `Source` select `File`
* Load the map you just downloaded by clicking: `Loaded map` |rarr| `Choose file`
* [Optional] Tweak the visualization options under `Render voxels` and `Render slice`


Newer College dataset
*********************
The Newer College dataset is available `here <https://ori-drs.github.io/newer-college-dataset/download/>`__. To get the sharpest maps, we recommend supplying wavemap with a high-rate odometry estimate and turning on wavemap's built-in pointcloud motion undistortion. In our experiments, we got these estimates by modifying FastLIO2 to publish its forward-integrated IMU poses. If you would like to run FastLIO2 yourself, our public fork is `available here <https://github.com/ethz-asl/fast_lio>`_. Alternatively, we provide rosbags with pre-recorded odometry for the Multi-Cam Cloister, Park, Math-easy and Mine-easy sequences `here <https://drive.google.com/drive/folders/1sTmDBUt97wwE220gVFwCq88JT5IOQlk5>`__.

To run wavemap on the Cloister sequence used in the paper, run::

    roslaunch wavemap_ros newer_college_os0_cloister.launch rosbag_dir:=<path_to_downloaded_dataset_directory>

For additional options, please refer to the launch file's documented arguments
:gh_file:`here <interfaces/ros1/wavemap_ros/launch/datasets/newer_college/newer_college_os0_cloister.launch>`. To experiment with wavemap's configuration, modify :gh_file:`this config file <interfaces/ros1/wavemap_ros/config/wavemap_ouster_os0.yaml>`.

Panoptic mapping dataset
************************
The Panoptic Mapping flat dataset is available `here <https://projects.asl.ethz.ch/datasets/doku.php?id=panoptic_mapping>`__. You can automatically download it using::

    export FLAT_DATA_DIR="/home/$USER/data/panoptic_mapping" # Set to path of your preference
    bash <(curl -s https://raw.githubusercontent.com/ethz-asl/panoptic_mapping/3926396d92f6e3255748ced61f5519c9b102570f/panoptic_mapping_utils/scripts/download_flat_dataset.sh)

To process it with wavemap, run::

    roslaunch wavemap_ros panoptic_mapping_rgbd_flat.launch base_path:="${FLAT_DATA_DIR}"/flat_dataset/run1

To experiment with different wavemap settings, modify :gh_file:`this config file <interfaces/ros1/wavemap_ros/config/wavemap_panoptic_mapping_rgbd.yaml>`.

Multi-sensor live demo
**********************
This section provides instructions to reproduce the interactive multi-sensor, multi-resolution mapping demo we performed at several events, including `RSS 2023 <https://roboticsconference.org/2023/program/papers/065/>`__ and the `Swiss Robotics Day 2023 <https://swissroboticsday.ch/>`__. In this demo, wavemap fuses measurements from a depth camera up to a resolution of 1cm and a LiDAR up to a range of 15m in real-time on a laptop. The odometry is obtained by running FastLIO2 using the LiDAR's pointclouds and built-in IMU.

Hardware
========
Wavemap can be :doc:`configured <parameters/index>` to work with almost any depth input. In case you want to replicate our exact setup, we used a Livox MID-360 LiDAR at RSS and an Ouster OS0-128 at the Swiss Robotics Day. We chose the PMD Pico Monstar depth camera for both demos, but the Azure Kinect also works well. Although any recent laptop will suffice, we recommend using a laptop with a discrete GPU as it will help Rviz run much smoother.

Docker
======
For convenience, the entire demo can be run in Docker using :gh_file:`this Docker image <tooling/docker/ros1/live_demo.Dockerfile>`, which includes the sensor drivers, FastLIO and wavemap. You can build it by running::

    docker build --tag=wavemap_demo --pull - <<< $(curl -s https://raw.githubusercontent.com/ethz-asl/wavemap/main/tooling/docker/ros1/live_demo.Dockerfile)

We provide a convenience script that you can use to run the Docker image. To download it, run::

    curl -o demo_in_docker.sh https://raw.githubusercontent.com/ethz-asl/wavemap/main/tooling/scripts/demo_in_docker.sh
    sudo chmod +x demo_in_docker.sh

Configure
=========
Start by configuring the laptop's network interface to receive the LiDAR data. By default, the Livox we used required the laptop's IP to be set to ``192.168.1.50``, while the Ouster required the laptop's IP to be set to ``192.168.10.50``. In case you're using another LiDAR, the expected receiver IP is usually mentioned in the LiDARs manual or driver documentation.

Next, update the sensor calibrations to match your sensor setup. If you're using the exact sensors we used, you mainly need to update the extrinsics. These are defined through static TFs, at the bottom of the launch file for your sensor setup (e.g. :gh_file:`here <interfaces/ros1/wavemap_ros/launch/online/livox_mid360_pico_monstar.launch#L76>` or :gh_file:`here <interfaces/ros1/wavemap_ros/launch/online/ouster_os0_pico_monstar.launch#L78>`). If you're using different sensors, you will also need to update the wavemap, FastLIO and sensor driver config files. To help you get started quickly, we provide examples for many different sensor setups for :gh_file:`wavemap <interfaces/ros1/wavemap_ros/config>` and :gh_file:`FastLIO <interfaces/ros1/wavemap_ros/config/other_packages/fast_lio>`. In case we do not yet have an example FastLIO config for your LiDAR, you might find one in the `official the FastLIO repository <https://github.com/hku-mars/FAST_LIO>`__.

Note that the ``demo_in_docker.sh`` script saves and restores the `ros/wavemap_ros` folder across runs (explained :gh_file:`here <tooling/scripts/demo_in_docker.sh#L9>`), so you can configure wavemap by simply running ``demo_in_docker.sh`` and editing the launch and config files through the resulting interactive shell.

Run
===
Once the configured, you can directly run the demo by calling the ``demo_in_docker.sh`` script followed by the launch command. For example::

    demo_in_docker.sh roslaunch wavemap_ros ouster_os0_pico_monstar.launch
