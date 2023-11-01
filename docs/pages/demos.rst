Demos
#####
.. highlight:: bash
.. include:: <isonum.txt>
.. rstcheck: ignore-roles=gh_file

Quick start
***********
To get an impression of the maps wavemap can generate, you can download the maps of our `RSS paper's <https://www.roboticsproceedings.org/rss19/p065.pdf>`__ and directly view them in Rviz. To do so,

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
:gh_file:`here <ros/wavemap_ros/launch/datasets/newer_college/newer_college_os0_cloister.launch>`. To experiment with wavemap's configuration, modify :gh_file:`this config file <ros/wavemap_ros/config/ouster_os0.yaml>`.

Panoptic mapping dataset
************************
The Panoptic Mapping flat dataset is available `here <https://projects.asl.ethz.ch/datasets/doku.php?id=panoptic_mapping>`__. You can automatically download it using::

    export FLAT_DATA_DIR="/home/$USER/data/panoptic_mapping" # Set to path of your preference
    bash <(curl -s https://raw.githubusercontent.com/ethz-asl/panoptic_mapping/3926396d92f6e3255748ced61f5519c9b102570f/panoptic_mapping_utils/scripts/download_flat_dataset.sh)

To process it with wavemap, run::

    roslaunch wavemap_ros panoptic_mapping_rgbd_flat.launch base_path:="${FLAT_DATA_DIR}"/flat_dataset/run1

To experiment with different wavemap settings, modify :gh_file:`this config file <ros/wavemap_ros/config/panoptic_mapping_rgbd.yaml>`.

Your own data
*************
The only requirements for running wavemap are:

1. a source of depth measurements,
2. sensor pose (estimates) for each measurement.

We usually use depth measurements from depth cameras or 3D LiDARs, but any source would work as long as a sufficiently good :ref:`projection <configuration_projection_models>` and :ref:`measurement <configuration_measurement_models>` model is available. Wavemap's ROS interface natively supports depth image and pointcloud inputs, and automatically queries the sensor poses from the TF tree.

To help you get started quickly, we provide various example :gh_file:`config <ros/wavemap_ros/config>` and ROS :gh_file:`launch <ros/wavemap_ros/launch>` files.
For a brief introduction on how to edit wavemap configs and an overview of all the available settings, see the :doc:`configuration page <configuration>`.
