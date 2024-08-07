Inputs
######
.. highlight:: YAML
.. rstcheck: ignore-directives=doxygenstruct

The settings in the config's ``inputs`` section are used to add measurement inputs to wavemap, similar to dataloaders in a machine learning framework. The expected format is a YAML list, where each item corresponds to a separate measurement input handler.

C++ Library
***********
We believe that input handlers for standard datasets like KITTI, Redwood, 3DMatch, Replica, and others would be valuable additions to wavemap's C++ library. However, these have not been implemented yet. If you're interested in contributing, please reach out to us `here <https://github.com/ethz-asl/wavemap/issues>`_.

ROS1 Interface
**************
Each input subscribes to a measurement ROS topic, retrieves the pose for each received measurement using `TF2`, and then calls the integrators specified under `measurement_integrator_names` to add the measurement to the map.

Pointcloud input handler
========================
Selected by setting ``inputs[i]/type`` to ``"pointcloud_topic"``.

The remaining settings are available under ``inputs[i]`` will then be:

.. doxygenstruct:: wavemap::PointcloudTopicInputConfig
    :project: wavemap_ros1
    :members:

Depth image input handler
=========================
Selected by setting ``inputs[i]/type`` to ``"depth_image_topic"``.

The remaining settings are available under ``inputs[i]`` will then be:

.. doxygenstruct:: wavemap::DepthImageTopicInputConfig
    :project: wavemap_ros1
    :members:
