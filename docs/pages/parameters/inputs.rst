Inputs
######
.. highlight:: YAML
.. rstcheck: ignore-directives=doxygenstruct

These settings control the measurement input handlers.
They are nested in the top level config under ``inputs``. The ``inputs`` key takes a YAML list, and one input handler will be created for each item in the list.

Note that wavemap currently only provides input handlers for ROS1. Input handlers for additional interfaces are likely to be added in the future.

..
   _TODO: Explain how this can be used as a plugin system.

ROS1 Interface
**************
Each input subscribes to a measurement ROS topic, looks up the pose for each measurement, and then calls the integrators whose names are specified under `measurement_integrator_names` to integrate the measurement into the map.

Pointcloud input handler
========================
Selected by setting ``inputs[i]/type: "pointcloud"``.

The remaining settings are available under ``inputs[i]`` will then be:

.. doxygenstruct:: wavemap::PointcloudTopicInputConfig
    :project: wavemap_ros1
    :members:

Depth image input handler
=========================
Selected by setting ``inputs[i]/type: "depth_image"``.

The remaining settings are available under ``inputs[i]`` will then be:

.. doxygenstruct:: wavemap::DepthImageTopicInputConfig
    :project: wavemap_ros1
    :members:
