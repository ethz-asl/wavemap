ROS1 Interface
##############
.. rstcheck: ignore-roles=gh_file

On the :doc:`demos page <../demos>`, we saw several examples of how to run wavemap on existing datasets. In the sections that follow, we'll first discuss how the ROS packages that are included in wavemap's ROS1 interface can be configured to process your own sensor data. We'll then explain how you can interact with wavemaps in your own ROS nodes to enable new, custom use cases.

Your own data
*************
The only requirements to run wavemap on your own dataset or robot are:

1. a source of depth measurements,
2. sensor pose (estimates) for each measurement.

We usually use depth measurements from depth cameras or 3D LiDARs, but any source would work as long as a sufficiently good :ref:`projection <configuration_projection_models>` and :ref:`measurement <configuration_measurement_models>` model is available. Wavemap's ROS interface natively supports depth image and pointcloud inputs, and automatically queries the sensor poses from the TF tree.

To help you get started quickly, we provide example :gh_file:`config <ros/wavemap_ros/config>` and ROS :gh_file:`launch <ros/wavemap_ros/launch>` files for various sensor setups and use cases. An overview of all the available settings is provided on the :doc:`configuration page <../configuration/index>`.

Your own code
*************

Since wavemap's ROS1 interface extends its C++ API, all of the :doc:`C++ API's usage examples <cpp>` can directly be used in ROS.

The only code required to receive maps over a ROS topic in your own ROS node is:

.. literalinclude:: ../../../examples/ros1/io/receive_map_over_ros.cc
    :language: c++

To send a map, the following code can be used:

.. literalinclude:: ../../../examples/ros1/io/send_map_over_ros.cc
    :language: c++

Note that an example catkin package that includes the above code is provided :gh_file:`here <examples/ros1>`.
