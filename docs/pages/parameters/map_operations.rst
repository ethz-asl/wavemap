Map operations
##############
.. rstcheck: ignore-directives=doxygenstruct

The settings in the config's ``map_operations`` section are used to add operations that will be applied to wavemap's map. The expected format is a YAML list, where each item represents a separate map operation. These operations function like plugins, receiving a callback each time the map is updated by a new measurement. For efficiency, most plugins can be throttled to run at most once every *n* seconds.

C++ Library
***********
This subsection describes the operations available in wavemap's C++ library.

Threshold map
=============
Selected by setting ``map_operations[i]/type`` to ``"threshold_map"``.

This operation thresholds the map's occupancy values to stay within the range specified by `min_log_odds` and `max_log_odds`, which are configured under ``map``.

.. doxygenstruct:: wavemap::ThresholdMapOperationConfig
    :project: wavemap_cpp
    :members:

Prune map
=========
Selected by setting ``map_operations[i]/type`` to ``"prune_map"``.

This operation frees up memory by pruning octree nodes that are no longer needed -- specifically, nodes whose wavelet coefficients are all zero and which have no children. Note that this pruning operation is lossless and does not alter the estimated occupancy posterior.

.. doxygenstruct:: wavemap::PruneMapOperationConfig
    :project: wavemap_cpp
    :members:

ROS1 Interface
**************
In addition to the map operations provided by the C++ library, wavemap's ROS1 interface offers several ROS-specific operations.

Publish map
===========
Selected by setting ``map_operations[i]/type`` to ``"publish_map"``.

This operation publishes the current map to a ROS topic, allowing other ROS nodes to access and utilize the map's data.

.. doxygenstruct:: wavemap::PublishMapOperationConfig
    :project: wavemap_ros1
    :members:

Publish pointcloud
==================
Selected by setting ``map_operations[i]/type`` to ``"publish_pointcloud"``.

This operation publishes a pointcloud representation of the map to a ROS topic, enabling processing by ROS nodes without requiring wavemap's C++ library.

.. doxygenstruct:: wavemap::PublishPointcloudOperationConfig
    :project: wavemap_ros1
    :members:

Crop map
========
Selected by setting ``map_operations[i]/type`` to ``"crop_map"``.

This operation crops the map to a specified radius around the robot, to enable local mapping.

.. doxygenstruct:: wavemap::CropMapOperationConfig
    :project: wavemap_ros1
    :members:
