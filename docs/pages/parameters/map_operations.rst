Map operations
##############
.. rstcheck: ignore-directives=doxygenstruct

..
   _TODO: Explain how this can be used as a plugin system.

C++ Library
***********
Selected by setting ``map_operations[i]/type: "threshold_map"``.

.. doxygenstruct:: wavemap::ThresholdMapOperationConfig
    :project: wavemap_cpp
    :members:

Selected by setting ``map_operations[i]/type: "prune_map"``.

.. doxygenstruct:: wavemap::PruneMapOperationConfig
    :project: wavemap_cpp
    :members:

ROS1 Interface
**************
Selected by setting ``map_operations[i]/type: "publish_map"``.

.. doxygenstruct:: wavemap::PublishMapOperationConfig
    :project: wavemap_ros1
    :members:

Selected by setting ``map_operations[i]/type: "publish_pointcloud"``.

.. doxygenstruct:: wavemap::PublishPointcloudOperationConfig
    :project: wavemap_ros1
    :members:

Selected by setting ``map_operations[i]/type: "crop_map"``.

.. doxygenstruct:: wavemap::CropMapOperationConfig
    :project: wavemap_ros1
    :members:
