Configuration
#############
.. highlight:: YAML
.. rstcheck: ignore-directives=doxygenstruct
.. rstcheck: ignore-roles=gh_file

Wavemap's configuration system is designed to be flexible and expressive. Let's start with an example to illustrate the overall structure:

.. literalinclude:: ../../ros/wavemap_ros/config/wavemap_ouster_os0.yaml
    :language: YAML
    :caption: :gh_file:`Example config to use wavemap with an Ouster OS0-128 LiDAR<ros/wavemap_ros/config/wavemap_ouster_os0.yaml>`

The map's general settings, such as the coordinate frame in which it's represented and the rate at which it's published, are configured under ``map/general``.
The data structure used to store the volumetric map is configured under ``map/data_structure``.
Finally, a list of inputs can be specified.

You might have noticed two peculiarities in wavemap's configs.
The first is that subsections often feature a **type** tag. This tag determines the type of the object that should be created for the corresponding element. For example, setting ``data_structure/type: hashed_chunked_wavelet_octree`` tells wavemap to create a *HashedChunkedWaveletOctree* data structure to store the map.
The second is that **units** are specified explicitly. This eliminates misunderstandings and enables automatic conversions from derived units, such as ``{ millimeters: 27.67 }`` or ``{ degrees: 180.0 }``, to SI base units, such as ``{ meters: 0.02767 }`` or ``{ radians: 3.14159 }``. Wavemap's code internally always operates in SI base units.

Note that the wavemap ROS server prints warnings for all unrecognized params at startup. This can be helpful for debugging and to quickly find typos in config param names.

To get started quickly, we recommend skimming through these :gh_file:`example configs <ros/wavemap_ros/config>`.
For reference, an overview of all available config options is provided below.

.. toctree::
    :caption: Full options
    :maxdepth: 2

    general
    map
    map_operations
    measurement_integrators
    inputs
