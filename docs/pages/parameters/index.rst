Parameters
##########
.. highlight:: YAML
.. rstcheck: ignore-roles=gh_file

Wavemap's configuration system is designed to be flexible and expressive. Let's start with an example to illustrate the overall structure:

.. literalinclude:: ../../../interfaces/ros1/wavemap_ros/config/wavemap_ouster_os0.yaml
    :language: YAML
    :caption: :gh_file:`Example config to use wavemap with an Ouster OS0-128 in ROS1 LiDAR<interfaces/ros1/wavemap_ros/config/wavemap_ouster_os0.yaml>`

The framework's general settings, such as its `logging_level` (verbosity) and the coordinate frame in which the map is represented, are configured under ``general``.
The data structure used to store the volumetric map is configured under ``map``.

Next, a list of ``map_operations`` can be specified that are called after each map update -- possibly throttled to happen at most once every *n* seconds, for efficiency.

The last two sections control how new measurements are added to the map. The ``inputs`` param allows you to specify a list of inputs which take care of receiving depth measurements and looking up their poses, before forwarding them to measurement integrators defined in the ``measurement_integrator`` dictionary. In the above example, we create a single input of ``type: pointcloud_topic`` which subscribes to a pointcloud topic in ROS, looks up the pose of each measurement using ROS' TF2 and then forwards it to a measurement integrator we named `ouster_os0`.

You might have noticed two peculiarities in wavemap's configs.
The first is that subsections often contain a **type** tag. This tag determines the type of the object that should be created for the corresponding element. For example, setting ``map/type: hashed_chunked_wavelet_octree`` tells wavemap to create a *HashedChunkedWaveletOctree* data structure to store the map.
The second is that **units** are specified explicitly. This eliminates misunderstandings and enables automatic conversions from derived units, such as ``{ millimeters: 27.67 }`` or ``{ degrees: 180.0 }``, to SI base units, such as ``{ meters: 0.02767 }`` or ``{ radians: 3.14159 }``. Wavemap's code internally always operates in SI base units.

.. tip::

    Wavemap's ROS server prints warnings for all unrecognized params at startup. This can be helpful for debugging and to quickly find typos in config param names.

To get started quickly, we recommend skimming through these :gh_file:`example configs <interfaces/ros1/wavemap_ros/config>`.
For reference, an overview of all available config options is provided below.

.. toctree::
    :caption: Parameter sets
    :maxdepth: 1

    general
    map
    map_operations
    measurement_integrators
    inputs
