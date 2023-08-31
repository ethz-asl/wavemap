Configuration
#############
.. highlight:: YAML
.. rstcheck: ignore-directives=doxygenstruct
.. rstcheck: ignore-roles=repo_file

Wavemap's configuration system is designed to be flexible and expressive. Let's start with an example to illustrate the overall structure:

.. literalinclude:: ../../ros/wavemap_ros/config/wavemap_ouster_os0.yaml
    :language: YAML
    :caption: :repo_file:`Example config to use wavemap with an Ouster OS0-128 LiDAR<ros/wavemap_ros/config/wavemap_ouster_os0.yaml>`

The map's general settings, such as the coordinate frame in which it's represented and the rate at which its published, are configured under ``map/general``.
The data structure used to store the volumetric map is configured under ``map/data_structure``.
Finally, a list of inputs can be specified.

You might have noticed two peculiarities in wavemap's configs.
The first is that subsections often feature a **type** tag. This tag determines the type of the object that should be created for the corresponding element. For example, setting ``data_structure/type: hashed_chunked_wavelet_octree`` tells wavemap to create a *HashedChunkedWaveletOctree* data structure to store the map.
The second is that **units** are specified explicitly. This eliminates misunderstandings and enables automatic conversions from derived units, such as ``{ millimeters: 27.67 }`` or ``{ degrees: 180.0 }``, to SI base units, such as ``{ meters: 0.02767 }`` or ``{ radians: 3.14159 }``. Wavemap's code internally always operates in SI base units.

Note that the wavemap ROS server prints warnings for all unrecognized params at startup. This can be helpful for debugging and to quickly find typos in config param names.

To get started quickly, we recommend skimming through these :repo_file:`example configs <ros/wavemap_ros/config>`.
For reference, an overview of all available config options is provided below.

ROS server
**********
These settings control the general behavior of the ROS server.
They are nested in the top level config under ``map/general``.

.. doxygenstruct:: wavemap::WavemapServerConfig
    :project: wavemap
    :members:

Map data structures
*******************
These settings control the data structure that is used to store the map.
They are nested in the top level config under ``map/data_structure``.

The following three data structures are fully supported by wavemap's ROS server: ``wavelet_octree``, ``hashed_wavelet_octree`` or ``hashed_chunked_wavelet_octree``. For general use, we recommend the ``hashed_wavelet_octree`` data structure. If you need the best possible performance, the ``hashed_chunked_wavelet_octree`` data structure is faster and uses less RAM. However, it is still under active development.

Wavelet octree
==============
Selected by setting ``map/data_structure/type: "wavelet_octree"``.

The ``wavelet_octree`` is the simplest of the wavelet-based data structures and stores the wavelet coefficients in a standard octree. This data structure can be useful in case you need the entire map to be contained in a single tree and don't mind sacrificing performance.

.. doxygenstruct:: wavemap::WaveletOctreeConfig
    :project: wavemap
    :members:

Hashed wavelet octree
=====================
Selected by setting ``map/data_structure/type: "hashed_wavelet_octree"``.

The ``hashed_wavelet_octree`` combines the strengths of wavelet octrees with block-hashing. At the top level, the map is split into blocks which are accessed through a hash table. Each block in turn contains a small wavelet octree.

.. doxygenstruct:: wavemap::HashedWaveletOctreeConfig
    :project: wavemap
    :members:

Hashed chunked wavelet octree
=============================
Selected by setting ``map/data_structure/type: "hashed_chunked_wavelet_octree"``.

The ``hashed_chunked_wavelet_octree`` is similar to the ``hashed_wavelet_octree``, but instead of storing all octree nodes separately it groups them into chunks.

.. doxygenstruct:: wavemap::HashedChunkedWaveletOctreeConfig
    :project: wavemap
    :members:


Measurement inputs
******************
These settings control the measurement input handlers.
They are nested in the top level config under ``inputs``. Note that ``inputs`` takes a YAML list, and one input handler will be created for each item in the list.

Each input subscribes to a measurement ROS topic, looks up the pose for each measurement, and then calls its integrators to integrate the measurement into the map. We cover the measurement input handler types and their general settings first, followed by the integrator settings.

Pointcloud input handler
========================
Selected by setting ``inputs[i]/type: "pointcloud"``.

The settings are available under ``inputs[i]/general`` are:

.. doxygenstruct:: wavemap::PointcloudInputHandlerConfig
    :project: wavemap
    :members:

Depth image input handler
=========================
Selected by setting ``inputs[i]/type: "depth_image"``.

The settings are available under ``inputs[i]/general`` are:

.. doxygenstruct:: wavemap::DepthImageInputHandlerConfig
    :project: wavemap
    :members:

Measurement integrators
=======================
These settings specify which integrators will be applied whenever the input they belong to receives a new measurement. In similar fashion to the ``inputs`` key, ``inputs[i]/integrators`` takes a YAML list, and one integrator will be created for each item in the list. In most cases, a single integrator will suffice. However, the option of specifying multiple integrators can, for example, be used to integrate a given input at a very high resolution close to the robot (e.g. 1 to 3m) with one integrator, and at a lower resolution over a long range (e.g. 3 to 30m) with another integrator.

For each integrator, you need to specify the ``integration_method``, ``projection_model`` and ``measurement_model``.

Integration method
------------------
These settings control the algorithm that is used to apply measurement updates to the map.
They are nested in the top level config under ``inputs[i]/integrators[j]/integration_method``.

Compatible map data structure and integration method types:

.. list-table::
   :widths: 50 50
   :header-rows: 1

   * - Data structure
     - Integration method
   * - ``wavelet_octree``
     - ``wavelet_integrator``
   * - ``hashed_wavelet_octree``
     - ``hashed_wavelet_integrator``
   * - ``hashed_chunked_wavelet_octree``
     - ``hashed_chunked_wavelet_integrator``

They support the following settings:

.. doxygenstruct:: wavemap::ProjectiveIntegratorConfig
    :project: wavemap
    :members:

.. _configuration_projection_models:

Projection models
-----------------
These settings control the projection model that is used to convert cartesian coordinates to/from sensor coordinates.
They are nested in the top level config under ``inputs[i]/integrators[j]/projection_model``.

Spherical projection model
^^^^^^^^^^^^^^^^^^^^^^^^^^
Selected by setting ``inputs[i]/integrators[j]/projection_model/type: 'spherical_projector'``.

This projection model is appropriate for most LiDARs, including Velodynes. It also works with dome LiDARs such as the Robosense RS-Bpearl.

The remaining settings available under ``inputs[i]/integrators[j]/projection_model`` are:

.. doxygenstruct:: wavemap::SphericalProjectorConfig
    :project: wavemap
    :members:

Ouster LiDAR projection model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Selected by setting ``inputs[i]/integrators[j]/projection_model/type: 'ouster_projector'``.

This projection model improves the reconstruction accuracy for Ouster based LiDARs including the OS0 and OS1, by taking their staggered beam pattern into account.

The remaining settings available under ``inputs[i]/integrators[j]/projection_model`` are:

.. doxygenstruct:: wavemap::OusterProjectorConfig
    :project: wavemap
    :members:

Pinhole camera projection model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Selected by setting ``inputs[i]/integrators[j]/projection_model/type: 'pinhole_camera_projector'``.

This projection model is appropriate for most depth cameras.

The remaining settings available under ``inputs[i]/integrators[j]/projection_model`` are:

.. doxygenstruct:: wavemap::PinholeCameraProjectorConfig
    :project: wavemap
    :members:

.. _configuration_measurement_models:

Measurement models
------------------
These settings control the measurement model that is used to convert measurements into corresponding occupancy updates.
They are nested in the top level config under ``inputs[i]/integrators[j]/measurement_model``.

Continuous ray measurement model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Selected by setting ``inputs[i]/integrators[j]/measurement_model/type: 'continuous_ray'``.

This measurement model accounts for uncertainty along each measured ray, but does not include angular uncertainty. It is appropriate for high density sensors with a limited range, such as most depth cameras. Considering angular uncertainty usually does not significantly improve reconstruction accuracy when the rays densely cover the entire observed volume, in which case the computational overhead is unnecessary.

The remaining settings available under ``inputs[i]/integrators[j]/measurement_model`` are:

.. doxygenstruct:: wavemap::ContinuousRayConfig
    :project: wavemap
    :members:

Continuous beam measurement model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Selected by setting ``inputs[i]/integrators[j]/measurement_model/type: 'continuous_beam'``.

This measurement model extends the continuous ray model, by including angular uncertainty. For LiDAR sensors, whose ray density is low at long range, it significantly improves the reconstruction quality and recall on thin objects.

The remaining settings available under ``inputs[i]/integrators[j]/measurement_model`` are:

.. doxygenstruct:: wavemap::ContinuousBeamConfig
    :project: wavemap
    :members:
