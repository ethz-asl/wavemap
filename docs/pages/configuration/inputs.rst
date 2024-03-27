Inputs
######
.. highlight:: YAML
.. rstcheck: ignore-directives=doxygenstruct

These settings control the measurement input handlers.
They are nested in the top level config under ``inputs``. Note that ``inputs`` takes a YAML list, and one input handler will be created for each item in the list.

Each input subscribes to a measurement ROS topic, looks up the pose for each measurement, and then calls its integrators to integrate the measurement into the map. We cover the measurement input handler types and their general settings first, followed by the integrator settings.

Input handlers
**************

Pointcloud input handler
========================
Selected by setting ``inputs[i]/type: "pointcloud"``.

The settings are available under ``inputs[i]/general`` are:

.. doxygenstruct:: wavemap::PointcloudInputConfig
    :project: wavemap
    :members:

Depth image input handler
=========================
Selected by setting ``inputs[i]/type: "depth_image"``.

The settings are available under ``inputs[i]/general`` are:

.. doxygenstruct:: wavemap::DepthImageInputConfig
    :project: wavemap
    :members:

Measurement integrators
***********************
These settings specify which integrators will be applied whenever the input they belong to receives a new measurement. In similar fashion to the ``inputs`` key, ``inputs[i]/integrators`` takes a YAML list, and one integrator will be created for each item in the list. In most cases, a single integrator will suffice. However, the option of specifying multiple integrators can, for example, be used to integrate a given input at a very high resolution close to the robot (e.g. 1 to 3m) with one integrator, and at a lower resolution over a long range (e.g. 3 to 30m) with another integrator.

For each integrator, you need to specify the ``integration_method``, ``projection_model`` and ``measurement_model``.

Integration method
==================
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
=================
These settings control the projection model that is used to convert cartesian coordinates to/from sensor coordinates.
They are nested in the top level config under ``inputs[i]/integrators[j]/projection_model``.

Spherical projection model
--------------------------
Selected by setting ``inputs[i]/integrators[j]/projection_model/type: 'spherical_projector'``.

This projection model is appropriate for most LiDARs, including Velodynes. It also works with dome LiDARs such as the Robosense RS-Bpearl.

The remaining settings available under ``inputs[i]/integrators[j]/projection_model`` are:

.. doxygenstruct:: wavemap::SphericalProjectorConfig
    :project: wavemap
    :members:

Ouster LiDAR projection model
-----------------------------
Selected by setting ``inputs[i]/integrators[j]/projection_model/type: 'ouster_projector'``.

This projection model improves the reconstruction accuracy for Ouster based LiDARs including the OS0 and OS1, by taking their staggered beam pattern into account.

The remaining settings available under ``inputs[i]/integrators[j]/projection_model`` are:

.. doxygenstruct:: wavemap::OusterProjectorConfig
    :project: wavemap
    :members:

Pinhole camera projection model
-------------------------------
Selected by setting ``inputs[i]/integrators[j]/projection_model/type: 'pinhole_camera_projector'``.

This projection model is appropriate for most depth cameras.

The remaining settings available under ``inputs[i]/integrators[j]/projection_model`` are:

.. doxygenstruct:: wavemap::PinholeCameraProjectorConfig
    :project: wavemap
    :members:

.. _configuration_measurement_models:

Measurement models
==================
These settings control the measurement model that is used to convert measurements into corresponding occupancy updates.
They are nested in the top level config under ``inputs[i]/integrators[j]/measurement_model``.

Continuous ray measurement model
--------------------------------
Selected by setting ``inputs[i]/integrators[j]/measurement_model/type: 'continuous_ray'``.

This measurement model accounts for uncertainty along each measured ray, but does not include angular uncertainty. It is appropriate for high density sensors with a limited range, such as most depth cameras. Considering angular uncertainty usually does not significantly improve reconstruction accuracy when the rays densely cover the entire observed volume, in which case the computational overhead is unnecessary.

The remaining settings available under ``inputs[i]/integrators[j]/measurement_model`` are:

.. doxygenstruct:: wavemap::ContinuousRayConfig
    :project: wavemap
    :members:

Continuous beam measurement model
---------------------------------
Selected by setting ``inputs[i]/integrators[j]/measurement_model/type: 'continuous_beam'``.

This measurement model extends the continuous ray model, by including angular uncertainty. For LiDAR sensors, whose ray density is low at long range, it significantly improves the reconstruction quality and recall on thin objects.

The remaining settings available under ``inputs[i]/integrators[j]/measurement_model`` are:

.. doxygenstruct:: wavemap::ContinuousBeamConfig
    :project: wavemap
    :members:
