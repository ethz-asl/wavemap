Measurement integrators
#######################
.. rstcheck: ignore-directives=doxygenstruct

The settings in the config's ``measurement_integrators`` section are used to add measurement integrators to wavemap. The expected format is a YAML dictionary, where each item represents a separate measurement integrator. The item's key defines the integrator's name, while its value specifies the integrator's settings. Note that the integrator's name can be chosen freely. It will later be used in the ``inputs`` section to direct measurements to specific integrators.

In most cases, a single integrator per sensor type is sufficient. However, the system also allows you to forward a single input to multiple integrators. This can be useful, for example, to integrate data at a very high resolution close to the robot (e.g., 1 to 3 meters) with one integrator, while using another integrator for lower resolution updates over a longer range (e.g., 3 to 30 meters).

For each integrator, you need to specify the ``projection_model``, ``measurement_model`` and ``integration_method``.

.. _configuration_projection_models:

Projection models
*****************
These settings control the projection model that is used to convert coordinates in the sensor's frame to/from cartesian coordinates.
They are nested in the top level config under ``measurement_integrators["your_integrator"]/projection_model``.

Spherical projection model
==========================
Selected by setting ``projection_model/type`` to ``'spherical_projector'``.

This projection model is appropriate for most LiDARs, including Velodynes. It also works with dome LiDARs such as the Robosense RS-Bpearl.

The remaining settings available under ``projection_model`` will then be:

.. doxygenstruct:: wavemap::SphericalProjectorConfig
    :project: wavemap_cpp
    :members:

Ouster LiDAR projection model
=============================
Selected by setting ``projection_model/type`` to ``'ouster_projector'``.

This projection model improves the reconstruction accuracy for Ouster based LiDARs including the OS0 and OS1, by taking their staggered beam pattern into account.

The remaining settings available under ``projection_model`` will then be:

.. doxygenstruct:: wavemap::OusterProjectorConfig
    :project: wavemap_cpp
    :members:

Pinhole camera projection model
===============================
Selected by setting ``projection_model/type`` to ``'pinhole_camera_projector'``.

This projection model is appropriate for most depth cameras.

The remaining settings available under ``projection_model`` will then be:

.. doxygenstruct:: wavemap::PinholeCameraProjectorConfig
    :project: wavemap_cpp
    :members:

.. _configuration_measurement_models:

Measurement models
******************
These settings control the measurement model that is used to convert measurements into corresponding occupancy updates.
They are nested in the top level config under ``measurement_integrators["your_integrator"]/measurement_model``.

Continuous ray measurement model
================================
Selected by setting ``measurement_model/type`` to ``'continuous_ray'``.

This measurement model accounts for uncertainty along each measured ray, but does not include angular uncertainty. It is appropriate for high density sensors with a limited range, such as most depth cameras. Considering angular uncertainty usually does not significantly improve reconstruction accuracy when the rays densely cover the entire observed volume, in which case the computational overhead is unnecessary.

The remaining settings available under ``measurement_model`` will then be:

.. doxygenstruct:: wavemap::ContinuousRayConfig
    :project: wavemap_cpp
    :members:

Continuous beam measurement model
=================================
Selected by setting ``measurement_model/type`` to ``'continuous_beam'``.

This measurement model extends the continuous ray model, by including angular uncertainty. For LiDAR sensors, whose ray density is low at long range, it significantly improves the reconstruction quality and recall on thin objects.

The remaining settings available under ``measurement_model`` will then be:

.. doxygenstruct:: wavemap::ContinuousBeamConfig
    :project: wavemap_cpp
    :members:

Integration method
******************
These settings control the algorithm that is used to apply measurement updates to the map.
They are nested in the top level config under ``measurement_integrators["your_integrator"]/integration_method``.

The following map data structure and integration method types are compatible:

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

The integration methods support the following settings:

.. doxygenstruct:: wavemap::ProjectiveIntegratorConfig
    :project: wavemap_cpp
    :members:
