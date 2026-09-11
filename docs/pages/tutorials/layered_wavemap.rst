.. _layered-wavemap-tutorial:

Layered Wavemap
###############

Layered Wavemap extends the original occupancy map with environmental
attributes that use the same spatial coordinates. This guide takes you from a
fresh checkout to a running RGB example, then shows how to use real point-cloud
fields and define a new layer.

What is stored
**************

Every layered map owns two related forms of storage:

* Occupancy and wavelet-compatible continuous states share one wavelet octree.
  Examples are reflectivity, signal strength, near-infrared intensity, and RGB.
* Categorical or finite-state values use sparse side layers aligned to the
  minimum-resolution voxel indices. The current discrete backend stores a
  dominant value per block, observed offsets, and exact exceptions.

Occupancy always remains part of the continuous voxel and is not listed in a
user schema. Continuous and discrete describe the mathematical operations that
are valid for a value. An integer class ID is discrete because averaging class
IDs has no useful meaning.

.. code-block:: text

    posed point cloud
      +-- XYZ ----------------> original Wavemap occupancy integrator
      +-- point attributes ---> layer update policy
                                  +-- continuous: shared wavelet voxel
                                  +-- discrete: aligned side layer

Requirements
************

The supported ROS setup is Ubuntu 20.04 with ROS Noetic. Docker is the fastest
way to try the project when that environment is not already installed. Native
installation is more convenient while changing and rebuilding the code.

Docker quick start
==================

Install Docker and configure it for use without ``sudo``. Then clone this
repository and build the ROS image from the local checkout:

.. code-block:: bash

    git clone https://github.com/GuilhermeCabaco/wavemap-digital-twin.git
    cd wavemap-digital-twin
    docker build --tag=wavemap_ros1 \
      --file tooling/docker/ros1/full.Dockerfile .

The full Dockerfile copies the current checkout into a Catkin workspace,
installs its dependencies, and builds ``wavemap_all``. Downloading the upstream
Wavemap image alone does not include this repository's layered extensions.

The supplied runner forwards an X11 display for RViz. It expects a host data
directory at ``/home/$USER/data``:

.. code-block:: bash

    mkdir -p /home/$USER/data
    chmod +x tooling/scripts/run_in_docker.sh
    ./tooling/scripts/run_in_docker.sh \
      roslaunch wavemap_ros synthetic_rgb_layered_map.launch

The runner is intended for Linux/X11. Other display systems require the Docker
GUI forwarding appropriate to that platform.

Native ROS Noetic installation
==============================

Install ROS Noetic, followed by the build tools:

.. code-block:: bash

    sudo apt update
    sudo apt install git build-essential python3-rosdep python3-catkin-tools

Create and configure a workspace:

.. code-block:: bash

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    source /opt/ros/noetic/setup.bash
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

Clone this repository into the source space:

.. code-block:: bash

    cd ~/catkin_ws/src
    git clone https://github.com/GuilhermeCabaco/wavemap-digital-twin.git wavemap

Initialize rosdep once, install dependencies, and build:

.. code-block:: bash

    sudo rosdep init
    rosdep update
    cd ~/catkin_ws
    rosdep install -y --from-paths src --ignore-src
    catkin build wavemap_all
    source devel/setup.bash

If rosdep is already initialized, continue with ``rosdep update``. Source
``devel/setup.bash`` in each new terminal.

Run the dataset-free RGB demo
*****************************

The synthetic demo constructs a small colored forest, publishes it as a
layered map, and opens RViz:

.. code-block:: bash

    source ~/catkin_ws/devel/setup.bash
    roslaunch wavemap_ros synthetic_rgb_layered_map.launch

No bag, sensor driver, or localization system is needed. RViz subscribes to
``/synthetic_rgb_layered_map/layered_map``. Use
``Render voxels / Layer`` to switch between ``rgb`` and ``occupancy``. The node
prints a completion message after inserting all voxels.

Control the animation with:

.. code-block:: bash

    roslaunch wavemap_ros synthetic_rgb_layered_map.launch \
      voxels_per_update:=300 update_period:=0.03

Save the completed map with:

.. code-block:: bash

    roslaunch wavemap_ros synthetic_rgb_layered_map.launch \
      save_path:=$HOME/synthetic_rgb.lwvmp

After the completion message, stop the process with Ctrl-C. To inspect the file
later, add a ``LayeredMap`` display in RViz, change ``Source`` from ``Topic`` to
``File``, and select ``Load map from disk``. The built-in RGB schema handler
recognizes the saved file.

Run a layered server with Ouster data
*************************************

The included Ouster applications expect:

* Point clouds on ``/ouster/points``.
* A valid TF transform from each cloud frame to the configured map frame.
* ``reflectivity`` values between 0 and 255.
* ``intensity`` and ``ambient`` values between 0 and 65535 when selected.

The all-layer application stores occupancy, reflectivity, signal, near
infrared, and a geometrically derived ground/obstacle class:

.. code-block:: bash

    source ~/catkin_ws/devel/setup.bash
    rosparam load \
      $(rospack find wavemap_ros)/config/wavemap_ouster_all_layers_demo.yaml \
      /wavemap
    rosrun wavemap_ros signal_near_ir_class_layered_server __name:=wavemap

In another terminal, run the sensor, localization, or bag playback that
publishes the cloud and TF transforms. Open the supplied RViz configuration:

.. code-block:: bash

    source ~/catkin_ws/devel/setup.bash
    rviz -d $(rospack find wavemap_ros)/config/rviz/layered_map.rviz

Choose ``occupancy``, ``reflectivity``, ``signal``, ``near_ir``, or ``class``
from the display's Layer property. Class 1 is ground and class 2 is obstacle.
This is an example based on local elevation, not a learned semantic model.

Available applications
======================

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Executable
     - Additional layers
   * - ``occupancy_layered_server``
     - None; exercises the layered lifecycle
   * - ``reflectivity_layered_server``
     - reflectivity
   * - ``reflectivity_class_layered_server``
     - reflectivity and class
   * - ``signal_layered_server``
     - reflectivity and signal
   * - ``signal_near_ir_layered_server``
     - reflectivity, signal, and near infrared
   * - ``signal_class_layered_server``
     - reflectivity, signal, and class
   * - ``signal_near_ir_class_layered_server``
     - reflectivity, signal, near infrared, and class

Hashed and chunked versions also exist for the occupancy and reflectivity
examples. Their source is under
``interfaces/ros1/wavemap_ros/app/ouster_layered``.

Save and restore layered maps
*****************************

A layered server named ``/wavemap`` exposes these services:

.. code-block:: bash

    rosservice call /wavemap/save_layered_map \
      "file_path: '$HOME/forest_map.lwvmp'"

    rosservice call /wavemap/load_layered_map \
      "file_path: '$HOME/forest_map.lwvmp'"

    rosservice call /wavemap/layered_map_request_full

The ``.lwvmp`` format stores its version, ordered layer schema, typed continuous
wavelet map, and compressed discrete side layers. Loading rejects incompatible
schemas and map geometry. Start the same typed server used to create the file
before calling ``load_layered_map``. A successful load publishes the full map.

Define and configure a layer
****************************

A layer is a compile-time type. This scalar example uses exponential smoothing
with an observation weight of 0.2:

.. code-block:: c++

    using ReflectivityPolicy =
        wavemap::layered::ExponentialScalarLayerUpdatePolicy<1, 5>;

    struct ReflectivityLayer
        : wavemap::layered::schema::ContinuousLayer<
              float, ReflectivityPolicy> {
      static constexpr std::string_view name = "reflectivity";
      static constexpr wavemap::layered::ScalarLayerVisualization
          visualization{{0.05f, 0.05f, 0.05f}, {1.f, 1.f, 1.f}};
    };

A class layer uses discrete storage because arithmetic on identifiers would not
preserve their meaning:

.. code-block:: c++

    struct ClassLayer
        : wavemap::layered::schema::DiscreteLayer<
              int, wavemap::layered::ReplaceLayerUpdatePolicy<int>> {
      static constexpr std::string_view name = "class";
      static constexpr std::array categories{
          wavemap::layered::IntegerCategoryLabel{
              1, "Ground", {0.2f, 0.8f, 0.2f}},
          wavemap::layered::IntegerCategoryLabel{
              2, "Obstacle", {0.9f, 0.2f, 0.15f}}};
    };

Combine the tags into a schema and derive the map type:

.. code-block:: c++

    using Schema = wavemap::layered::schema::LayerSchema<
        ReflectivityLayer, ClassLayer>;
    using Definition = wavemap::layered::LayeredMapDefinition<Schema>;

Occupancy is included automatically. Pass
``wavemap::layered::HashedChunkedWaveletOctreeBackend`` as the second
``LayeredMapDefinition`` template argument to select the chunked backend.

Configure map geometry, pruning, bounds, and discrete grouping:

.. code-block:: c++

    wavemap::layered::LayeredMapConfigBuilder<Definition> config;
    config.map()
        .minCellWidth(0.25f)
        .occupancyLogOdds(-2.f, 4.f)
        .treeHeight(7)
        .onlyPruneBlocksIfUnusedFor(5.f);
    config.occupancy().pruningScale(1e-3f).pruningWeight(1.f);
    config.layer<ReflectivityLayer>()
        .storageBounds(0.f, 1.f)
        .pruningScale(1e-3f)
        .pruningWeight(1.f);
    config.discreteBlockHeight(2);
    config.combinedPruningThreshold(1.f);

A discrete block height of 2 groups 4 x 4 x 4 minimum-resolution positions. It
changes compression grouping, not map resolution.

Choose an update policy
=======================

Reusable policies live in
``library/cpp/include/wavemap/layered/integration/layer_update_policy.h``. They
include replacement, minimum, maximum, accumulation, exponential smoothing,
confidence blending, logical operations, and a stateful weighted mean.

For ordinary layers, Value and State are identical. Use
``StatefulContinuousLayer`` when observations need a richer stored state. A
custom wavelet-compatible state must provide:

* ``ContinuousValueTraits`` operations for addition, subtraction, scaling,
  thresholding, and magnitude.
* A ``LayerStreamCodec`` for persistence.
* A ``LayerStateConversion`` when State differs from public Value.
* A ``LayerRosCodec`` when using the generic ROS converter.

The RGB and weighted-mean types under
``library/cpp/include/wavemap/layered/types`` are compact examples.

Bind sensor data
================

Install the typed map with ``LayeredRosServerBuilder``, then register how sensor
fields populate it. Numeric PointCloud2 fields can be normalized directly:

.. code-block:: c++

    server.bindNormalizedPointField<ReflectivityLayer>(
        extension, "reflectivity", 0.f, 255.f,
        wavemap::layered::EndpointRange{0.5f, 25.f});

Other helpers include:

* ``bindDirectPointField`` for values already in the layer's units.
* ``bindComputedPointField`` for values derived from numeric fields.
* ``bindGeometricClass`` for the included local-elevation classifier.
* A custom endpoint adapter for structured or application-specific decoding.

Bindings default to endpoint-only integration. Continuous layers may select an
along-ray ``PointcloudLayerIntegrationMode``. Discrete fields are restricted to
endpoints.

Expose and visualize the application
====================================

Add the executable to ``interfaces/ros1/wavemap_ros/CMakeLists.txt``, link it to
``wavemap_ros``, and rebuild:

.. code-block:: bash

    cd ~/catkin_ws
    catkin build wavemap_ros wavemap_ros_conversions wavemap_rviz_plugin
    source devel/setup.bash

Topic visualization obtains schema and color metadata from the ``LayeredMap``
message. Loading a custom ``.lwvmp`` directly in RViz additionally requires a
typed ``LayeredMapFactory`` registration for the new schema.

Verification and troubleshooting
********************************

Run the affected builds and tests after changing schemas, codecs, storage, or
ROS conversions:

.. code-block:: bash

    cd ~/catkin_ws
    catkin build wavemap wavemap_ros_conversions wavemap_ros \
      wavemap_rviz_plugin
    catkin run_tests wavemap wavemap_ros_conversions wavemap_ros \
      wavemap_rviz_plugin
    source /opt/ros/noetic/setup.bash
    catkin_test_results --all build

No map appears
  Check the display topic, the fixed frame, TF availability, and whether the
  node has published a map message.

Point clouds are dropped
  Verify cloud timestamps and frames, the map-to-sensor transform, and
  ``max_wait_for_pose``. Motion undistortion needs usable per-point timing.

An attribute remains empty
  Inspect the PointCloud2 fields. Its name and numeric range must match the
  registered binding.

A saved map will not load
  Use the same schema, backend, resolution, tree geometry, and discrete block
  configuration that created it.

Fine attribute detail disappears
  Reduce the layer's pruning scale or increase its pruning weight. Continuous
  fields contribute to one combined pruning decision.

Relevant source locations
*************************

* ``library/cpp/include/wavemap/layered/schema``: declarations and bundles.
* ``library/cpp/include/wavemap/layered/integration``: observations and update
  policies.
* ``library/cpp/include/wavemap/layered/map``: map types, backends, and storage.
* ``library/cpp/include/wavemap/layered/io``: schema-aware persistence.
* ``interfaces/ros1/wavemap_ros/app/ouster_layered``: complete applications.
* ``interfaces/ros1/wavemap_ros_conversions``: ROS codecs.
* ``interfaces/ros1/wavemap_rviz_plugin``: topic and file visualization.
