^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wavemap_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.2 (2024-11-20)
------------------
* Report CPU, wall time and RAM usage when rosbag_processor completes
* Adjust wavemap config schemas to resolve false positive validation warnings caused by CLion bug IJPL-63581
* Contributors: Victor Reijgwart

2.1.1 (2024-10-24)
------------------
* Address warnings from new cpplint version (v2.0)
* Contributors: Victor Reijgwart

2.1.0 (2024-09-16)
------------------

2.0.1 (2024-08-30)
------------------
* Fix outdated Livox callback code
* Contributors: Zhihuan Hu, Victor Reijgwart

2.0.0 (2024-08-12)
------------------
* New features

  * Implement a plugin system for map operations, including plugins to

    * Crop the map for local mapping
    * Publish obstacles as a fixed-resolution pointcloud

  * Separate configuration of inputs and measurement integrators

    * Allows one set of integrator settings to be used by multiple inputs (e.g. on a robot with 6 identical depth cameras)
    * Cleaner coupling between ROS interface and ROS-agnostic integration pipeline

* Improvements

  * Migrate functional code from ROS packages to C++ library

    * Why?

      * Goal is to keep the interfaces thin
      * Simplifies adding new interfaces in future (e.g. ROS2, Zenoh,...)
      * Reduces code duplication

    * Make measurement integration and map operation pipeline ROS-agnostic
    * Move pointcloud undistortion code into ROS-agnostic library
    * Make the C++ library's logging level (verbosity) configurable through ROS params
    * Rename wavemap server to ros_server to make room for ROS agnostic server

  * Use new feature of wavemap's C++ library

    * Improved parameter handling and TypeSelector syntax and methods

  * Tidy up CMake files

    * Switch from catkin_simple to vanilla catkin
    * Remove dependencies on catkinized gflags, glog and Eigen

  * Improve user friendliness

    * Make it easier to configure an integrator's max update resolution, by allowing it to set it explicitly instead of through `termination_height`
    * Print clear error msgs when measurement_integrator_names fail to parse

* Contributors: Victor Reijgwart

1.6.3 (2023-12-21)
------------------

1.6.2 (2023-12-11)
------------------
* Include <optional> for std::optional
* Contributors: Lucas Walter

1.6.1 (2023-11-20)
------------------
* Add config for Ouster OS0 + Pico Monstar
* Improve demo settings and tune FastLIO
* Make depth camera startup more reliable
* Document how to replicate the live demo
* Contributors: Victor Reijgwart

1.6.0 (2023-10-17)
------------------
* New features

  * Add service and button to reset the wavemap_server's map
  * Make ROS logging level configurable through ROS params
  * Make the number of threads to use configurable through ROS params
  * Add option to directly query the most up-to-date transform to TF handler

* Improvements

  * Share a single thread pool among all integrators
  * Multi-thread block to ROS msg serialization
  * Update incremental transmission and Rviz to remove deleted blocks
  * Simplify incremental map transmission
  * Improve Rviz plugin block drawing scheduling
  * Do not latch map topic
  * Improve example configs
  * Consistently use ROS logging in ROS packages
  * Refactor wavemap utils

* Bug fixes

  * Fix bug causing delays when transmitting blocks with identical timestamps

* Contributors: Victor Reijgwart

1.5.3 (2023-09-28)
------------------

1.5.2 (2023-09-19)
------------------
* Add missing install rules for wavemap
* Contributors: Alexander Stumpf

1.5.1 (2023-09-08)
------------------

1.5.0 (2023-09-05)
------------------
* Annotate code for profiling with Tracy Profiler
* Improve error messages when reading/writing a file fails
* Contributors: Victor Reijgwart

1.4.0 (2023-08-30)
------------------
* Document how to configure wavemap
* Update all configs to recommended settings
* Improve config parameter unit management
* Make warnings/errors that can occur when loading configs more descriptive
* Silence cmake warning when no livox ros driver is found
* Define schema for wavemap configs (enables code completion + validation in IDEs)
* Configure .pre-commit to automatically lint wavemap configs using above schema
* Contributors: Alexander Stumpf, Victor Reijgwart

1.3.2 (2023-08-28)
------------------
* Fix empty header of wavemap msgs
* Contributors: Alexander Stumpf

1.3.1 (2023-08-22)
------------------
* Release the code under the BSD-3 license

1.3.0 (2023-08-17)
------------------
* Update map <-> ROS msg conversion methods to be consistent with map <-> byte stream conversions
* Incremental map transmission
  Only publish changed map blocks and add option to control the max message size. This improves transmission stability over unreliable networks and resolves the issue of roscpp dropping messages >1GB.
* Standardize time definitions
* General code cleanup
* Contributors: Victor Reijgwart

1.2.0 (2023-08-11)
------------------

1.1.0 (2023-08-09)
------------------
* Enable file saving in ROS server
* Contributors: Victor Reijgwart

1.0.0 (2023-08-08)
------------------
* First public release
* Contributors: Victor Reijgwart
