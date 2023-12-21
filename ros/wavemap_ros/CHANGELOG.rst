^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wavemap_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
