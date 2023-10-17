^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wavemap_ros_conversions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update incremental transmission and Rviz to remove deleted blocks
* Multi-thread block to ROS msg serialization
* Add tests for map to ROS msg conversions
* Consistently use ROS logging in ROS packages
* Refactor wavemap utils
* Contributors: Victor Reijgwart

1.5.3 (2023-09-28)
------------------

1.5.2 (2023-09-19)
------------------

1.5.1 (2023-09-08)
------------------

1.5.0 (2023-09-05)
------------------
* Annotate code for profiling with Tracy Profiler
* Contributors: Victor Reijgwart

1.4.0 (2023-08-30)
------------------
* Make warnings/errors that can occur when loading configs more descriptive
* Contributors: Victor Reijgwart

1.3.2 (2023-08-28)
------------------

1.3.1 (2023-08-22)
------------------
* Release the code under the BSD-3 license

1.3.0 (2023-08-17)
------------------
* Update map <-> ROS msg conversion methods to be consistent with map <-> byte stream conversions
* Incremental map transmission
  Only publish changed map blocks and add option to control the max message size. This improves transmission stability over unreliable networks and resolves the issue of roscpp dropping messages >1GB.
* Contributors: Victor Reijgwart

1.2.0 (2023-08-11)
------------------

1.1.0 (2023-08-09)
------------------

1.0.0 (2023-08-08)
------------------
* First public release
* Contributors: Victor Reijgwart
