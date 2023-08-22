^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wavemap_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
