^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wavemap_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.5.1 (2023-09-08)
------------------

1.5.0 (2023-09-05)
------------------

1.4.0 (2023-08-30)
------------------

1.3.2 (2023-08-28)
------------------

1.3.1 (2023-08-22)
------------------
* Release the code under the BSD-3 license

1.3.0 (2023-08-17)
------------------
New features:

* Major refactoring of Rviz plugin architecture and UI

  * Support different render modes (slice; grid) in a single WavemapDisplay instance

    * Each render mode can be configured through its own foldout menu
    * The map is only stored once per plugin instance and shared among the render mode handlers

  * Grid render mode

    * Expose grid color options in the UI
    * Add option to set maximum grid drawing resolution (e.g. to improve frame rate when displaying large maps on computers with no GPU)

  * Improve default settings s.t. it can be used with minimal tuning in most scenarios
  * Remove the option to render map meshes

    * This render mode does not yet produce good iso-surfaces and is currently very slow. It will be reintroduced once its more mature.

* Improve Rviz plugin performance

  * Only redraw map blocks that changed
  * Render grid blocks with Level of Detail based on their distance to the camera
  * Use a work queue and limit the update time per frame, to avoid stalling Rviz when large map changes occur
  * Interface directly with Ogre, instead of using rviz::Pointcloud as an intermediary for rendering

* General

  * Update map <-> ROS msg conversion methods to be consistent with map <-> byte stream conversions
  * Incremental map transmission
    Only publish changed map blocks and add option to control the max message size. This improves transmission stability over unreliable networks and resolves the issue of roscpp dropping messages >1GB.
  * Standardize time definitions

* Contributors: Victor Reijgwart

1.2.0 (2023-08-11)
------------------

1.1.0 (2023-08-09)
------------------

1.0.0 (2023-08-08)
------------------
* First public release
* Contributors: Victor Reijgwart
