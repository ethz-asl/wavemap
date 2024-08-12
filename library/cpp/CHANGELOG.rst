^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wavemap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-08-12)
------------------
* New features

  * Make wavemap's C++ library available as a pure CMake package

    * Installable and compatible with find_package (and FetchContent)
    * Supports automatically fetching missing system deps with FetchContent
    * Embeds header-only minkindr library, including minor cleanup of its code

  * Generic spatial data structures

    * Decouple our spatial data structures from our occupancy map types
    * Directly usable for path planning, semantic mapping and other tasks
    * Improved code quality, performance and consistency between interfaces

  * Euclidean Signed Distance Fields

    * Quasi and Fully Euclidean ESDF generators
    * Extensive unit tests
    * (De)serialization to files

  * Hierarchical, multi-resolution classified maps

    * Classifies space based on occupancy or collision radius (ESDF) thresholds
    * Enables fast, hierarchical traversability and visibility checking
    * Provides convenience functions, e.g. visit all free & observed map leaves

  * Additional utilities for path planning, to simplify

    * Iterating over cell neighborhoods
    * Determining adjacency types between fixed and multi-resolution cells
    * Sampling random points

      * Supports occupancy, observedness, and collision radius constraints

    * Computing best and worst-case distances between AABBs

  * Class that emulates single-node pointers chunked octrees

    * Enables traversing and manipulating chunked trees as if they're regular trees
    * Improves code readability and reduces chance of bookkeeping bugs

* Improvements

  * Migrate functional code from ROS packages into C++ library

    * Make measurement integration and map operation pipeline ROS-agnostic
    * Move pointcloud undistortion code into C++ library

  * Extend query accelerator

    * Add support for ClassifiedMaps and most of our generic spatial data structures
    * Expose height (resolution) of query result, useful for multi-resolution ray tracing

  * Refactoring and code quality improvements

    * Improve consistency and reduce code duplication between occupancy map types
    * Update all factories to return unique_ptrs

      * Users are then free to choose if they keep it unique or auto convert it to a shared ptr
    * Reorganize and refactor unit tests
    * Define and use bit_ops::is_bit_set to make bit manipulation code easier to read
    * Simplify DECLARE_CONFIG_MEMBERS macro

  * Improve user friendliness

    * Improve structure of utils directory
    * Interaction with configs

      * Improve syntax for handling TypeSelectors
      * Improve syntax and add helper methods for handling nested params
      * Make it possible to pass expressions in param validators

    * Add helper methods to configure the C++ library's logging level (verbosity)
    * Improve index conversion vector op syntax
    * Make it easier to configure an integrator's max update resolution, by allowing it to set it explicitly instead of through `termination_height`

* Bug fixes

  * Fix GCC hanging when compiling with UBSAN enabled

* Contributors: Victor Reijgwart

1.6.3 (2023-12-21)
------------------

1.6.2 (2023-12-11)
------------------
* Include <optional> for std::optional
* Contributors: Lucas Walter

1.6.1 (2023-11-20)
------------------

1.6.0 (2023-10-17)
------------------
* New features

  * Map query accelerator
  * Trilinear interpolator

* Improvements

  * Optimize measurement integration

    * Replace stack with recursion (faster and easier to read)
    * Vectorize batched leaf updater
    * Reduce memory move and copy overheads
    * Simplify measurement model math
    * Postpone image offset error norm root computation
    * Share a single thread pool among all integrators

  * Refactor wavemap utils
  * Add tests for nearest index and offset methods
  * Add initial usage examples

* Contributors: Victor Reijgwart

1.5.3 (2023-09-28)
------------------
* Address failing DCHECKs for Morton conversions of negative indices
* Contributors: Victor Reijgwart

1.5.2 (2023-09-19)
------------------
* Add missing install rules for wavemap
* Contributors: Alexander Stumpf

1.5.1 (2023-09-08)
------------------

1.5.0 (2023-09-05)
------------------
* Annotate code for profiling with Tracy Profiler
* Switch to custom atan2 in LiDAR projection models

  * Speeds up wavemap by roughly 20% when using LiDAR inputs
  * No compromise in accuracy (slightly improves AUC, accuracy and recall)

* Minor general optimizations
* Add option to enable DCHECKs even when not compiling in debug mode
* Contributors: Victor Reijgwart

1.4.0 (2023-08-30)
------------------
* Document how to configure wavemap
* Improve config parameter unit management
* Make warnings/errors that can occur when loading configs more descriptive
* Contributors: Victor Reijgwart

1.3.2 (2023-08-28)
------------------

1.3.1 (2023-08-22)
------------------
* Release the code under the BSD-3 license

1.3.0 (2023-08-17)
------------------
* Standardize time definitions
* Add option to limit maximum resolution in forEachLeaf visitor
* Also consider the root_scale_coefficient in block.empty() checks
  Otherwise blocks with no child nodes (i.e. detail coefficients) will be pruned away. This leads to information loss, as the block might have told us the area is fully free or occupied, and pruning it away resets it to being unknown. By also checking the root scale (average value of the block), we can make sure to only mark blocks empty if they're fully unknown.
* Contributors: Victor Reijgwart

1.2.0 (2023-08-11)
------------------

1.1.0 (2023-08-09)
------------------

1.0.0 (2023-08-08)
------------------
* First public release
* Contributors: Victor Reijgwart
