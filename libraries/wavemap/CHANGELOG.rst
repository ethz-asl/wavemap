^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wavemap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
