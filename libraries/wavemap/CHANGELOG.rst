^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wavemap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
