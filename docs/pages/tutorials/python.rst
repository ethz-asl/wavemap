Python API
##########
.. highlight:: python
.. rstcheck: ignore-roles=gh_file

In this tutorial, we illustrate how you can use wavemap's Python API in your own projects.

Setup
*****
Before you start, make sure you :doc:`installed pywavemap <../installation/python>`. In case you used a virtual environment, activate it by running the following command from your terminal:

.. code-block:: bash

    source <path_to_your_virtual_env>/bin/activate

In your python files, you can then load the API by simply calling::

    import pywavemap

Code examples
*************

In the following sections, we provide sample code for common tasks. If you'd like to request examples for additional tasks or contribute new examples, please don't hesitate to `contact us <https://github.com/ethz-asl/wavemap/issues>`_.

.. note::

    All of the examples scripts that follow can be found :gh_file:`here <examples/python>`.

Serialization
=============

Files
-----
Saving maps to files:

.. literalinclude:: ../../../examples/python/io/save_map_to_file.py
    :language: python

Loading maps from files:

.. literalinclude:: ../../../examples/python/io/load_map_from_file.py
    :language: python

Queries
=======
In this section, we illustrate how you can query the map and classify whether a point or region of interest is occupied.

Node indices
------------
The map models the environment by filling it with cubes of variable sizes, arranged as the nodes of an octree. Node indices are defined as integer [X, Y, Z, height] coordinates, whose XYZ values correspond to the node's position in the octree's grid at the given *height*, or level in the tree. Height 0 corresponds to the map's maximum resolution, and the grid resolution is halved for each subsequent height level.

Fixed resolution
^^^^^^^^^^^^^^^^
Querying the value of a single node in the highest resolution grid (*height=0*) can be done as follows.

.. literalinclude:: ../../../examples/python/queries/fixed_resolution.py
    :language: python

Multi-res averages
^^^^^^^^^^^^^^^^^^
It is also possible to query lower resolution nodes, whose values correspond to the average estimated occupancy of the volume they cover.

.. literalinclude:: ../../../examples/python/queries/multi_resolution.py
    :language: python

Accelerators
^^^^^^^^^^^^
In case you intend to look up multiple node values, we recommend using wavemap's query accelerator which traverses the octree significantly faster by caching parent nodes.

.. literalinclude:: ../../../examples/python/queries/accelerated_queries.py
    :language: python

Real coordinates
----------------
Many applications require occupancy estimates at arbitrary 3D points, with real-valued coordinates. Such estimates are computed by interpolating the map.

.. note::

    In case the query points are expressed in a different coordinate frame than the map, do not forget to transform them into the map frame before you continue.

Nearest neighbor interpolation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The simplest form of interpolation simply looks up the value of the map node that is closest to the query point.

.. literalinclude:: ../../../examples/python/queries/nearest_neighbor_interpolation.py
    :language: python

Trilinear interpolation
^^^^^^^^^^^^^^^^^^^^^^^
Another option is to linearly interpolate the map along the x, y, and z axes. This method produces cleaner, more accurate results at the cost of being slightly slower, since it needs to query 8 neighboring map nodes.

.. literalinclude:: ../../../examples/python/queries/trilinear_interpolation.py
    :language: python

Occupancy classification
------------------------
Once the estimated occupancy at a node or point has been retrieved, it can be classified as follows.

.. literalinclude:: ../../../examples/python/queries/classification.py
    :language: python

Mapping
=======

Full pipeline
-------------

.. literalinclude:: ../../../examples/python/mapping/full_pipeline.py
    :language: python
