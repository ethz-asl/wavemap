C++ API
#######
.. rstcheck: ignore-roles=gh_file

In this tutorial, we illustrate how you can use wavemap's C++ API in your own projects.

.. tip::

    An example package that combines the setup steps and code examples that follow can be found :gh_file:`here <examples/cpp>`.

CMake target setup
******************
Once you included wavemap's C++ library in your CMake project, for example by following our :doc:`installation instructions <../installation/cmake>`, the last remaining step to start using it is to configure your CMake target (e.g. library or executable) to use it.

Wavemap's C++ library contains three main components:

* ``wavemap_core``: The framework's core algorithms, data structures and utilities
* ``wavemap_io``: Functions to read and write maps to streams and files
* ``wavemap_pipeline``: A measurement integration and map management pipeline

For an example executable that performs some operations on a map after reading it from a file, you would need to link:

.. code-block:: cmake

      add_executable(your_executable your_code.cc)
      target_link_libraries(your_executable
            PUBLIC wavemap::wavemap_core wavemap::wavemap_io)

Note that this will automatically make wavemap's include directories (headers) available to `your_code.cc`.

We **strongly recommend** to also call the ``set_wavemap_target_properties`` function to ensure your target's compilation options are compatible with those of wavemap:

.. code-block:: cmake

      set_wavemap_target_properties(your_executable)

.. _cpp-code-examples:

Code examples
*************
In the following sections, you'll find sample code for common tasks. If you'd like to request examples for additional tasks or contribute new examples, please don't hesitate to `contact us <https://github.com/ethz-asl/wavemap/issues>`_.

Serializing maps
================
In this section, we'll demonstrate how to serialize and deserialize maps using wavemap's lightweight and efficient binary format. This format is consistent across wavemap's C++, Python, and ROS interfaces. For instance, you can create maps on a robot with ROS and later load them into a rendering engine plugin that only depends on wavemap's C++ library.

Binary files
------------
Maps can be saved to disk with

.. literalinclude:: ../../../examples/cpp/io/save_map_to_file.cc
    :language: c++

.. _cpp-code-examples-read-map:

and read using

.. literalinclude:: ../../../examples/cpp/io/load_map_from_file.cc
    :language: c++

Byte streams
------------
We also provide an alternative, lower-level interface to convert maps to (byte) streams

.. literalinclude:: ../../../examples/cpp/io/save_map_to_stream.cc
    :language: c++

and read them with

.. literalinclude:: ../../../examples/cpp/io/load_map_from_stream.cc
    :language: c++


Queries
=======
In this section, we illustrate how you can query the map and classify whether a point or region of interest is occupied.

Node indices
------------
The map models the environment by filling it with cubes of variable sizes, arranged as the nodes of an octree. Node indices are defined as integer [X, Y, Z, height] coordinates, whose XYZ values correspond to the node's position in the octree's grid at the given *height*, or level in the tree. Height 0 corresponds to the map's maximum resolution, and the grid resolution is halved for each subsequent height level.

Fixed resolution
^^^^^^^^^^^^^^^^
Querying the value of a single node in the highest resolution grid (*height=0*) can be done as follows.

.. literalinclude:: ../../../examples/cpp/queries/fixed_resolution.cc
    :language: c++

Multi-res averages
^^^^^^^^^^^^^^^^^^
It is also possible to query lower resolution nodes, whose values correspond to the average estimated occupancy of the volume they cover.

.. literalinclude:: ../../../examples/cpp/queries/multi_resolution.cc
    :language: c++

Accelerators
^^^^^^^^^^^^
In case you intend to look up multiple node values, we recommend using wavemap's query accelerator which traverses the octree significantly faster by caching parent nodes.

.. literalinclude:: ../../../examples/cpp/queries/accelerated_queries.cc
    :language: c++

.. _cpp-code-examples-interpolation:

Real coordinates
----------------
Many applications require occupancy estimates at arbitrary 3D points, with real-valued coordinates. Such estimates are computed by interpolating the map.

.. caution::

    If your query points are expressed in a different coordinate frame than the map, do not forget to transform them into the map frame before you continue.

Nearest neighbor interpolation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The simplest form of interpolation simply looks up the value of the map node that is closest to the query point.

.. literalinclude:: ../../../examples/cpp/queries/nearest_neighbor_interpolation.cc
    :language: c++

Trilinear interpolation
^^^^^^^^^^^^^^^^^^^^^^^
Another option is to linearly interpolate the map along the x, y, and z axes. This method produces cleaner, more accurate results at the cost of being slightly slower, since it needs to query 8 neighboring map nodes.

.. literalinclude:: ../../../examples/cpp/queries/trilinear_interpolation.cc
    :language: c++

.. _cpp-code-examples-classification:

Occupancy classification
------------------------
Once the estimated occupancy at a node or point has been retrieved, it can be classified as follows.

.. literalinclude:: ../../../examples/cpp/queries/classification.cc
    :language: c++
