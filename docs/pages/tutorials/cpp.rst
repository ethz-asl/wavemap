C++ API
#######
.. rstcheck: ignore-roles=gh_file

In this tutorial, we illustrate how you can use wavemap's C++ API in your own project. Note that an example CMake package that contains all of the examples that follow can be found :gh_file:`here <examples/cpp>`.

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

Code examples
*************
.. _cpp-code-examples:

In the following sections, you'll find sample code for common tasks. If you'd like to request examples for additional tasks or contribute new examples, please don't hesitate to `contact us <https://github.com/ethz-asl/wavemap/issues>`_.

Serialization
=============

Files
-----
Saving maps to files:

.. literalinclude:: ../../../examples/cpp/io/save_map_to_file.cc
    :language: c++

Loading maps from files:

.. literalinclude:: ../../../examples/cpp/io/load_map_from_file.cc
    :language: c++

Queries
=======

Fixed resolution
----------------
.. literalinclude:: ../../../examples/cpp/queries/fixed_resolution.cc
    :language: c++

Multi-res averages
------------------
.. literalinclude:: ../../../examples/cpp/queries/multi_resolution.cc
    :language: c++

Accelerators
------------
.. literalinclude:: ../../../examples/cpp/queries/accelerated_queries.cc
    :language: c++

Interpolation
-------------
.. _cpp-code-examples-interpolation:

Nearest neighbor interpolation:

.. literalinclude:: ../../../examples/cpp/queries/nearest_neighbor_interpolation.cc
    :language: c++

Trilinear interpolation:

.. literalinclude:: ../../../examples/cpp/queries/trilinear_interpolation.cc
    :language: c++

Classification
--------------
.. _cpp-code-examples-classification:

.. literalinclude:: ../../../examples/cpp/queries/classification.cc
    :language: c++
