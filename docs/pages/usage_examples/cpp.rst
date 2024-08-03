C++ API
#######
.. rstcheck: ignore-roles=gh_file

Before you begin, make sure your code has access to wavemap's C++ library. For example, by following the instructions for :doc:`CMake-based C++ projects <../installation/cmake>` or :doc:`ROS1 packages <../installation/ros1>`.

Note that an example CMake package that contains all of the examples that follow can be found :gh_file:`here <examples/cpp>`.

Serialization
*************

Files
=====
Saving maps to files:

.. literalinclude:: ../../../examples/cpp/io/save_map_to_file.cc
    :language: c++

Loading maps from files:

.. literalinclude:: ../../../examples/cpp/io/load_map_from_file.cc
    :language: c++

Queries
*******

Fixed resolution
================
.. literalinclude:: ../../../examples/cpp/queries/fixed_resolution.cc
    :language: c++

Multi-res averages
==================
.. literalinclude:: ../../../examples/cpp/queries/multi_resolution.cc
    :language: c++

Accelerators
============
.. literalinclude:: ../../../examples/cpp/queries/accelerated_queries.cc
    :language: c++

Interpolation
=============
.. _examples-interpolation:

Nearest neighbor interpolation:

.. literalinclude:: ../../../examples/cpp/queries/nearest_neighbor_interpolation.cc
    :language: c++

Trilinear interpolation:

.. literalinclude:: ../../../examples/cpp/queries/trilinear_interpolation.cc
    :language: c++

Classification
==============
.. _examples-classification:

.. literalinclude:: ../../../examples/cpp/queries/classification.cc
    :language: c++
