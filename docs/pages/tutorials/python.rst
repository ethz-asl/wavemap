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

    import pywavemap as wave

Code examples
*************

In the following sections, we provide sample code for common tasks. If you'd like to request examples for additional tasks or contribute new examples, please don't hesitate to `contact us <https://github.com/ethz-asl/wavemap/issues>`_.

.. tip::

    All of the examples scripts that follow can be found :gh_file:`here <examples/python>`.

Mapping
=======
The only requirements to build wavemap maps are that you have a set of

1. depth measurements,
2. sensor pose (estimates) for each measurement.

We usually use depth measurements from depth cameras or 3D LiDARs, but any source would work as long as a corresponding :ref:`projection <configuration_projection_models>` and :ref:`measurement <configuration_measurement_models>` model is available. To help you get started quickly, we provide example configs for various sensor setups :gh_file:`here <interfaces/ros1/wavemap_ros/config>`. An overview of all the available settings is provided on the :doc:`parameters page <../parameters/index>`.

Example pipeline
----------------

.. literalinclude:: ../../../examples/python/mapping/full_pipeline.py
    :language: python

Serialization
=============
Next, we show how you can serialize and deserialize common wavemap objects, for example to save and load them from files.

Maps
----
Wavemap uses a lightweight, efficient binary format to serialize its maps. The same format is used across wavemap's C++, Python and ROS interfaces. You could therefore, for example, create maps on a robot with ROS and subsequently analyze them in Python.

Binary files
^^^^^^^^^^^^
Maps can be saved to disk using

.. literalinclude:: ../../../examples/python/io/save_map_to_file.py
    :language: python

.. _python-code-examples-read-map:

and read with

.. literalinclude:: ../../../examples/python/io/load_map_from_file.py
    :language: python

Configs
-------
In the previous mapping pipeline example, the configuration parameters for the map and the measurement integration components were hard-coded. To make your setup more flexible, you can use configuration files. We will demonstrate how to work with YAML files, which is the format we use for wavemap's :gh_file:`example configs <interfaces/ros1/wavemap_ros/config>`. However, pywavemap is flexible and can support any parameter format that can be read into a Python `dict`.


YAML files
^^^^^^^^^^

.. literalinclude:: ../../../examples/python/io/load_params_from_file.py
    :language: python


Queries
=======
In this section, we show how you can query wavemap maps and classify whether a point or region of interest is occupied.

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
If you need to look up multiple node values, we recommend using our batched query functions. These functions deliver significant speedups by utilizing wavemap's QueryAccelerator.

.. literalinclude:: ../../../examples/python/queries/accelerated_queries.py
    :language: python

.. _python-code-examples-interpolation:

Real coordinates
----------------
Many applications require occupancy estimates at arbitrary 3D points, with real-valued coordinates. Such estimates are computed by interpolating the map.

.. caution::

    If your query points are expressed in a different coordinate frame than the map, do not forget to transform them into the map frame before you continue.

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

.. _python-code-examples-classification:

Occupancy classification
------------------------
Once the estimated occupancy at a node or point has been retrieved, it can be classified as follows.

.. literalinclude:: ../../../examples/python/queries/classification.py
    :language: python
