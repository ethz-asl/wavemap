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

.. literalinclude:: ../../../examples/python/io_save_map_to_file.py
    :language: python

Loading maps from files:

.. literalinclude:: ../../../examples/python/io_load_map_from_file.py
    :language: python

Queries
=======

Fixed resolution
----------------
.. literalinclude:: ../../../examples/python/queries_fixed_resolution.py
    :language: python

Multi-res averages
------------------
.. literalinclude:: ../../../examples/python/queries_multi_resolution.py
    :language: python

Accelerators
------------
.. literalinclude:: ../../../examples/python/queries_accelerated_queries.py
    :language: python

Interpolation
-------------

Nearest neighbor interpolation:

.. literalinclude:: ../../../examples/python/queries_nearest_neighbor_interpolation.py
    :language: python

Trilinear interpolation:

.. literalinclude:: ../../../examples/python/queries_trilinear_interpolation.py
    :language: python

Classification
--------------

.. literalinclude:: ../../../examples/python/queries_classification.py
    :language: python

Mapping
=======

Full pipeline
-------------

.. literalinclude:: ../../../examples/python/mapping_pipeline.py
    :language: python
