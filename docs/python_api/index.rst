Python API
##########
.. rstcheck: ignore-directives=automodule
.. rstcheck: ignore-directives=autoclass
.. rstcheck: ignore-directives=automethod

.. automodule:: pywavemap

.. autoclass:: pywavemap.Map
    :members:
.. autoclass:: pywavemap.HashedWaveletOctree
    :show-inheritance:
    :members:
.. autoclass:: pywavemap.HashedChunkedWaveletOctree
    :show-inheritance:
    :members:

.. autoclass:: pywavemap.HashedWaveletOctreeQueryAccelerator
    :members:

.. autoclass:: pywavemap.OctreeIndex
    :members:

.. autoclass:: pywavemap.Rotation
    :members:
.. autoclass:: pywavemap.Pose
    :members:

.. autoclass:: pywavemap.Pointcloud
    :members:
.. autoclass:: pywavemap.PosedPointcloud
    :members:

.. autoclass:: pywavemap.Image
    :members:
.. autoclass:: pywavemap.PosedImage
    :members:

.. autoclass:: pywavemap.Pipeline
    :members:

.. automodule:: pywavemap.convert
    :members:
.. automethod:: pywavemap.convert.cell_width_to_height
.. automethod:: pywavemap.convert.height_to_cell_width
.. automethod:: pywavemap.convert.point_to_nearest_index
.. automethod:: pywavemap.convert.point_to_node_index

.. automodule:: pywavemap.logging
.. automethod:: pywavemap.logging.set_level
.. automethod:: pywavemap.logging.enable_prefix

.. automodule:: pywavemap.param
.. autoclass:: pywavemap.param.Value
    :members:
