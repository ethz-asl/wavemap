"""
convert
*******
Submodule with common conversion functions for wavemap index types.
"""

import numpy as _np
from ._pywavemap_bindings import OctreeIndex


def cell_width_to_height(cell_width, min_cell_width):
    """
    Compute the minimum node height (resolution level) required to reach
    a given width.
    """
    return int(_np.ceil(_np.log2(cell_width / min_cell_width)))


def height_to_cell_width(min_cell_width, height):
    """Compute the node width at a given height."""
    return min_cell_width * float(_np.exp2(height))


def scaled_point_to_nearest_index(point):
    """Compute the nearest index to a point on the unit grid."""
    return (point - 0.5).round().astype(int)


def point_to_nearest_index(point, cell_width):
    """
    Compute the nearest index to a point on a grid with a given cell width.
    """
    return scaled_point_to_nearest_index(point / cell_width)


def point_to_node_index(point, min_cell_width, height):
    """
    Compute the index of a node containing a given point.

    :param min_cell_width: The grid resolution at height 0 (max map resolution).
    :param height: The desired height (resolution level) of the node index.
    """
    node_width = height_to_cell_width(min_cell_width, height)
    position_index = point_to_nearest_index(point, node_width)
    return OctreeIndex(height, position_index)
