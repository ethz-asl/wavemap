#include "pywavemap/convert.h"

#include <nanobind/eigen/dense.h>
#include <wavemap/core/indexing/index_conversions.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_convert_module(nb::module_& m_convert) {
  m_convert.def(
      "cell_width_to_height",
      [](FloatingPoint cell_width, FloatingPoint min_cell_width) {
        return convert::cellWidthToHeight(cell_width, 1.f / min_cell_width);
      },
      "cell_width"_a, "min_cell_width"_a,
      "Compute the minimum node height (resolution level) required to reach"
      "a given node width.\n\n"
      "    :param cell_width: The desired node width.\n"
      "    :param min_cell_width: The grid resolution at height 0 "
      "(max map resolution).");

  m_convert.def("height_to_cell_width", &convert::heightToCellWidth,
                "min_cell_width"_a, "height"_a,
                "Compute the node width at a given height.\n\n"
                "    :param min_cell_width: The grid resolution at height 0 "
                "(max map resolution).\n"
                "    :param height: The desired height (resolution level) of "
                "the node index.");

  m_convert.def(
      "point_to_nearest_index",
      [](const Point3D& point, FloatingPoint cell_width) {
        return convert::pointToNearestIndex<3>(point, 1.f / cell_width);
      },
      "point"_a, "cell_width"_a,
      "Compute the nearest index to a point on a grid with a given "
      "cell width.");

  m_convert.def("point_to_node_index", &convert::pointToNodeIndex<3>, "point"_a,
                "min_cell_width"_a, "height"_a,
                "Compute the index of a node containing a given point.\n\n"
                "    :param min_cell_width: The grid resolution at height 0 "
                "(max map resolution).\n"
                "    :param height: The desired height (resolution level) of "
                "the node index.");
}
}  // namespace wavemap
