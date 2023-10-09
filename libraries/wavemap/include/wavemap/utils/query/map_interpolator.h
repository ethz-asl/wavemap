#ifndef WAVEMAP_UTILS_QUERY_MAP_INTERPOLATOR_H_
#define WAVEMAP_UTILS_QUERY_MAP_INTERPOLATOR_H_

#include "wavemap/common.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/indexing/index_conversions.h"

namespace wavemap::interpolate {
FloatingPoint trilinear(const wavemap::VolumetricDataStructureBase& map,
                        const wavemap::Point3D& position) {
  const FloatingPoint cell_width = map.getMinCellWidth();
  const FloatingPoint cell_width_inv = 1.f / cell_width;
  const auto min_corner_index =
      wavemap::convert::pointToFloorIndex(position, cell_width_inv);

  const Vector3D a = (position - wavemap::convert::indexToCenterPoint(
                                     min_corner_index, cell_width)) *
                     cell_width_inv;
  const Vector3D a_comp = 1.f - a.array();
  DCHECK((-0.001 < a.array() && a.array() < 1.001).all()) << a;
  DCHECK((-0.001 < a_comp.array() && a_comp.array() < 1.001).all()) << a_comp;

  std::array<FloatingPoint, 8> cube_corners{};
  for (int neighbor_idx = 0; neighbor_idx < 8; ++neighbor_idx) {
    const Index3D offset{(neighbor_idx >> 2) & 1, (neighbor_idx >> 1) & 1,
                         neighbor_idx & 1};
    cube_corners[neighbor_idx] = map.getCellValue(min_corner_index + offset);
  }
  std::array<FloatingPoint, 4> plane_corners{};
  for (int corner_idx = 0; corner_idx < 4; ++corner_idx) {
    plane_corners[corner_idx] = a_comp[0] * cube_corners[corner_idx] +
                                a[0] * cube_corners[corner_idx + 0b100];
  }
  std::array<FloatingPoint, 2> line_corners{};
  for (int side_idx = 0; side_idx < 2; ++side_idx) {
    line_corners[side_idx] = a_comp[1] * plane_corners[side_idx] +
                             a[1] * plane_corners[side_idx + 0b10];
  }
  return a_comp[2] * line_corners[0] + a[2] * line_corners[1];
}
}  // namespace wavemap::interpolate

#endif  // WAVEMAP_UTILS_QUERY_MAP_INTERPOLATOR_H_
