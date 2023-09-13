#ifndef WAVEMAP_UTILS_INTERPOLATION_UTILS_H_
#define WAVEMAP_UTILS_INTERPOLATION_UTILS_H_

#include "wavemap/common.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/indexing/index_conversions.h"

namespace wavemap {
FloatingPoint interpolateTrilinear(
    const wavemap::VolumetricDataStructureBase& map,
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
  const float c000 = map.getCellValue(min_corner_index + Index3D(0, 0, 0));
  const float c001 = map.getCellValue(min_corner_index + Index3D(0, 0, 1));
  const float c010 = map.getCellValue(min_corner_index + Index3D(0, 1, 0));
  const float c011 = map.getCellValue(min_corner_index + Index3D(0, 1, 1));
  const float c100 = map.getCellValue(min_corner_index + Index3D(1, 0, 0));
  const float c101 = map.getCellValue(min_corner_index + Index3D(1, 0, 1));
  const float c110 = map.getCellValue(min_corner_index + Index3D(1, 1, 0));
  const float c111 = map.getCellValue(min_corner_index + Index3D(1, 1, 1));
  const float c00 = a_comp[0] * c000 + a[0] * c100;
  const float c01 = a_comp[0] * c001 + a[0] * c101;
  const float c10 = a_comp[0] * c010 + a[0] * c110;
  const float c11 = a_comp[0] * c011 + a[0] * c111;
  const float c0 = a_comp[1] * c00 + a[1] * c10;
  const float c1 = a_comp[1] * c01 + a[1] * c11;
  const float c = a_comp[2] * c0 + a[2] * c1;
  return c;
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_INTERPOLATION_UTILS_H_
