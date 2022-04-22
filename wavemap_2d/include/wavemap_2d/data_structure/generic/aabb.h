#ifndef WAVEMAP_2D_DATA_STRUCTURE_GENERIC_AABB_H_
#define WAVEMAP_2D_DATA_STRUCTURE_GENERIC_AABB_H_

#include <limits>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
struct AABB {
  Point min = Point::Constant(std::numeric_limits<FloatingPoint>::max());
  Point max = Point::Constant(std::numeric_limits<FloatingPoint>::lowest());

  void includePoint(const Point& point) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_GENERIC_AABB_H_
