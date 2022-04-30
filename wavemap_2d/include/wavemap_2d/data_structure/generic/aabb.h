#ifndef WAVEMAP_2D_DATA_STRUCTURE_GENERIC_AABB_H_
#define WAVEMAP_2D_DATA_STRUCTURE_GENERIC_AABB_H_

#include <limits>
#include <string>

#include "wavemap_2d/common.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
template <typename T>
struct AABB {
  T min = T::Constant(std::numeric_limits<typename T::value_type>::max());
  T max = T::Constant(std::numeric_limits<typename T::value_type>::lowest());

  void includePoint(const T& point) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }
  bool containsPoint(const T& point) const {
    return (min.array() <= point.array() && point.array() <= max.array()).all();
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "[min =" << EigenFormat::oneLine(min)
       << ", max =" << EigenFormat::oneLine(max) << "]";
    return ss.str();
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_GENERIC_AABB_H_
