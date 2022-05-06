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

  T closestPointTo(const T& point) const {
    T closest_point = point.cwiseMax(min);
    closest_point = closest_point.cwiseMin(max);
    return closest_point;
  }
  T furthestPointFrom(const T& point) const {
    const T aabb_center = (max - min) / static_cast<typename T::Scalar>(2);
    const T furthest_point =
        (aabb_center.array() < point.array()).select(min, max);
    return furthest_point;
  }

  FloatingPoint minSquaredDistanceTo(const T& point) const {
    return (point - closestPointTo(point)).squaredNorm();
  }
  FloatingPoint maxSquaredDistanceFrom(const T& point) const {
    return (point - furthestPointFrom(point)).squaredNorm();
  }
  FloatingPoint minDistanceTo(const T& point) const {
    return (point - closestPointTo(point)).norm();
  }
  FloatingPoint maxDistanceFrom(const T& point) const {
    return (point - furthestPointFrom(point)).norm();
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
