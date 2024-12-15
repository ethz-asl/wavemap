#ifndef WAVEMAP_CORE_UTILS_GEOMETRY_SPHERE_H_
#define WAVEMAP_CORE_UTILS_GEOMETRY_SPHERE_H_

#include <algorithm>
#include <string>
#include <utility>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/print/eigen.h"

namespace wavemap {
template <typename PointT>
struct Sphere {
  static constexpr int kDim = dim_v<PointT>;
  using PointType = PointT;
  using ScalarType = typename PointType::Scalar;

  PointType center;
  ScalarType radius;

  Sphere() = default;
  Sphere(const PointT& center, ScalarType radius)
      : center(center), radius(radius) {}
  Sphere(PointT&& center, ScalarType radius)
      : center(std::move(center)), radius(radius) {}

  operator AABB<PointT>() const {
    return AABB<PointT>(center.array() - radius, center.array() + radius);
  }

  bool contains(const PointType& point) const {
    return (point - center).squaredNorm() <= radius * radius;
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "[center =" << print::eigen::oneLine(center)
       << ", radius = " << radius << "]";
    return ss.str();
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_GEOMETRY_SPHERE_H_
