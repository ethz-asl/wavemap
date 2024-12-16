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
  using ScalarType = typename PointT::Scalar;

  PointT center = PointT::Constant(kNaN);
  ScalarType radius = static_cast<ScalarType>(0);

  Sphere() = default;
  Sphere(const PointT& center, ScalarType radius)
      : center(center), radius(radius) {}
  Sphere(PointT&& center, ScalarType radius)
      : center(std::move(center)), radius(radius) {}

  operator AABB<PointT>() const {
    if (std::isnan(center[0])) {
      return {};
    }
    return {center.array() - radius, center.array() + radius};
  }

  // TODO(victorr): Add tests, incl. behavior after default construction
  bool contains(const PointT& point) const {
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
