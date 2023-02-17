#ifndef WAVEMAP_DATA_STRUCTURE_AABB_H_
#define WAVEMAP_DATA_STRUCTURE_AABB_H_

#include <limits>
#include <string>

#include "wavemap/common.h"
#include "wavemap/utils/eigen_format.h"
#include "wavemap/utils/int_math.h"

namespace wavemap {
template <typename PointT>
struct AABB {
  static constexpr int kDim = dim_v<PointT>;
  static constexpr int kNumCorners = int_math::exp2(kDim);
  using PointType = PointT;
  using ScalarType = typename PointType::Scalar;
  using Corners = Eigen::Matrix<ScalarType, kDim, kNumCorners>;

  static constexpr auto kInitialMin = std::numeric_limits<ScalarType>::max();
  static constexpr auto kInitialMax = std::numeric_limits<ScalarType>::lowest();

  PointType min = PointType::Constant(kInitialMin);
  PointType max = PointType::Constant(kInitialMax);

  void includePoint(const PointType& point) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }
  bool containsPoint(const PointType& point) const {
    return (min.array() <= point.array() && point.array() <= max.array()).all();
  }

  PointType closestPointTo(const PointType& point) const {
    PointType closest_point = point.cwiseMax(min);
    closest_point = closest_point.cwiseMin(max);
    return closest_point;
  }
  PointType furthestPointFrom(const PointType& point) const {
    const PointType aabb_center = (min + max) / static_cast<ScalarType>(2);
    const PointType furthest_point =
        (aabb_center.array() < point.array()).select(min, max);
    return furthest_point;
  }

  FloatingPoint minSquaredDistanceTo(const PointType& point) const {
    return (point - closestPointTo(point)).squaredNorm();
  }
  FloatingPoint maxSquaredDistanceTo(const PointType& point) const {
    return (point - furthestPointFrom(point)).squaredNorm();
  }
  FloatingPoint minDistanceTo(const PointType& point) const {
    return (point - closestPointTo(point)).norm();
  }
  FloatingPoint maxDistanceTo(const PointType& point) const {
    return (point - furthestPointFrom(point)).norm();
  }

  template <int dim>
  ScalarType width() const {
    return max[dim] - min[dim];
  }

  Corners corner_matrix() const {
    Eigen::Matrix<FloatingPoint, kDim, kNumCorners> corners;
    for (int corner_idx = 0; corner_idx < kNumCorners; ++corner_idx) {
      for (int dim_idx = 0; dim_idx < kDim; ++dim_idx) {
        corners(dim_idx, corner_idx) = corner_coordinate(dim_idx, corner_idx);
      }
    }
    return corners;
  }

  PointType corner_point(int corner_idx) const {
    PointType corner;
    for (int dim_idx = 0; dim_idx < kDim; ++dim_idx) {
      corner[dim_idx] = corner_coordinate(dim_idx, corner_idx);
    }
    return corner;
  }

  ScalarType corner_coordinate(int dim_idx, int corner_idx) const {
    if (corner_idx & (0b1 << dim_idx)) {
      return max[dim_idx];
    } else {
      return min[dim_idx];
    }
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "[min =" << EigenFormat::oneLine(min)
       << ", max =" << EigenFormat::oneLine(max) << "]";
    return ss.str();
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_AABB_H_
