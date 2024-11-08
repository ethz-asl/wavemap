#ifndef WAVEMAP_CORE_DATA_STRUCTURE_AABB_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_AABB_H_

#include <algorithm>
#include <limits>
#include <string>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/math/int_math.h"
#include "wavemap/core/utils/print/eigen.h"

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

  AABB() = default;
  AABB(PointT min, PointT max) : min(min), max(max) {}

  void includePoint(const PointType& point) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }
  bool containsPoint(const PointType& point) const {
    return (min.array() <= point.array() && point.array() <= max.array()).all();
  }

  PointType closestPointTo(const PointType& point) const {
    PointType closest_point = point.cwiseMax(min).cwiseMin(max);
    return closest_point;
  }
  PointType furthestPointFrom(const PointType& point) const {
    const PointType aabb_center = (min + max) / static_cast<ScalarType>(2);
    PointType furthest_point =
        (aabb_center.array() < point.array()).select(min, max);
    return furthest_point;
  }

  PointType minOffsetTo(const PointType& point) const {
    return point - closestPointTo(point);
  }
  PointType maxOffsetTo(const PointType& point) const {
    return point - furthestPointFrom(point);
  }
  // TODO(victorr): Check correctness with unit tests
  PointType minOffsetTo(const AABB& aabb) const {
    const PointType greatest_min = min.cwiseMax(aabb.min);
    const PointType smallest_max = max.cwiseMin(aabb.max);
    return (greatest_min - smallest_max).cwiseMax(0);
  }
  // TODO(victorr): Check correctness with unit tests. Pay particular
  //                attention to whether the offset signs are correct.
  PointType maxOffsetTo(const AABB& aabb) const {
    const PointType diff_1 = min - aabb.max;
    const PointType diff_2 = max - aabb.min;
    PointType offset =
        (diff_2.array().abs() < diff_1.array().abs()).select(diff_1, diff_2);
    return offset;
  }

  template <typename GeometricEntityT>
  ScalarType minSquaredDistanceTo(const GeometricEntityT& entity) const {
    return minOffsetTo(entity).squaredNorm();
  }
  template <typename GeometricEntityT>
  ScalarType maxSquaredDistanceTo(const GeometricEntityT& entity) const {
    return maxOffsetTo(entity).squaredNorm();
  }

  template <typename GeometricEntityT>
  ScalarType minDistanceTo(const GeometricEntityT& entity) const {
    return minOffsetTo(entity).norm();
  }
  template <typename GeometricEntityT>
  ScalarType maxDistanceTo(const GeometricEntityT& entity) const {
    return maxOffsetTo(entity).norm();
  }

  template <int dim>
  ScalarType width() const {
    return max[dim] - min[dim];
  }
  PointType widths() const { return max - min; }

  Corners corner_matrix() const {
    Eigen::Matrix<ScalarType, kDim, kNumCorners> corners;
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
    if (bit_ops::is_bit_set(corner_idx, dim_idx)) {
      return max[dim_idx];
    } else {
      return min[dim_idx];
    }
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "[min =" << print::eigen::oneLine(min)
       << ", max =" << print::eigen::oneLine(max) << "]";
    return ss.str();
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_AABB_H_
