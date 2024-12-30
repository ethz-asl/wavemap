#ifndef WAVEMAP_CORE_UTILS_SHAPE_AABB_H_
#define WAVEMAP_CORE_UTILS_SHAPE_AABB_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/math/int_math.h"
#include "wavemap/core/utils/print/eigen.h"

namespace wavemap {
template <typename PointT>
struct AABB {
  static constexpr int kDim = dim_v<PointT>;
  static constexpr int kNumCorners = int_math::exp2(kDim);
  using PointType = PointT;
  using ScalarType = typename PointT::Scalar;
  using Corners = Eigen::Matrix<ScalarType, kDim, kNumCorners>;

  static constexpr auto kInitialMin = std::numeric_limits<ScalarType>::max();
  static constexpr auto kInitialMax = std::numeric_limits<ScalarType>::lowest();

  PointT min = PointT::Constant(kInitialMin);
  PointT max = PointT::Constant(kInitialMax);

  AABB() = default;
  AABB(const PointT& min, const PointT& max) : min(min), max(max) {}
  AABB(PointT&& min, PointT&& max) : min(std::move(min)), max(std::move(max)) {}

  void insert(const PointT& point) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }
  bool contains(const PointT& point) const {
    return (min.array() <= point.array() && point.array() <= max.array()).all();
  }

  PointT closestPointTo(const PointT& point) const {
    PointT closest_point = point.cwiseMax(min).cwiseMin(max);
    return closest_point;
  }
  PointT furthestPointFrom(const PointT& point) const {
    const PointT aabb_center = (min + max) / static_cast<ScalarType>(2);
    PointT furthest_point =
        (aabb_center.array() < point.array()).select(min, max);
    return furthest_point;
  }

  PointT minOffsetTo(const PointT& point) const {
    return point - closestPointTo(point);
  }
  PointT maxOffsetTo(const PointT& point) const {
    return point - furthestPointFrom(point);
  }
  // TODO(victorr): Check correctness with unit tests
  PointT minOffsetTo(const AABB& other) const {
    const PointT greatest_min = min.cwiseMax(other.min);
    const PointT smallest_max = max.cwiseMin(other.max);
    return (greatest_min - smallest_max).cwiseMax(0);
  }
  // TODO(victorr): Check correctness with unit tests. Pay particular
  //                attention to whether the offset signs are correct.
  PointT maxOffsetTo(const AABB& other) const {
    const PointT diff_1 = min - other.max;
    const PointT diff_2 = max - other.min;
    PointT offset =
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
  PointT widths() const { return max - min; }

  Corners corner_matrix() const {
    Eigen::Matrix<ScalarType, kDim, kNumCorners> corners;
    for (int corner_idx = 0; corner_idx < kNumCorners; ++corner_idx) {
      for (int dim_idx = 0; dim_idx < kDim; ++dim_idx) {
        corners(dim_idx, corner_idx) = corner_coordinate(dim_idx, corner_idx);
      }
    }
    return corners;
  }

  PointT corner_point(int corner_idx) const {
    PointT corner;
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

#endif  // WAVEMAP_CORE_UTILS_SHAPE_AABB_H_
