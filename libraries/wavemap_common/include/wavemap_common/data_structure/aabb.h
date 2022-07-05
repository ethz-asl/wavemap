#ifndef WAVEMAP_2D_DATA_STRUCTURE_GENERIC_AABB_H_
#define WAVEMAP_2D_DATA_STRUCTURE_GENERIC_AABB_H_

#include <limits>
#include <string>

#include "wavemap_common/common.h"
#include "wavemap_common/utils/eigen_format.h"
#include "wavemap_common/utils/int_math.h"

namespace wavemap {
template <typename T>
struct AABB {
  static constexpr int kDim = T::RowsAtCompileTime;
  static constexpr int kNumCorners = int_math::exp2(kDim);
  using ValueType = typename T::Scalar;
  using Corners = Eigen::Matrix<ValueType, kDim, kNumCorners>;

  T min = T::Constant(std::numeric_limits<ValueType>::max());
  T max = T::Constant(std::numeric_limits<ValueType>::lowest());

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
    const T aabb_center = (min + max) / static_cast<ValueType>(2);
    const T furthest_point =
        (aabb_center.array() < point.array()).select(min, max);
    return furthest_point;
  }

  FloatingPoint minSquaredDistanceTo(const T& point) const {
    return (point - closestPointTo(point)).squaredNorm();
  }
  FloatingPoint maxSquaredDistanceTo(const T& point) const {
    return (point - furthestPointFrom(point)).squaredNorm();
  }
  FloatingPoint minDistanceTo(const T& point) const {
    return (point - closestPointTo(point)).norm();
  }
  FloatingPoint maxDistanceTo(const T& point) const {
    return (point - furthestPointFrom(point)).norm();
  }

  template <int dim>
  ValueType width() const {
    return max[dim] - min[dim];
  }

  Corners corners() const {
    // TODO(victorr): Vectorize this
    Eigen::Matrix<FloatingPoint, kDim, kNumCorners> corners;
    for (int corner_idx = 0; corner_idx < kNumCorners; ++corner_idx) {
      for (int dim_idx = 0; dim_idx < kDim; ++dim_idx) {
        if (corner_idx & (0b1 << dim_idx)) {
          corners(dim_idx, corner_idx) = max[dim_idx];
        } else {
          corners(dim_idx, corner_idx) = min[dim_idx];
        }
      }
    }
    return corners;
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "[min =" << EigenFormat::oneLine(min)
       << ", max =" << EigenFormat::oneLine(max) << "]";
    return ss.str();
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_GENERIC_AABB_H_
