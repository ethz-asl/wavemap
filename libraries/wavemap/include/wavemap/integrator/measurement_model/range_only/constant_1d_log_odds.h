#ifndef WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_RANGE_ONLY_CONSTANT_1D_LOG_ODDS_H_
#define WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_RANGE_ONLY_CONSTANT_1D_LOG_ODDS_H_

#include "wavemap/common.h"
#include "wavemap/indexing/index_conversions.h"

namespace wavemap {
class Constant1DLogOdds {
 public:
  static constexpr FloatingPoint kLogOddsOccupied = 0.85f;
  static constexpr FloatingPoint kLogOddsFree = -0.4f;

  explicit Constant1DLogOdds(FloatingPoint min_cell_width)
      : min_cell_width_(min_cell_width) {}

  void setStartPoint(const Point3D& start_point) {
    W_start_point_ = start_point;
    updateCachedVariables();
  }
  void setEndPoint(const Point3D& end_point) {
    W_end_point_ = end_point;
    updateCachedVariables();
  }
  const Point3D& getStartPoint() const { return W_start_point_; }
  const Point3D& getEndPoint() const { return W_end_point_; }
  FloatingPoint getMeasuredDistance() const { return measured_distance_; }

  Index3D getBottomLeftUpdateIndex() const {
    const Point3D bottom_left_point = W_start_point_.cwiseMin(W_end_point_);
    return convert::pointToFloorIndex(bottom_left_point, min_cell_width_inv_);
  }
  Index3D getTopRightUpdateIndex() const {
    const Point3D top_right_point = W_start_point_.cwiseMax(W_end_point_);
    return convert::pointToCeilIndex(top_right_point, min_cell_width_inv_);
  }

  // NOTE: This method assumes queried indices always lies on the ray's line
  //       segment.
  FloatingPoint computeUpdate(const Index3D& index) const {
    if (index == end_point_index_) {
      return kLogOddsOccupied;
    } else {
      return kLogOddsFree;
    }
  }

 private:
  const FloatingPoint min_cell_width_;
  const FloatingPoint min_cell_width_inv_ = 1.f / min_cell_width_;

  Point3D W_start_point_ = Point3D::Zero();
  Point3D W_end_point_ = Point3D::Zero();
  FloatingPoint measured_distance_ = 0.f;

  Index3D end_point_index_;

  void updateCachedVariables() {
    measured_distance_ = (W_end_point_ - W_start_point_).norm();
    end_point_index_ =
        convert::pointToNearestIndex(W_end_point_, min_cell_width_inv_);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_RANGE_ONLY_CONSTANT_1D_LOG_ODDS_H_
