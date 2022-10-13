#ifndef WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_ONLY_CONSTANT_1D_LOG_ODDS_H_
#define WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_ONLY_CONSTANT_1D_LOG_ODDS_H_

#include <wavemap_common/common.h>
#include <wavemap_common/indexing/index_conversions.h>

namespace wavemap {
template <int dim>
class Constant1DLogOdds {
 public:
  static constexpr FloatingPoint kLogOddsOccupied = 0.85f;
  static constexpr FloatingPoint kLogOddsFree = -0.4f;

  explicit Constant1DLogOdds(FloatingPoint min_cell_width)
      : min_cell_width_(min_cell_width) {}

  void setStartPoint(const Point<dim>& start_point) {
    W_start_point_ = start_point;
    measured_distance_ = (W_end_point_ - W_start_point_).norm();
    updateCachedVariables();
  }
  void setEndPoint(const Point<dim>& end_point) {
    W_end_point_ = end_point;
    measured_distance_ = (W_end_point_ - W_start_point_).norm();
    updateCachedVariables();
  }
  const Point<dim>& getStartPoint() const { return W_start_point_; }
  const Point<dim>& getEndPoint() const { return W_end_point_; }
  FloatingPoint getMeasuredDistance() const { return measured_distance_; }

  bool isMeasurementValid() const {
    if (W_end_point_.hasNaN()) {
      LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                   << W_end_point_;
      return false;
    }
    if (measured_distance_ < kEpsilon) {
      return false;
    }
    if (1e3 < measured_distance_) {
      LOG(INFO) << "Skipping measurement with suspicious length: "
                << measured_distance_;
      return false;
    }
    return true;
  }
  bool exceedsMaxRange(FloatingPoint max_range) const {
    return max_range < measured_distance_;
  }
  Point<dim> getEndPointOrMaxRange(FloatingPoint max_range) const {
    if (max_range < measured_distance_) {
      return W_start_point_ +
             max_range / measured_distance_ * (W_end_point_ - W_start_point_);
    } else {
      return W_end_point_;
    }
  }

  Index<dim> getBottomLeftUpdateIndex() const {
    const Point<dim> bottom_left_point = W_start_point_.cwiseMin(W_end_point_);
    return convert::pointToFloorIndex(bottom_left_point, min_cell_width_inv_);
  }
  Index<dim> getTopRightUpdateIndex() const {
    const Point<dim> top_right_point = W_start_point_.cwiseMax(W_end_point_);
    return convert::pointToCeilIndex(top_right_point, min_cell_width_inv_);
  }

  // NOTE: This method assumes queried indices always lies on the ray's line
  //       segment.
  FloatingPoint computeUpdate(const Index<dim>& index) const {
    if (index == end_point_index_) {
      return kLogOddsOccupied;
    } else {
      return kLogOddsFree;
    }
  }

 private:
  const FloatingPoint min_cell_width_;
  const FloatingPoint min_cell_width_inv_ = 1.f / min_cell_width_;

  Point<dim> W_start_point_ = Point<dim>::Zero();
  Point<dim> W_end_point_ = Point<dim>::Zero();
  FloatingPoint measured_distance_ = 0.f;

  Index<dim> end_point_index_;

  void updateCachedVariables() {
    end_point_index_ =
        convert::pointToNearestIndex(W_end_point_, min_cell_width_inv_);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_ONLY_CONSTANT_1D_LOG_ODDS_H_
