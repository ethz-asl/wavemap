#ifndef WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONSTANT_RAY_H_
#define WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONSTANT_RAY_H_

#include "wavemap/common.h"
#include "wavemap/config/config_base.h"
#include "wavemap/indexing/index_conversions.h"

namespace wavemap {
/**
 * Config struct for the constant ray measurement model.
 */
struct ConstantRayConfig : ConfigBase<ConstantRayConfig, 2> {
  FloatingPoint log_odds_occupied = 0.85f;
  FloatingPoint log_odds_free = -0.4f;

  static MemberMap memberMap;

  // Constructors
  ConstantRayConfig() = default;
  ConstantRayConfig(FloatingPoint log_odds_occupied,
                    FloatingPoint log_odds_free)
      : log_odds_occupied(log_odds_occupied), log_odds_free(log_odds_free) {}

  bool isValid(bool /*verbose*/) const override { return true; }
};

class ConstantRay {
 public:
  explicit ConstantRay(FloatingPoint min_cell_width)
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

  // NOTE: This method assumes queried indices always lies on the ray's line
  //       segment.
  FloatingPoint computeUpdate(const Index3D& index) const {
    if (index == end_point_index_) {
      return config_.log_odds_occupied;
    } else {
      return config_.log_odds_free;
    }
  }

 private:
  const ConstantRayConfig config_;

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

#endif  // WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_CONSTANT_RAY_H_
