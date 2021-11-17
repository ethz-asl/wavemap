#ifndef WAVEMAP_2D_BEAM_MODEL_H_
#define WAVEMAP_2D_BEAM_MODEL_H_

#include "wavemap_2d/common.h"

namespace wavemap_2d {
class BeamModel {
 public:
  explicit BeamModel(FloatingPoint resolution)
      : resolution_(resolution),
        resolution_inv_(1.f / resolution),
        measured_distance_(0.f) {}

  void setStartPoint(const Point& start_point) {
    W_start_point_ = start_point;
    updateCachedVariables();
  }
  void setEndPoint(const Point& end_point) {
    W_end_point_ = end_point;
    updateCachedVariables();
  }

  FloatingPoint getLength() const { return measured_distance_; }
  Index getBottomLeftUpdateIndex() {
    const Point bottom_left_point = W_start_point_.cwiseMin(W_end_point_) -
                                    Point::Constant(kRangeThreshold);
    const Point bottom_left_point_scaled = bottom_left_point * resolution_inv_;
    return bottom_left_point_scaled.array().floor().cast<IndexElement>();
  }
  Index getTopRightUpdateIndex() {
    const Point top_right_point = W_start_point_.cwiseMax(W_end_point_) +
                                  Point::Constant(kRangeThreshold);
    const Point top_right_point_scaled = top_right_point * resolution_inv_;
    return top_right_point_scaled.array().ceil().cast<IndexElement>();
  }

  FloatingPoint computeUpdateAt(const Index& index) {
    const Point W_cell_center = index.cast<FloatingPoint>() * resolution_;
    const Point C_cell_center = W_cell_center - W_start_point_;
    const FloatingPoint distance = C_cell_center.norm();

    if (kEpsilon < distance &&
        distance <= measured_distance_ + kRangeThreshold) {
      const FloatingPoint dot_prod_normalized =
          C_cell_center.dot(C_end_point_normalized_) / distance;
      const FloatingPoint angle = std::acos(dot_prod_normalized);
      if (angle <= kAngleThresh) {
        const FloatingPoint f = (distance - measured_distance_) / kSigmaRange;
        const FloatingPoint g = angle / kSigmaAngle;
        const FloatingPoint range_contrib =
            q_cdf(f) - 0.5f * q_cdf(f - 3.f) - 0.5f;
        const FloatingPoint angle_contrib = q_cdf(g + 3.f) - q_cdf(g - 3.f);
        const FloatingPoint contribs = range_contrib * angle_contrib;
        const FloatingPoint scaled_contribs =
            ((contribs < 0.f) ? kFreeScaling * contribs
                              : kOccScaling * contribs);
        const FloatingPoint p = scaled_contribs + 0.5f;
        const FloatingPoint odds = std::log(p / (1.f - p));
        return odds;

        // TODO(victorr): Set up comparisons using the equations below and DDA
        // if (distance < measured_distance_) {
        //   return -0.4f;
        // } else {
        //   return 0.85f;
        // }
      }
    }
    return 0.f;
  }

 protected:
  static constexpr FloatingPoint kRangeThreshold = 0.1f;
  static constexpr FloatingPoint kAngleThresh = 0.0174533f;
  static constexpr FloatingPoint kSigmaRange = kRangeThreshold / 3.f;
  static constexpr FloatingPoint kSigmaAngle = kAngleThresh / 3.f;
  static constexpr FloatingPoint kFreeScaling = 0.2f;
  static constexpr FloatingPoint kOccScaling = 0.4f;

  const FloatingPoint resolution_;
  const FloatingPoint resolution_inv_;

  Point W_start_point_;
  Point W_end_point_;

  FloatingPoint measured_distance_;
  Point C_end_point_;
  Point C_end_point_normalized_;

  void updateCachedVariables() {
    C_end_point_ = W_end_point_ - W_start_point_;
    measured_distance_ = C_end_point_.norm();
    C_end_point_normalized_ = C_end_point_ / measured_distance_;
  }

  static FloatingPoint q_cdf(const FloatingPoint t) {
    if (t < -3.f) {
      return 0.f;
    } else if (-3.f <= t && t <= -1.f) {
      return 1.f / 48.f * static_cast<FloatingPoint>(std::pow(3.f + t, 3));
    } else if (-1.f < t && t < 1.f) {
      return 1.f / 2.f + 1.f / 24.f * t * (3.f + t) * (3.f - t);
    } else if (1.f <= t && t <= 3.f) {
      return 1.f -
             1.f / 48.f * static_cast<FloatingPoint>(std::pow(3.f - t, 3));
    } else {
      return 1.f;
    }
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_BEAM_MODEL_H_
