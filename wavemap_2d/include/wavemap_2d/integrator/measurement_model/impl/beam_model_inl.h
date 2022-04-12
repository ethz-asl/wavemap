#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_

#include "wavemap_2d/indexing/index_conversions.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
inline FloatingPoint BeamModel::computeUpdateAt(const Index& index) const {
  const Point W_cell_center = computeCenterFromIndex(index, resolution_);
  const Point C_cell_center = W_cell_center - W_start_point_;

  const FloatingPoint distance = C_cell_center.norm();
  if (kRangeMax < distance) {
    return 0.f;
  }

  if (kEpsilon < distance &&
      distance <= measured_distance_ + kRangeDeltaThresh) {
    const FloatingPoint dot_prod_normalized =
        C_cell_center.dot(C_end_point_normalized_) / distance;
    if (dot_prod_normalized < 0.f) {
      return 0.f;
    }
    FloatingPoint angle = 0.f;
    if (dot_prod_normalized <= 1.f) {
      angle = std::acos(dot_prod_normalized);
    }
    DCHECK(!std::isnan(angle));
    if (angle <= kAngleThresh) {
      const FloatingPoint g = angle / kAngleSigma;
      const FloatingPoint f = (distance - measured_distance_) / kRangeSigma;
      const FloatingPoint angle_contrib = Qcdf(g + 3.f) - Qcdf(g - 3.f);
      const FloatingPoint range_contrib = Qcdf(f) - 0.5f * Qcdf(f - 3.f) - 0.5f;
      const FloatingPoint contribs = range_contrib * angle_contrib;
      const FloatingPoint scaled_contribs = kScaling * contribs;
      const FloatingPoint p = scaled_contribs + 0.5f;
      const FloatingPoint log_odds = std::log(p / (1.f - p));
      DCHECK(!std::isnan(log_odds) && std::isfinite(log_odds));
      return log_odds;
    }
  }
  return 0.f;
}

inline FloatingPoint BeamModel::Qcdf(FloatingPoint t) {
  if (t < -3.f) {
    return 0.f;
  } else if (t <= -1.f) {
    return 1.f / 48.f * static_cast<FloatingPoint>(std::pow(t + 3.f, 3));
  } else if (t < 1.f) {
    return 1.f / 2.f + 1.f / 24.f * t * (3.f + t) * (3.f - t);
  } else if (t <= 3.f) {
    return 1.f - 1.f / 48.f * static_cast<FloatingPoint>(std::pow(3.f - t, 3));
  } else {
    return 1.f;
  }
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_
