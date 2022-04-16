#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_

#include "wavemap_2d/indexing/index_conversions.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
inline FloatingPoint BeamModel::computeUpdateAt(const Index& index) const {
  const Point W_cell_center = computeCenterFromIndex(index, resolution_);
  const Point C_cell_center = W_cell_center - W_start_point_;

  // Compute the distance to the sensor
  const FloatingPoint distance = C_cell_center.norm();
  // Return early if the point is beyond the beam's max range
  if (kRangeMax < distance) {
    return 0.f;
  }
  // Return early if the point is inside the sensor or far behind the surface
  if (distance < kEpsilon ||
      measured_distance_ + kRangeDeltaThresh < distance) {
    return 0.f;
  }

  // Compute the angle w.r.t. the ray
  const FloatingPoint dot_prod_normalized =
      C_cell_center.dot(C_end_point_normalized_) / distance;
  // Return early if the point is behind the sensor
  if (dot_prod_normalized < 0.f) {
    return 0.f;
  }
  FloatingPoint angle;
  if (dot_prod_normalized <= 1.f) {
    // The normalized dot product is within the arc cosine's valid range
    angle = std::acos(dot_prod_normalized);
  } else {
    // Due to floating point precision, the normalized dot product can slightly
    // exceed 1.0 for points on the beam's centerline (i.e. if the angle is 0).
    angle = 0.f;
  }
  DCHECK(!std::isnan(angle));
  // Return early if the point is not inside the beam's non-zero region
  if (kAngleThresh < angle) {
    return 0.f;
  }

  // Compute the full measurement update
  const FloatingPoint g = angle / kAngleSigma;
  const FloatingPoint f = (distance - measured_distance_) / kRangeSigma;
  const FloatingPoint angle_contrib = Qcdf(g + 3.f) - Qcdf(g - 3.f);
  const FloatingPoint range_contrib = Qcdf(f) - 0.5f * Qcdf(f - 3.f) - 0.5f;
  const FloatingPoint scaled_contribs =
      kScaling * range_contrib * angle_contrib;
  const FloatingPoint p = scaled_contribs + 0.5f;
  const FloatingPoint log_odds = std::log(p / (1.f - p));
  DCHECK(!std::isnan(log_odds) && std::isfinite(log_odds));
  return log_odds;
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
