#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_

#include "wavemap_2d/indexing/index_conversions.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
inline FloatingPoint BeamModel::computeUpdateAt(const Index& index) const {
  const Point W_cell_center =
      convert::indexToCenterPoint(index, min_cell_width_);
  const Point C_cell_center = W_cell_center - W_start_point_;

  // Compute the distance to the sensor
  const FloatingPoint cell_to_sensor_distance = C_cell_center.norm();
  // Return early if the point is beyond the beam's max range
  if (kRangeMax < cell_to_sensor_distance) {
    return 0.f;
  }
  // Return early if the point is inside the sensor or far behind the surface
  if (cell_to_sensor_distance < kEpsilon ||
      measured_distance_ + kRangeDeltaThresh < cell_to_sensor_distance) {
    return 0.f;
  }

  // Compute the angle w.r.t. the ray
  // NOTE: This relative angle computation method works in 2D and 3D.
  const FloatingPoint dot_prod_normalized =
      C_cell_center.dot(C_end_point_normalized_) / cell_to_sensor_distance;
  // Return early if the point is behind the sensor
  if (dot_prod_normalized < 0.f) {
    return 0.f;
  }
  FloatingPoint cell_to_beam_angle;
  if (dot_prod_normalized <= 1.f) {
    // The normalized dot product is within the arc cosine's valid range
    cell_to_beam_angle = std::acos(dot_prod_normalized);
  } else {
    // Due to floating point precision, the normalized dot product can slightly
    // exceed 1.0 for points on the beam's centerline (i.e. if the angle is 0).
    cell_to_beam_angle = 0.f;
  }
  DCHECK(!std::isnan(cell_to_beam_angle));
  // Return early if the point is outside the beam's non-zero angular region
  if (kAngleThresh < cell_to_beam_angle) {
    return 0.f;
  }

  // Compute the full measurement update
  return computeUpdate(cell_to_sensor_distance, cell_to_beam_angle,
                       measured_distance_);
}

inline FloatingPoint BeamModel::computeUpdate(
    FloatingPoint cell_to_sensor_distance, FloatingPoint cell_to_beam_angle,
    FloatingPoint measured_distance) {
  // Compute the full measurement update
  const FloatingPoint g = cell_to_beam_angle / kAngleSigma;
  const FloatingPoint f =
      (cell_to_sensor_distance - measured_distance) / kRangeSigma;
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
  const FloatingPoint t_plus_three = t + 3.f;
  const FloatingPoint three_min_t = 3.f - t;
  if (t < -3.f) {
    return 0.f;
  } else if (t <= -1.f) {
    return 1.f / 48.f * t_plus_three * t_plus_three * t_plus_three;
  } else if (t < 1.f) {
    return 1.f / 2.f + 1.f / 24.f * t * t_plus_three * three_min_t;
  } else if (t <= 3.f) {
    return 1.f - 1.f / 48.f * three_min_t * three_min_t * three_min_t;
  } else {
    return 1.f;
  }
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_
