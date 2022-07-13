#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_

#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/utils/eigen_format.h>

namespace wavemap {
inline FloatingPoint BeamModel::computeUpdateAt(const Index2D& index) const {
  const Point2D W_cell_center =
      convert::indexToCenterPoint(index, min_cell_width_);
  const Point2D C_cell_center = W_cell_center - W_start_point_;

  // Compute the distance to the sensor
  const FloatingPoint d_C_cell = C_cell_center.norm();
  // Return early if the point is inside the sensor, beyond the beam's max
  // range, or far behind the surface
  if (d_C_cell < kEpsilon || kRangeMax < d_C_cell ||
      measured_distance_ + kRangeDeltaThresh < d_C_cell) {
    return 0.f;
  }

  // Compute the angle w.r.t. the ray
  // NOTE: For a method that works in 3D as well as 2D, see commit
  //       4abe1af9fc66cc069697727cc14577ade3abe092 or earlier.
  const FloatingPoint cell_angle =
      std::atan2(C_cell_center.y(), C_cell_center.x());
  const FloatingPoint cell_to_beam_angle = std::abs(cell_angle - beam_angle_);

  // Return early if the point is outside the beam's non-zero angular region
  if (kAngleThresh < cell_to_beam_angle) {
    return 0.f;
  }

  // Compute the full measurement update
  return computeUpdate(d_C_cell, cell_to_beam_angle, measured_distance_);
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
    return (1.f / 48.f) * t_plus_three * t_plus_three * t_plus_three;
  } else if (t < 1.f) {
    return (1.f / 2.f) + (1.f / 24.f) * t * t_plus_three * three_min_t;
  } else if (t <= 3.f) {
    return 1.f - (1.f / 48.f) * three_min_t * three_min_t * three_min_t;
  } else {
    return 1.f;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_IMPL_BEAM_MODEL_INL_H_
