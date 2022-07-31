#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_IMPL_SCAN_INTEGRATOR_INL_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_IMPL_SCAN_INTEGRATOR_INL_H_

#include "wavemap_2d/integrator/measurement_model/beam_model.h"

namespace wavemap {
inline FloatingPoint ScanIntegrator::sampleUpdateAtPoint(
    const RangeImage& range_image, const CircleProjector& circle_projector,
    FloatingPoint d_C_cell, FloatingPoint azimuth_angle_C_cell) {
  if (d_C_cell < kEpsilon || BeamModel::kRangeMax < d_C_cell) {
    return 0.f;
  }

  const IndexElement idx =
      circle_projector.angleToNearestIndex(azimuth_angle_C_cell);
  if (idx < 0 || range_image.getNumBeams() <= idx) {
    return 0.f;
  }
  const FloatingPoint measured_distance = range_image[idx];
  if (measured_distance + BeamModel::kRangeDeltaThresh < d_C_cell) {
    return 0.f;
  }

  const FloatingPoint beam_azimuth_angle = circle_projector.indexToAngle(idx);
  const FloatingPoint cell_to_beam_angle =
      std::abs(azimuth_angle_C_cell - beam_azimuth_angle);
  if (BeamModel::kAngleThresh < cell_to_beam_angle) {
    return 0.f;
  }

  return BeamModel::computeUpdate(d_C_cell, cell_to_beam_angle,
                                  measured_distance);
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_IMPL_SCAN_INTEGRATOR_INL_H_
