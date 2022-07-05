#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_IMPL_FIXED_RESOLUTION_INTEGRATOR_INL_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_IMPL_FIXED_RESOLUTION_INTEGRATOR_INL_H_

#include <algorithm>

namespace wavemap {
inline FloatingPoint FixedResolutionIntegrator::computeUpdateForCell(
    const RangeImage& range_image, const Point& C_cell_center) {
  const FloatingPoint d_C_cell = C_cell_center.norm();
  if (d_C_cell < kEpsilon || BeamModel::kRangeMax < d_C_cell) {
    return 0.f;
  }

  const FloatingPoint cell_azimuth_angle =
      RangeImage::bearingToAngle(C_cell_center);
  const auto idx = range_image.angleToNearestIndex(cell_azimuth_angle);
  if (idx < 0 || range_image.getNumBeams() <= idx) {
    return 0.f;
  }

  const FloatingPoint measured_distance = range_image[idx];
  if (measured_distance + BeamModel::kRangeDeltaThresh < d_C_cell) {
    return 0.f;
  }

  const FloatingPoint beam_azimuth_angle = range_image.indexToAngle(idx);
  const FloatingPoint cell_to_beam_angle =
      cell_azimuth_angle - beam_azimuth_angle;
  if (BeamModel::kAngleThresh < cell_to_beam_angle) {
    return 0.f;
  }

  return BeamModel::computeUpdate(d_C_cell, cell_to_beam_angle,
                                  measured_distance);
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_IMPL_FIXED_RESOLUTION_INTEGRATOR_INL_H_
