#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_IMPL_FIXED_RESOLUTION_INTEGRATOR_INL_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_IMPL_FIXED_RESOLUTION_INTEGRATOR_INL_H_

#include <algorithm>

namespace wavemap_2d {
inline FloatingPoint FixedResolutionIntegrator::computeUpdateForCell(
    const RangeImage& range_image, const Point& C_cell_center) {
  const FloatingPoint d_C_cell = C_cell_center.norm();
  if (d_C_cell < kEpsilon || BeamModel::kRangeMax < d_C_cell) {
    return 0.f;
  }

  const FloatingPoint cell_azimuth_angle =
      RangeImage::bearingToAngle(C_cell_center);

  const auto first_idx =
      std::max(0, range_image.angleToCeilIndex(cell_azimuth_angle -
                                               BeamModel::kAngleThresh));
  const auto last_idx =
      std::min(range_image.getNumBeams() - 1,
               range_image.angleToFloorIndex(cell_azimuth_angle +
                                             BeamModel::kAngleThresh));
  FloatingPoint total_update = 0.f;
  for (RangeImageIndex idx = first_idx; idx <= last_idx; ++idx) {
    const FloatingPoint measured_distance = range_image[idx];
    if (measured_distance + BeamModel::kRangeDeltaThresh < d_C_cell) {
      continue;
    }

    const FloatingPoint beam_azimuth_angle = range_image.indexToAngle(idx);
    const FloatingPoint cell_to_beam_angle =
        std::abs(cell_azimuth_angle - beam_azimuth_angle);
    if (BeamModel::kAngleThresh < cell_to_beam_angle) {
      continue;
    }

    total_update += BeamModel::computeUpdate(d_C_cell, cell_to_beam_angle,
                                             measured_distance);
  }

  return total_update;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_IMPL_FIXED_RESOLUTION_INTEGRATOR_INL_H_
