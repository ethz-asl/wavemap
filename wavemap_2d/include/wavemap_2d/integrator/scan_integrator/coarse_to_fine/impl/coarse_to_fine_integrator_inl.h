#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_INL_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_INL_H_

#include <algorithm>

namespace wavemap_2d {
inline FloatingPoint CoarseToFineIntegrator::computeMaxApproximationError(
    RangeImageIntersector::IntersectionType intersection_type,
    FloatingPoint sphere_center_distance, FloatingPoint sphere_diameter) {
  switch (intersection_type) {
    case RangeImageIntersector::IntersectionType::kFullyInside:
      return kMaxGradientOverRangeFullyInside * sphere_center_distance *
             sphere_diameter;
    case RangeImageIntersector::IntersectionType::kIntersectsBoundary:
      return kMaxGradientOverRangeOnBoundary * sphere_center_distance *
             sphere_diameter;
    case RangeImageIntersector::IntersectionType::kFullyOutside:
    default:
      return 0.f;
  }
}

inline FloatingPoint CoarseToFineIntegrator::computeUpdateForCell(
    const RangeImage& range_image, const Point& C_cell_center) {
  const FloatingPoint cell_to_sensor_distance = C_cell_center.norm();
  const FloatingPoint cell_azimuth_angle =
      RangeImage::bearingToAngle(C_cell_center);

  const auto first_idx =
      std::max(0, range_image.angleToCeilIndex(cell_azimuth_angle -
                                               BeamModel::kAngleThresh));
  const auto last_idx =
      std::min(range_image.getNBeams() - 1,
               range_image.angleToFloorIndex(cell_azimuth_angle +
                                             BeamModel::kAngleThresh));
  FloatingPoint total_update = 0.f;
  for (RangeImageIndex idx = first_idx; idx <= last_idx; ++idx) {
    const FloatingPoint measured_distance = range_image[idx];
    if (BeamModel::kRangeMax < cell_to_sensor_distance) {
      continue;
    }
    if (cell_to_sensor_distance < kEpsilon ||
        measured_distance + BeamModel::kRangeDeltaThresh <
            cell_to_sensor_distance) {
      continue;
    }

    const FloatingPoint beam_azimuth_angle = range_image.indexToAngle(idx);
    const FloatingPoint cell_to_beam_angle =
        std::abs(cell_azimuth_angle - beam_azimuth_angle);
    if (BeamModel::kAngleThresh < cell_to_beam_angle) {
      continue;
    }

    total_update += BeamModel::computeUpdate(
        cell_to_sensor_distance, cell_to_beam_angle, measured_distance);
  }

  return total_update;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_INL_H_
