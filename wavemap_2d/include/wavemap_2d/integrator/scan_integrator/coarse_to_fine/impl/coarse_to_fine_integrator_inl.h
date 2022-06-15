#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_INL_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_INL_H_

#include <algorithm>

namespace wavemap_2d {
inline bool CoarseToFineIntegrator::isApproximationErrorAcceptable(
    RangeImageIntersector::IntersectionType intersection_type,
    FloatingPoint sphere_center_distance,
    FloatingPoint bounding_sphere_radius) {
  switch (intersection_type) {
    case RangeImageIntersector::IntersectionType::kFreeOrUnknown:
      return bounding_sphere_radius / sphere_center_distance <
             kMaxAcceptableUpdateError / kMaxGradientOverRangeFullyInside;
    case RangeImageIntersector::IntersectionType::kPossiblyOccupied:
      return bounding_sphere_radius <
             kMaxAcceptableUpdateError / kMaxGradientOnBoundary;
    case RangeImageIntersector::IntersectionType::kFullyUnknown:
    default:
      return true;
  }
}

inline FloatingPoint CoarseToFineIntegrator::computeUpdateForCell(
    const RangeImage& range_image, FloatingPoint d_C_cell,
    FloatingPoint azimuth_angle_C_cell) {
  if (d_C_cell < kEpsilon || BeamModel::kRangeMax < d_C_cell) {
    return 0.f;
  }

  const auto idx = range_image.angleToNearestIndex(azimuth_angle_C_cell);
  if (idx < 0 || range_image.getNumBeams() <= idx) {
    return 0.f;
  }
  const FloatingPoint measured_distance = range_image[idx];
  if (measured_distance + BeamModel::kRangeDeltaThresh < d_C_cell) {
    return 0.f;
  }

  const FloatingPoint beam_azimuth_angle = range_image.indexToAngle(idx);
  const FloatingPoint cell_to_beam_angle =
      std::abs(azimuth_angle_C_cell - beam_azimuth_angle);
  if (BeamModel::kAngleThresh < cell_to_beam_angle) {
    return 0.f;
  }

  return BeamModel::computeUpdate(d_C_cell, cell_to_beam_angle,
                                  measured_distance);
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_INL_H_
