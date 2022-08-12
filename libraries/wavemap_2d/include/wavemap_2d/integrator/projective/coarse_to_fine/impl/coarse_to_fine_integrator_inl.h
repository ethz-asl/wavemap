#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_INL_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_INL_H_

namespace wavemap {
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
    default:
      return true;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_INL_H_
