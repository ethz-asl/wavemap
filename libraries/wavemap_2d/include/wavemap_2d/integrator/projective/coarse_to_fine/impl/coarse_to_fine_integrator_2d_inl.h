#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_2D_INL_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_2D_INL_H_

namespace wavemap {
inline bool CoarseToFineIntegrator2D::isApproximationErrorAcceptable(
    RangeImage1DIntersector::IntersectionType intersection_type,
    FloatingPoint sphere_center_distance,
    FloatingPoint bounding_sphere_radius) {
  switch (intersection_type) {
    case RangeImage1DIntersector::IntersectionType::kFreeOrUnknown:
      return bounding_sphere_radius / sphere_center_distance <
             kMaxAcceptableUpdateError / kMaxGradientOverRangeFullyInside;
    case RangeImage1DIntersector::IntersectionType::kPossiblyOccupied:
      return bounding_sphere_radius <
             kMaxAcceptableUpdateError / kMaxGradientOnBoundary;
    default:
      return true;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_2D_INL_H_
