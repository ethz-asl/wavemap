#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_3D_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_3D_INL_H_

namespace wavemap {
inline bool CoarseToFineIntegrator3D::isApproximationErrorAcceptable(
    IntersectionType intersection_type, FloatingPoint sphere_center_distance,
    FloatingPoint bounding_sphere_radius) const {
  switch (intersection_type) {
    case IntersectionType::kFreeOrUnknown:
      return bounding_sphere_radius / sphere_center_distance <
             kMaxAcceptableUpdateError / max_gradient_over_range_fully_inside_;
    case IntersectionType::kPossiblyOccupied:
      return bounding_sphere_radius <
             kMaxAcceptableUpdateError / max_gradient_on_boundary_;
    default:
      return true;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_3D_INL_H_
