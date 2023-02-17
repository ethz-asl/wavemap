#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_2D_INL_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_2D_INL_H_

namespace wavemap {
inline bool CoarseToFineIntegrator2D::isApproximationErrorAcceptable(
    UpdateType update_type, FloatingPoint sphere_center_distance,
    FloatingPoint bounding_sphere_radius) const {
  switch (update_type) {
    case UpdateType::kFreeOrUnobserved:
      return bounding_sphere_radius / sphere_center_distance <
             kMaxAcceptableUpdateError / max_gradient_over_range_fully_inside_;
    case UpdateType::kPossiblyOccupied:
      return bounding_sphere_radius <
             kMaxAcceptableUpdateError / max_gradient_on_boundary_;
    default:
      return true;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_2D_INL_H_
