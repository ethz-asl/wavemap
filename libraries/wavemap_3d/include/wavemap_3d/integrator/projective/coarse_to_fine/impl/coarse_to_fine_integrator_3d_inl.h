#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_3D_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_3D_INL_H_

namespace wavemap {
inline bool CoarseToFineIntegrator3D::isApproximationErrorAcceptable(
    UpdateType update_type, FloatingPoint sphere_center_distance,
    FloatingPoint bounding_sphere_radius) const {
  switch (update_type) {
    case UpdateType::kFreeOrUnobserved:
      return bounding_sphere_radius < (kMaxAcceptableUpdateError /
                                       max_gradient_over_range_fully_inside_) *
                                          sphere_center_distance;
    case UpdateType::kPossiblyOccupied:
      return bounding_sphere_radius <
             kMaxAcceptableUpdateError / max_gradient_on_boundary_;
    default:
      return true;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_COARSE_TO_FINE_INTEGRATOR_3D_INL_H_
