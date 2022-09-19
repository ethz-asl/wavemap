#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_3D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_3D_H_

#include <memory>

#include <wavemap_common/integrator/projective/intersection_type.h>

#include "wavemap_3d/data_structure/volumetric_octree_interface.h"
#include "wavemap_3d/integrator/projective/scanwise_integrator_3d.h"

namespace wavemap {
class CoarseToFineIntegrator3D : public ScanwiseIntegrator3D {
 public:
  static constexpr FloatingPoint kMaxAcceptableUpdateError = 0.1f;

  explicit CoarseToFineIntegrator3D(
      VolumetricDataStructure3D::Ptr occupancy_map);

  void integratePointcloud(const PosedPointcloud<Point3D>& pointcloud) override;

 private:
  VolumetricOctreeInterface* volumetric_octree_;

  const FloatingPoint min_cell_width_;
  std::shared_ptr<PosedRangeImage2D> posed_range_image_;

  static constexpr FloatingPoint kMaxGradientOverRangeFullyInside =
      ContinuousVolumetricLogOdds<3>::kScaling * 572.957795130823f;
  static constexpr FloatingPoint kMaxGradientOnBoundary =
      ContinuousVolumetricLogOdds<3>::kScaling * 14.9999999999997f;
  static constexpr FloatingPoint kUnitCubeHalfDiagonal = 1.73205080757f / 2.f;

  static bool isApproximationErrorAcceptable(
      IntersectionType intersection_type, FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius);
};
}  // namespace wavemap

#include "wavemap_3d/integrator/projective/coarse_to_fine/impl/coarse_to_fine_integrator_3d_inl.h"

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_3D_H_
