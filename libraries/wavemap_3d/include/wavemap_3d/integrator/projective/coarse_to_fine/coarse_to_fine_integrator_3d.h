#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_3D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_3D_H_

#include <memory>

#include <wavemap_common/integrator/projective/intersection_type.h>

#include "wavemap_3d/data_structure/volumetric_octree_interface.h"
#include "wavemap_3d/integrator/projective/coarse_to_fine/range_image_2d_intersector.h"
#include "wavemap_3d/integrator/projective/scanwise_integrator_3d.h"

namespace wavemap {
class CoarseToFineIntegrator3D : public ScanwiseIntegrator3D {
 public:
  static constexpr FloatingPoint kMaxAcceptableUpdateError = 0.1f;

  CoarseToFineIntegrator3D(
      const PointcloudIntegratorConfig& config,
      std::shared_ptr<const Image2DProjectionModel> projection_model,
      ContinuousVolumetricLogOdds<3> measurement_model,
      VolumetricDataStructure3D::Ptr occupancy_map);

  void integratePointcloud(const PosedPointcloud<Point3D>& pointcloud) override;

 private:
  VolumetricOctreeInterface* volumetric_octree_;

  const FloatingPoint min_cell_width_;
  std::shared_ptr<RangeImage2DIntersector> range_image_intersector_;

  // TODO(victorr): Auto update these based on the projection model config
  const FloatingPoint max_gradient_over_range_fully_inside_ =
      measurement_model_.getConfig().scaling_free * 366.692988883727f;
  const FloatingPoint max_gradient_on_boundary_ =
      measurement_model_.getConfig().scaling_occupied * 3.75000000000002f;
  static constexpr FloatingPoint kUnitCubeHalfDiagonal = 1.73205080757f / 2.f;

  bool isApproximationErrorAcceptable(
      IntersectionType intersection_type, FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius) const;
};
}  // namespace wavemap

#include "wavemap_3d/integrator/projective/coarse_to_fine/impl/coarse_to_fine_integrator_3d_inl.h"

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_3D_H_
