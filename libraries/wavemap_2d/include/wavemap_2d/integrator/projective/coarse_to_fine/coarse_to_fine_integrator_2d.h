#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_2D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_2D_H_

#include <memory>

#include "wavemap_2d/data_structure/volumetric_quadtree_interface.h"
#include "wavemap_2d/integrator/projective/coarse_to_fine/range_image_1d_intersector.h"
#include "wavemap_2d/integrator/projective/range_image_1d.h"
#include "wavemap_2d/integrator/projective/scanwise_integrator_2d.h"

namespace wavemap {
class CoarseToFineIntegrator2D : public ScanwiseIntegrator2D {
 public:
  static constexpr FloatingPoint kMaxAcceptableUpdateError = 0.1f;

  explicit CoarseToFineIntegrator2D(
      const PointcloudIntegratorConfig& config,
      CircularProjector projection_model,
      ContinuousVolumetricLogOdds<2> measurement_model,
      VolumetricDataStructure2D::Ptr occupancy_map);

  void integratePointcloud(const PosedPointcloud<Point2D>& pointcloud) override;

 private:
  VolumetricQuadtreeInterface::Ptr volumetric_quadtree_;

  const FloatingPoint min_cell_width_;
  std::shared_ptr<RangeImage1DIntersector> range_image_intersector_;

  // TODO(victorr): Auto update these based on the projection model config
  const FloatingPoint max_gradient_over_range_fully_inside_ =
      measurement_model_.getConfig().scaling_free * 572.957795130823f;
  const FloatingPoint max_gradient_on_boundary_ =
      measurement_model_.getConfig().scaling_occupied * 14.9999999999997f;
  static constexpr FloatingPoint kUnitSquareHalfDiagonal = 1.41421356237f / 2.f;

  bool isApproximationErrorAcceptable(
      UpdateType update_type, FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius) const;
};
}  // namespace wavemap

#include "wavemap_2d/integrator/projective/coarse_to_fine/impl/coarse_to_fine_integrator_2d_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_2D_H_
