#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_

#include <memory>

#include "wavemap/data_structure/volumetric/volumetric_octree_interface.h"
#include "wavemap/integrator/projective/coarse_to_fine/range_image_2d_intersector.h"
#include "wavemap/integrator/projective/scanwise_integrator.h"
#include "wavemap/integrator/projective/update_type.h"

namespace wavemap {
class CoarseToFineIntegrator : public ScanwiseIntegrator {
 public:
  static constexpr FloatingPoint kMaxAcceptableUpdateError = 0.1f;

  CoarseToFineIntegrator(
      const PointcloudIntegratorConfig& config,
      std::shared_ptr<const Image2DProjectionModel> projection_model,
      ContinuousVolumetricLogOdds measurement_model,
      VolumetricDataStructureBase::Ptr occupancy_map);

 private:
  VolumetricOctreeInterface::Ptr volumetric_octree_;

  const FloatingPoint min_cell_width_;
  std::shared_ptr<RangeImage2DIntersector> range_image_intersector_;

  // TODO(victorr): Move this to the measurement model
  const FloatingPoint max_gradient_over_range_fully_inside_ =
      measurement_model_.getConfig().scaling_free * 366.692988883727f;
  const FloatingPoint max_gradient_on_boundary_ =
      measurement_model_.getConfig().scaling_occupied * 3.75000000000002f;
  static constexpr FloatingPoint kUnitCubeHalfDiagonal = 1.73205080757f / 2.f;

  bool isApproximationErrorAcceptable(
      UpdateType update_type, FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius) const;

  void updateMap() override;
};
}  // namespace wavemap

#include "wavemap/integrator/projective/coarse_to_fine/impl/coarse_to_fine_integrator_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_
