#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_

#include <memory>

#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/data_structure/volumetric/wavelet_octree_interface.h"
#include "wavemap/integrator/projective/coarse_to_fine/range_image_2d_intersector.h"
#include "wavemap/integrator/projective/scanwise_integrator.h"

namespace wavemap {
class WaveletIntegrator : public ScanwiseIntegrator {
 public:
  static constexpr FloatingPoint kMaxAcceptableUpdateError = 0.1f;

  WaveletIntegrator(
      const PointcloudIntegratorConfig& config,
      std::shared_ptr<const Image2DProjectionModel> projection_model,
      ContinuousVolumetricLogOdds measurement_model,
      VolumetricDataStructureBase::Ptr occupancy_map);

 private:
  WaveletOctreeInterface::Ptr wavelet_tree_;

  const FloatingPoint min_cell_width_;
  std::shared_ptr<RangeImage2DIntersector> range_image_intersector_;

  // TODO(victorr): Move this to the measurement model
  const FloatingPoint max_gradient_over_range_fully_inside_ =
      measurement_model_.getConfig().scaling_free * 366.692988883727f;
  const FloatingPoint max_gradient_on_boundary_ =
      measurement_model_.getConfig().scaling_occupied * 3.75000000000002f;
  static constexpr FloatingPoint kUnitCubeHalfDiagonal = 1.73205080757f / 2.f;

  WaveletOctreeInterface::Coefficients::Scale recursiveSamplerCompressor(
      const OctreeIndex& node_index, FloatingPoint node_value,
      WaveletOctreeInterface::NodeType& parent_node,
      OctreeIndex ::RelativeChild relative_child_index);

  bool isApproximationErrorAcceptable(
      UpdateType update_type, FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius) const;

  void updateMap() override;
};
}  // namespace wavemap

#include "wavemap/integrator/projective/coarse_to_fine/impl/wavelet_integrator_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_
