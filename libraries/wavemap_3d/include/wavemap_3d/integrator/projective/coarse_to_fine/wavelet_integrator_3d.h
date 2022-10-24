#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_3D_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_3D_H_

#include <memory>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"
#include "wavemap_3d/data_structure/wavelet_octree_interface.h"
#include "wavemap_3d/integrator/projective/coarse_to_fine/range_image_2d_intersector.h"
#include "wavemap_3d/integrator/projective/scanwise_integrator_3d.h"

namespace wavemap {
class WaveletIntegrator3D : public ScanwiseIntegrator3D {
 public:
  static constexpr FloatingPoint kMaxAcceptableUpdateError = 0.1f;

  WaveletIntegrator3D(
      const PointcloudIntegratorConfig& config,
      std::shared_ptr<const Image2DProjectionModel> projection_model,
      ContinuousVolumetricLogOdds<3> measurement_model,
      VolumetricDataStructure3D::Ptr occupancy_map);

  void integratePointcloud(const PosedPointcloud<Point3D>& pointcloud) override;

 private:
  WaveletOctreeInterface* wavelet_tree_;

  const FloatingPoint min_cell_width_;
  std::shared_ptr<RangeImage2DIntersector> range_image_intersector_;

  // TODO(victorr): Auto update these based on the projection model config
  const FloatingPoint max_gradient_over_range_fully_inside_ =
      measurement_model_.getConfig().scaling_free * 366.692988883727f;
  const FloatingPoint max_gradient_on_boundary_ =
      measurement_model_.getConfig().scaling_occupied * 3.75000000000002f;
  static constexpr FloatingPoint kUnitCubeHalfDiagonal = 1.73205080757f / 2.f;

  WaveletOctreeInterface::Coefficients::Scale recursiveSamplerCompressor(
      const OctreeIndex& node_index, FloatingPoint node_value,
      WaveletOctreeInterface::NodeType& parent_node,
      OctreeIndex ::RelativeChild relative_child_index,
      RangeImage2DIntersector::Cache cache);

  bool isApproximationErrorAcceptable(
      IntersectionType intersection_type, FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius) const;
};
}  // namespace wavemap

#include "wavemap_3d/integrator/projective/coarse_to_fine/impl/wavelet_integrator_3d_inl.h"

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_3D_H_
