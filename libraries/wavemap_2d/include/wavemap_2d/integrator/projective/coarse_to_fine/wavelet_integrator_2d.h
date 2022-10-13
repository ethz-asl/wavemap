#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_2D_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_2D_H_

#include <memory>

#include "wavemap_2d/data_structure/wavelet_quadtree_interface.h"
#include "wavemap_2d/integrator/projective/coarse_to_fine/range_image_1d_intersector.h"
#include "wavemap_2d/integrator/projective/range_image_1d.h"
#include "wavemap_2d/integrator/projective/scanwise_integrator_2d.h"

namespace wavemap {
class WaveletIntegrator2D : public ScanwiseIntegrator2D {
 public:
  static constexpr FloatingPoint kMaxAcceptableUpdateError = 0.1f;

  WaveletIntegrator2D(const PointcloudIntegratorConfig& config,
                      CircularProjector projection_model,
                      ContinuousVolumetricLogOdds<2> measurement_model,
                      VolumetricDataStructure2D::Ptr occupancy_map);

  void integratePointcloud(const PosedPointcloud<Point2D>& pointcloud) override;

 private:
  WaveletQuadtreeInterface* wavelet_tree_;

  const FloatingPoint min_cell_width_;
  std::shared_ptr<PosedRangeImage1D> posed_range_image_;
  std::shared_ptr<RangeImage1DIntersector> range_image_intersector_;

  const FloatingPoint max_gradient_over_range_fully_inside_ =
      measurement_model_.getConfig().scaling_free * 572.957795130823f;
  const FloatingPoint max_gradient_on_boundary_ =
      measurement_model_.getConfig().scaling_occupied * 14.9999999999997f;
  static constexpr FloatingPoint kUnitSquareHalfDiagonal = 1.41421356237f / 2.f;

  WaveletQuadtreeInterface::Coefficients::Scale recursiveSamplerCompressor(
      const QuadtreeIndex& node_index,
      WaveletQuadtreeInterface::NodeType& parent_node,
      QuadtreeIndex ::RelativeChild relative_child_index);

  bool isApproximationErrorAcceptable(
      IntersectionType intersection_type, FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius) const;
};
}  // namespace wavemap

#include "wavemap_2d/integrator/projective/coarse_to_fine/impl/wavelet_integrator_2d_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_2D_H_
