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

  explicit WaveletIntegrator2D(VolumetricDataStructure2D::Ptr occupancy_map);

  void integratePointcloud(const PosedPointcloud<Point2D>& pointcloud) override;

 private:
  WaveletQuadtreeInterface* wavelet_tree_;

  const FloatingPoint min_cell_width_;
  std::shared_ptr<PosedRangeImage1D> posed_range_image_;
  std::shared_ptr<RangeImage1DIntersector> range_image_intersector_;

  static constexpr FloatingPoint kMaxGradientOverRangeFullyInside =
      ContinuousVolumetricLogOdds<2>::kScalingFree * 572.957795130823f;
  static constexpr FloatingPoint kMaxGradientOnBoundary =
      ContinuousVolumetricLogOdds<2>::kScalingOccupied * 14.9999999999997f;
  static constexpr FloatingPoint kUnitSquareHalfDiagonal = 1.41421356237f / 2.f;

  WaveletQuadtreeInterface::Coefficients::Scale recursiveSamplerCompressor(
      const QuadtreeIndex& node_index,
      WaveletQuadtreeInterface::NodeType& parent_node,
      QuadtreeIndex ::RelativeChild relative_child_index);

  static bool isApproximationErrorAcceptable(
      IntersectionType intersection_type, FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius);
};
}  // namespace wavemap

#include "wavemap_2d/integrator/projective/coarse_to_fine/impl/wavelet_integrator_2d_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_2D_H_
