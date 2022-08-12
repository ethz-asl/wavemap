#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_

#include <memory>
#include <utility>

#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>

#include "wavemap_2d/data_structure/wavelet_quadtree_interface.h"
#include "wavemap_2d/integrator/projective/coarse_to_fine/range_image_intersector.h"
#include "wavemap_2d/integrator/projective/range_image_1d.h"
#include "wavemap_2d/integrator/projective/scanwise_integrator_2d.h"

namespace wavemap {
class WaveletIntegrator : public ScanwiseIntegrator2D {
 public:
  static constexpr FloatingPoint kMaxAcceptableUpdateError = 0.1f;

  explicit WaveletIntegrator(VolumetricDataStructure2D::Ptr occupancy_map);

  void integratePointcloud(
      const PosedPointcloud<Point2D, Transformation2D>& pointcloud) override;

 private:
  WaveletQuadtreeInterface* wavelet_tree_;

  const FloatingPoint min_cell_width_;
  std::shared_ptr<PosedRangeImage1D> posed_range_image_;
  std::shared_ptr<RangeImageIntersector> range_image_intersector_;

  static constexpr FloatingPoint kMaxGradientOverRangeFullyInside =
      ContinuousVolumetricLogOdds<2>::kScaling * 572.957795130823f;
  static constexpr FloatingPoint kMaxGradientOnBoundary =
      ContinuousVolumetricLogOdds<2>::kScaling * 14.9999999999997f;
  static constexpr FloatingPoint kUnitCubeHalfDiagonal = 1.41421356237f / 2.f;

  WaveletQuadtreeInterface::Coefficients::Scale recursiveSamplerCompressor(
      const QuadtreeIndex& node_index,
      WaveletQuadtreeInterface::NodeType& parent_node,
      QuadtreeIndex ::RelativeChild relative_child_index);

  static bool isApproximationErrorAcceptable(
      RangeImageIntersector::IntersectionType intersection_type,
      FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius);
};
}  // namespace wavemap

#include "wavemap_2d/integrator/projective/coarse_to_fine/impl/wavelet_integrator_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_
