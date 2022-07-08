#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_

#include <memory>
#include <utility>

#include "wavemap_2d/data_structure/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/wavelet_tree_interface.h"
#include "wavemap_2d/integrator/pointcloud_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/range_image_intersector.h"
#include "wavemap_2d/integrator/scan_integrator/posed_range_image.h"

namespace wavemap {
class WaveletIntegrator : public PointcloudIntegrator {
 public:
  static constexpr FloatingPoint kMaxAcceptableUpdateError = 0.1f;

  explicit WaveletIntegrator(VolumetricDataStructure::Ptr occupancy_map);

  void integratePointcloud(
      const PosedPointcloud<Point2D, Transformation2D>& pointcloud) override;

 private:
  WaveletTreeInterface* wavelet_tree_;

  const FloatingPoint min_cell_width_;
  std::shared_ptr<PosedRangeImage> posed_range_image_;
  std::shared_ptr<RangeImageIntersector> range_image_intersector_;

  static constexpr FloatingPoint kMaxGradientOverRangeFullyInside =
      BeamModel::kScaling * 572.957795130823f;
  static constexpr FloatingPoint kMaxGradientOnBoundary =
      BeamModel::kScaling * 14.9999999999997f;
  static constexpr FloatingPoint kUnitCubeHalfDiagonal = 1.41421356237f / 2.f;

  WaveletTreeInterface::ScaleCoefficient recursiveSamplerCompressor(
      const QuadtreeIndex& node_index,
      WaveletTreeInterface::NodeType& parent_node,
      QuadtreeIndex ::RelativeChild relative_child_index);

  static bool isApproximationErrorAcceptable(
      RangeImageIntersector::IntersectionType intersection_type,
      FloatingPoint sphere_center_distance,
      FloatingPoint bounding_sphere_radius);

  static FloatingPoint sampleUpdateAtPoint(const RangeImage& range_image,
                                           FloatingPoint d_C_cell,
                                           FloatingPoint azimuth_angle_C_cell);
};
}  // namespace wavemap

#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/impl/wavelet_integrator_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_
