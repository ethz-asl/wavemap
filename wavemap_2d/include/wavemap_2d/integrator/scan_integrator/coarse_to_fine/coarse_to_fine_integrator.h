#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_

#include <algorithm>
#include <utility>

#include "wavemap_2d/integrator/pointcloud_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/range_image_intersector.h"
#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap_2d {
class CoarseToFineIntegrator : public PointcloudIntegrator {
 public:
  explicit CoarseToFineIntegrator(VolumetricDataStructure::Ptr occupancy_map)
      : PointcloudIntegrator(std::move(occupancy_map)) {}

  void integratePointcloud(const PosedPointcloud<>& pointcloud) override;

  static RangeImage computeRangeImage(const PosedPointcloud<>& pointcloud,
                                      FloatingPoint min_angle,
                                      FloatingPoint max_angle,
                                      Eigen::Index num_beams);

 private:
  static constexpr QuadtreeIndex::Element kMaxDepth = 14;
  static constexpr FloatingPoint kMaxAcceptableError = 1e-2;
  static constexpr FloatingPoint kMaxGradientOverRangeFullyInside = 1e-3;
  static constexpr FloatingPoint kMaxGradientOverRangeOnBoundary = 1e-3;

  static FloatingPoint computeMaxApproximationError(
      RangeImageIntersector::IntersectionType intersection_type,
      FloatingPoint sphere_center_distance, FloatingPoint sphere_diameter);

  static FloatingPoint computeUpdateForCell(const RangeImage& range_image,
                                            const Point& C_cell_center);
};
}  // namespace wavemap_2d

#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/impl/coarse_to_fine_integrator_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_COARSE_TO_FINE_INTEGRATOR_H_