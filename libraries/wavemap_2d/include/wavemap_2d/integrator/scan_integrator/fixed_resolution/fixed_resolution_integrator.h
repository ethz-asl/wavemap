#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_

#include <utility>

#include <wavemap_common/data_structure/aabb.h>

#include "wavemap_2d/integrator/pointcloud_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap {
class FixedResolutionIntegrator : public PointcloudIntegrator {
 public:
  explicit FixedResolutionIntegrator(VolumetricDataStructure::Ptr occupancy_map)
      : PointcloudIntegrator(std::move(occupancy_map)) {}

  void integratePointcloud(const PosedPointcloud<>& pointcloud) override;

  static std::pair<RangeImage, AABB<Point2D>> computeRangeImageAndAABB(
      const PosedPointcloud<>& pointcloud, FloatingPoint min_angle,
      FloatingPoint max_angle, Eigen::Index num_beams);

 private:
  static FloatingPoint computeUpdateForCell(const RangeImage& range_image,
                                            const Point2D& C_cell_center);
};
}  // namespace wavemap

#include "wavemap_2d/integrator/scan_integrator/fixed_resolution/impl/fixed_resolution_integrator_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
