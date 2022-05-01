#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_

#include <utility>

#include "wavemap_2d/data_structure/generic/aabb.h"
#include "wavemap_2d/integrator/pointcloud_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap_2d {
class FixedResolutionIntegrator : public PointcloudIntegrator {
 public:
  explicit FixedResolutionIntegrator(VolumetricDataStructure::Ptr occupancy_map)
      : PointcloudIntegrator(std::move(occupancy_map)) {}

  void integratePointcloud(const PosedPointcloud<>& pointcloud) override;

 private:
  static void computeRangeImageAndAABB(const PosedPointcloud<>& pointcloud,
                                       RangeImage& range_image,
                                       AABB<Point>& aabb);

  static FloatingPoint computeUpdateForCell(const RangeImage& range_image,
                                            const Point& C_cell_center);
};
}  // namespace wavemap_2d

#include "wavemap_2d/integrator/scan_integrator/fixed_resolution/impl/fixed_resolution_integrator_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_FIXED_RESOLUTION_FIXED_RESOLUTION_INTEGRATOR_H_
