#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_SCAN_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_SCAN_INTEGRATOR_H_

#include <utility>

#include "wavemap_2d/data_structure/generic/aabb.h"
#include "wavemap_2d/data_structure/generic/range_image.h"
#include "wavemap_2d/integrator/pointcloud_integrator.h"

namespace wavemap_2d {
class ScanIntegrator : public PointcloudIntegrator {
 public:
  explicit ScanIntegrator(VolumetricDataStructure::Ptr occupancy_map)
      : PointcloudIntegrator(std::move(occupancy_map)) {}

  void integratePointcloud(const PosedPointcloud<>& pointcloud) override;

 private:
  static void computeRangeImageAndAABB(const PosedPointcloud<>& pointcloud,
                                       RangeImage& range_image, AABB& aabb);

  static FloatingPoint computeUpdateForCell(const RangeImage& range_image,
                                            const Point& C_cell_center);
};
}  // namespace wavemap_2d

#include "wavemap_2d/integrator/scan_integrator/impl/scan_integrator_inl.h"

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_SCAN_INTEGRATOR_H_
