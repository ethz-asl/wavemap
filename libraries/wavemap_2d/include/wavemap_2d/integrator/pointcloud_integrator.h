#ifndef WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
#define WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_

#include <memory>

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"

namespace wavemap {
class PointcloudIntegrator {
 public:
  using Ptr = std::shared_ptr<PointcloudIntegrator>;

  PointcloudIntegrator() = delete;
  explicit PointcloudIntegrator(VolumetricDataStructure2D::Ptr occupancy_map)
      : occupancy_map_(CHECK_NOTNULL(occupancy_map)) {}
  virtual ~PointcloudIntegrator() = default;

  virtual void integratePointcloud(
      const PosedPointcloud<Point2D, Transformation2D>& pointcloud) = 0;

 protected:
  VolumetricDataStructure2D::Ptr occupancy_map_;

  static bool isPointcloudValid(
      const PosedPointcloud<Point2D, Transformation2D>& pointcloud);
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
