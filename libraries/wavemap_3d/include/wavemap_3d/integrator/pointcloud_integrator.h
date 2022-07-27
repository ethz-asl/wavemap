#ifndef WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
#define WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_

#include <memory>

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"

namespace wavemap {
class PointcloudIntegrator {
 public:
  using Ptr = std::shared_ptr<PointcloudIntegrator>;

  PointcloudIntegrator() = delete;
  explicit PointcloudIntegrator(VolumetricDataStructure3D::Ptr occupancy_map)
      : occupancy_map_(CHECK_NOTNULL(occupancy_map)) {}
  virtual ~PointcloudIntegrator() = default;

  virtual void integratePointcloud(
      const PosedPointcloud<Point3D, Transformation3D>& pointcloud) = 0;

 protected:
  VolumetricDataStructure3D::Ptr occupancy_map_;

  static bool isPointcloudValid(
      const PosedPointcloud<Point3D, Transformation3D>& pointcloud);
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
