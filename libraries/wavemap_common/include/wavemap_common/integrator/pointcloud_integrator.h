#ifndef WAVEMAP_COMMON_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_

#include <memory>

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>

#include "wavemap_common/data_structure/volumetric/volumetric_data_structure_base.h"

namespace wavemap {
template <int dim>
class PointcloudIntegrator {
 public:
  using Ptr = std::shared_ptr<PointcloudIntegrator>;

  PointcloudIntegrator() = delete;
  explicit PointcloudIntegrator(
      typename VolumetricDataStructureBase<dim>::Ptr occupancy_map)
      : occupancy_map_(CHECK_NOTNULL(occupancy_map)) {}
  virtual ~PointcloudIntegrator() = default;

  virtual void integratePointcloud(
      const PosedPointcloud<Point<dim>>& pointcloud) = 0;

 protected:
  typename VolumetricDataStructureBase<dim>::Ptr occupancy_map_;

  static bool isPointcloudValid(const PosedPointcloud<Point<dim>>& pointcloud) {
    if (const Point<dim>& origin = pointcloud.getOrigin(); origin.hasNaN()) {
      LOG(WARNING) << "Ignoring request to integrate pointcloud whose origin "
                      "contains NaNs:\n"
                   << origin;
      return false;
    }

    return true;
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_POINTCLOUD_INTEGRATOR_H_
