#include "wavemap_2d/integrator/pointcloud_integrator.h"

#include "wavemap_2d/integrator/grid_iterator.h"
#include "wavemap_2d/integrator/ray_iterator.h"

namespace wavemap_2d {
void PointcloudIntegrator::integratePointcloud(
    const PosedPointcloud& pointcloud) {
  measurement_model_->setStartPoint(pointcloud.getOrigin());
  for (const auto& point : pointcloud.getPointsGlobal()) {
    measurement_model_->setEndPoint(point);
    measurement_model_->updateMap(*occupancy_map_);
  }
}
}  // namespace wavemap_2d
