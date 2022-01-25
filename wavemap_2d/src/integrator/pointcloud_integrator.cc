#include "wavemap_2d/integrator/pointcloud_integrator.h"

#include "wavemap_2d/integrator/grid_iterator.h"
#include "wavemap_2d/integrator/ray_iterator.h"

namespace wavemap_2d {
void PointcloudIntegrator::integratePointcloud(
    const PosedPointcloud& pointcloud) {
  const Point origin = pointcloud.getOrigin();
  if (origin.hasNaN()) {
    LOG(WARNING) << "Ignoring request to integrate pointcloud whose origin "
                    "contains NaNs:\n"
                 << origin;
    return;
  }
  measurement_model_->setStartPoint(origin);
  for (const auto& point : pointcloud.getPointsGlobal()) {
    if (point.hasNaN()) {
      LOG(WARNING) << "Skipping beam whose endpoint contains NaNs:\n" << origin;
      continue;
    }
    const FloatingPoint ray_length = (point - origin).norm();
    if (1e3 < ray_length) {
      LOG(INFO) << "Skipping beam with suspicious length: " << ray_length;
      continue;
    }
    measurement_model_->setEndPoint(point);
    measurement_model_->updateMap(*occupancy_map_);
  }
}
}  // namespace wavemap_2d
