#include "wavemap_2d/integrator/pointcloud_integrator.h"

namespace wavemap_2d {
bool PointcloudIntegrator::isPointcloudValid(
    const PosedPointcloud<>& pointcloud) {
  const Point& origin = pointcloud.getOrigin();
  if (origin.hasNaN()) {
    LOG(WARNING) << "Ignoring request to integrate pointcloud whose origin "
                    "contains NaNs:\n"
                 << origin;
    return false;
  }
  return true;
}
}  // namespace wavemap_2d
