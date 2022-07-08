#include "wavemap_2d/integrator/pointcloud_integrator.h"

namespace wavemap {
bool PointcloudIntegrator::isPointcloudValid(
    const PosedPointcloud<Point2D, Transformation2D>& pointcloud) {
  if (const Point2D& origin = pointcloud.getOrigin(); origin.hasNaN()) {
    LOG(WARNING) << "Ignoring request to integrate pointcloud whose origin "
                    "contains NaNs:\n"
                 << origin;
    return false;
  }

  return true;
}
}  // namespace wavemap
