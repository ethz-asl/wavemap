#include "wavemap_3d/integrator/pointcloud_integrator.h"

namespace wavemap {
bool PointcloudIntegrator::isPointcloudValid(
    const wavemap::PosedPointcloud<wavemap::Point3D, wavemap::Transformation3D>&
        pointcloud) {
  if (const Point3D& origin = pointcloud.getOrigin(); origin.hasNaN()) {
    LOG(WARNING) << "Ignoring request to integrate pointcloud whose origin "
                    "contains NaNs:\n"
                 << origin;
    return false;
  }

  return true;
}
}  // namespace wavemap
