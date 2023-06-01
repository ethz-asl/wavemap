#include "wavemap/integrator/integrator_base.h"

namespace wavemap {
bool IntegratorBase::isPointcloudValid(const PosedPointcloud<>& pointcloud) {
  if (const Point3D& origin = pointcloud.getOrigin(); origin.hasNaN()) {
    LOG(WARNING) << "Ignoring request to integrate pointcloud whose origin "
                    "contains NaNs:\n"
                 << origin;
    return false;
  }

  return true;
}
}  // namespace wavemap
