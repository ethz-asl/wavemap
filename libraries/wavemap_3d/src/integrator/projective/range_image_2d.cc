#include "wavemap_3d/integrator/projective/range_image_2d.h"

#include <wavemap_common/utils/eigen_format.h>

namespace wavemap {
void RangeImage2D::importPointcloud(
    const Pointcloud<Point3D>& pointcloud,
    const SphericalProjector& spherical_projector) {
  for (const auto& C_point : pointcloud) {
    // Filter out noisy points and compute point's range
    if (C_point.hasNaN()) {
      LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                   << C_point;
      continue;
    }
    const FloatingPoint range = C_point.norm();
    if (range < 1e-3f) {
      continue;
    }
    if (1e3f < range) {
      LOG(INFO) << "Skipping measurement with suspicious length: " << range;
      continue;
    }

    // Add the point to the range image
    const Index2D range_image_index =
        spherical_projector.bearingToNearestIndex(C_point);

    // Prevent out-of-bounds access
    if ((range_image_index.array() < 0).any() ||
        (getDimensions().array() <= range_image_index.array()).any()) {
      LOG(WARNING) << "\nTried update range image with dimensions "
                   << EigenFormat::oneLine(getDimensions())
                   << " at out of bounds index "
                   << EigenFormat::oneLine(range_image_index)
                   << " with range value " << range << " from original C_point "
                   << C_point;
      continue;
    }

    operator[](range_image_index) = range;
  }
}
}  // namespace wavemap
