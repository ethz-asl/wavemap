#include "wavemap_3d/integrator/projective/range_image_2d.h"

#include <wavemap_common/utils/eigen_format.h>

namespace wavemap {
void RangeImage2D::importPointcloud(
    const Pointcloud<Point3D>& pointcloud,
    const SphericalProjector& spherical_projector) {
  resetToInitialValue();
  for (const auto& C_point : pointcloud) {
    // Filter out noisy points and compute point's range
    if (C_point.hasNaN()) {
      LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                   << C_point;
      continue;
    }
    const FloatingPoint range = C_point.norm();
    if (range < kEpsilon) {
      continue;
    }
    if (1e3f < range) {
      LOG(INFO) << "Skipping measurement with suspicious length: " << range;
      continue;
    }

    // Add the point to the range image
    const Index2D range_image_index =
        spherical_projector.bearingToNearestIndex(C_point);
    if ((range_image_index.array() < 0).any() ||
        (getDimensions().array() <= range_image_index.array()).any()) {
      // Prevent out-of-bounds access
      continue;
    }

    operator[](range_image_index) = range;
  }
}
}  // namespace wavemap
