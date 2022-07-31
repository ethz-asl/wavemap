#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap {
void RangeImage::importPointcloud(const Pointcloud<Point2D>& pointcloud,
                                  const CircleProjector& circle_projector) {
  for (const auto& C_point : pointcloud) {
    // Filter out noisy points and compute point's range
    if (C_point.hasNaN()) {
      LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                   << C_point;
      continue;
    }
    const FloatingPoint range = C_point.norm();
    if (1e3 < range) {
      LOG(INFO) << "Skipping measurement with suspicious length: " << range;
      continue;
    }

    // Add the point to the range image
    const IndexElement range_image_index =
        circle_projector.bearingToNearestIndex(C_point);
    operator[](range_image_index) = range;
  }
}
}  // namespace wavemap
