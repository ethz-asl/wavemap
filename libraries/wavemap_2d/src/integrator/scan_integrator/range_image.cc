#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap {
RangeImage::RangeImage(FloatingPoint min_angle, FloatingPoint max_angle,
                       Eigen::Index num_beams)
    : data_(RangeImageData::Zero(1, num_beams)),
      min_angle_(min_angle),
      max_angle_(max_angle),
      angle_increment_((max_angle_ - min_angle_) /
                       static_cast<FloatingPoint>(data_.cols() - 1)),
      angle_increment_inv_(1.f / angle_increment_) {
  CHECK_LT(min_angle_, max_angle_);
}

void RangeImage::importPointcloud(const Pointcloud<Point2D>& pointcloud) {
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
    const RangeImageIndex range_image_index = bearingToNearestIndex(C_point);
    operator[](range_image_index) = range;
  }
}
}  // namespace wavemap
