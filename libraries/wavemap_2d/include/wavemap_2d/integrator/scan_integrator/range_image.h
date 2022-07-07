#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_H_

#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/pointcloud.h>

namespace wavemap {
using RangeImageIndex = int;

class RangeImage {
 public:
  using RangeImageData = Eigen::Matrix<FloatingPoint, 1, Eigen::Dynamic>;

  RangeImage(FloatingPoint min_angle, FloatingPoint max_angle,
             Eigen::Index num_beams);

  RangeImage(FloatingPoint min_angle, FloatingPoint max_angle,
             Eigen::Index num_beams, const Pointcloud<>& pointcloud)
      : RangeImage(min_angle, max_angle, num_beams) {
    importPointcloud(pointcloud);
  }

  void importPointcloud(const Pointcloud<>& pointcloud);

  bool empty() const { return !size(); }
  size_t size() const { return data_.cols(); }
  void resize(const unsigned int n_points) { data_.resize(1, n_points); }
  void clear() { data_.resize(1, 0); }

  FloatingPoint getMinAngle() const { return min_angle_; }
  FloatingPoint getMaxAngle() const { return max_angle_; }
  RangeImageIndex getNumBeams() const {
    return static_cast<RangeImageIndex>(data_.cols());
  }
  const RangeImageData& getData() const { return data_; }

  FloatingPoint& operator[](RangeImageIndex index) {
    DCHECK_GE(index, 0);
    DCHECK_LT(index, data_.cols());
    return data_(0, index);
  }
  const FloatingPoint& operator[](RangeImageIndex index) const {
    DCHECK_GE(index, 0);
    DCHECK_LT(index, data_.cols());
    return data_(0, index);
  }

  static FloatingPoint bearingToAngle(const Vector2D& bearing) {
    return std::atan2(bearing.y(), bearing.x());
  }
  static Vector2D angleToBearing(FloatingPoint angle) {
    return {std::cos(angle), std::sin(angle)};
  }

  RangeImageIndex angleToNearestIndex(FloatingPoint angle) const {
    return static_cast<RangeImageIndex>(std::round(angleToScaledAngle(angle)));
  }
  RangeImageIndex angleToFloorIndex(FloatingPoint angle) const {
    return static_cast<RangeImageIndex>(std::floor(angleToScaledAngle(angle)));
  }
  RangeImageIndex angleToCeilIndex(FloatingPoint angle) const {
    return static_cast<RangeImageIndex>(std::ceil(angleToScaledAngle(angle)));
  }
  FloatingPoint indexToAngle(RangeImageIndex index) const {
    return min_angle_ + static_cast<FloatingPoint>(index) * angle_increment_;
  }

  RangeImageIndex bearingToNearestIndex(const Vector2D& bearing) const {
    const FloatingPoint angle = bearingToAngle(bearing);
    const auto range_image_index = angleToNearestIndex(angle);
    return range_image_index;
  }
  Vector2D indexToBearing(RangeImageIndex index) const {
    const FloatingPoint angle = indexToAngle(index);
    return angleToBearing(angle);
  }

 private:
  RangeImageData data_;

  const FloatingPoint min_angle_;
  const FloatingPoint max_angle_;
  const FloatingPoint angle_increment_;
  const FloatingPoint angle_increment_inv_;
  FloatingPoint angleToScaledAngle(FloatingPoint angle) const {
    return (angle - min_angle_) * angle_increment_inv_;
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_RANGE_IMAGE_H_
