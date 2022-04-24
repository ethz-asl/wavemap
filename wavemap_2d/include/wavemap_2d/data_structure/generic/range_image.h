#ifndef WAVEMAP_2D_DATA_STRUCTURE_GENERIC_RANGE_IMAGE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_GENERIC_RANGE_IMAGE_H_

namespace wavemap_2d {
using RangeImageIndex = Eigen::Index;

class RangeImage {
 public:
  using RangeImageData = Eigen::Matrix<FloatingPoint, 1, Eigen::Dynamic>;

  RangeImage(FloatingPoint min_angle, FloatingPoint max_angle,
             Eigen::Index n_beams)
      : data_(RangeImageData::Zero(1, n_beams)),
        min_angle_(min_angle),
        max_angle_(max_angle),
        angle_increment_((max_angle_ - min_angle_) /
                         static_cast<FloatingPoint>(data_.cols() - 1)),
        angle_increment_inv_(1.f / angle_increment_) {
    CHECK_LT(min_angle_, max_angle_);
  }

  bool empty() const { return !size(); }
  size_t size() const { return data_.cols(); }
  void resize(const unsigned int n_points) { data_.resize(1, n_points); }
  void clear() { data_.resize(1, 0); }

  FloatingPoint getMinAngle() const { return min_angle_; }
  FloatingPoint getMaxAngle() const { return max_angle_; }
  Eigen::Index getNBeams() const { return data_.cols(); }

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

  static FloatingPoint bearingToAngle(const Vector& bearing) {
    return std::atan2(bearing.y(), bearing.x());
  }
  static Vector angleToBearing(FloatingPoint angle) {
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

  RangeImageIndex bearingToNearestIndex(const Vector& bearing) const {
    const FloatingPoint angle = bearingToAngle(bearing);
    const auto range_image_index = angleToNearestIndex(angle);
    return range_image_index;
  }
  Vector indexToBearing(RangeImageIndex index) const {
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
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_GENERIC_RANGE_IMAGE_H_
