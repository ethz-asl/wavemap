#ifndef WAVEMAP_COMMON_INTEGRATOR_CIRCLE_PROJECTOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_CIRCLE_PROJECTOR_H_

namespace wavemap {
class CircleProjector {
 public:
  CircleProjector(FloatingPoint min_angle, FloatingPoint max_angle,
                  IndexElement num_cells)
      : min_angle_(min_angle),
        max_angle_(max_angle),
        num_cells_(num_cells),
        angle_increment_((max_angle_ - min_angle_) /
                         static_cast<FloatingPoint>(num_cells - 1)),
        angle_increment_inv_(1.f / angle_increment_) {
    CHECK_LT(min_angle_, max_angle_);
  }

  FloatingPoint getMinAngle() const { return min_angle_; }
  FloatingPoint getMaxAngle() const { return max_angle_; }
  IndexElement getNumCells() const { return num_cells_; }

  static FloatingPoint bearingToAngle(const Vector2D& bearing) {
    return std::atan2(bearing.y(), bearing.x());
  }
  static Vector2D angleToBearing(FloatingPoint angle) {
    return {std::cos(angle), std::sin(angle)};
  }

  IndexElement angleToNearestIndex(FloatingPoint angle) const {
    return static_cast<IndexElement>(std::round(angleToScaledAngle(angle)));
  }
  IndexElement angleToFloorIndex(FloatingPoint angle) const {
    return static_cast<IndexElement>(std::floor(angleToScaledAngle(angle)));
  }
  IndexElement angleToCeilIndex(FloatingPoint angle) const {
    return static_cast<IndexElement>(std::ceil(angleToScaledAngle(angle)));
  }
  FloatingPoint indexToAngle(IndexElement index) const {
    return min_angle_ + static_cast<FloatingPoint>(index) * angle_increment_;
  }

  IndexElement bearingToNearestIndex(const Vector2D& bearing) const {
    const FloatingPoint angle = bearingToAngle(bearing);
    const auto range_image_index = angleToNearestIndex(angle);
    return range_image_index;
  }
  Vector2D indexToBearing(IndexElement index) const {
    const FloatingPoint angle = indexToAngle(index);
    return angleToBearing(angle);
  }

 private:
  const FloatingPoint min_angle_;
  const FloatingPoint max_angle_;
  const IndexElement num_cells_;

  const FloatingPoint angle_increment_;
  const FloatingPoint angle_increment_inv_;
  FloatingPoint angleToScaledAngle(FloatingPoint angle) const {
    return (angle - min_angle_) * angle_increment_inv_;
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_CIRCLE_PROJECTOR_H_