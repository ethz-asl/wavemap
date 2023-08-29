#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_CIRCULAR_PROJECTOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_CIRCULAR_PROJECTOR_H_

#include "wavemap/common.h"
#include "wavemap/config/config_base.h"

namespace wavemap {
/**
 * Config struct for the circular projection model.
 */
struct CircularProjectorConfig : ConfigBase<CircularProjectorConfig, 3> {
  //! Minimum angle along this axis.
  FloatingPoint min_angle = 0.f;
  //! Maximum angle along this axis.
  FloatingPoint max_angle = 0.f;
  //! Resolution of the image along this axis,
  //! set as the number of cells along the axis.
  IndexElement num_cells = 0;

  static MemberMap memberMap;

  // Constructors
  CircularProjectorConfig() = default;
  CircularProjectorConfig(FloatingPoint min_angle, FloatingPoint max_angle,
                          IndexElement num_cells)
      : min_angle(min_angle), max_angle(max_angle), num_cells(num_cells) {}

  bool isValid(bool verbose) const override;
};

class CircularProjector {
 public:
  explicit CircularProjector(const CircularProjectorConfig& config)
      : config_(config.checkValid()) {}

  FloatingPoint getMinAngle() const { return config_.min_angle; }
  FloatingPoint getMaxAngle() const { return config_.max_angle; }
  IndexElement getNumCells() const { return config_.num_cells; }

  static FloatingPoint bearingToAngle(const Vector2D& bearing) {
    return std::atan2(bearing.y(), bearing.x());
  }
  static Vector2D angleToBearing(FloatingPoint angle) {
    return {std::cos(angle), std::sin(angle)};
  }

  IndexElement angleToNearestIndex(FloatingPoint angle) const {
    return static_cast<IndexElement>(std::round(angleToScaledAngle(angle)));
  }
  IndexElement angleToNearestIndex(FloatingPoint angle,
                                   FloatingPoint& angle_remainder) const {
    const FloatingPoint scaled_angle = angleToScaledAngle(angle);
    const FloatingPoint scaled_angle_rounded = std::round(scaled_angle);
    angle_remainder = (scaled_angle - scaled_angle_rounded) * angle_increment_;
    return static_cast<IndexElement>(scaled_angle_rounded);
  }
  IndexElement angleToFloorIndex(FloatingPoint angle) const {
    return static_cast<IndexElement>(std::floor(angleToScaledAngle(angle)));
  }
  IndexElement angleToCeilIndex(FloatingPoint angle) const {
    return static_cast<IndexElement>(std::ceil(angleToScaledAngle(angle)));
  }
  FloatingPoint indexToAngle(IndexElement index) const {
    return config_.min_angle +
           static_cast<FloatingPoint>(index) * angle_increment_;
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
  const CircularProjectorConfig config_;

  const FloatingPoint angle_increment_ =
      (config_.max_angle - config_.min_angle) /
      static_cast<FloatingPoint>(config_.num_cells - 1);
  const FloatingPoint angle_increment_inv_ = 1.f / angle_increment_;

  FloatingPoint angleToScaledAngle(FloatingPoint angle) const {
    return (angle - config_.min_angle) * angle_increment_inv_;
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_CIRCULAR_PROJECTOR_H_
