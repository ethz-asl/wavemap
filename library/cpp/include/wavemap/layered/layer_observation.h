#ifndef WAVEMAP_LAYERED_LAYER_OBSERVATION_H_
#define WAVEMAP_LAYERED_LAYER_OBSERVATION_H_

#include <optional>
#include <string>
#include <utility>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/time/time.h>

namespace wavemap::layered {

// A sensor-independent observation of one layer at a spatial position.
// Observation generation stays outside the map. The update policy decides how
// this value is fused.
template <typename ValueT>
struct LayerObservation {
  using Value = ValueT;

  Point3D position = Point3D::Zero();
  ValueT value{};
  std::optional<FloatingPoint> confidence;
  std::optional<Timestamp> timestamp;
  std::string source;

  LayerObservation() = default;
  LayerObservation(Point3D observed_position, ValueT observed_value)
      : position(std::move(observed_position)),
        value(std::move(observed_value)) {}
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYER_OBSERVATION_H_
