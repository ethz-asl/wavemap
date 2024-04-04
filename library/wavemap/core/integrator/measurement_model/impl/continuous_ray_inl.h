#ifndef WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_IMPL_CONTINUOUS_RAY_INL_H_
#define WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_IMPL_CONTINUOUS_RAY_INL_H_

#include "wavemap/core/integrator/measurement_model/approximate_gaussian_distribution.h"

namespace wavemap {
inline FloatingPoint ContinuousRay::computeWorstCaseApproximationError(
    UpdateType update_type, FloatingPoint /*sphere_center_distance*/,
    FloatingPoint cell_bounding_radius) const {
  if (update_type == UpdateType::kFullyUnobserved ||
      update_type == UpdateType::kFullyFree) {
    return 0.f;
  }

  if (update_type == UpdateType::kFreeOrUnobserved) {
    // TODO(victorr): Check this
    return 0.5f * config_.scaling_free;
  }

  const FloatingPoint worst_dz = cell_bounding_radius;
  const FloatingPoint worst_dz_error =
      3.f / 8.f * config_.scaling_occupied / config_.range_sigma * worst_dz;

  return worst_dz_error;
}

inline FloatingPoint ContinuousRay::computeUpdate(
    const SensorCoordinates& sensor_coordinates) const {
  switch (config_.beam_selector_type) {
    case BeamSelectorType::kNearestNeighbor: {
      const auto image_index =
          projection_model_->imageToNearestIndex(sensor_coordinates.image);
      return computeBeamUpdate(sensor_coordinates, image_index);
    }
    case BeamSelectorType::kAllNeighbors: {
      FloatingPoint update = 0.f;
      const auto image_indices =
          projection_model_->imageToNearestIndices(sensor_coordinates.image);
      for (int neighbor_idx = 0; neighbor_idx < 4; ++neighbor_idx) {
        const Index2D& image_index = image_indices.col(neighbor_idx);
        update += computeBeamUpdate(sensor_coordinates, image_index);
      }
      return update;
    }
    default:
      return 0.f;
  }
}

inline FloatingPoint ContinuousRay::computeBeamUpdate(
    const SensorCoordinates& sensor_coordinates,
    const Index2D& image_index) const {
  if (!range_image_->isIndexWithinBounds(image_index)) {
    return 0.f;
  }

  const FloatingPoint measured_distance = range_image_->at(image_index);

  if (range_threshold_back_ < sensor_coordinates.depth - measured_distance) {
    return 0.f;
  }

  if (sensor_coordinates.depth < measured_distance - range_threshold_front_) {
    return computeFreeSpaceBeamUpdate();
  } else {
    return computeFullBeamUpdate(sensor_coordinates.depth, measured_distance);
  }
}

inline FloatingPoint ContinuousRay::computeFullBeamUpdate(
    FloatingPoint cell_to_sensor_distance,
    FloatingPoint measured_distance) const {
  const FloatingPoint f =
      (cell_to_sensor_distance - measured_distance) / config_.range_sigma;
  const FloatingPoint range_contrib =
      ApproximateGaussianDistribution::cumulative(f) -
      0.5f * ApproximateGaussianDistribution::cumulative(f - 3.f) - 0.5f;

  const FloatingPoint scaled_contribs =
      (range_contrib < 0.f) ? config_.scaling_free * range_contrib
                            : config_.scaling_occupied * range_contrib;

  const FloatingPoint p = scaled_contribs + 0.5f;
  const FloatingPoint log_odds = std::log(p / (1.f - p));
  DCHECK(!std::isnan(log_odds) && std::isfinite(log_odds));
  return log_odds;
}

inline FloatingPoint ContinuousRay::computeFreeSpaceBeamUpdate() const {
  const FloatingPoint p = -0.5f * config_.scaling_free + 0.5f;
  const FloatingPoint log_odds = std::log(p / (1.f - p));
  DCHECK(!std::isnan(log_odds) && std::isfinite(log_odds));
  return log_odds;
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_IMPL_CONTINUOUS_RAY_INL_H_
