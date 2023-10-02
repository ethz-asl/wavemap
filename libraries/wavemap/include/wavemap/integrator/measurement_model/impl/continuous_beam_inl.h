#ifndef WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_IMPL_CONTINUOUS_BEAM_INL_H_
#define WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_IMPL_CONTINUOUS_BEAM_INL_H_

#include <algorithm>

#include "wavemap/integrator/measurement_model/approximate_gaussian_distribution.h"

namespace wavemap {
inline FloatingPoint ContinuousBeam::computeWorstCaseApproximationError(
    UpdateType update_type, FloatingPoint cell_to_sensor_distance,
    FloatingPoint cell_bounding_radius) const {
  if (update_type == UpdateType::kFullyUnobserved) {
    return 0.f;
  }

  const FloatingPoint worst_angle =
      cell_bounding_radius / cell_to_sensor_distance;
  const FloatingPoint worst_angular_error =
      3.f / 16.f * config_.scaling_free / config_.angle_sigma * worst_angle;

  if (update_type == UpdateType::kFreeOrUnobserved ||
      update_type == UpdateType::kFullyFree) {
    return worst_angular_error;
  }

  const FloatingPoint worst_dz = cell_bounding_radius;
  const FloatingPoint worst_dz_error =
      3.f / 8.f * config_.scaling_occupied / config_.range_sigma * worst_dz;

  return std::max(worst_angular_error, worst_dz_error);
}

inline FloatingPoint ContinuousBeam::computeUpdate(
    const SensorCoordinates& sensor_coordinates) const {
  const auto& range_image = *range_image_;
  const auto& beam_offset_image = *beam_offset_image_;
  const auto& projection_model = *projection_model_;
  const FloatingPoint cell_to_sensor_distance = sensor_coordinates.normal;

  switch (config_.beam_selector_type.toTypeId()) {
    case BeamSelectorType::kNearestNeighbor: {
      // Get the measured distance and cell to beam offset
      const auto [image_index, cell_offset] =
          projection_model.imageToNearestIndexAndOffset(
              sensor_coordinates.image);
      if (!range_image.isIndexWithinBounds(image_index)) {
        return 0.f;
      }
      const FloatingPoint measured_distance = range_image.at(image_index);
      const Vector2D cell_to_beam_offset =
          beam_offset_image.at(image_index) - cell_offset;

      // Compute the image error norm
      const FloatingPoint cell_to_beam_image_error_norm_squared =
          projection_model.imageOffsetToErrorSquaredNorm(
              sensor_coordinates.image, cell_to_beam_offset);

      // Compute the update
      return computeBeamUpdate(cell_to_sensor_distance,
                               cell_to_beam_image_error_norm_squared,
                               measured_distance);
    }

    case BeamSelectorType::kAllNeighbors: {
      // Get the measured distances and cell to beam offsets
      std::array<FloatingPoint, 4> measured_distances{};
      auto [image_indices, cell_to_beam_offsets] =
          projection_model.imageToNearestIndicesAndOffsets(
              sensor_coordinates.image);
      constexpr bool row_major = Image<>::Data::IsRowMajor;
      const int stride = static_cast<int>(range_image.getData().outerStride());
      for (int neighbor_idx = 0; neighbor_idx < 4; ++neighbor_idx) {
        const auto& image_index = image_indices.col(neighbor_idx);
        // Get the measured distance and cell to beam offset
        if (range_image.isIndexWithinBounds(image_index)) {
          const int linear_index =
              row_major ? stride * image_index[0] + image_index[1]  // NOLINT
                        : stride * image_index[1] + image_index[0];
          measured_distances[neighbor_idx] =
              range_image.getData().coeff(linear_index);
          cell_to_beam_offsets.col(neighbor_idx) -=
              beam_offset_image.getData().coeffRef(linear_index);
        }
      }

      // Compute the image error norms
      const auto cell_to_beam_image_error_norms_sq =
          projection_model.imageOffsetsToErrorSquaredNorms(
              sensor_coordinates.image, cell_to_beam_offsets);

      // Compute the update
      FloatingPoint update = 0.f;
      for (int neighbor_idx = 0; neighbor_idx < 4; ++neighbor_idx) {
        update +=
            computeBeamUpdate(cell_to_sensor_distance,
                              cell_to_beam_image_error_norms_sq[neighbor_idx],
                              measured_distances[neighbor_idx]);
      }
      return update;
    }

    default:
      return 0.f;
  }
}

inline FloatingPoint ContinuousBeam::computeBeamUpdate(
    FloatingPoint cell_to_sensor_distance,
    FloatingPoint cell_to_beam_image_error_norm_squared,
    FloatingPoint measured_distance) const {
  const bool fully_in_unknown_space =
      angle_threshold_squared < cell_to_beam_image_error_norm_squared ||
      measured_distance + range_threshold_back_ < cell_to_sensor_distance;
  if (fully_in_unknown_space) {
    return 0.f;
  }

  const FloatingPoint g =
      std::sqrt(cell_to_beam_image_error_norm_squared) / config_.angle_sigma;
  // NOTE: As derived in our paper, angle_contrib = C(g + 3.f) - C(g - 3.f)
  //       where C is ApproximateGaussianDistribution::cumulative. Since 0 <= g,
  //       C is always 1 and angle_contrib simplifies to 1 - C(g - 3.f).
  const FloatingPoint angle_contrib =
      1.f - ApproximateGaussianDistribution::cumulative(g - 3.f);

  const bool fully_in_free_space =
      cell_to_sensor_distance < measured_distance - range_threshold_front;
  constexpr FloatingPoint kFreeSpaceRangeContrib = -0.5f;
  FloatingPoint range_contrib = kFreeSpaceRangeContrib;
  if (!fully_in_free_space) {
    const FloatingPoint f =
        (cell_to_sensor_distance - measured_distance) / config_.range_sigma;
    range_contrib =
        ApproximateGaussianDistribution::cumulative(f) -
        0.5f * ApproximateGaussianDistribution::cumulative(f - 3.f) - 0.5f;
  }

  const FloatingPoint contribs = range_contrib * angle_contrib;
  const FloatingPoint scaled_contribs =
      (fully_in_free_space || contribs < 0.f)
          ? config_.scaling_free * contribs
          : config_.scaling_occupied * contribs;

  const FloatingPoint p = scaled_contribs + 0.5f;
  const FloatingPoint log_odds = std::log(p / (1.f - p));
  DCHECK(!std::isnan(log_odds) && std::isfinite(log_odds));
  return log_odds;
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_MEASUREMENT_MODEL_IMPL_CONTINUOUS_BEAM_INL_H_
