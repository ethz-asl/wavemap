#include "wavemap/integrator/projective/projective_integrator.h"

namespace wavemap {
bool ProjectiveIntegratorConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_range, 0.f, verbose);
  is_valid &= IS_PARAM_GT(max_range, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_range, max_range, verbose);
  is_valid &= IS_PARAM_GE(termination_height, 0, verbose);
  is_valid &= IS_PARAM_GT(termination_update_error, 0.f, verbose);

  return is_valid;
}

ProjectiveIntegratorConfig ProjectiveIntegratorConfig::from(
    const param::Map& params) {
  ProjectiveIntegratorConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(min_range)) {
      config.min_range =
          param::convert::toMeters(param_value, config.min_range);
    } else if (param_name == NAMEOF(max_range)) {
      config.max_range =
          param::convert::toMeters(param_value, config.max_range);
    } else if (param_name == NAMEOF(termination_height)) {
      if (param_value.holds<NdtreeIndexElement>()) {
        config.termination_height = param_value.get<NdtreeIndexElement>();
      }
    } else if (param_name == NAMEOF(termination_update_error)) {
      if (param_value.holds<FloatingPoint>()) {
        config.termination_update_error = param_value.get<FloatingPoint>();
      }
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}

void ProjectiveIntegrator::integratePointcloud(
    const PosedPointcloud<Point<3>>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }
  importPointcloud(pointcloud);
  updateMap();
}

void ProjectiveIntegrator::integrateRangeImage(
    const PosedImage<>& range_image) {
  importRangeImage(range_image);
  updateMap();
}

void ProjectiveIntegrator::importPointcloud(
    const PosedPointcloud<>& pointcloud) {
  // Reset the posed range image and the beam offset image
  posed_range_image_->resetToInitialValue();
  posed_range_image_->setPose(pointcloud.getPose());
  beam_offset_image_->resetToInitialValue();

  // Import all the points
  for (const auto& C_point : pointcloud.getPointsLocal()) {
    // Filter out noisy points and compute point's range
    if (!isMeasurementValid(C_point)) {
      continue;
    }

    // Calculate the range image index
    const Vector3D sensor_coordinates =
        projection_model_->cartesianToSensor(C_point);

    const auto [range_image_index, beam_to_pixel_offset] =
        projection_model_->imageToNearestIndexAndOffset(
            sensor_coordinates.head<2>());
    if (!posed_range_image_->isIndexWithinBounds(range_image_index)) {
      // Prevent out-of-bounds access
      continue;
    }

    // Add the point to the range image, if multiple points hit the same image
    // pixel, keep the closest point
    const FloatingPoint range = sensor_coordinates[2];
    const FloatingPoint old_range_value =
        posed_range_image_->at(range_image_index);
    if (old_range_value < config_.min_range || range < old_range_value) {
      posed_range_image_->at(range_image_index) = range;
      beam_offset_image_->at(range_image_index) = beam_to_pixel_offset;
    }
  }
}

void ProjectiveIntegrator::importRangeImage(
    const PosedImage<>& range_image_input) {
  *posed_range_image_ = range_image_input;
  beam_offset_image_->resetToInitialValue();
}
}  // namespace wavemap
