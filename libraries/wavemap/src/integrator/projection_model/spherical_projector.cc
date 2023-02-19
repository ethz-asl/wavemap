#include "wavemap/integrator/projection_model/spherical_projector.h"

#include "wavemap/utils/angle_utils.h"

namespace wavemap {
Eigen::Matrix<bool, 3, 1> SphericalProjector::sensorAxisIsPeriodic() const {
  const FloatingPoint x_difference =
      angle_math::normalize_near(config_.elevation.max_angle +
                                 index_to_image_scale_factor_.x()) -
      config_.elevation.min_angle;
  const FloatingPoint y_difference =
      angle_math::normalize_near(config_.azimuth.max_angle +
                                 index_to_image_scale_factor_.y()) -
      config_.azimuth.min_angle;
  return {
      0.f <= x_difference && x_difference <= index_to_image_scale_factor_.x(),
      0.f <= y_difference && y_difference <= index_to_image_scale_factor_.y(),
      false};
}

bool SphericalProjectorConfig::isValid(bool verbose) const {
  return elevation.isValid(verbose) && azimuth.isValid(verbose);
}

SphericalProjectorConfig SphericalProjectorConfig::from(
    const param::Map& params) {
  SphericalProjectorConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(elevation)) {
      config.elevation =
          CircularProjectorConfig::from(param_value.get<param::Map>());
    } else if (param_name == NAMEOF(azimuth)) {
      config.azimuth =
          CircularProjectorConfig::from(param_value.get<param::Map>());
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}
}  // namespace wavemap
