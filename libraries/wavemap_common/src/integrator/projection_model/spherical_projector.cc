#include "wavemap_common/integrator/projection_model/image_2d/spherical_projector.h"

#include "wavemap_common/utils/angle_utils.h"

namespace wavemap {
bool SphericalProjector::isXAxisWrapping() const {
  const FloatingPoint difference = angle_math::normalize_near(
      config_.elevation.max_angle - config_.elevation.min_angle);
  return (difference - index_to_image_scale_factor_.x()) <= 1e-3f;
}

bool SphericalProjector::isYAxisWrapping() const {
  const FloatingPoint difference = angle_math::normalize_near(
      config_.azimuth.max_angle - config_.azimuth.min_angle);
  return (difference - index_to_image_scale_factor_.y()) <= 1e-3f;
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
