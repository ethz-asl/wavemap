#include "wavemap_common/integrator/projection_model/image_2d/spherical_projector.h"

namespace wavemap {
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
