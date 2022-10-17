#include "wavemap_common/integrator/projection_model/ouster_projector.h"

namespace wavemap {
bool OusterProjectorConfig::isValid(bool verbose) const {
  return elevation.isValid(verbose) && azimuth.isValid(verbose);
}

OusterProjectorConfig OusterProjectorConfig::from(const param::Map& params) {
  OusterProjectorConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(elevation)) {
      config.elevation =
          CircularProjectorConfig::from(param_value.get<param::Map>());
    } else if (param_name == NAMEOF(azimuth)) {
      config.azimuth =
          CircularProjectorConfig::from(param_value.get<param::Map>());
    } else if (param_name == NAMEOF(lidar_origin_to_beam_origin)) {
      config.lidar_origin_to_beam_origin =
          param::convert::toMeters(param_value.get<param::Map>());
    } else if (param_name == NAMEOF(lidar_origin_to_sensor_origin_z_offset)) {
      config.lidar_origin_to_sensor_origin_z_offset =
          param::convert::toMeters(param_value.get<param::Map>());
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}
}  // namespace wavemap
