#include "wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h"

namespace wavemap {
bool ContinuousVolumetricLogOddsConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(angle_sigma, 0.f, verbose);
  is_valid &= IS_PARAM_GT(range_sigma, 0.f, verbose);

  is_valid &= IS_PARAM_GT(scaling_free, 0.f, verbose);
  is_valid &= IS_PARAM_GT(scaling_occupied, 0.f, verbose);

  return is_valid;
}

ContinuousVolumetricLogOddsConfig ContinuousVolumetricLogOddsConfig::from(
    const param::Map& params) {
  ContinuousVolumetricLogOddsConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(angle_sigma)) {
      config.angle_sigma =
          param::convert::toRadians(param_value, config.angle_sigma);
    } else if (param_name == NAMEOF(range_sigma)) {
      config.range_sigma =
          param::convert::toMeters(param_value, config.range_sigma);
    } else if (param_name == NAMEOF(scaling_free)) {
      config.scaling_free = param_value.get<FloatingPoint>();
    } else if (param_name == NAMEOF(scaling_occupied)) {
      config.scaling_occupied = param_value.get<FloatingPoint>();
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}
}  // namespace wavemap
