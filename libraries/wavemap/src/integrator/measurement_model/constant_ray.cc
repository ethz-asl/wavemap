#include "wavemap/integrator/measurement_model/constant_ray.h"

namespace wavemap {
ConstantRayConfig ConstantRayConfig::from(const param::Map& params) {
  ConstantRayConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(log_odds_occupied)) {
      config.log_odds_occupied = param_value.get<FloatingPoint>();
    } else if (param_name == NAMEOF(log_odds_free)) {
      config.log_odds_free = param_value.get<FloatingPoint>();
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}
}  // namespace wavemap
