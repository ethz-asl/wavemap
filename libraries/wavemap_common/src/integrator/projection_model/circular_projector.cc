#include "wavemap_common/integrator/projection_model/circular_projector.h"

namespace wavemap {
bool CircularProjectorConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GE(min_angle, -kPi, verbose);
  is_valid &= IS_PARAM_LE(min_angle, kPi, verbose);

  is_valid &= IS_PARAM_GE(max_angle, -kPi, verbose);
  is_valid &= IS_PARAM_LE(max_angle, kPi, verbose);

  is_valid &= IS_PARAM_LT(min_angle, max_angle, verbose);

  is_valid &= IS_PARAM_GT(num_cells, 1, verbose);

  return is_valid;
}

CircularProjectorConfig CircularProjectorConfig::from(
    const param::Map& params) {
  CircularProjectorConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(min_angle)) {
      config.min_angle = param::convert::toRadians(param_value);
    } else if (param_name == NAMEOF(max_angle)) {
      config.max_angle = param::convert::toRadians(param_value);
    } else if (param_name == NAMEOF(num_cells)) {
      config.num_cells = param_value.get<IndexElement>();
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}
}  // namespace wavemap
