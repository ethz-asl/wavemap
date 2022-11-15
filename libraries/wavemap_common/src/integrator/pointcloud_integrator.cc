#include "wavemap_common/integrator/pointcloud_integrator.h"

namespace wavemap {
bool PointcloudIntegratorConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_range, 0.f, verbose);
  is_valid &= IS_PARAM_GT(max_range, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_range, max_range, verbose);
  is_valid &= IS_PARAM_GE(termination_height, 0, verbose);

  return is_valid;
}

PointcloudIntegratorConfig PointcloudIntegratorConfig::from(
    const param::Map& params) {
  PointcloudIntegratorConfig config;

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
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}
}  // namespace wavemap
