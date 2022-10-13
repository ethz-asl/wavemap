#include "wavemap_common/integrator/pointcloud_integrator.h"

namespace wavemap {
bool PointcloudIntegratorConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_range, 0.f, verbose);
  is_valid &= IS_PARAM_GT(max_range, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_range, max_range, verbose);

  return is_valid;
}

PointcloudIntegratorConfig PointcloudIntegratorConfig::from(
    const param::Map& params) {
  PointcloudIntegratorConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(min_range)) {
      config.min_range = param::convert::toMeters(param_value);
    } else if (param_name == NAMEOF(max_range)) {
      config.max_range = param::convert::toMeters(param_value);
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}
}  // namespace wavemap
