#include "wavemap_common/data_structure/volumetric/volumetric_data_structure_base.h"

#include "wavemap_common/utils/nameof.h"

namespace wavemap {
bool VolumetricDataStructureConfig::isValid(bool verbose) const {
  return IS_PARAM_GT(min_cell_width, 0.f, verbose);
}

VolumetricDataStructureConfig VolumetricDataStructureConfig::from(
    const param::Map& params) {
  VolumetricDataStructureConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(min_cell_width)) {
      config.min_cell_width = param::convert::toMeters(param_value);
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}
}  // namespace wavemap
