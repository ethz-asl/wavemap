#include "wavemap_common/data_structure/volumetric/volumetric_data_structure_base.h"

namespace wavemap {
VolumetricDataStructureConfig VolumetricDataStructureConfig::fromParams(
    const param::Map& params) {
  VolumetricDataStructureConfig config;
  for (const auto& [param_name, param_value] : params) {
    if (param_name == "min_cell_width") {
      config.min_cell_width = param::convert::toMeters(param_value);
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }
  return config;
}
}  // namespace wavemap
