#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"

#include "wavemap/utils/nameof.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(VolumetricDataStructureConfig,
                       (min_cell_width, SiUnit::kMeters), (min_log_odds),
                       (max_log_odds));

bool VolumetricDataStructureConfig::isValid(bool verbose) const {
  return IS_PARAM_GT(min_cell_width, 0.f, verbose);
}
}  // namespace wavemap
