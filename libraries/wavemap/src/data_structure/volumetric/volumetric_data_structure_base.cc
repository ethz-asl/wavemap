#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"

#include "wavemap/utils/nameof.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(VolumetricDataStructureConfig,
                      (min_cell_width)
                      (min_log_odds)
                      (max_log_odds));

bool VolumetricDataStructureConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_cell_width, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_log_odds, max_log_odds, verbose);

  return is_valid;
}
}  // namespace wavemap
