#include "wavemap/integrator/projection_model/circular_projector.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(CircularProjectorConfig,
                      (min_angle)
                      (max_angle)
                      (num_cells));

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
}  // namespace wavemap
