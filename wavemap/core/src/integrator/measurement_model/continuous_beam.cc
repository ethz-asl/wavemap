#include "wavemap/integrator/measurement_model/continuous_beam.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(ContinuousBeamConfig,
                      (angle_sigma)
                      (range_sigma)
                      (scaling_free)
                      (scaling_occupied)
                      (beam_selector_type));

bool ContinuousBeamConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(angle_sigma, 0.f, verbose);
  is_valid &= IS_PARAM_GT(range_sigma, 0.f, verbose);

  is_valid &= IS_PARAM_GT(scaling_free, 0.f, verbose);
  is_valid &= IS_PARAM_GT(scaling_occupied, 0.f, verbose);

  return is_valid;
}
}  // namespace wavemap
