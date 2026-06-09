#include "wavemap/core/map/hashed_wavelet_octree.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(HashedWaveletOctreeConfig,
                      (min_cell_width)
                      (min_log_odds)
                      (max_log_odds)
                      (tree_height)
                      (only_prune_blocks_if_unused_for));

bool HashedWaveletOctreeConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_cell_width, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_log_odds, max_log_odds, verbose);
  is_valid &= IS_PARAM_GT(tree_height, 0, verbose);

  return is_valid;
}
}  // namespace wavemap
