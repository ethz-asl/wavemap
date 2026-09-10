#include "wavemap/core/map/hashed_chunked_wavelet_octree.h"

#include <unordered_set>

#include <wavemap/core/utils/profile/profiler_interface.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(HashedChunkedWaveletOctreeConfig,
                      (min_cell_width)
                      (min_log_odds)
                      (max_log_odds)
                      (tree_height)
                      (only_prune_blocks_if_unused_for));

bool HashedChunkedWaveletOctreeConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_cell_width, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_log_odds, max_log_odds, verbose);
  is_valid &= IS_PARAM_GT(tree_height, 0, verbose);
  is_valid &= IS_PARAM_LE(tree_height, kMaxSupportedTreeHeight, verbose);

  return is_valid;
}

}  // namespace wavemap
