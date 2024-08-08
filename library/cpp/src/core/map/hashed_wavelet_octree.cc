#include "wavemap/core/map/hashed_wavelet_octree.h"

#include <unordered_set>

#include <wavemap/core/utils/profiler_interface.h>

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

void HashedWaveletOctree::threshold() {
  ProfilerZoneScoped;
  forEachBlock([](const BlockIndex& /*block_index*/, Block& block) {
    block.threshold();
  });
}

void HashedWaveletOctree::prune() {
  ProfilerZoneScoped;
  block_map_.eraseBlockIf([](const BlockIndex& /*block_index*/, Block& block) {
    block.prune();
    return block.empty();
  });
}

void HashedWaveletOctree::pruneSmart() {
  ProfilerZoneScoped;
  block_map_.eraseBlockIf(
      [&config = config_](const BlockIndex& /*block_index*/, Block& block) {
        if (config.only_prune_blocks_if_unused_for <
            block.getTimeSinceLastUpdated()) {
          block.prune();
        }
        return block.empty();
      });
}

size_t HashedWaveletOctree::getMemoryUsage() const {
  ProfilerZoneScoped;
  // TODO(victorr): Also include the memory usage of the unordered map itself
  size_t memory_usage = 0u;
  forEachBlock(
      [&memory_usage](const BlockIndex& /*block_index*/, const Block& block) {
        memory_usage += block.getMemoryUsage();
      });
  return memory_usage;
}

Index3D HashedWaveletOctree::getMinIndex() const {
  return cells_per_block_side_ * getMinBlockIndex();
}

Index3D HashedWaveletOctree::getMaxIndex() const {
  if (empty()) {
    return Index3D::Zero();
  }
  return cells_per_block_side_ * (getMaxBlockIndex().array() + 1) - 1;
}

void HashedWaveletOctree::forEachLeaf(
    MapBase::IndexedLeafVisitorFunction visitor_fn) const {
  forEachBlock(
      [&visitor_fn](const BlockIndex& block_index, const Block& block) {
        block.forEachLeaf(block_index, visitor_fn);
      });
}

}  // namespace wavemap
