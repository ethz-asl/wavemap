#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"

#include <unordered_set>

#include <tracy/Tracy.hpp>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(HashedWaveletOctreeConfig,
                      (min_cell_width, SiUnit::kMeters)
                      (min_log_odds)
                      (max_log_odds)
                      (tree_height)
                      (only_prune_blocks_if_unused_for, SiUnit::kSeconds));

bool HashedWaveletOctreeConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_cell_width, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_log_odds, max_log_odds, verbose);
  is_valid &= IS_PARAM_GT(tree_height, 0, verbose);

  return is_valid;
}

void HashedWaveletOctree::threshold() {
  ZoneScoped;
  for (auto& [block_index, block] : blocks_) {
    block.threshold();
  }
}

void HashedWaveletOctree::prune() {
  ZoneScoped;
  std::unordered_set<BlockIndex, IndexHash<kDim>> blocks_to_remove;
  for (auto& [block_index, block] : blocks_) {
    block.prune();
    if (block.empty()) {
      blocks_to_remove.emplace(block_index);
    }
  }
  for (const auto& index : blocks_to_remove) {
    blocks_.erase(index);
  }
}

void HashedWaveletOctree::pruneDistant() {
  ZoneScoped;
  std::unordered_set<BlockIndex, IndexHash<kDim>> blocks_to_remove;
  for (auto& [block_index, block] : blocks_) {
    if (config_.only_prune_blocks_if_unused_for <
        block.getTimeSinceLastUpdated()) {
      block.prune();
    }
    if (block.empty()) {
      blocks_to_remove.emplace(block_index);
    }
  }
  for (const auto& index : blocks_to_remove) {
    blocks_.erase(index);
  }
}

size_t HashedWaveletOctree::getMemoryUsage() const {
  ZoneScoped;
  // TODO(victorr): Also include the memory usage of the unordered map itself
  size_t memory_usage = 0u;
  for (const auto& [block_index, block] : blocks_) {
    memory_usage += block.getMemoryUsage();
  }
  return memory_usage;
}

Index3D HashedWaveletOctree::getMinIndex() const {
  if (empty()) {
    return Index3D::Zero();
  }

  Index3D min_block_index =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  for (const auto& [block_index, block] : blocks_) {
    min_block_index = min_block_index.cwiseMin(block_index);
  }
  return cells_per_block_side_ * min_block_index;
}

Index3D HashedWaveletOctree::getMaxIndex() const {
  if (empty()) {
    return Index3D::Zero();
  }

  Index3D max_block_index =
      Index3D::Constant(std::numeric_limits<IndexElement>::lowest());
  for (const auto& [block_index, block] : blocks_) {
    max_block_index = max_block_index.cwiseMax(block_index);
  }
  return cells_per_block_side_ * (max_block_index + Index3D::Ones());
}
}  // namespace wavemap
