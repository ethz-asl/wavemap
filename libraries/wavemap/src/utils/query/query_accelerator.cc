#include "wavemap/utils/query/query_accelerator.h"

namespace wavemap {
void QueryAccelerator<HashedWaveletOctree>::reset() {
  node_stack_ = std::array<const NodeType*, morton::kMaxTreeHeight<3>>{};
  value_stack_ = std::array<FloatingPoint, morton::kMaxTreeHeight<3>>{};

  block_index_ = BlockIndex::Constant(std::numeric_limits<IndexElement>::max());
  morton_code_ = std::numeric_limits<MortonIndex>::max();
  height_ = tree_height_;
}

FloatingPoint QueryAccelerator<HashedWaveletOctree>::getCellValue(
    const OctreeIndex& index) {
  // Remember previous query indices and compute new ones
  const BlockIndex previous_block_index = block_index_;
  const MortonIndex previous_morton_code = morton_code_;
  const IndexElement previous_height = height_;
  block_index_ = map_.indexToBlockIndex(index);
  morton_code_ = convert::nodeIndexToMorton(index);

  // Check whether we're in the same block as last time
  if (block_index_ == previous_block_index) {
    // If the block is the same, but it doesn't exist, return 'unknown'
    if (!node_stack_[tree_height_]) {
      return 0.f;
    }
    // Compute the last ancestor the current and previous query had in common
    auto last_common_ancestor = OctreeIndex::computeLastCommonAncestorHeight(
        morton_code_, index.height, previous_morton_code, previous_height);
    height_ = last_common_ancestor;
    DCHECK_LE(height_, tree_height_);
  } else {
    // Test if the queried block exists
    const auto* current_block = map_.getBlock(block_index_);
    if (current_block) {
      // If yes, load it
      node_stack_[tree_height_] = &current_block->getRootNode();
      value_stack_[tree_height_] = current_block->getRootScale();
      height_ = tree_height_;
    } else {
      // Otherwise remember that it doesn't exist and return 'unknown'
      node_stack_[tree_height_] = nullptr;
      value_stack_[tree_height_] = 0.f;
      height_ = tree_height_;
      return 0.f;
    }
  }

  // If the requested value was already decompressed in the last query, return
  if (height_ == index.height) {
    return value_stack_[height_];
  }

  // Load the node at height_ if it was not yet loaded last time
  if (previous_height != tree_height_ && height_ == previous_height) {
    const HashedWaveletOctree::Block::NodeType* parent_node =
        node_stack_[height_ + 1];
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code_, height_ + 1);
    if (!parent_node->hasChild(child_index)) {
      return value_stack_[height_];
    }
    node_stack_[height_] = parent_node->getChild(child_index);
  }

  // Walk down the tree from height_ to index.height
  while (true) {
    const HashedWaveletOctree::Block::NodeType* parent_node =
        node_stack_[height_];
    const FloatingPoint parent_value = value_stack_[height_];
    const NdtreeIndexRelativeChild child_idx =
        OctreeIndex::computeRelativeChildIndex(morton_code_, height_);
    --height_;
    value_stack_[height_] =
        HashedWaveletOctree::Block::Transform::backwardSingleChild(
            {parent_value, parent_node->data()}, child_idx);
    if (height_ == index.height || !parent_node->hasChild(child_idx)) {
      break;
    }
    node_stack_[height_] = parent_node->getChild(child_idx);
  }

  return value_stack_[height_];
}
}  // namespace wavemap
