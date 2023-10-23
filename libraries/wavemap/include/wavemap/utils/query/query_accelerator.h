#ifndef WAVEMAP_UTILS_QUERY_QUERY_ACCELERATOR_H_
#define WAVEMAP_UTILS_QUERY_QUERY_ACCELERATOR_H_

#include <limits>
#include <unordered_map>

#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"
#include "wavemap/indexing/index_hashes.h"

namespace wavemap {
class QueryAccelerator {
 public:
  static constexpr int kDim = 3;

  explicit QueryAccelerator(const HashedWaveletOctree& map)
      : block_map_(map.getBlocks()), tree_height_(map.getTreeHeight()) {}

  FloatingPoint getCellValue(const Index3D& index) {
    return getCellValue(OctreeIndex{0, index});
  }

  FloatingPoint getCellValue(const OctreeIndex& index) {
    // Remember previous query indices and compute new ones
    const BlockIndex previous_block_index = block_index_;
    const MortonIndex previous_morton_code = morton_code_;
    const IndexElement previous_height = height_;
    block_index_ = computeBlockIndexFromIndex(index);
    morton_code_ = convert::nodeIndexToMorton(index);

    // Check whether we're in the same block as last time
    if (block_index_ == previous_block_index) {
      // Compute the last ancestor the current and previous query had in common
      auto last_common_ancestor = OctreeIndex::computeLastCommonAncestorHeight(
          morton_code_, index.height, previous_morton_code, previous_height);
      height_ = last_common_ancestor;
      DCHECK_LE(height_, tree_height_);
    } else {
      // Test if the queried block exists
      if (block_map_.count(block_index_)) {
        // If yes, load it
        const auto& current_block = block_map_.at(block_index_);
        node_stack_[tree_height_] = &current_block.getRootNode();
        value_stack_[tree_height_] = current_block.getRootScale();
        height_ = tree_height_;
      } else {
        // Otherwise return ignore this query and return 'unknown'
        block_index_ = previous_block_index;
        morton_code_ = previous_morton_code;
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
      value_stack_[height_] = Transform::backwardSingleChild(
          {parent_value, parent_node->data()}, child_idx);
      if (height_ == index.height || !parent_node->hasChild(child_idx)) {
        break;
      }
      node_stack_[height_] = parent_node->getChild(child_idx);
    }

    return value_stack_[height_];
  }

 private:
  using Coefficients = HaarCoefficients<FloatingPoint, kDim>;
  using Transform = HaarTransform<FloatingPoint, kDim>;
  using BlockIndex = Index3D;
  using BlockMap =
      std::unordered_map<BlockIndex, HashedWaveletOctreeBlock, IndexHash<kDim>>;
  using NodeType = NdtreeNode<typename Coefficients::Details, kDim>;

  const BlockMap& block_map_;
  const IndexElement tree_height_;

  std::array<const NodeType*, morton::kMaxTreeHeight<3>> node_stack_{};
  std::array<FloatingPoint, morton::kMaxTreeHeight<3>> value_stack_{};

  Index3D block_index_ =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  MortonIndex morton_code_ = std::numeric_limits<MortonIndex>::max();
  IndexElement height_ = tree_height_;

  BlockIndex computeBlockIndexFromIndex(const OctreeIndex& node_index) const {
    const Index3D index = convert::nodeIndexToMinCornerIndex(node_index);
    return int_math::div_exp2_floor(index, tree_height_);
  }

  friend class QueryAcceleratorTest_Equivalence_Test;
};
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_QUERY_ACCELERATOR_H_
