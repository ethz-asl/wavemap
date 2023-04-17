#include "wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h"

#include <unordered_set>

namespace wavemap {
void HashedChunkedWaveletOctree::threshold() {
  for (auto& [block_index, block] : blocks_) {
    block.threshold();
  }
}

void HashedChunkedWaveletOctree::prune() {
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

void HashedChunkedWaveletOctree::pruneDistant() {
  std::unordered_set<BlockIndex, IndexHash<kDim>> blocks_to_remove;
  for (auto& [block_index, block] : blocks_) {
    if (kDoNotPruneIfUsedInLastNSec < block.getTimeSinceLastUpdated()) {
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

size_t HashedChunkedWaveletOctree::getMemoryUsage() const {
  // TODO(victorr): Also include the memory usage of the unordered map itself
  size_t memory_usage = 0u;
  for (const auto& [block_index, block] : blocks_) {
    memory_usage += block.getMemoryUsage();
  }
  return memory_usage;
}

Index3D HashedChunkedWaveletOctree::getMinIndex() const {
  if (empty()) {
    return Index3D::Zero();
  }

  Index3D min_block_index =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  for (const auto& [block_index, block] : blocks_) {
    min_block_index = min_block_index.cwiseMin(block_index);
  }
  return kCellsPerBlockSide * min_block_index;
}

Index3D HashedChunkedWaveletOctree::getMaxIndex() const {
  if (empty()) {
    return Index3D::Zero();
  }

  Index3D max_block_index =
      Index3D::Constant(std::numeric_limits<IndexElement>::lowest());
  for (const auto& [block_index, block] : blocks_) {
    max_block_index = max_block_index.cwiseMax(block_index);
  }
  return kCellsPerBlockSide * (max_block_index + Index3D::Ones());
}

void HashedChunkedWaveletOctree::Block::threshold() {
  if (getNeedsThresholding()) {
    root_scale_coefficient_ = recursiveThreshold(chunked_ndtree_.getRootChunk(),
                                                 root_scale_coefficient_);
    setNeedsThresholding(false);
  }
}

void HashedChunkedWaveletOctree::Block::prune() {
  if (getNeedsPruning()) {
    threshold();
    recursivePrune(chunked_ndtree_.getRootChunk());
    setNeedsPruning(false);
  }
}

void HashedChunkedWaveletOctree::Block::setCellValue(const OctreeIndex& index,
                                                     FloatingPoint new_value) {
  setNeedsPruning();
  setNeedsThresholding();
  setLastUpdatedStamp();

  // Descend the tree chunk by chunk while decompressing, and caching chunk ptrs
  const MortonCode morton_code = convert::nodeIndexToMorton(index);
  std::array<NodeChunkType*, kTreeHeight / kChunkHeight> chunk_ptrs{};
  chunk_ptrs[0] = &chunked_ndtree_.getRootChunk();
  FloatingPoint current_value = root_scale_coefficient_;
  for (int chunk_top_height = kTreeHeight; index.height < chunk_top_height;
       chunk_top_height -= kChunkHeight) {
    // Get the current chunk
    const int chunk_depth = (kTreeHeight - chunk_top_height) / kChunkHeight;
    NodeChunkType* const current_chunk = chunk_ptrs[chunk_depth];
    // Decompress level by level
    for (int parent_height = chunk_top_height;
         chunk_top_height - kChunkHeight < parent_height; --parent_height) {
      // Perform one decompression stage
      const LinearIndex value_index = OctreeIndex::computeTreeTraversalDistance(
          morton_code, chunk_top_height, parent_height);
      const NdtreeIndexRelativeChild relative_child_index =
          OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
      current_value = Transform::backwardSingleChild(
          {current_value, current_chunk->data(value_index)},
          relative_child_index);
      // If we've reached the requested resolution, stop descending
      if (parent_height == index.height + 1) {
        break;
      }
    }
    if (chunk_top_height - kChunkHeight <= index.height + 1) {
      break;
    }

    // Descend to the next chunk
    const LinearIndex linear_child_index =
        OctreeIndex::computeLevelTraversalDistance(
            morton_code, chunk_top_height, chunk_top_height - kChunkHeight);
    if (current_chunk->hasChild(linear_child_index)) {
      chunk_ptrs[chunk_depth + 1] = current_chunk->getChild(linear_child_index);
    } else {
      chunk_ptrs[chunk_depth + 1] =
          current_chunk->allocateChild(linear_child_index);
    }
  }

  Coefficients::Parent coefficients{new_value - current_value, {}};
  for (int parent_height = index.height + 1; parent_height <= kTreeHeight;
       ++parent_height) {
    // Get the current chunk
    const int chunk_depth = (kTreeHeight - parent_height) / kChunkHeight;
    NodeChunkType* current_chunk = chunk_ptrs[chunk_depth];
    // Get the index of the data w.r.t. the chunk
    const int chunk_top_height = kTreeHeight - chunk_depth * kChunkHeight;
    const LinearIndex value_index = OctreeIndex::computeTreeTraversalDistance(
        morton_code, chunk_top_height, parent_height);
    // Compute and apply the transformed update
    const NdtreeIndexRelativeChild relative_child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, relative_child_index);
    current_chunk->data(value_index) += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

void HashedChunkedWaveletOctree::Block::addToCellValue(const OctreeIndex& index,
                                                       FloatingPoint update) {
  setNeedsPruning();
  setNeedsThresholding();
  setLastUpdatedStamp();

  const MortonCode morton_code = convert::nodeIndexToMorton(index);
  std::array<NodeChunkType*, kTreeHeight / kChunkHeight> chunk_ptrs{};
  chunk_ptrs[0] = &chunked_ndtree_.getRootChunk();
  const int last_chunk_depth = (kTreeHeight - index.height - 1) / kChunkHeight;
  for (int chunk_depth = 1; chunk_depth <= last_chunk_depth; ++chunk_depth) {
    const int parent_chunk_top_height =
        kTreeHeight - (chunk_depth - 1) * kChunkHeight;
    const int chunk_top_height = kTreeHeight - chunk_depth * kChunkHeight;
    const LinearIndex linear_child_index =
        OctreeIndex::computeLevelTraversalDistance(
            morton_code, parent_chunk_top_height, chunk_top_height);
    NodeChunkType* current_chunk = chunk_ptrs[chunk_depth - 1];
    if (current_chunk->hasChild(linear_child_index)) {
      chunk_ptrs[chunk_depth] = current_chunk->getChild(linear_child_index);
    } else {
      chunk_ptrs[chunk_depth] =
          current_chunk->allocateChild(linear_child_index);
    }
  }

  Coefficients::Parent coefficients{update, {}};
  for (int parent_height = index.height + 1; parent_height <= kTreeHeight;
       ++parent_height) {
    // Get the current chunk
    const int chunk_depth = (kTreeHeight - parent_height) / kChunkHeight;
    NodeChunkType* current_chunk = chunk_ptrs[chunk_depth];
    // Get the index of the data w.r.t. the chunk
    const int chunk_top_height = kTreeHeight - chunk_depth * kChunkHeight;
    const LinearIndex value_index = OctreeIndex::computeTreeTraversalDistance(
        morton_code, chunk_top_height, parent_height);
    // Compute and apply the transformed update
    const NdtreeIndexRelativeChild relative_child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, relative_child_index);
    current_chunk->data(value_index) += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

void HashedChunkedWaveletOctree::Block::forEachLeaf(
    const BlockIndex& block_index,
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn) const {
  if (empty()) {
    return;
  }

  std::stack<StackElement> stack;
  stack.emplace(StackElement{{kTreeHeight, block_index},
                             chunked_ndtree_.getRootChunk(),
                             root_scale_coefficient_});
  while (!stack.empty()) {
    const OctreeIndex index = stack.top().node_index;
    const NodeChunkType& chunk = stack.top().chunk;
    const FloatingPoint scale_coefficient = stack.top().scale_coefficient;
    stack.pop();

    const MortonCode morton_code = convert::nodeIndexToMorton(index);
    const int chunk_top_height =
        kChunkHeight * int_math::div_round_up(index.height, kChunkHeight);
    const LinearIndex value_index = OctreeIndex::computeTreeTraversalDistance(
        morton_code, chunk_top_height, index.height);
    const Coefficients::CoefficientsArray child_scale_coefficients =
        Transform::backward({scale_coefficient, {chunk.data(value_index)}});

    for (NdtreeIndexRelativeChild relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      const OctreeIndex child_index =
          index.computeChildIndex(relative_child_idx);
      const FloatingPoint child_scale_coefficient =
          child_scale_coefficients[relative_child_idx];

      if (child_index.height % kChunkHeight != 0) {
        stack.emplace(
            StackElement{child_index, chunk, child_scale_coefficient});
        continue;
      }

      const MortonCode child_morton = convert::nodeIndexToMorton(child_index);
      const LinearIndex linear_child_index =
          OctreeIndex::computeLevelTraversalDistance(
              child_morton, chunk_top_height, child_index.height);

      if (chunk.hasChild(linear_child_index)) {
        const NodeChunkType& child_chunk = *chunk.getChild(linear_child_index);
        stack.emplace(
            StackElement{child_index, child_chunk, child_scale_coefficient});
      } else {
        visitor_fn(child_index, child_scale_coefficient);
      }
    }
  }
}

HashedChunkedWaveletOctree::Coefficients::Scale
HashedChunkedWaveletOctree::Block::recursiveThreshold(  // NOLINT
    HashedChunkedWaveletOctree::NodeChunkType& chunk, float scale_coefficient) {
  constexpr auto tree_size = [](auto tree_height) {
    return static_cast<int>(
        tree_math::perfect_tree::num_total_nodes_fast<kDim>(tree_height));
  };
  constexpr auto level_size = [](auto level_height) {
    return static_cast<int>(
        tree_math::perfect_tree::num_leaf_nodes<kDim>(level_height));
  };

  // Decompress
  std::array<Coefficients::Scale, tree_size(kChunkHeight + 1)>
      chunk_scale_coefficients{};
  chunk_scale_coefficients[0] = scale_coefficient;
  for (int level_idx = 0; level_idx < kChunkHeight; ++level_idx) {
    const int first_idx = tree_size(level_idx);
    const int last_idx = tree_size(level_idx + 1);
    for (int relative_idx = 0; relative_idx < level_size(level_idx + 1);
         ++relative_idx) {
      const int src_idx = first_idx + relative_idx;
      const Coefficients::CoefficientsArray child_scale_coefficients =
          Transform::backward(
              {chunk_scale_coefficients[src_idx], chunk.data(src_idx)});
      const int first_dest_idx = last_idx + 8 * relative_idx;
      std::move(child_scale_coefficients.begin(),
                child_scale_coefficients.end(),
                chunk_scale_coefficients.begin() + first_dest_idx);
    }
  }

  // Threshold
  const int first_leaf_idx = tree_size(kChunkHeight);
  for (LinearIndex child_idx = 0; child_idx < NodeChunkType::kNumChildren;
       ++child_idx) {
    const LinearIndex array_idx = first_leaf_idx + child_idx;
    if (chunk.hasChild(child_idx)) {
      NodeChunkType& child_chunk = *chunk.getChild(child_idx);
      chunk_scale_coefficients[array_idx] =
          recursiveThreshold(child_chunk, chunk_scale_coefficients[array_idx]);
    } else {
      chunk_scale_coefficients[array_idx] =
          parent_->clamp(chunk_scale_coefficients[array_idx]);
    }
  }

  // Compress
  for (int level_idx = kChunkHeight - 1; 0 <= level_idx; --level_idx) {
    const int first_idx = tree_size(level_idx);
    const int last_idx = tree_size(level_idx + 1);
    for (int relative_idx = level_size(level_idx + 1) - 1; 0 <= relative_idx;
         --relative_idx) {
      const int first_src_idx = last_idx + 8 * relative_idx;
      Coefficients::CoefficientsArray scale_coefficients_subset{};
      std::move(chunk_scale_coefficients.begin() + first_src_idx,
                chunk_scale_coefficients.begin() + first_src_idx + 8,
                scale_coefficients_subset.begin());

      const int dst_idx = first_idx + relative_idx;
      auto [scale_update, detail_updates] =
          Transform::forward(scale_coefficients_subset);
      chunk.data(dst_idx) = detail_updates;
      chunk_scale_coefficients[dst_idx] = scale_update;
    }
  }

  return chunk_scale_coefficients[0];
}

void HashedChunkedWaveletOctree::Block::recursivePrune(  // NOLINT
    HashedChunkedWaveletOctree::NodeChunkType& chunk) {
  constexpr FloatingPoint kNonzeroCoefficientThreshold = 1e-3f;
  bool has_at_least_one_child = false;
  for (LinearIndex linear_child_idx = 0;
       linear_child_idx < NodeChunkType::kNumChildren; ++linear_child_idx) {
    if (chunk.hasChild(linear_child_idx)) {
      NodeChunkType& child_chunk = *chunk.getChild(linear_child_idx);
      recursivePrune(child_chunk);
      if (!child_chunk.hasChildrenArray() &&
          !child_chunk.hasNonzeroData(kNonzeroCoefficientThreshold)) {
        chunk.deleteChild(linear_child_idx);
      } else {
        has_at_least_one_child = true;
      }
    }
  }
  if (!has_at_least_one_child) {
    chunk.deleteChildrenArray();
  }
}
}  // namespace wavemap
