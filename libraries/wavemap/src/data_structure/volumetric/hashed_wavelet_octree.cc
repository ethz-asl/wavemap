#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"

#include <unordered_set>

namespace wavemap {
void HashedWaveletOctree::threshold() {
  for (auto& [block_index, block] : blocks_) {
    block.threshold();
  }
}

void HashedWaveletOctree::prune() {
  std::unordered_set<BlockIndex, VoxbloxIndexHash<3>> blocks_to_remove;
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

size_t HashedWaveletOctree::getMemoryUsage() const {
  // TODO(victorr): Also include the memory usage of the unordered map itself
  size_t memory_usage = 0u;
  for (const auto& [block_index, block] : blocks_) {
    memory_usage += block.getMemoryUsage();
  }
  return memory_usage;
}

Index3D HashedWaveletOctree::getMinIndex() const {
  if (!empty()) {
    Index3D min_block_index =
        Index3D::Constant(std::numeric_limits<IndexElement>::max());
    for (const auto& [block_index, block] : blocks_) {
      min_block_index = min_block_index.cwiseMin(block_index);
    }
    return kCellsPerBlockSide * min_block_index;
  }
  return Index3D::Zero();
}

Index3D HashedWaveletOctree::getMaxIndex() const {
  if (!empty()) {
    Index3D max_block_index =
        Index3D::Constant(std::numeric_limits<IndexElement>::lowest());
    for (const auto& [block_index, block] : blocks_) {
      max_block_index = max_block_index.cwiseMax(block_index);
    }
    return kCellsPerBlockSide * (max_block_index + Index3D::Ones());
  }
  return Index3D::Zero();
}

void HashedWaveletOctree::Block::threshold() {
  if (getNeedsThresholding()) {
    root_scale_coefficient_ -=
        recursiveThreshold(ndtree_.getRootNode(), root_scale_coefficient_);
    setNeedsThresholding(false);
  }
}

void HashedWaveletOctree::Block::prune() {
  if (getNeedsPruning() &&
      kDoNotPruneIfUsedInLastNSec < getTimeSinceLastUpdated()) {
    threshold();
    recursivePrune(ndtree_.getRootNode());
    setNeedsPruning(false);
  }
}

void HashedWaveletOctree::Block::setCellValue(const OctreeIndex& index,
                                              FloatingPoint new_value) {
  setNeedsPruning();
  setNeedsThresholding();
  setLastUpdatedStamp();
  const MortonCode morton_code = index.computeMortonCode();
  std::vector<NodeType*> node_ptrs;
  const int height_difference = kTreeHeight - index.height;
  node_ptrs.reserve(height_difference);
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  FloatingPoint current_value = root_scale_coefficient_;
  for (int parent_height = kTreeHeight; index.height + 1 < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    NodeType* current_parent = node_ptrs.back();
    current_value = Transform::backwardSingleChild(
        {current_value, current_parent->data()}, child_index);
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }
  DCHECK_EQ(node_ptrs.size(), height_difference);

  Coefficients::Parent coefficients{new_value - current_value, {}};
  for (int parent_height = index.height + 1; parent_height <= kTreeHeight;
       ++parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    NodeType* current_node = node_ptrs.back();
    node_ptrs.pop_back();
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, child_index);
    current_node->data() += coefficients.details;
  }

  root_scale_coefficient_ += coefficients.scale;
}

void HashedWaveletOctree::Block::addToCellValue(const OctreeIndex& index,
                                                FloatingPoint update) {
  setNeedsPruning();
  setNeedsThresholding();
  setLastUpdatedStamp();
  const MortonCode morton_code = index.computeMortonCode();

  std::vector<NodeType*> node_ptrs;
  const int height_difference = kTreeHeight - index.height;
  node_ptrs.reserve(height_difference);
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  for (int parent_height = kTreeHeight; index.height + 1 < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    NodeType* current_parent = node_ptrs.back();
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }
  DCHECK_EQ(node_ptrs.size(), height_difference);

  Coefficients::Parent coefficients{update, {}};
  for (int parent_height = index.height + 1; parent_height <= kTreeHeight;
       ++parent_height) {
    NodeType* current_node = node_ptrs.back();
    node_ptrs.pop_back();
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, child_index);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

void HashedWaveletOctree::Block::forEachLeaf(
    const BlockIndex& block_index,
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn) const {
  if (empty()) {
    return;
  }

  std::stack<StackElement> stack;
  stack.emplace(StackElement{OctreeIndex{kTreeHeight, block_index},
                             ndtree_.getRootNode(), root_scale_coefficient_});
  while (!stack.empty()) {
    const OctreeIndex node_index = stack.top().node_index;
    const NodeType& node = stack.top().node;
    const FloatingPoint node_scale_coefficient = stack.top().scale_coefficient;
    stack.pop();

    const Coefficients::CoefficientsArray child_scale_coefficients =
        Transform::backward({node_scale_coefficient, {node.data()}});
    for (NdtreeIndexRelativeChild child_idx = 0;
         child_idx < OctreeIndex::kNumChildren; ++child_idx) {
      const OctreeIndex child_node_index =
          node_index.computeChildIndex(child_idx);
      const FloatingPoint child_scale_coefficient =
          child_scale_coefficients[child_idx];
      if (node.hasChild(child_idx)) {
        const NodeType& child_node = *node.getChild(child_idx);
        stack.emplace(StackElement{child_node_index, child_node,
                                   child_scale_coefficient});
      } else {
        visitor_fn(child_node_index, child_scale_coefficient);
      }
    }
  }
}

HashedWaveletOctree::Coefficients::Scale
HashedWaveletOctree::Block::recursiveThreshold(  // NOLINT
    HashedWaveletOctree::NodeType& node, float scale_coefficient) {
  Coefficients::CoefficientsArray child_scale_coefficients =
      Transform::backward({scale_coefficient, node.data()});

  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    if (node.hasChild(child_idx)) {
      NodeType& child_node = *node.getChild(child_idx);
      child_scale_coefficients[child_idx] =
          recursiveThreshold(child_node, child_scale_coefficients[child_idx]);
    } else {
      child_scale_coefficients[child_idx] -=
          parent_->clamp(child_scale_coefficients[child_idx]);
    }
  }

  const auto [scale_update, detail_updates] =
      Transform::forward(child_scale_coefficients);
  node.data() -= detail_updates;

  return scale_update;
}

void HashedWaveletOctree::Block::recursivePrune(  // NOLINT
    HashedWaveletOctree::NodeType& node) {
  bool has_at_least_one_child = false;
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    if (node.hasChild(child_idx)) {
      NodeType& child_node = *node.getChild(child_idx);
      recursivePrune(child_node);
      const bool detail_coefficients_all_zero = std::all_of(
          child_node.data().cbegin(), child_node.data().cend(),
          [](auto coefficient) { return std::abs(coefficient) < 1e-3f; });
      if (!child_node.hasChildrenArray() && detail_coefficients_all_zero) {
        node.deleteChild(child_idx);
      } else {
        has_at_least_one_child = true;
      }
    }
  }
  if (!has_at_least_one_child) {
    node.deleteChildrenArray();
  }
}
}  // namespace wavemap
