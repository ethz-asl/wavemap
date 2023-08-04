#include "wavemap/data_structure/volumetric/hashed_wavelet_octree_block.h"

#include <tracy/Tracy.hpp>

namespace wavemap {
void HashedWaveletOctreeBlock::threshold() {
  ZoneScoped;
  if (getNeedsThresholding()) {
    root_scale_coefficient_ -=
        recursiveThreshold(ndtree_.getRootNode(), root_scale_coefficient_);
    setNeedsThresholding(false);
  }
}

void HashedWaveletOctreeBlock::prune() {
  ZoneScoped;
  if (getNeedsPruning()) {
    threshold();
    recursivePrune(ndtree_.getRootNode());
    setNeedsPruning(false);
  }
}

void HashedWaveletOctreeBlock::clear() {
  ZoneScoped;
  root_scale_coefficient_ = Coefficients::Scale{};
  ndtree_.clear();
  last_updated_stamp_ = Clock::now();
}

void HashedWaveletOctreeBlock::setCellValue(const OctreeIndex& index,
                                            FloatingPoint new_value) {
  setNeedsPruning();
  setNeedsThresholding();
  setLastUpdatedStamp();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  std::vector<NodeType*> node_ptrs;
  const int height_difference = tree_height_ - index.height;
  node_ptrs.reserve(height_difference);
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  FloatingPoint current_value = root_scale_coefficient_;
  for (int parent_height = tree_height_; index.height + 1 < parent_height;
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
  for (int parent_height = index.height + 1; parent_height <= tree_height_;
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

void HashedWaveletOctreeBlock::addToCellValue(const OctreeIndex& index,
                                              FloatingPoint update) {
  setNeedsPruning();
  setNeedsThresholding();
  setLastUpdatedStamp();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);

  std::vector<NodeType*> node_ptrs;
  const int height_difference = tree_height_ - index.height;
  node_ptrs.reserve(height_difference);
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  for (int parent_height = tree_height_; index.height + 1 < parent_height;
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
  for (int parent_height = index.height + 1; parent_height <= tree_height_;
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

void HashedWaveletOctreeBlock::forEachLeaf(
    const BlockIndex& block_index,
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn,
    IndexElement termination_height) const {
  ZoneScoped;
  if (empty()) {
    return;
  }

  struct StackElement {
    const OctreeIndex node_index;
    const NodeType& node;
    const Coefficients::Scale scale_coefficient{};
  };
  std::stack<StackElement> stack;
  stack.emplace(StackElement{OctreeIndex{tree_height_, block_index},
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
      if (node.hasChild(child_idx) &&
          termination_height < child_node_index.height) {
        const NodeType& child_node = *node.getChild(child_idx);
        stack.emplace(StackElement{child_node_index, child_node,
                                   child_scale_coefficient});
      } else {
        visitor_fn(child_node_index, child_scale_coefficient);
      }
    }
  }
}

HashedWaveletOctreeBlock::Coefficients::Scale
HashedWaveletOctreeBlock::recursiveThreshold(  // NOLINT
    HashedWaveletOctreeBlock::NodeType& node, FloatingPoint scale_coefficient) {
  Coefficients::CoefficientsArray child_scale_coefficients =
      Transform::backward({scale_coefficient, node.data()});

  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    if (node.hasChild(child_idx)) {
      NodeType& child_node = *node.getChild(child_idx);
      child_scale_coefficients[child_idx] =
          recursiveThreshold(child_node, child_scale_coefficients[child_idx]);
    } else {
      child_scale_coefficients[child_idx] -= std::clamp(
          child_scale_coefficients[child_idx], min_log_odds_, max_log_odds_);
    }
  }

  const auto [scale_update, detail_updates] =
      Transform::forward(child_scale_coefficients);
  node.data() -= detail_updates;

  return scale_update;
}

void HashedWaveletOctreeBlock::recursivePrune(  // NOLINT
    HashedWaveletOctreeBlock::NodeType& node) {
  bool has_at_least_one_child = false;
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    if (node.hasChild(child_idx)) {
      NodeType& child_node = *node.getChild(child_idx);
      recursivePrune(child_node);
      if (!child_node.hasChildrenArray() && !child_node.hasNonzeroData(1e-3f)) {
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
