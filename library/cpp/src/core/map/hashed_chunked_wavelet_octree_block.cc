#include "wavemap/core/map/hashed_chunked_wavelet_octree_block.h"

#include <stack>
#include <utility>
#include <vector>

#include <wavemap/core/utils/profile/profiler_interface.h>

namespace wavemap {
void HashedChunkedWaveletOctreeBlock::threshold() {
  ProfilerZoneScoped;
  if (getNeedsThresholding()) {
    recursiveThreshold(ndtree_.getRootNode(), root_scale_coefficient_);
    setNeedsThresholding(false);
  }
}

void HashedChunkedWaveletOctreeBlock::prune() {
  ProfilerZoneScoped;
  if (getNeedsPruning()) {
    threshold();
    recursivePrune(ndtree_.getRootNode());
    setNeedsPruning(false);
  }
}

void HashedChunkedWaveletOctreeBlock::clear() {
  ProfilerZoneScoped;
  root_scale_coefficient_ = Coefficients::Scale{};
  ndtree_.clear();
  setLastUpdatedStamp();
}

void HashedChunkedWaveletOctreeBlock::setCellValue(const OctreeIndex& index,
                                                   FloatingPoint new_value) {
  setNeedsPruning();
  setNeedsThresholding();
  setLastUpdatedStamp();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  std::vector<OctreeType::NodeRefType> ancestors;
  const int height_difference = tree_height_ - index.height;
  ancestors.reserve(height_difference);
  ancestors.emplace_back(ndtree_.getRootNode());
  FloatingPoint current_value = root_scale_coefficient_;
  for (int parent_height = tree_height_; index.height + 1 < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    OctreeType::NodeRefType current_parent = ancestors.back();
    current_value = Transform::backwardSingleChild(
        {current_value, current_parent.data()}, child_index);
    OctreeType::NodeRefType child =
        current_parent.getOrAllocateChild(child_index);
    ancestors.emplace_back(child);
  }
  DCHECK_EQ(ancestors.size(), height_difference);

  Coefficients::Parent coefficients{new_value - current_value, {}};
  for (int parent_height = index.height + 1; parent_height <= tree_height_;
       ++parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    OctreeType::NodeRefType current_node = ancestors.back();
    ancestors.pop_back();
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, child_index);
    current_node.data() += coefficients.details;
  }

  root_scale_coefficient_ += coefficients.scale;
}

void HashedChunkedWaveletOctreeBlock::addToCellValue(const OctreeIndex& index,
                                                     FloatingPoint update) {
  setNeedsPruning();
  setNeedsThresholding();
  setLastUpdatedStamp();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);

  std::vector<OctreeType::NodeRefType> ancestors;
  const int height_difference = tree_height_ - index.height;
  ancestors.reserve(height_difference);
  ancestors.emplace_back(ndtree_.getRootNode());
  for (int parent_height = tree_height_; index.height + 1 < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    OctreeType::NodeRefType current_parent = ancestors.back();
    OctreeType::NodeRefType child =
        current_parent.getOrAllocateChild(child_index);
    ancestors.emplace_back(child);
  }
  DCHECK_EQ(ancestors.size(), height_difference);

  Coefficients::Parent coefficients{update, {}};
  for (int parent_height = index.height + 1; parent_height <= tree_height_;
       ++parent_height) {
    OctreeType::NodeRefType current_node = ancestors.back();
    ancestors.pop_back();
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, child_index);
    current_node.data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

void HashedChunkedWaveletOctreeBlock::forEachLeaf(
    const BlockIndex& block_index,
    MapBase::IndexedLeafVisitorFunction visitor_fn,
    IndexElement termination_height) const {
  ProfilerZoneScoped;
  if (empty()) {
    return;
  }

  struct StackElement {
    const OctreeIndex node_index;
    OctreeType::NodeConstRefType node;
    const Coefficients::Scale scale_coefficient{};
  };
  std::stack<StackElement> stack;
  stack.emplace(StackElement{{tree_height_, block_index},
                             ndtree_.getRootNode(),
                             root_scale_coefficient_});
  while (!stack.empty()) {
    const OctreeIndex index = stack.top().node_index;
    const auto node = stack.top().node;
    const FloatingPoint scale_coefficient = stack.top().scale_coefficient;
    stack.pop();

    const Coefficients::CoefficientsArray child_scale_coefficients =
        Transform::backward({scale_coefficient, {node.data()}});
    for (NdtreeIndexRelativeChild child_idx = 0;
         child_idx < OctreeIndex::kNumChildren; ++child_idx) {
      const OctreeIndex child_node_index = index.computeChildIndex(child_idx);
      const FloatingPoint child_scale_coefficient =
          child_scale_coefficients[child_idx];
      if (auto child_node = node.getChild(child_idx);
          child_node && termination_height < child_node_index.height) {
        stack.emplace(StackElement{child_node_index, *child_node,
                                   child_scale_coefficient});
      } else {
        visitor_fn(child_node_index, child_scale_coefficient);
      }
    }
  }
}

void HashedChunkedWaveletOctreeBlock::recursiveThreshold(  // NOLINT
    OctreeType::NodeRefType node, Coefficients::Scale& node_scale_coefficient) {
  // Decompress child values
  auto& node_detail_coefficients = node.data();
  Coefficients::CoefficientsArray child_scale_coefficients =
      Transform::backward({node_scale_coefficient, node_detail_coefficients});

  // Handle each child
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    Coefficients::Scale& child_scale = child_scale_coefficients[child_idx];
    if (auto child_node = node.getChild(child_idx); child_node) {
      recursiveThreshold(*child_node, child_scale);
    } else {
      child_scale = std::clamp(child_scale, min_log_odds_, max_log_odds_);
    }
  }

  // Compress
  const auto [new_scale, new_details] =
      Transform::forward(child_scale_coefficients);
  node_detail_coefficients = new_details;
  node_scale_coefficient = new_scale;
}

void HashedChunkedWaveletOctreeBlock::recursivePrune(  // NOLINT
    HashedChunkedWaveletOctreeBlock::OctreeType::NodeRefType node) {
  bool has_at_least_one_child = false;
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    if (OctreeType::NodePtrType child_node = node.getChild(child_idx);
        child_node) {
      recursivePrune(*child_node);
      if (!child_node->hasAtLeastOneChild() &&
          !child_node->hasNonzeroData(1e-3f)) {
        node.eraseChild(child_idx);
      } else {
        has_at_least_one_child = true;
      }
    }
  }
  if (!has_at_least_one_child) {
    node.hasAtLeastOneChild() = false;
  }
}
}  // namespace wavemap
