#ifndef WAVEMAP_CORE_MAP_IMPL_HASHED_WAVELET_OCTREE_BLOCK_INL_H_
#define WAVEMAP_CORE_MAP_IMPL_HASHED_WAVELET_OCTREE_BLOCK_INL_H_

#include <algorithm>
#include <stack>
#include <vector>

#include "wavemap/core/utils/profile/profiler_interface.h"
#include "wavemap/core/utils/query/occupancy_classifier.h"

namespace wavemap {
namespace detail {
template <typename CellDataT>
FloatingPoint getOccupancy(const CellDataT& value) {
  return static_cast<FloatingPoint>(value);
}

template <typename CellDataT>
void setOccupancy(CellDataT& value, FloatingPoint occupancy) {
  value = occupancy;
}
}  // namespace detail

template <typename CellDataT>
bool HashedWaveletOctreeBlockT<CellDataT>::empty() const {
  return ndtree_.empty() && OccupancyClassifier::isUnobserved(
                                detail::getOccupancy(root_scale_coefficient_));
}

template <typename CellDataT>
FloatingPoint HashedWaveletOctreeBlockT<CellDataT>::getTimeSinceLastUpdated()
    const {
  return time::to_seconds<FloatingPoint>(Time::now() - last_updated_stamp_);
}

template <typename CellDataT>
FloatingPoint HashedWaveletOctreeBlockT<CellDataT>::getCellValue(
    const OctreeIndex& index) const {
  return detail::getOccupancy(getVoxelValue(index));
}

template <typename CellDataT>
CellDataT HashedWaveletOctreeBlockT<CellDataT>::getVoxelValue(
    const OctreeIndex& index) const {
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  typename OctreeType::NodeConstPtrType node = &ndtree_.getRootNode();
  CellDataT value = root_scale_coefficient_;
  for (int parent_height = tree_height_; node && index.height < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    value = Transform::backwardSingleChild({value, node->data()}, child_index);
    node = node->getChild(child_index);
  }
  return value;
}

template <typename CellDataT>
void HashedWaveletOctreeBlockT<CellDataT>::threshold() {
  ProfilerZoneScoped;
  if (getNeedsThresholding()) {
    recursiveThreshold(ndtree_.getRootNode(), root_scale_coefficient_);
    setNeedsThresholding(false);
  }
}

template <typename CellDataT>
void HashedWaveletOctreeBlockT<CellDataT>::prune() {
  ProfilerZoneScoped;
  if (getNeedsPruning()) {
    threshold();
    recursivePrune(ndtree_.getRootNode());
    setNeedsPruning(false);
  }
}

template <typename CellDataT>
void HashedWaveletOctreeBlockT<CellDataT>::clear() {
  ProfilerZoneScoped;
  root_scale_coefficient_ = typename Coefficients::Scale{};
  ndtree_.clear();
  setLastUpdatedStamp();
}

template <typename CellDataT>
void HashedWaveletOctreeBlockT<CellDataT>::setCellValue(
    const OctreeIndex& index, FloatingPoint new_value) {
  CellDataT voxel = getVoxelValue(index);
  detail::setOccupancy(voxel, new_value);
  setVoxelValue(index, voxel);
}

template <typename CellDataT>
void HashedWaveletOctreeBlockT<CellDataT>::setVoxelValue(
    const OctreeIndex& index, const CellDataT& new_value) {
  setNeedsPruning();
  setNeedsThresholding();
  setLastUpdatedStamp();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  std::vector<typename OctreeType::NodePtrType> ancestors;
  const int height_difference = tree_height_ - index.height;
  ancestors.reserve(height_difference);
  ancestors.emplace_back(&ndtree_.getRootNode());
  CellDataT current_value = root_scale_coefficient_;
  for (int parent_height = tree_height_; index.height + 1 < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    typename OctreeType::NodePtrType current_parent = ancestors.back();
    current_value = Transform::backwardSingleChild(
        {current_value, current_parent->data()}, child_index);
    typename OctreeType::NodeRefType child =
        current_parent->getOrAllocateChild(child_index);
    ancestors.emplace_back(&child);
  }
  DCHECK_EQ(ancestors.size(), height_difference);

  typename Coefficients::Parent coefficients{new_value - current_value, {}};
  for (int parent_height = index.height + 1; parent_height <= tree_height_;
       ++parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    typename OctreeType::NodePtrType current_node = ancestors.back();
    ancestors.pop_back();
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, child_index);
    current_node->data() += coefficients.details;
  }

  root_scale_coefficient_ += coefficients.scale;
}

template <typename CellDataT>
void HashedWaveletOctreeBlockT<CellDataT>::addToCellValue(
    const OctreeIndex& index, FloatingPoint update) {
  addToVoxelValue(index, CellDataT{update});
}

template <typename CellDataT>
void HashedWaveletOctreeBlockT<CellDataT>::addToVoxelValue(
    const OctreeIndex& index, const CellDataT& update) {
  setNeedsPruning();
  setNeedsThresholding();
  setLastUpdatedStamp();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);

  std::vector<typename OctreeType::NodePtrType> ancestors;
  const int height_difference = tree_height_ - index.height;
  ancestors.reserve(height_difference);
  ancestors.emplace_back(&ndtree_.getRootNode());
  for (int parent_height = tree_height_; index.height + 1 < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    typename OctreeType::NodePtrType current_parent = ancestors.back();
    typename OctreeType::NodeRefType child =
        current_parent->getOrAllocateChild(child_index);
    ancestors.emplace_back(&child);
  }
  DCHECK_EQ(ancestors.size(), height_difference);

  typename Coefficients::Parent coefficients{update, {}};
  for (int parent_height = index.height + 1; parent_height <= tree_height_;
       ++parent_height) {
    typename OctreeType::NodePtrType current_node = ancestors.back();
    ancestors.pop_back();
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, child_index);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

template <typename CellDataT>
void HashedWaveletOctreeBlockT<CellDataT>::forEachLeaf(
    const BlockIndex& block_index,
    MapBase::IndexedLeafVisitorFunction visitor_fn,
    IndexElement termination_height) const {
  forEachVoxelLeaf(
      block_index,
      [&visitor_fn](const OctreeIndex& node_index,
                    const CellDataT& voxel_value) {
        visitor_fn(node_index, detail::getOccupancy(voxel_value));
      },
      termination_height);
}

template <typename CellDataT>
template <typename IndexedVoxelLeafVisitorFunction>
void HashedWaveletOctreeBlockT<CellDataT>::forEachVoxelLeaf(
    const BlockIndex& block_index, IndexedVoxelLeafVisitorFunction visitor_fn,
    IndexElement termination_height) const {
  ProfilerZoneScoped;
  if (empty()) {
    return;
  }

  struct StackElement {
    const OctreeIndex node_index;
    typename OctreeType::NodeConstRefType node;
    const typename Coefficients::Scale scale_coefficient{};
  };
  std::stack<StackElement> stack;
  stack.emplace(StackElement{OctreeIndex{tree_height_, block_index},
                             ndtree_.getRootNode(), root_scale_coefficient_});
  while (!stack.empty()) {
    const OctreeIndex node_index = stack.top().node_index;
    typename OctreeType::NodeConstRefType node = stack.top().node;
    const typename Coefficients::Scale node_scale_coefficient =
        stack.top().scale_coefficient;
    stack.pop();

    const typename Coefficients::CoefficientsArray child_scale_coefficients =
        Transform::backward({node_scale_coefficient, {node.data()}});
    for (NdtreeIndexRelativeChild child_idx = 0;
         child_idx < OctreeIndex::kNumChildren; ++child_idx) {
      const OctreeIndex child_node_index =
          node_index.computeChildIndex(child_idx);
      const typename Coefficients::Scale child_scale_coefficient =
          child_scale_coefficients[child_idx];
      typename OctreeType::NodeConstPtrType child_node = node.getChild(child_idx);
      if (child_node && termination_height < child_node_index.height) {
        stack.emplace(StackElement{child_node_index, *child_node,
                                   child_scale_coefficient});
      } else {
        visitor_fn(child_node_index, child_scale_coefficient);
      }
    }
  }
}

template <typename CellDataT>
void HashedWaveletOctreeBlockT<CellDataT>::recursiveThreshold(
    typename OctreeType::NodeRefType node,
    typename Coefficients::Scale& node_scale_coefficient) {
  auto& node_detail_coefficients = node.data();
  typename Coefficients::CoefficientsArray child_scale_coefficients =
      Transform::backward({node_scale_coefficient, node_detail_coefficients});

  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    typename Coefficients::Scale& child_scale = child_scale_coefficients[child_idx];
    if (auto child_node = node.getChild(child_idx); child_node) {
      recursiveThreshold(*child_node, child_scale);
    } else {
      detail::setOccupancy(
          child_scale, std::clamp(detail::getOccupancy(child_scale),
                                  min_log_odds_, max_log_odds_));
    }
  }

  const auto [new_scale, new_details] =
      Transform::forward(child_scale_coefficients);
  node_detail_coefficients = new_details;
  node_scale_coefficient = new_scale;
}

template <typename CellDataT>
void HashedWaveletOctreeBlockT<CellDataT>::recursivePrune(
    typename OctreeType::NodeRefType node) {
  bool has_at_least_one_child = false;
  for (NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    if (typename OctreeType::NodePtrType child_node = node.getChild(child_idx);
        child_node) {
      recursivePrune(*child_node);
      if (!child_node->hasChildrenArray() &&
          !child_node->hasNonzeroData(1e-3f)) {
        node.eraseChild(child_idx);
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

#endif  // WAVEMAP_CORE_MAP_IMPL_HASHED_WAVELET_OCTREE_BLOCK_INL_H_
