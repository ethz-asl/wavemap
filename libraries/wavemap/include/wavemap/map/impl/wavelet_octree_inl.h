#ifndef WAVEMAP_MAP_IMPL_WAVELET_OCTREE_INL_H_
#define WAVEMAP_MAP_IMPL_WAVELET_OCTREE_INL_H_

#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap/utils/query/occupancy_classifier.h"

namespace wavemap {
inline bool WaveletOctree::empty() const {
  // Check if all cells in the map are equal to zero
  // NOTE: Aside from checking whether the map contains no detail
  //       coefficients, we also need to check whether its scale coefficient
  //       (average value over the whole map) is zero.
  return ndtree_.empty() &&
         OccupancyClassifier::isUnobserved(root_scale_coefficient_);
}

inline void WaveletOctree::clear() {
  ndtree_.clear();
  root_scale_coefficient_ = {};
}

inline OctreeIndex::ChildArray WaveletOctree::getFirstChildIndices() const {
  OctreeIndex::ChildArray first_child_indices =
      getInternalRootNodeIndex().computeChildIndices();
  for (auto& child : first_child_indices) {
    child = toExternalNodeIndex(child);
  }
  return first_child_indices;
}

inline Index3D WaveletOctree::getMinPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMinCornerIndex(getInternalRootNodeIndex()));
}

inline Index3D WaveletOctree::getMaxPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMaxCornerIndex(getInternalRootNodeIndex()));
}

inline FloatingPoint WaveletOctree::getCellValue(const Index3D& index) const {
  const OctreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  return getCellValue(node_index);
}

inline FloatingPoint WaveletOctree::getCellValue(
    const OctreeIndex& index) const {
  const OctreeIndex internal_index = toInternal(index);
  const MortonIndex morton_code = convert::nodeIndexToMorton(internal_index);
  const NodeType* node = &ndtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (int parent_height = config_.tree_height;
       internal_index.height < parent_height; --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    value = Transform::backwardSingleChild({value, node->data()}, child_index);
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
  }
  return value;
}

inline void WaveletOctree::setCellValue(const Index3D& index,
                                        FloatingPoint new_value) {
  const OctreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  setCellValue(node_index, new_value);
}

inline void WaveletOctree::setCellValue(const OctreeIndex& index,
                                        FloatingPoint new_value) {
  const OctreeIndex internal_index = toInternal(index);
  const MortonIndex morton_code = convert::nodeIndexToMorton(internal_index);
  std::vector<NodeType*> node_ptrs;
  const int height_difference = config_.tree_height - internal_index.height;
  node_ptrs.reserve(height_difference);
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  FloatingPoint current_value = root_scale_coefficient_;
  for (int parent_height = config_.tree_height;
       internal_index.height + 1 < parent_height; --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    NodeType* current_parent = node_ptrs.back();
    current_value = Transform::backwardSingleChild(
        {current_value, current_parent->data()}, child_index);
    NodeType& child = current_parent->getOrAllocateChild(child_index);
    node_ptrs.emplace_back(&child);
  }
  DCHECK_EQ(node_ptrs.size(), height_difference);

  Coefficients::Parent coefficients{new_value - current_value, {}};
  for (int parent_height = internal_index.height + 1;
       parent_height <= config_.tree_height; ++parent_height) {
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

inline void WaveletOctree::addToCellValue(const Index3D& index,
                                          FloatingPoint update) {
  const OctreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  addToCellValue(node_index, update);
}

inline void WaveletOctree::addToCellValue(const OctreeIndex& index,
                                          FloatingPoint update) {
  const OctreeIndex internal_index = toInternal(index);
  const MortonIndex morton_code = convert::nodeIndexToMorton(internal_index);

  std::vector<NodeType*> node_ptrs;
  const int height_difference = config_.tree_height - internal_index.height;
  node_ptrs.reserve(height_difference);
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  for (int parent_height = config_.tree_height;
       internal_index.height + 1 < parent_height; --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    NodeType* current_parent = node_ptrs.back();
    NodeType& child = current_parent->getOrAllocateChild(child_index);
    node_ptrs.emplace_back(&child);
  }
  DCHECK_EQ(node_ptrs.size(), height_difference);

  Coefficients::Parent coefficients{update, {}};
  for (int parent_height = internal_index.height + 1;
       parent_height <= config_.tree_height; ++parent_height) {
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
}  // namespace wavemap

#endif  // WAVEMAP_MAP_IMPL_WAVELET_OCTREE_INL_H_
