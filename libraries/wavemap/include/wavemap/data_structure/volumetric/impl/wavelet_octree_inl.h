#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_OCTREE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_OCTREE_INL_H_

#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap/data_structure/volumetric/cell_types/occupancy_state.h"

namespace wavemap {
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
  const MortonCode morton_code = internal_index.computeMortonCode();
  const NodeType* node = &ndtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (int parent_height = max_height_; internal_index.height < parent_height;
       --parent_height) {
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
  const MortonCode morton_code = internal_index.computeMortonCode();
  std::vector<NodeType*> node_ptrs;
  const int height_difference = max_height_ - internal_index.height;
  node_ptrs.reserve(height_difference);
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  FloatingPoint current_value = root_scale_coefficient_;
  for (int parent_height = max_height_;
       internal_index.height + 1 < parent_height; --parent_height) {
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
  for (int parent_height = internal_index.height + 1;
       parent_height <= max_height_; ++parent_height) {
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
  const MortonCode morton_code = internal_index.computeMortonCode();

  std::vector<NodeType*> node_ptrs;
  const int height_difference = max_height_ - internal_index.height;
  node_ptrs.reserve(height_difference);
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  for (int parent_height = max_height_;
       internal_index.height + 1 < parent_height; --parent_height) {
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
  for (int parent_height = internal_index.height + 1;
       parent_height <= max_height_; ++parent_height) {
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

inline WaveletOctree::NodeType* WaveletOctree::getNode(
    const OctreeIndex& index) {
  const OctreeIndex internal_index = toInternal(index);
  const MortonCode morton_code = internal_index.computeMortonCode();
  NodeType* node = &ndtree_.getRootNode();
  for (int parent_height = max_height_; internal_index.height < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    if (!node->hasChild(child_index)) {
      node->template allocateChild(child_index);
    }
    node = node->getChild(child_index);
  }
  return node;
}

inline const WaveletOctree::NodeType* WaveletOctree::getNode(
    const OctreeIndex& index) const {
  const OctreeIndex internal_index = toInternal(index);
  const MortonCode morton_code = internal_index.computeMortonCode();
  const NodeType* node = &ndtree_.getRootNode();
  for (int parent_height = max_height_; internal_index.height < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    if (!node->hasChild(child_index)) {
      return nullptr;
    }
    node = node->getChild(child_index);
  }
  return node;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_OCTREE_INL_H_
