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
  const OctreeIndex deepest_possible_node_index = toInternal(index);
  const std::vector<OctreeIndex::RelativeChild> child_indices =
      deepest_possible_node_index
          .template computeRelativeChildIndices<kMaxHeight>();
  const NodeType* node = &ndtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (const OctreeIndex::RelativeChild child_index : child_indices) {
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

inline void WaveletOctree::setCellValue(const OctreeIndex& node_index,
                                        FloatingPoint new_value) {
  const OctreeIndex internal_node_index = toInternal(node_index);
  const std::vector<OctreeIndex::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<kMaxHeight>();
  std::vector<NodeType*> node_ptrs;
  node_ptrs.reserve(child_indices.size());
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  FloatingPoint current_value = root_scale_coefficient_;
  for (size_t depth = 0; depth < child_indices.size() - 1; ++depth) {
    const OctreeIndex::RelativeChild child_index = child_indices[depth];
    NodeType* current_parent = node_ptrs.back();
    current_value = Transform::backwardSingleChild(
        {current_value, current_parent->data()}, child_index);
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }

  Coefficients::Parent coefficients{new_value - current_value, {}};
  for (int depth = static_cast<int>(child_indices.size()) - 1; 0 <= depth;
       --depth) {
    const OctreeIndex::RelativeChild relative_child_idx = child_indices[depth];
    NodeType* current_node = node_ptrs[depth];
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, relative_child_idx);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

inline void WaveletOctree::addToCellValue(const Index3D& index,
                                          FloatingPoint update) {
  const OctreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  addToCellValue(node_index, update);
}

inline void WaveletOctree::addToCellValue(const OctreeIndex& node_index,
                                          FloatingPoint update) {
  const OctreeIndex internal_node_index = toInternal(node_index);
  const std::vector<OctreeIndex::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<kMaxHeight>();
  std::vector<NodeType*> node_ptrs;
  node_ptrs.reserve(child_indices.size());
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  for (size_t depth = 0; depth < child_indices.size() - 1; ++depth) {
    const OctreeIndex::RelativeChild child_index = child_indices[depth];
    NodeType* current_parent = node_ptrs.back();
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }

  Coefficients::Parent coefficients{update, {}};
  for (int depth = static_cast<int>(child_indices.size()) - 1; 0 <= depth;
       --depth) {
    NodeType* current_node = node_ptrs[depth];
    const OctreeIndex::RelativeChild relative_child_idx = child_indices[depth];
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, relative_child_idx);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

inline WaveletOctree::NodeType* WaveletOctree::getNode(
    const OctreeIndex& node_index) {
  const OctreeIndex internal_node_index = toInternal(node_index);
  const std::vector<OctreeIndex::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<kMaxHeight>();
  NodeType* node = &ndtree_.getRootNode();
  for (const OctreeIndex::RelativeChild child_index : child_indices) {
    if (!node->hasChild(child_index)) {
      node->template allocateChild(child_index);
    }
    node = node->getChild(child_index);
  }
  return node;
}

inline const WaveletOctree::NodeType* WaveletOctree::getNode(
    const OctreeIndex& node_index) const {
  const OctreeIndex internal_node_index = toInternal(node_index);
  const std::vector<OctreeIndex::RelativeChild> child_indices =
      internal_node_index.template computeRelativeChildIndices<kMaxHeight>();
  const NodeType* node = &ndtree_.getRootNode();
  for (const OctreeIndex::RelativeChild child_index : child_indices) {
    if (!node->hasChild(child_index)) {
      return nullptr;
    }
    node = node->getChild(child_index);
  }
  return node;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_WAVELET_OCTREE_INL_H_
