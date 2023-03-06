#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_VOLUMETRIC_OCTREE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_VOLUMETRIC_OCTREE_INL_H_

#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "wavemap/data_structure/volumetric/cell_types/occupancy_state.h"

namespace wavemap {
inline OctreeIndex::ChildArray VolumetricOctree::getFirstChildIndices() const {
  OctreeIndex::ChildArray first_child_indices =
      getInternalRootNodeIndex().computeChildIndices();
  for (auto& child : first_child_indices) {
    child = toExternalNodeIndex(child);
  }
  return first_child_indices;
}

inline Index3D VolumetricOctree::getMinPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMinCornerIndex(getInternalRootNodeIndex()));
}

inline Index3D VolumetricOctree::getMaxPossibleIndex() const {
  return toExternalIndex(
      convert::nodeIndexToMaxCornerIndex(getInternalRootNodeIndex()));
}

inline FloatingPoint VolumetricOctree::getCellValue(
    const Index3D& index) const {
  const NodeType* deepest_node_at_index = getDeepestNodeAtIndex(index);
  if (deepest_node_at_index) {
    return deepest_node_at_index->data();
  }
  return 0.f;
}

inline void VolumetricOctree::setCellValue(const Index3D& index,
                                           FloatingPoint new_value) {
  const OctreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  setCellValue(node_index, new_value);
}

inline void VolumetricOctree::setCellValue(const OctreeIndex& node_index,
                                           FloatingPoint new_value) {
  constexpr bool kAutoAllocate = true;
  const OctreeIndex internal_node_index = toInternal(node_index);
  NodeType* node = ndtree_.getNode(internal_node_index, kAutoAllocate);
  if (node) {
    node->data() = new_value;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << node_index.toString();
  }
}

inline void VolumetricOctree::addToCellValue(const Index3D& index,
                                             FloatingPoint update) {
  const OctreeIndex node_index = convert::indexAndHeightToNodeIndex(index, 0);
  addToCellValue(node_index, update);
}

inline void VolumetricOctree::addToCellValue(const OctreeIndex& node_index,
                                             FloatingPoint update) {
  constexpr bool kAutoAllocate = true;
  const OctreeIndex internal_node_index = toInternal(node_index);
  NodeType* node = ndtree_.getNode(internal_node_index, kAutoAllocate);
  if (node) {
    node->data() += update;
  } else {
    LOG(ERROR) << "Failed to allocate cell at index: " << node_index.toString();
  }
}

inline const VolumetricOctree::NodeType*
VolumetricOctree::getDeepestNodeAtIndex(const Index3D& index) const {
  const OctreeIndex deepest_possible_internal_node_index = toInternal(index);
  const NodeType* node = &ndtree_.getRootNode();
  const MortonCode morton_code =
      deepest_possible_internal_node_index.computeMortonCode();
  for (int parent_height = kMaxHeight; 0 < parent_height; --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
  }
  return node;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_VOLUMETRIC_OCTREE_INL_H_
