#ifndef WAVEMAP_CORE_UTILS_EDIT_MULTIPLY_H_
#define WAVEMAP_CORE_UTILS_EDIT_MULTIPLY_H_

#include "wavemap/core/common.h"

namespace wavemap {

template <typename MapType>
void multiplyNodeRecursive(
    typename MapType::Block::OctreeType::NodeRefType node,
    FloatingPoint multiplier) {
  // Multiply
  node.data() *= multiplier;

  // Recursively handle all children
  for (int child_idx = 0; child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    if (auto child_node = node.getChild(child_idx); child_node) {
      multiplyNodeRecursive<MapType>(*child_node, multiplier);
    }
  }
}

template <typename MapType>
void multiply(MapType& map, FloatingPoint multiplier) {
  map.forEachBlock([multiplier](const Index3D& /*block_index*/, auto& block) {
    block.getRootScale() *= multiplier;
    multiplyNodeRecursive<MapType>(block.getRootNode(), multiplier);
    block.setLastUpdatedStamp();
  });
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_EDIT_MULTIPLY_H_
