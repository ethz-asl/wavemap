#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INDEX_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INDEX_INL_H_

#include <bitset>
#include <string>
#include <vector>

namespace wavemap_2d {
NodeIndex NodeIndex::computeParentIndex() const {
  DCHECK_GT(depth, 0);

  NodeIndex parent_index = *this;
  parent_index.position.x() >>= 1;
  parent_index.position.y() >>= 1;
  parent_index.depth = depth - 1;

  return parent_index;
}

NodeIndex NodeIndex::computeParentIndex(NodeIndexElement parent_depth) const {
  DCHECK_GT(parent_depth, 0);
  DCHECK_LT(parent_depth, depth);
  const NodeIndexElement depth_difference = depth - parent_depth;

  NodeIndex parent_index = *this;
  parent_index.position.x() >>= depth_difference;
  parent_index.position.y() >>= depth_difference;
  parent_index.depth = parent_depth;

  return parent_index;
}

std::vector<NodeIndex> NodeIndex::computeParentIndices() const {
  if (depth == 0) {
    return {};
  }

  std::vector<NodeIndex> parent_indices(depth);
  parent_indices[depth - 1] = computeParentIndex();
  for (NodeIndexElement depth_idx = depth - 2; 0 <= depth_idx; --depth_idx) {
    parent_indices[depth_idx] =
        parent_indices[depth_idx + 1].computeParentIndex();
  }

  return parent_indices;
}

NodeIndex NodeIndex::computeChildIndex(
    NodeRelativeChildIndex relative_child_index) const {
  NodeIndex child_index = *this;

  // Compute index of first child
  child_index.position *= 2;
  child_index.depth += 1;

  // Add offset to current child
  std::bitset<kMapDimension> child_bitset(relative_child_index);
  for (int i = 0; i < kMapDimension; ++i) {
    child_index.position[i] += child_bitset[i];
  }

  return child_index;
}

std::vector<NodeIndex> NodeIndex::computeChildIndices() const {
  std::vector<NodeIndex> child_indices(kNumChildren);
  for (NodeRelativeChildIndex relative_child_idx = 0;
       relative_child_idx < kNumChildren; ++relative_child_idx) {
    child_indices[relative_child_idx] = computeChildIndex(relative_child_idx);
  }
  return child_indices;
}

NodeRelativeChildIndex NodeIndex::computeRelativeChildIndex() const {
  NodeRelativeChildIndex child_index = 0;
  std::bitset<kMapDimension> child_bitset;
  for (int i = 0; i < kMapDimension; ++i) {
    child_bitset.set(i, position[i] & 0b1);
  }
  child_index = child_bitset.to_ulong();
  return child_index;
}

std::vector<NodeRelativeChildIndex> NodeIndex::computeRelativeChildIndices()
    const {
  std::vector<NodeRelativeChildIndex> child_indices(depth);
  NodeIndex node_index = *this;
  for (NodeIndexElement depth_idx = depth - 1; 0 <= depth_idx; --depth_idx) {
    child_indices[depth_idx] = node_index.computeRelativeChildIndex();
    node_index = node_index.computeParentIndex();
  }
  return child_indices;
}

std::string NodeIndex::toString() const {
  std::stringstream ss;
  ss << "[depth=";
  ss << depth << ", position=[";
  for (int i = 0; i < kMapDimension; ++i) {
    if (i) {
      ss << ", ";
    }
    ss << position[i];
  }
  ss << "]]";
  return ss.str();
}

}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INDEX_INL_H_
