#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INDEX_INL_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INDEX_INL_H_

#include <bitset>
#include <string>
#include <vector>

namespace wavemap_2d {
inline NodeIndex NodeIndex::computeParentIndex() const {
  CHECK_GT(depth, 0u);
  NodeIndex parent_index;

  parent_index.position = (position.cast<FloatingPoint>().array() / 2.f)
                              .floor()
                              .cast<NodeIndexElement>();
  parent_index.depth = depth - 1;

  return parent_index;
}

inline NodeIndex NodeIndex::computeParentIndex(
    NodeIndexElement parent_depth) const {
  CHECK_LT(parent_depth, depth);
  NodeIndex parent_index;
  NodeIndexElement depth_difference = depth - parent_depth;

  parent_index.position =
      (position.cast<FloatingPoint>().array() / std::exp2(depth_difference))
          .floor()
          .cast<NodeIndexElement>();
  parent_index.depth = parent_depth;

  return parent_index;
}

inline std::vector<NodeIndex> NodeIndex::computeParentIndices() const {
  std::vector<NodeIndex> parent_indices(depth);

  if (depth == 0u) {
    return parent_indices;
  }

  parent_indices[depth - 1] = computeParentIndex();
  for (int depth_idx = static_cast<int>(depth) - 2; 0 <= depth_idx;
       --depth_idx) {
    parent_indices[depth_idx] =
        parent_indices[depth_idx + 1].computeParentIndex();
  }

  return parent_indices;
}

inline NodeIndex NodeIndex::computeChildIndex(
    NodeRelativeChildIndex relative_child_index) const {
  NodeIndex child_index = *this;

  // Compute index of first child
  child_index.position *= 2;
  child_index.depth += 1;

  // Add offset to current child
  std::bitset<kMapDimension> child_bitset(relative_child_index);
  for (int i = 0; i < kMapDimension; ++i) {
    if (child_bitset[i]) child_index.position[i] += 1;
  }

  return child_index;
}

inline std::vector<NodeIndex> NodeIndex::computeChildIndices() const {
  std::vector<NodeIndex> child_indices(kNumChildren);

  for (NodeRelativeChildIndex relative_child_idx = 0;
       relative_child_idx < kNumChildren; ++relative_child_idx) {
    child_indices[relative_child_idx] = computeChildIndex(relative_child_idx);
  }

  return child_indices;
}

inline NodeRelativeChildIndex NodeIndex::computeRelativeChildIndex() const {
  NodeRelativeChildIndex child_index = 0;
  for (int i = 0; i < kMapDimension; ++i) {
    if (position[i] % 2) child_index += constexpr_functions::exp2(i);
  }
  return child_index;
}

inline std::vector<NodeRelativeChildIndex>
NodeIndex::computeRelativeChildIndices() const {
  std::vector<NodeRelativeChildIndex> child_indices(depth);
  NodeIndex node_index = *this;

  for (int depth_idx = static_cast<int>(depth) - 1; 0 <= depth_idx;
       --depth_idx) {
    child_indices[depth_idx] = node_index.computeRelativeChildIndex();
    node_index = node_index.computeParentIndex();
  }

  return child_indices;
}

inline std::string NodeIndex::toString() const {
  std::stringstream ss;
  ss << "[";
  for (int i = 0; i < kMapDimension; ++i) {
    ss << position[i] << ", ";
  }
  ss << depth << "]";
  return ss.str();
}

}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_NODE_INDEX_INL_H_
