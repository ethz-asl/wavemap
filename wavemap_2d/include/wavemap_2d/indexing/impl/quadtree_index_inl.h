#ifndef WAVEMAP_2D_INDEXING_IMPL_QUADTREE_INDEX_INL_H_
#define WAVEMAP_2D_INDEXING_IMPL_QUADTREE_INDEX_INL_H_

#include <vector>

namespace wavemap_2d {
inline QuadtreeIndex QuadtreeIndex::computeParentIndex() const {
  DCHECK_GT(depth, 0);

  QuadtreeIndex parent_index = *this;
  parent_index.position.x() >>= 1;
  parent_index.position.y() >>= 1;
  parent_index.depth = depth - 1;

  return parent_index;
}

inline QuadtreeIndex QuadtreeIndex::computeParentIndex(
    NodeIndexElement parent_depth) const {
  DCHECK_GE(parent_depth, 0);
  DCHECK_LT(parent_depth, depth);
  const NodeIndexElement depth_difference = depth - parent_depth;

  QuadtreeIndex parent_index = *this;
  parent_index.position.x() >>= depth_difference;
  parent_index.position.y() >>= depth_difference;
  parent_index.depth = parent_depth;

  return parent_index;
}

inline std::vector<QuadtreeIndex> QuadtreeIndex::computeParentIndices() const {
  if (depth == 0) {
    return {};
  }

  std::vector<QuadtreeIndex> parent_indices(depth);
  parent_indices[depth - 1] = computeParentIndex();
  for (NodeIndexElement depth_idx = depth - 2; 0 <= depth_idx; --depth_idx) {
    parent_indices[depth_idx] =
        parent_indices[depth_idx + 1].computeParentIndex();
  }

  return parent_indices;
}

inline QuadtreeIndex QuadtreeIndex::computeChildIndex(
    NodeRelativeChildIndex relative_child_index) const {
  QuadtreeIndex child_index = *this;

  // Compute index of first child
  child_index.position *= 2;
  child_index.depth += 1;

  // Add offset to current child
  for (int i = 0; i < kMapDimension; ++i) {
    child_index.position[i] += (relative_child_index >> i) & 0b1;
  }

  return child_index;
}

inline std::vector<QuadtreeIndex> QuadtreeIndex::computeChildIndices() const {
  std::vector<QuadtreeIndex> child_indices(kNumChildren);
  for (NodeRelativeChildIndex relative_child_idx = 0;
       relative_child_idx < kNumChildren; ++relative_child_idx) {
    child_indices[relative_child_idx] = computeChildIndex(relative_child_idx);
  }
  return child_indices;
}

inline NodeRelativeChildIndex QuadtreeIndex::computeRelativeChildIndex() const {
  NodeRelativeChildIndex child_index = 0;
  for (int i = 0; i < kMapDimension; ++i) {
    child_index += (position[i] & 0b1) << i;
  }
  return child_index;
}

inline std::vector<NodeRelativeChildIndex>
QuadtreeIndex::computeRelativeChildIndices() const {
  std::vector<NodeRelativeChildIndex> child_indices(depth);
  QuadtreeIndex node_index = *this;
  for (NodeIndexElement depth_idx = depth - 1; 0 <= depth_idx; --depth_idx) {
    child_indices[depth_idx] = node_index.computeRelativeChildIndex();
    node_index = node_index.computeParentIndex();
  }
  return child_indices;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INDEXING_IMPL_QUADTREE_INDEX_INL_H_
