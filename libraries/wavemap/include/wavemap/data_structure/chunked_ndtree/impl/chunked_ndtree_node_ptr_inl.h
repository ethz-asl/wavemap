#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_PTR_INL_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_PTR_INL_H_

#include <utility>

namespace wavemap {
template <typename ChunkType>
bool ChunkedNdtreeNodePtr<ChunkType>::hasNonzeroData() const {
  return chunk_->nodeHasNonzeroData(computeRelativeNodeIndex());
}

template <typename ChunkType>
bool ChunkedNdtreeNodePtr<ChunkType>::hasNonzeroData(
    FloatingPoint threshold) const {
  return chunk_->nodeHasNonzeroData(computeRelativeNodeIndex(), threshold);
}

template <typename ChunkType>
auto& ChunkedNdtreeNodePtr<ChunkType>::data() {
  return chunk_->nodeData(computeRelativeNodeIndex());
}

template <typename ChunkType>
const auto& ChunkedNdtreeNodePtr<ChunkType>::data() const {
  return chunk_->nodeData(computeRelativeNodeIndex());
}

template <typename ChunkType>
typename ChunkType::BitRef
ChunkedNdtreeNodePtr<ChunkType>::hasAtLeastOneChild() {
  return chunk_->nodeHasAtLeastOneChild(computeRelativeNodeIndex());
}

template <typename ChunkType>
bool ChunkedNdtreeNodePtr<ChunkType>::hasAtLeastOneChild() const {
  return chunk_->nodeHasAtLeastOneChild(computeRelativeNodeIndex());
}

template <typename ChunkType>
bool ChunkedNdtreeNodePtr<ChunkType>::hasChild(
    NdtreeIndexRelativeChild child_index) const {
  return getChild(child_index);
}

template <typename ChunkType>
ChunkedNdtreeNodePtr<ChunkType> ChunkedNdtreeNodePtr<ChunkType>::getChild(
    NdtreeIndexRelativeChild child_index) {
  const IndexElement child_depth = relative_node_depth_ + 1;
  const LinearIndex child_level_traversal_distance =
      computeChildLevelTraversalDistance(child_index);
  if (child_depth % ChunkType::kHeight == 0) {
    auto* child_chunk = chunk_->getChild(child_level_traversal_distance);
    return {child_chunk, 0, 0u};
  } else {
    return {chunk_, child_depth, child_level_traversal_distance};
  }
}

template <typename ChunkType>
ChunkedNdtreeNodePtr<const ChunkType> ChunkedNdtreeNodePtr<ChunkType>::getChild(
    NdtreeIndexRelativeChild child_index) const {
  const IndexElement child_depth = relative_node_depth_ + 1;
  const LinearIndex child_level_traversal_distance =
      computeChildLevelTraversalDistance(child_index);
  if (child_depth % ChunkType::kHeight == 0) {
    const auto* child_chunk = chunk_->getChild(child_level_traversal_distance);
    return {child_chunk, 0, 0u};
  } else {
    return {chunk_, child_depth, child_level_traversal_distance};
  }
}

template <typename ChunkType>
template <typename... DefaultArgs>
ChunkedNdtreeNodePtr<ChunkType>
ChunkedNdtreeNodePtr<ChunkType>::getOrAllocateChild(
    NdtreeIndexRelativeChild child_index, DefaultArgs&&... args) const {
  const IndexElement child_depth = relative_node_depth_ + 1;
  const LinearIndex child_level_traversal_distance =
      computeChildLevelTraversalDistance(child_index);
  if (child_depth % ChunkType::kHeight == 0) {
    auto& child_chunk = chunk_->getOrAllocateChild(
        child_level_traversal_distance, std::forward<DefaultArgs>(args)...);
    return {child_chunk, 0, 0};
  } else {
    return {chunk_, child_depth, child_level_traversal_distance};
  }
}

template <typename ChunkType>
LinearIndex ChunkedNdtreeNodePtr<ChunkType>::computeRelativeNodeIndex() const {
  const LinearIndex parent_to_first_child_distance =
      tree_math::perfect_tree::num_total_nodes_fast<kDim>(relative_node_depth_);
  return parent_to_first_child_distance + level_traversal_distance_;
}

template <typename ChunkType>
LinearIndex ChunkedNdtreeNodePtr<ChunkType>::computeChildLevelTraversalDistance(
    NdtreeIndexRelativeChild child_index) const {
  DCHECK_GE(child_index, 0);
  DCHECK_LT(child_index, 1 << kDim);
  return (level_traversal_distance_ << kDim) | child_index;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_PTR_INL_H_
