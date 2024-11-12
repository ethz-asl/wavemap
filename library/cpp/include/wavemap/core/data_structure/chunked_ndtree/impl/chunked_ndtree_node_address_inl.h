#ifndef WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_ADDRESS_INL_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_ADDRESS_INL_H_

#include <utility>

namespace wavemap {
template <typename ChunkType>
ChunkedNdtreeNodePtr<ChunkType>::ChunkedNdtreeNodePtr(
    ChunkType* chunk, IndexElement relative_node_depth,
    LinearIndex level_traversal_distance)
    : node_(chunk ? std::make_optional<NodeRef>(*chunk, relative_node_depth,
                                                level_traversal_distance)
                  : std::nullopt) {}

template <typename ChunkType>
ChunkedNdtreeNodePtr<ChunkType>::ChunkedNdtreeNodePtr(
    const ChunkedNdtreeNodePtr& other)
    : node_(other ? std::make_optional<NodeRef>(other.node_.value())
                  : std::nullopt) {}

template <typename ChunkType>
ChunkedNdtreeNodePtr<ChunkType>::ChunkedNdtreeNodePtr(
    ChunkedNdtreeNodePtr&& other) noexcept
    : node_(other ? std::make_optional<NodeRef>(std::move(other.node_.value()))
                  : std::nullopt) {}

template <typename ChunkType>
ChunkedNdtreeNodePtr<ChunkType>& ChunkedNdtreeNodePtr<ChunkType>::operator=(
    const ChunkedNdtreeNodePtr& other) {
  if (other) {
    node_.emplace(other.node_.value());
  } else {
    reset();
  }
  return *this;
}

template <typename ChunkType>
ChunkedNdtreeNodePtr<ChunkType>& ChunkedNdtreeNodePtr<ChunkType>::operator=(
    ChunkedNdtreeNodePtr&& other) noexcept {
  if (other) {
    node_.emplace(std::move(other.node_.value()));
  } else {
    reset();
  }
  return *this;
}

template <typename ChunkType>
ChunkedNdtreeNodeRef<ChunkType>::ChunkedNdtreeNodeRef(
    ChunkType& chunk, IndexElement relative_node_depth,
    LinearIndex level_traversal_distance)
    : chunk_(chunk),
      relative_node_depth_(relative_node_depth),
      level_traversal_distance_(level_traversal_distance) {}

template <typename ChunkType>
ChunkedNdtreeNodeRef<ChunkType>::ChunkedNdtreeNodeRef(
    const ChunkedNdtreeNodeRef& other)
    : ChunkedNdtreeNodeRef(other.chunk_, other.relative_node_depth_,
                           other.level_traversal_distance_) {}

template <typename ChunkType>
ChunkedNdtreeNodeRef<ChunkType>::ChunkedNdtreeNodeRef(
    ChunkedNdtreeNodeRef&& other) noexcept
    : chunk_(other.chunk_),
      relative_node_depth_(other.relative_node_depth_),
      level_traversal_distance_(other.level_traversal_distance_) {}

template <typename ChunkType>
ChunkedNdtreeNodeRef<ChunkType>::operator ChunkedNdtreeNodeRef<
    const ChunkType>() const {
  return {chunk_, relative_node_depth_, level_traversal_distance_};
}

template <typename ChunkType>
typename ChunkedNdtreeNodeRef<ChunkType>::NodePtr
ChunkedNdtreeNodeRef<ChunkType>::operator&() const {  // NOLINT
  return {&chunk_, relative_node_depth_, level_traversal_distance_};
}

template <typename ChunkType>
bool ChunkedNdtreeNodeRef<ChunkType>::empty() const {
  return !hasAtLeastOneChild() && !hasNonzeroData();
}

template <typename ChunkType>
bool ChunkedNdtreeNodeRef<ChunkType>::hasNonzeroData() const {
  return chunk_.nodeHasNonzeroData(computeRelativeNodeIndex());
}

template <typename ChunkType>
bool ChunkedNdtreeNodeRef<ChunkType>::hasNonzeroData(
    FloatingPoint threshold) const {
  return chunk_.nodeHasNonzeroData(computeRelativeNodeIndex(), threshold);
}

template <typename ChunkType>
auto& ChunkedNdtreeNodeRef<ChunkType>::data() const {
  return chunk_.nodeData(computeRelativeNodeIndex());
}

template <typename ChunkType>
auto ChunkedNdtreeNodeRef<ChunkType>::hasAtLeastOneChild() const {
  return chunk_.nodeHasAtLeastOneChild(computeRelativeNodeIndex());
}

template <typename ChunkType>
bool ChunkedNdtreeNodeRef<ChunkType>::hasChild(
    NdtreeIndexRelativeChild child_index) const {
  return getChild(child_index);
}

template <typename ChunkType>
void ChunkedNdtreeNodeRef<ChunkType>::eraseChild(
    NdtreeIndexRelativeChild child_index) const {
  if (!hasAtLeastOneChild()) {
    return;
  }
  LinearIndex child_start_idx = computeChildLevelTraversalDistance(child_index);
  LinearIndex child_end_idx = child_start_idx + 1;
  for (int child_depth = relative_node_depth_ + 1;
       child_depth <= ChunkType::kHeight; ++child_depth) {
    for (LinearIndex level_child_idx = child_start_idx;
         level_child_idx < child_end_idx; ++level_child_idx) {
      if (child_depth == ChunkType::kHeight) {
        chunk_.eraseChild(level_child_idx);
      } else {
        const LinearIndex level_offset =
            tree_math::perfect_tree::num_total_nodes_fast<kDim>(child_depth);
        const LinearIndex chunk_child_idx = level_offset + level_child_idx;
        chunk_.nodeData(chunk_child_idx) = {};
        chunk_.nodeHasAtLeastOneChild(chunk_child_idx) = false;
      }
    }
    child_start_idx <<= kDim;
    child_end_idx <<= kDim;
  }
}

template <typename ChunkType>
typename ChunkedNdtreeNodeRef<ChunkType>::NodePtr
ChunkedNdtreeNodeRef<ChunkType>::getChild(
    NdtreeIndexRelativeChild child_index) const {
  if (!hasAtLeastOneChild()) {
    return {nullptr, 0, 0u};
  }
  const IndexElement child_depth = relative_node_depth_ + 1;
  const LinearIndex child_level_traversal_distance =
      computeChildLevelTraversalDistance(child_index);
  if (child_depth == ChunkType::kHeight) {
    auto* child_chunk = chunk_.getChild(child_level_traversal_distance);
    return {child_chunk, 0, 0u};
  } else {
    return {&chunk_, child_depth, child_level_traversal_distance};
  }
}

template <typename ChunkType>
template <typename... DefaultArgs>
ChunkedNdtreeNodeRef<ChunkType>
ChunkedNdtreeNodeRef<ChunkType>::getOrAllocateChild(
    NdtreeIndexRelativeChild child_index, DefaultArgs&&... args) const {
  hasAtLeastOneChild() = true;
  const IndexElement child_depth = relative_node_depth_ + 1;
  const LinearIndex child_level_traversal_distance =
      computeChildLevelTraversalDistance(child_index);
  if (child_depth == ChunkType::kHeight) {
    auto& child_chunk = chunk_.getOrAllocateChild(
        child_level_traversal_distance, std::forward<DefaultArgs>(args)...);
    return {child_chunk, 0, 0};
  } else {
    return {chunk_, child_depth, child_level_traversal_distance};
  }
}

template <typename ChunkType>
LinearIndex ChunkedNdtreeNodeRef<ChunkType>::computeRelativeNodeIndex() const {
  const LinearIndex parent_to_first_child_distance =
      tree_math::perfect_tree::num_total_nodes_fast<kDim>(relative_node_depth_);
  return parent_to_first_child_distance + level_traversal_distance_;
}

template <typename ChunkType>
LinearIndex ChunkedNdtreeNodeRef<ChunkType>::computeChildLevelTraversalDistance(
    NdtreeIndexRelativeChild child_index) const {
  DCHECK_GE(child_index, 0);
  DCHECK_LT(child_index, 1 << kDim);
  return (level_traversal_distance_ << kDim) | child_index;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_ADDRESS_INL_H_
