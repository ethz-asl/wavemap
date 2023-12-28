#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_PTR_INL_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_PTR_INL_H_

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
ChunkedNdtreeNodeRef<ChunkType>::operator NodeConstRef() {
  return {chunk_, relative_node_depth_, level_traversal_distance_};
}

template <typename ChunkType>
typename ChunkedNdtreeNodeRef<ChunkType>::NodePtr
ChunkedNdtreeNodeRef<ChunkType>::operator&() {  // NOLINT
  return {&chunk_, relative_node_depth_, level_traversal_distance_};
}

template <typename ChunkType>
typename ChunkedNdtreeNodeRef<ChunkType>::NodeConstPtr
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
auto& ChunkedNdtreeNodeRef<ChunkType>::data() {
  return chunk_.nodeData(computeRelativeNodeIndex());
}

template <typename ChunkType>
const auto& ChunkedNdtreeNodeRef<ChunkType>::data() const {
  return chunk_.nodeData(computeRelativeNodeIndex());
}

template <typename ChunkType>
typename ChunkType::BitRef
ChunkedNdtreeNodeRef<ChunkType>::hasAtLeastOneChild() {
  return chunk_.nodeHasAtLeastOneChild(computeRelativeNodeIndex());
}

template <typename ChunkType>
bool ChunkedNdtreeNodeRef<ChunkType>::hasAtLeastOneChild() const {
  return chunk_.nodeHasAtLeastOneChild(computeRelativeNodeIndex());
}

template <typename ChunkType>
bool ChunkedNdtreeNodeRef<ChunkType>::hasChild(
    NdtreeIndexRelativeChild child_index) const {
  return getChild(child_index);
}

template <typename ChunkType>
typename ChunkedNdtreeNodeRef<ChunkType>::NodePtr
ChunkedNdtreeNodeRef<ChunkType>::getChild(
    NdtreeIndexRelativeChild child_index) {
  const IndexElement child_depth = relative_node_depth_ + 1;
  const LinearIndex child_level_traversal_distance =
      computeChildLevelTraversalDistance(child_index);
  if (child_depth % ChunkType::kHeight == 0) {
    auto* child_chunk = chunk_.getChild(child_level_traversal_distance);
    return {child_chunk, 0, 0u};
  } else {
    return {&chunk_, child_depth, child_level_traversal_distance};
  }
}

template <typename ChunkType>
typename ChunkedNdtreeNodeRef<ChunkType>::NodeConstPtr
ChunkedNdtreeNodeRef<ChunkType>::getChild(
    NdtreeIndexRelativeChild child_index) const {
  const IndexElement child_depth = relative_node_depth_ + 1;
  const LinearIndex child_level_traversal_distance =
      computeChildLevelTraversalDistance(child_index);
  if (child_depth % ChunkType::kHeight == 0) {
    const auto* child_chunk = chunk_.getChild(child_level_traversal_distance);
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
  const IndexElement child_depth = relative_node_depth_ + 1;
  const LinearIndex child_level_traversal_distance =
      computeChildLevelTraversalDistance(child_index);
  if (child_depth % ChunkType::kHeight == 0) {
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

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_PTR_INL_H_
