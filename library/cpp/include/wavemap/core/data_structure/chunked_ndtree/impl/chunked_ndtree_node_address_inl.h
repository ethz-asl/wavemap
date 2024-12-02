#ifndef WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_ADDRESS_INL_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_ADDRESS_INL_H_

#include <utility>

namespace wavemap {
template <typename ChunkT>
ChunkedNdtreeNodePtr<ChunkT>::ChunkedNdtreeNodePtr(ChunkT* chunk)
    : node_(chunk ? std::make_optional<NodeRef>(*chunk) : std::nullopt) {}

template <typename ChunkT>
ChunkedNdtreeNodePtr<ChunkT>::ChunkedNdtreeNodePtr(ChunkT* chunk,
                                                   OffsetType offset)
    : node_(chunk ? std::make_optional<NodeRef>(*chunk, offset)
                  : std::nullopt) {}

template <typename ChunkT>
ChunkedNdtreeNodePtr<ChunkT>::ChunkedNdtreeNodePtr(
    const ChunkedNdtreeNodePtr& other)
    : node_(other ? std::make_optional<NodeRef>(other.node_.value())
                  : std::nullopt) {}

template <typename ChunkT>
ChunkedNdtreeNodePtr<ChunkT>::ChunkedNdtreeNodePtr(
    ChunkedNdtreeNodePtr&& other) noexcept
    : node_(other ? std::make_optional<NodeRef>(std::move(other.node_.value()))
                  : std::nullopt) {}

template <typename ChunkT>
ChunkedNdtreeNodePtr<ChunkT>& ChunkedNdtreeNodePtr<ChunkT>::operator=(
    const ChunkedNdtreeNodePtr& other) {
  if (other) {
    node_.emplace(other.node_.value());
  } else {
    reset();
  }
  return *this;
}

template <typename ChunkT>
ChunkedNdtreeNodePtr<ChunkT>& ChunkedNdtreeNodePtr<ChunkT>::operator=(
    ChunkedNdtreeNodePtr&& other) noexcept {
  if (other) {
    node_.emplace(std::move(other.node_.value()));
  } else {
    reset();
  }
  return *this;
}

template <typename ChunkT>
ChunkedNdtreeNodeRef<ChunkT>::ChunkedNdtreeNodeRef(
    const ChunkedNdtreeNodeRef& other)
    : ChunkedNdtreeNodeRef(other.chunk_, other.offset_) {}

template <typename ChunkT>
ChunkedNdtreeNodeRef<ChunkT>::ChunkedNdtreeNodeRef(
    ChunkedNdtreeNodeRef&& other) noexcept
    : chunk_(other.chunk_), offset_(other.offset_) {}

template <typename ChunkT>
ChunkedNdtreeNodeRef<ChunkT>::operator ChunkedNdtreeNodeRef<const ChunkT>()
    const {
  return {chunk_, offset_};
}

template <typename ChunkT>
typename ChunkedNdtreeNodeRef<ChunkT>::NodePtr
ChunkedNdtreeNodeRef<ChunkT>::operator&() const {  // NOLINT
  return {&chunk_, offset_};
}

template <typename ChunkT>
bool ChunkedNdtreeNodeRef<ChunkT>::empty() const {
  return !hasAtLeastOneChild() && !hasNonzeroData();
}

template <typename ChunkT>
bool ChunkedNdtreeNodeRef<ChunkT>::hasNonzeroData() const {
  return chunk_.nodeHasNonzeroData(offset_);
}

template <typename ChunkT>
bool ChunkedNdtreeNodeRef<ChunkT>::hasNonzeroData(
    FloatingPoint threshold) const {
  return chunk_.nodeHasNonzeroData(offset_, threshold);
}

template <typename ChunkT>
auto& ChunkedNdtreeNodeRef<ChunkT>::data() const {
  return chunk_.nodeData(offset_);
}

template <typename ChunkT>
auto ChunkedNdtreeNodeRef<ChunkT>::hasAtLeastOneChild() const {
  return chunk_.nodeHasAtLeastOneChild(offset_);
}

template <typename ChunkT>
bool ChunkedNdtreeNodeRef<ChunkT>::hasChild(
    NdtreeIndexRelativeChild child_index) const {
  return getChild(child_index);
}

template <typename ChunkT>
void ChunkedNdtreeNodeRef<ChunkT>::eraseChild(
    NdtreeIndexRelativeChild child_index) const {
  if (!hasAtLeastOneChild()) {
    return;
  }
  const OffsetType child_offset = computeChildOffset(child_index);
  OffsetType child_start_idx =
      convert::nodeOffsetToLevelIndex<kDim>(child_offset);
  OffsetType child_end_idx = child_start_idx + 1;
  for (IndexElement child_depth =
           convert::nodeOffsetToDepth<kDim>(child_offset);
       child_depth <= ChunkT::kHeight; ++child_depth) {
    const OffsetType level_offset =
        tree_math::perfect_tree::num_total_nodes_fast<kDim>(child_depth);
    for (OffsetType level_child_idx = child_start_idx;
         level_child_idx < child_end_idx; ++level_child_idx) {
      if (child_depth == ChunkT::kHeight) {
        chunk_.eraseChild(level_child_idx);
      } else {
        const OffsetType chunk_child_idx = level_offset + level_child_idx;
        chunk_.nodeData(chunk_child_idx) = {};
        chunk_.nodeHasAtLeastOneChild(chunk_child_idx) = false;
      }
    }
    child_start_idx <<= kDim;
    child_end_idx <<= kDim;
  }
}

template <typename ChunkT>
typename ChunkedNdtreeNodeRef<ChunkT>::NodePtr
ChunkedNdtreeNodeRef<ChunkT>::getChild(
    NdtreeIndexRelativeChild child_index) const {
  if (!hasAtLeastOneChild()) {
    return {nullptr};
  }
  const OffsetType child_offset = computeChildOffset(child_index);
  if (kMaxOffsetInChunk < child_offset) {
    const OffsetType level_index =
        convert::nodeOffsetToLevelIndex<kDim>(child_offset);
    auto* child_chunk = chunk_.getChild(level_index);
    return {child_chunk, kRootOffset};
  }
  return {&chunk_, child_offset};
}

template <typename ChunkT>
template <typename... DefaultArgs>
ChunkedNdtreeNodeRef<ChunkT> ChunkedNdtreeNodeRef<ChunkT>::getOrAllocateChild(
    NdtreeIndexRelativeChild child_index, DefaultArgs&&... args) const {
  hasAtLeastOneChild() = true;
  const OffsetType child_offset = computeChildOffset(child_index);
  if (kMaxOffsetInChunk < child_offset) {
    const OffsetType level_index =
        convert::nodeOffsetToLevelIndex<kDim>(child_offset);
    auto& child_chunk = chunk_.getOrAllocateChild(
        level_index, std::forward<DefaultArgs>(args)...);
    return {child_chunk, kRootOffset};
  }
  return {chunk_, child_offset};
}

template <typename ChunkT>
IndexElement ChunkedNdtreeNodeRef<ChunkT>::computeDepthIndex() const {
  DCHECK_LE(offset_, kMaxOffsetInChunk);
  return convert::nodeOffsetToDepth<kDim>(offset_);
}

template <typename ChunkT>
typename ChunkedNdtreeNodeRef<ChunkT>::OffsetType
ChunkedNdtreeNodeRef<ChunkT>::computeLevelIndex() const {
  DCHECK_LE(offset_, kMaxOffsetInChunk);
  return convert::nodeOffsetToLevelIndex<kDim>(offset_);
}

template <typename ChunkT>
typename ChunkedNdtreeNodeRef<ChunkT>::OffsetType
ChunkedNdtreeNodeRef<ChunkT>::computeChildOffset(
    NdtreeIndexRelativeChild child_index) const {
  DCHECK_LE(offset_, kMaxOffsetInChunk);
  return convert::nodeOffsetToChildOffset<kDim>(offset_, child_index);
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_ADDRESS_INL_H_
