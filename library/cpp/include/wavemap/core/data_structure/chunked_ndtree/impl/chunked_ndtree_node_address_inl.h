#ifndef WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_ADDRESS_INL_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_ADDRESS_INL_H_

#include <utility>

namespace wavemap {
template <typename ChunkType>
ChunkedNdtreeNodePtr<ChunkType>::ChunkedNdtreeNodePtr(ChunkType* chunk)
    : node_(chunk ? std::make_optional<NodeRef>(*chunk) : std::nullopt) {}

template <typename ChunkType>
ChunkedNdtreeNodePtr<ChunkType>::ChunkedNdtreeNodePtr(ChunkType* chunk,
                                                      KeyType key)
    : node_(chunk ? std::make_optional<NodeRef>(*chunk, key) : std::nullopt) {}

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
    const ChunkedNdtreeNodeRef& other)
    : ChunkedNdtreeNodeRef(other.chunk_, other.key_) {}

template <typename ChunkType>
ChunkedNdtreeNodeRef<ChunkType>::ChunkedNdtreeNodeRef(
    ChunkedNdtreeNodeRef&& other) noexcept
    : chunk_(other.chunk_), key_(other.key_) {}

template <typename ChunkType>
ChunkedNdtreeNodeRef<ChunkType>::operator ChunkedNdtreeNodeRef<
    const ChunkType>() const {
  return {chunk_, key_};
}

template <typename ChunkType>
typename ChunkedNdtreeNodeRef<ChunkType>::NodePtr
ChunkedNdtreeNodeRef<ChunkType>::operator&() const {  // NOLINT
  return {&chunk_, key_};
}

template <typename ChunkType>
bool ChunkedNdtreeNodeRef<ChunkType>::empty() const {
  return !hasAtLeastOneChild() && !hasNonzeroData();
}

template <typename ChunkType>
bool ChunkedNdtreeNodeRef<ChunkType>::hasNonzeroData() const {
  return chunk_.nodeHasNonzeroData(computeIndexInChunk());
}

template <typename ChunkType>
bool ChunkedNdtreeNodeRef<ChunkType>::hasNonzeroData(
    FloatingPoint threshold) const {
  return chunk_.nodeHasNonzeroData(computeIndexInChunk(), threshold);
}

template <typename ChunkType>
auto& ChunkedNdtreeNodeRef<ChunkType>::data() const {
  return chunk_.nodeData(computeIndexInChunk());
}

template <typename ChunkType>
auto ChunkedNdtreeNodeRef<ChunkType>::hasAtLeastOneChild() const {
  return chunk_.nodeHasAtLeastOneChild(computeIndexInChunk());
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
  const KeyType child_key = computeChildKey(child_index);
  LinearIndex child_start_idx = computeIndexInLevel(child_key);
  LinearIndex child_end_idx = child_start_idx + 1;
  for (int child_depth = computeDepthIndex(child_key);
       child_depth <= ChunkType::kHeight; ++child_depth) {
    const LinearIndex level_offset =
        tree_math::perfect_tree::num_total_nodes_fast<kDim>(child_depth);
    for (LinearIndex level_child_idx = child_start_idx;
         level_child_idx < child_end_idx; ++level_child_idx) {
      if (child_depth == ChunkType::kHeight) {
        chunk_.eraseChild(level_child_idx);
      } else {
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
    return {nullptr};
  }
  const KeyType child_key = computeChildKey(child_index);
  if (kMaxKeyInChunk < child_key) {
    const LinearIndex level_index = computeIndexInLevel(child_key);
    auto* child_chunk = chunk_.getChild(level_index);
    return {child_chunk, kRootKey};
  }
  return {&chunk_, child_key};
}

template <typename ChunkType>
template <typename... DefaultArgs>
ChunkedNdtreeNodeRef<ChunkType>
ChunkedNdtreeNodeRef<ChunkType>::getOrAllocateChild(
    NdtreeIndexRelativeChild child_index, DefaultArgs&&... args) const {
  hasAtLeastOneChild() = true;
  const KeyType child_key = computeChildKey(child_index);
  if (kMaxKeyInChunk < child_key) {
    const LinearIndex level_index = computeIndexInLevel(child_key);
    auto& child_chunk = chunk_.getOrAllocateChild(
        level_index, std::forward<DefaultArgs>(args)...);
    return {child_chunk, kRootKey};
  }
  return {chunk_, child_key};
}

template <typename ChunkType>
IndexElement ChunkedNdtreeNodeRef<ChunkType>::computeDepthIndex(KeyType key) {
  DCHECK_GE(key, kRootKey);
  return int_math::log2_floor(key) / kDim;
}

template <typename ChunkType>
LinearIndex ChunkedNdtreeNodeRef<ChunkType>::computeIndexInLevel(KeyType key) {
  DCHECK_GE(key, kRootKey);
  const int depth_idx = computeDepthIndex(key);
  const LinearIndex level_mask = (1 << (kDim * depth_idx)) - 1;
  return key & level_mask;
}

template <typename ChunkType>
LinearIndex ChunkedNdtreeNodeRef<ChunkType>::computeIndexInChunk(KeyType key) {
  DCHECK_GE(key, kRootKey);
  const int depth_idx = computeDepthIndex(key);
  const LinearIndex prior_levels_size =
      tree_math::perfect_tree::num_total_nodes_fast<kDim>(depth_idx);
  const LinearIndex level_mask = (1 << (kDim * depth_idx)) - 1;
  const LinearIndex level_index = key & level_mask;
  return prior_levels_size + level_index;
}

template <typename ChunkType>
typename ChunkedNdtreeNodeRef<ChunkType>::KeyType
ChunkedNdtreeNodeRef<ChunkType>::computeChildKey(
    KeyType key, NdtreeIndexRelativeChild child_index) {
  DCHECK_GE(key, kRootKey);
  DCHECK_GE(child_index, 0);
  DCHECK_LT(child_index, 1 << kDim);
  return (key << kDim) | child_index;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_CHUNKED_NDTREE_IMPL_CHUNKED_NDTREE_NODE_ADDRESS_INL_H_
