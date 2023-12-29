#ifndef WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_ADDRESS_H_
#define WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_ADDRESS_H_

#include <optional>

#include "wavemap/common.h"
#include "wavemap/data_structure/chunked_ndtree/chunked_ndtree_chunk.h"
#include "wavemap/utils/data/comparisons.h"

namespace wavemap {
template <typename ChunkType>
class ChunkedNdtreeNodeRef;

template <typename ChunkType>
class ChunkedNdtreeNodePtr {
 public:
  using NodeRef = ChunkedNdtreeNodeRef<ChunkType>;
  using NodeConstRef = ChunkedNdtreeNodeRef<const ChunkType>;

  // Constructors
  ChunkedNdtreeNodePtr() = default;
  ChunkedNdtreeNodePtr(ChunkType* chunk, IndexElement relative_node_depth,
                       LinearIndex level_traversal_distance);

  // Copy/move constructors
  ChunkedNdtreeNodePtr(const ChunkedNdtreeNodePtr& other);
  ChunkedNdtreeNodePtr(ChunkedNdtreeNodePtr&& other) noexcept;

  // Copy/move assignment operators
  ChunkedNdtreeNodePtr& operator=(const ChunkedNdtreeNodePtr& other);
  ChunkedNdtreeNodePtr& operator=(ChunkedNdtreeNodePtr&& other) noexcept;

  // Emulate pointer semantics
  void reset() { node_.reset(); }
  operator bool() const { return node_.has_value(); }  // NOLINT
  NodeRef operator*() { return node_.value(); }
  NodeConstRef operator*() const { return node_.value(); }
  NodeRef* operator->() { return node_.operator->(); }
  const NodeConstRef* operator->() const { return node_.operator->(); }

 private:
  std::optional<NodeRef> node_;
};

template <typename ChunkType>
class ChunkedNdtreeNodeRef {
 public:
  using NodeRef = ChunkedNdtreeNodeRef<ChunkType>;
  using NodeConstRef = ChunkedNdtreeNodeRef<const ChunkType>;
  using NodePtr = ChunkedNdtreeNodePtr<ChunkType>;
  using NodeConstPtr = ChunkedNdtreeNodePtr<const ChunkType>;

  static constexpr int kDim = ChunkType::kDim;
  static constexpr int kNumChildren = NdtreeIndex<kDim>::kNumChildren;
  using NodeDataType = typename ChunkType::DataType;

  ChunkedNdtreeNodeRef(ChunkType& chunk, IndexElement relative_node_depth,
                       LinearIndex level_traversal_distance);

  // Copy/move constructor
  ChunkedNdtreeNodeRef(const ChunkedNdtreeNodeRef& other);
  ChunkedNdtreeNodeRef(ChunkedNdtreeNodeRef&& other) noexcept;

  // Copy/move assignment operators are deleted (to behave like a ref)
  ChunkedNdtreeNodeRef& operator=(const ChunkedNdtreeNodeRef&) = delete;
  ChunkedNdtreeNodeRef& operator=(ChunkedNdtreeNodeRef&&) = delete;

  // Conversion to const ref
  operator NodeConstRef();

  // Conversion to pointer
  NodePtr operator&();             // NOLINT
  NodeConstPtr operator&() const;  // NOLINT

  // Regular node methods
  bool empty() const;

  bool hasNonzeroData() const;
  bool hasNonzeroData(FloatingPoint threshold) const;
  auto& data();
  const auto& data() const;

  typename ChunkType::BitRef hasAtLeastOneChild();
  bool hasAtLeastOneChild() const;

  bool hasChild(NdtreeIndexRelativeChild child_index) const;

  NodePtr getChild(NdtreeIndexRelativeChild child_index);
  NodeConstPtr getChild(NdtreeIndexRelativeChild child_index) const;
  template <typename... DefaultArgs>
  NodeRef getOrAllocateChild(NdtreeIndexRelativeChild child_index,
                             DefaultArgs&&... args) const;

 private:
  ChunkType& chunk_;
  const IndexElement relative_node_depth_ = 0;
  const LinearIndex level_traversal_distance_ = 0u;

  LinearIndex computeRelativeNodeIndex() const;
  LinearIndex computeChildLevelTraversalDistance(
      NdtreeIndexRelativeChild child_index) const;

  template <typename T>
  friend class ChunkedNdtreeNodePtr;
};
}  // namespace wavemap

#include "wavemap/data_structure/chunked_ndtree/impl/chunked_ndtree_node_address_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_CHUNKED_NDTREE_CHUNKED_NDTREE_NODE_ADDRESS_H_
