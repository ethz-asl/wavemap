#ifndef WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_LINEAR_NDTREE_NODE_ADDRESS_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_LINEAR_NDTREE_NODE_ADDRESS_H_

namespace wavemap {
template <typename LinearNdtreeT>
class LinearNdtreeNodeRef;

template <typename LinearNdtreeT>
class LinearNdtreeNodePtr {
 public:
  using NodeOffsetType = typename LinearNdtreeT::NodeOffsetType;
  using NodeRef = LinearNdtreeNodeRef<LinearNdtreeT>;

  // Constructors
  LinearNdtreeNodePtr() = default;
  LinearNdtreeNodePtr(LinearNdtreeT* tree);  // NOLINT
  LinearNdtreeNodePtr(LinearNdtreeT* tree, NodeOffsetType offset);

  // Copy/move constructors
  LinearNdtreeNodePtr(const LinearNdtreeNodePtr& other);
  LinearNdtreeNodePtr(LinearNdtreeNodePtr&& other) noexcept;

  // Copy/move assignment operators
  LinearNdtreeNodePtr& operator=(const LinearNdtreeNodePtr& other);
  LinearNdtreeNodePtr& operator=(LinearNdtreeNodePtr&& other) noexcept;

  // Emulate pointer semantics
  void reset() { node_.reset(); }
  NodeRef operator*() const { return node_.value(); }
  NodeRef* operator->() { return node_.operator->(); }
  const NodeRef* operator->() const { return node_.operator->(); }

  // Emulate null check semantics
  operator bool() const { return node_.has_value(); }  // NOLINT
  bool operator==(std::nullptr_t) noexcept { return !node_.has_value(); }

  // Emulate comparisons
  bool operator==(const LinearNdtreeNodePtr& other) const;
  bool operator!=(const LinearNdtreeNodePtr& other) const;

 private:
  std::optional<NodeRef> node_{};
};

template <typename LinearNdtreeT>
class LinearNdtreeNodeRef {
 public:
  using NodeOffsetType = typename LinearNdtreeT::NodeOffsetType;
  using NodeRef = LinearNdtreeNodeRef;
  using NodePtr = LinearNdtreeNodePtr<LinearNdtreeT>;
  static constexpr int kDim = LinearNdtreeT::kDim;
  static constexpr NodeOffsetType kRootOffset = 0u;

  LinearNdtreeNodeRef() = delete;
  LinearNdtreeNodeRef(LinearNdtreeT& tree,  // NOLINT
                      NodeOffsetType offset = kRootOffset);

  // Copy/move constructor
  LinearNdtreeNodeRef(const LinearNdtreeNodeRef& other);
  LinearNdtreeNodeRef(LinearNdtreeNodeRef&& other) noexcept;

  // Copy/move assignment operators are deleted (to behave like a ref)
  LinearNdtreeNodeRef& operator=(const LinearNdtreeNodeRef&) = delete;
  LinearNdtreeNodeRef& operator=(LinearNdtreeNodeRef&&) = delete;

  // Conversion of non-const ref to const ref
  operator LinearNdtreeNodeRef<const LinearNdtreeT>() const;  // NOLINT

  // Conversion to pointer
  NodePtr operator&() const;  // NOLINT

  // Regular node methods
  bool empty() const;

  bool hasNonzeroData() const;
  bool hasNonzeroData(FloatingPoint threshold) const;
  auto& data() const;

  bool hasAtLeastOneChild() const;
  bool hasChild(NdtreeIndexRelativeChild child_index) const;
  NodePtr getChild(NdtreeIndexRelativeChild child_index) const;

 private:
  LinearNdtreeT& tree_;
  const NodeOffsetType offset_;

  std::optional<NodeOffsetType> getChildOffset(
      NdtreeIndexRelativeChild child_index) const;

  template <typename T>
  friend class LinearNdtreeNodePtr;
};
}  // namespace wavemap

#include "wavemap/core/data_structure/linear_ndtree/impl/linear_ndtree_node_address_inl.h"

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_LINEAR_NDTREE_NODE_ADDRESS_H_
