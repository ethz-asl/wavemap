#ifndef WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_IMPL_LINEAR_NDTREE_NODE_ADDRESS_INL_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_IMPL_LINEAR_NDTREE_NODE_ADDRESS_INL_H_

#include <utility>

namespace wavemap {
template <typename LinearNdtreeT>
LinearNdtreeNodePtr<LinearNdtreeT>::LinearNdtreeNodePtr(LinearNdtreeT* tree,
                                                        NodeOffsetType offset)
    : node_(tree ? std::make_optional<NodeRef>(*tree, offset) : std::nullopt) {}

template <typename LinearNdtreeT>
LinearNdtreeNodePtr<LinearNdtreeT>::LinearNdtreeNodePtr(
    const LinearNdtreeNodePtr& other)
    : node_(other ? std::make_optional<NodeRef>(other.node_.value())
                  : std::nullopt) {}

template <typename LinearNdtreeT>
LinearNdtreeNodePtr<LinearNdtreeT>::LinearNdtreeNodePtr(
    LinearNdtreeNodePtr&& other) noexcept
    : node_(other ? std::make_optional<NodeRef>(std::move(other.node_.value()))
                  : std::nullopt) {}

template <typename LinearNdtreeT>
LinearNdtreeNodePtr<LinearNdtreeT>&
LinearNdtreeNodePtr<LinearNdtreeT>::operator=(
    const LinearNdtreeNodePtr& other) {
  if (other) {
    node_.emplace(other.node_.value());
  } else {
    reset();
  }
  return *this;
}

template <typename LinearNdtreeT>
LinearNdtreeNodePtr<LinearNdtreeT>&
LinearNdtreeNodePtr<LinearNdtreeT>::operator=(
    LinearNdtreeNodePtr&& other) noexcept {
  if (other) {
    node_.emplace(std::move(other.node_.value()));
  } else {
    reset();
  }
  return *this;
}

template <typename LinearNdtreeT>
LinearNdtreeNodeRef<LinearNdtreeT>::LinearNdtreeNodeRef(LinearNdtreeT& tree,
                                                        NodeOffsetType offset)
    : tree_(tree), offset_(offset) {}

template <typename LinearNdtreeT>
LinearNdtreeNodeRef<LinearNdtreeT>::LinearNdtreeNodeRef(
    const LinearNdtreeNodeRef& other)
    : LinearNdtreeNodeRef(other.tree_, other.offset_) {}

template <typename LinearNdtreeT>
LinearNdtreeNodeRef<LinearNdtreeT>::LinearNdtreeNodeRef(
    LinearNdtreeNodeRef&& other) noexcept
    : tree_(other.tree_), offset_(other.offset_) {}

template <typename LinearNdtreeT>
LinearNdtreeNodeRef<LinearNdtreeT>::operator LinearNdtreeNodeRef<
    const LinearNdtreeT>() const {
  return {tree_, offset_};
}

template <typename LinearNdtreeT>
typename LinearNdtreeNodeRef<LinearNdtreeT>::NodePtr
LinearNdtreeNodeRef<LinearNdtreeT>::operator&() const {  // NOLINT
  return {&tree_, offset_};
}

template <typename LinearNdtreeT>
bool LinearNdtreeNodeRef<LinearNdtreeT>::empty() const {
  return !hasAtLeastOneChild() && !hasNonzeroData();
}

template <typename LinearNdtreeT>
bool LinearNdtreeNodeRef<LinearNdtreeT>::hasNonzeroData() const {
  return tree_.nodeHasNonzeroData(offset_);
}

template <typename LinearNdtreeT>
bool LinearNdtreeNodeRef<LinearNdtreeT>::hasNonzeroData(
    FloatingPoint threshold) const {
  return tree_.nodeHasNonzeroData(offset_, threshold);
}

template <typename LinearNdtreeT>
auto& LinearNdtreeNodeRef<LinearNdtreeT>::data() const {
  return tree_.nodeData(offset_);
}

template <typename LinearNdtreeT>
auto LinearNdtreeNodeRef<LinearNdtreeT>::hasAtLeastOneChild() const {
  return tree_.nodeHasAtLeastOneChild(offset_);
}

template <typename LinearNdtreeT>
bool LinearNdtreeNodeRef<LinearNdtreeT>::hasChild(
    NdtreeIndexRelativeChild child_index) const {
  return tree_.nodeHasChild(offset_, child_index);
}

template <typename LinearNdtreeT>
typename LinearNdtreeNodeRef<LinearNdtreeT>::NodePtr
LinearNdtreeNodeRef<LinearNdtreeT>::getChild(
    NdtreeIndexRelativeChild child_index) const {
  if (const auto child_offset = getChildOffset(child_index); child_offset) {
    return {&tree_, child_offset.value()};
  }
  return {nullptr};
}

template <typename LinearNdtreeT>
std::optional<typename LinearNdtreeNodeRef<LinearNdtreeT>::NodeOffsetType>
LinearNdtreeNodeRef<LinearNdtreeT>::getChildOffset(
    NdtreeIndexRelativeChild child_index) const {
  return tree_.getChildOffset(offset_, child_index);
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_IMPL_LINEAR_NDTREE_NODE_ADDRESS_INL_H_
