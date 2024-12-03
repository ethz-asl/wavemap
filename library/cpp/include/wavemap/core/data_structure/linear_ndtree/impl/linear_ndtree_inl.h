#ifndef WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_IMPL_LINEAR_NDTREE_INL_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_IMPL_LINEAR_NDTREE_INL_H_

#include <utility>

namespace wavemap {
template <typename NodeDataT, int dim>
LinearNdtree<NodeDataT, dim>::LinearNdtree(HeightType max_height)
    : max_height_(max_height) {
  CHECK_LE(max_height_, kMaxHeight);
}

template <typename NodeDataT, int dim>
void LinearNdtree<NodeDataT, dim>::clear() {
  first_child_offset_.clear();
  allocated_child_mask_.clear();
  node_data_.clear();
}

template <typename NodeDataT, int dim>
size_t LinearNdtree<NodeDataT, dim>::getMemoryUsage() const {
  return first_child_offset_.size() * sizeof(NodeOffsetType) +
         allocated_child_mask_.size() * sizeof(ChildAllocationMaskType) +
         node_data_.size() * sizeof(NodeDataType);
}

template <typename NodeDataT, int dim>
typename LinearNdtree<NodeDataT, dim>::NodePtrType
LinearNdtree<NodeDataT, dim>::getNode(const IndexType& index) {
  NodePtrType node = &getRootNode();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; node && index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    node = node->getChild(child_index);
  }
  return node;
}

template <typename NodeDataT, int dim>
typename LinearNdtree<NodeDataT, dim>::NodeConstPtrType
LinearNdtree<NodeDataT, dim>::getNode(const IndexType& index) const {
  NodeConstPtrType node = &getRootNode();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; node && index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    node = node->getChild(child_index);
  }
  return node;
}

template <typename NodeDataT, int dim>
std::pair<typename LinearNdtree<NodeDataT, dim>::NodePtrType,
          typename LinearNdtree<NodeDataT, dim>::HeightType>
LinearNdtree<NodeDataT, dim>::getNodeOrAncestor(const IndexType& index) {
  NodePtrType node = &getRootNode();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    // Check if the child is allocated
    NodePtrType child = node->getChild(child_index);
    if (!child) {
      return {node, node_height};
    }
    node = child;
  }
  return {node, index.height};
}

template <typename NodeDataT, int dim>
std::pair<typename LinearNdtree<NodeDataT, dim>::NodeConstPtrType,
          typename LinearNdtree<NodeDataT, dim>::HeightType>
LinearNdtree<NodeDataT, dim>::getNodeOrAncestor(const IndexType& index) const {
  NodeConstPtrType node = &getRootNode();
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  for (int node_height = max_height_; index.height < node_height;
       --node_height) {
    const NdtreeIndexRelativeChild child_index =
        NdtreeIndex<dim>::computeRelativeChildIndex(morton_code, node_height);
    // Check if the child is allocated
    NodeConstPtrType child = node->getChild(child_index);
    if (!child) {
      return {node, node_height};
    }
    node = child;
  }
  return {node, index.height};
}

template <typename NodeDataT, int dim>
bool LinearNdtree<NodeDataT, dim>::nodeHasNonzeroData(
    NodeOffsetType relative_node_index) const {
  DCHECK_LT(relative_node_index, node_data_.size());
  return data::is_nonzero(node_data_[relative_node_index]);
}

template <typename NodeDataT, int dim>
bool LinearNdtree<NodeDataT, dim>::nodeHasNonzeroData(
    NodeOffsetType relative_node_index, FloatingPoint threshold) const {
  DCHECK_LT(relative_node_index, node_data_.size());
  return data::is_nonzero(node_data_[relative_node_index], threshold);
}

template <typename NodeDataT, int dim>
NodeDataT& LinearNdtree<NodeDataT, dim>::nodeData(
    NodeOffsetType relative_node_index) {
  DCHECK_LT(relative_node_index, node_data_.size());
  return node_data_[relative_node_index];
}

template <typename NodeDataT, int dim>
const NodeDataT& LinearNdtree<NodeDataT, dim>::nodeData(
    NodeOffsetType relative_node_index) const {
  DCHECK_LT(relative_node_index, node_data_.size());
  return node_data_[relative_node_index];
}

template <typename NodeDataT, int dim>
bool LinearNdtree<NodeDataT, dim>::nodeHasAtLeastOneChild(
    NodeOffsetType relative_node_index) const {
  DCHECK_LT(relative_node_index, allocated_child_mask_.size());
  return allocated_child_mask_[relative_node_index];
}

template <typename NodeDataT, int dim>
bool LinearNdtree<NodeDataT, dim>::nodeHasChild(
    NodeOffsetType relative_node_index,
    NdtreeIndexRelativeChild child_index) const {
  DCHECK_LT(relative_node_index, allocated_child_mask_.size());
  const ChildAllocationMaskType node_child_mask =
      allocated_child_mask_[relative_node_index];
  return bit_ops::is_bit_set(node_child_mask, child_index);
}

template <typename NodeDataT, int dim>
std::optional<typename LinearNdtree<NodeDataT, dim>::NodeOffsetType>
LinearNdtree<NodeDataT, dim>::getChildOffset(
    NodeOffsetType relative_node_index,
    NdtreeIndexRelativeChild child_index) const {
  DCHECK_LT(relative_node_index, first_child_offset_.size());
  DCHECK_LT(relative_node_index, allocated_child_mask_.size());
  const NodeOffsetType node_first_child_offset =
      first_child_offset_[relative_node_index];
  const ChildAllocationMaskType node_child_mask =
      allocated_child_mask_[relative_node_index];
  const ChildAllocationMaskType relative_child_offset =
      bit_ops::popcount(node_child_mask & ((1 << child_index) - 1));
  return node_first_child_offset + relative_child_offset;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_LINEAR_NDTREE_IMPL_LINEAR_NDTREE_INL_H_
