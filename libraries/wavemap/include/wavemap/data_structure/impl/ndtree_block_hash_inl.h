#ifndef WAVEMAP_DATA_STRUCTURE_IMPL_NDTREE_BLOCK_HASH_INL_H_
#define WAVEMAP_DATA_STRUCTURE_IMPL_NDTREE_BLOCK_HASH_INL_H_

#include <stack>
#include <utility>

namespace wavemap {
template <typename CellDataT, int dim>
inline size_t NdtreeBlockHash<CellDataT, dim>::size() const {
  size_t size = 0u;
  forEachBlock([&size](const Index<dim>& /*block_index*/, const Block& block) {
    size += block.size();
  });
  return size;
}

template <typename CellDataT, int dim>
bool NdtreeBlockHash<CellDataT, dim>::hasBlock(
    const Index<dim>& block_index) const {
  return block_map_.hasBlock(block_index);
}

template <typename CellDataT, int dim>
bool NdtreeBlockHash<CellDataT, dim>::eraseBlock(
    const Index<dim>& block_index) {
  return block_map_.eraseBlock(block_index);
}

template <typename CellDataT, int dim>
template <typename IndexedBlockVisitor>
void NdtreeBlockHash<CellDataT, dim>::eraseBlockIf(
    IndexedBlockVisitor indicator_fn) {
  block_map_.eraseBlockIf(indicator_fn);
}

template <typename CellDataT, int dim>
inline typename NdtreeBlockHash<CellDataT, dim>::Block*
NdtreeBlockHash<CellDataT, dim>::getBlock(const Index<dim>& block_index) {
  return block_map_.getBlock(block_index);
}

template <typename CellDataT, int dim>
inline const typename NdtreeBlockHash<CellDataT, dim>::Block*
NdtreeBlockHash<CellDataT, dim>::getBlock(const Index<dim>& block_index) const {
  return block_map_.getBlock(block_index);
}

template <typename CellDataT, int dim>
inline typename NdtreeBlockHash<CellDataT, dim>::Block&
NdtreeBlockHash<CellDataT, dim>::getOrAllocateBlock(
    const Index<dim>& block_index) {
  return block_map_.getOrAllocateBlock(block_index, max_height_,
                                       default_value_);
}

template <typename CellDataT, int dim>
bool NdtreeBlockHash<CellDataT, dim>::hasNode(
    const NdtreeIndex<dim>& index) const {
  return getNode(index);
}

template <typename CellDataT, int dim>
bool NdtreeBlockHash<CellDataT, dim>::eraseNode(const NdtreeIndex<dim>& index) {
  const Index<dim> block_index = indexToBlockIndex(index);
  if (const Block* block = getBlock(block_index); block) {
    const NdtreeIndex<dim> cell_index = indexToCellIndex(index);
    return block->eraseNode(cell_index);
  }
  return false;
}

template <typename CellDataT, int dim>
typename NdtreeBlockHash<CellDataT, dim>::Node*
NdtreeBlockHash<CellDataT, dim>::getNode(const NdtreeIndex<dim>& index) {
  return const_cast<Node*>(std::as_const(*this).getNode(index));
}

template <typename CellDataT, int dim>
const typename NdtreeBlockHash<CellDataT, dim>::Node*
NdtreeBlockHash<CellDataT, dim>::getNode(const NdtreeIndex<dim>& index) const {
  const Index<dim> block_index = indexToBlockIndex(index);
  if (const Block* block = getBlock(block_index); block) {
    const NdtreeIndex<dim> cell_index = indexToCellIndex(index);
    return block->getNode(cell_index);
  }
  return nullptr;
}

template <typename CellDataT, int dim>
typename NdtreeBlockHash<CellDataT, dim>::Node&
NdtreeBlockHash<CellDataT, dim>::getOrAllocateNode(
    const NdtreeIndex<dim>& index) {
  const Index<dim> block_index = indexToBlockIndex(index);
  Block& block = getOrAllocateBlock(block_index);
  const NdtreeIndex<dim> cell_index = indexToCellIndex(index);
  return block.getOrAllocateNode(cell_index, default_value_);
}

template <typename CellDataT, int dim>
std::pair<typename NdtreeBlockHash<CellDataT, dim>::Node*, IndexElement>
NdtreeBlockHash<CellDataT, dim>::getNodeOrAncestor(
    const NdtreeIndex<dim>& index) {
  return const_cast<Node*>(std::as_const(*this).getNode(index));
}

template <typename CellDataT, int dim>
std::pair<const typename NdtreeBlockHash<CellDataT, dim>::Node*, IndexElement>
NdtreeBlockHash<CellDataT, dim>::getNodeOrAncestor(
    const NdtreeIndex<dim>& index) const {
  const Index<dim> block_index = indexToBlockIndex(index);
  if (const Block* block = getBlock(block_index); block) {
    const NdtreeIndex<dim> cell_index = indexToCellIndex(index);
    return block->getNodeOrAncestor(cell_index);
  }
  return {nullptr, max_height_};
}

template <typename CellDataT, int dim>
bool NdtreeBlockHash<CellDataT, dim>::hasValue(
    const NdtreeIndex<dim>& index) const {
  return hasNode(index);
}

template <typename CellDataT, int dim>
CellDataT* NdtreeBlockHash<CellDataT, dim>::getValue(
    const NdtreeIndex<dim>& index) {
  return const_cast<CellDataT*>(std::as_const(*this).getValue(index));
}

template <typename CellDataT, int dim>
const CellDataT* NdtreeBlockHash<CellDataT, dim>::getValue(
    const NdtreeIndex<dim>& index) const {
  if (const Node* node = getNode(index); node) {
    return &node->data();
  }
  return nullptr;
}

template <typename CellDataT, int dim>
CellDataT& NdtreeBlockHash<CellDataT, dim>::getOrAllocateValue(
    const NdtreeIndex<dim>& index) {
  return getOrAllocateNode(index).data();
}

template <typename CellDataT, int dim>
std::pair<CellDataT*, IndexElement>
NdtreeBlockHash<CellDataT, dim>::getValueOrAncestor(
    const NdtreeIndex<dim>& index) {
  auto rv = getNodeOrAncestor(index);
  return {rv.first ? &rv.first->data() : nullptr, rv.second};
}

template <typename CellDataT, int dim>
std::pair<const CellDataT*, IndexElement>
NdtreeBlockHash<CellDataT, dim>::getValueOrAncestor(
    const NdtreeIndex<dim>& index) const {
  auto rv = getNodeOrAncestor(index);
  return {rv.first ? &rv.first->data() : nullptr, rv.second};
}

template <typename CellDataT, int dim>
const CellDataT& NdtreeBlockHash<CellDataT, dim>::getValueOrDefault(
    const NdtreeIndex<dim>& index) const {
  if (const CellDataT* value = getValue(index); value) {
    return *value;
  }
  return default_value_;
}

template <typename CellDataT, int dim>
bool NdtreeBlockHash<CellDataT, dim>::equalsDefaultValue(
    const CellDataT& value) const {
  return value == default_value_;
}

template <typename CellDataT, int dim>
template <typename IndexedBlockVisitor>
void NdtreeBlockHash<CellDataT, dim>::forEachBlock(
    IndexedBlockVisitor visitor_fn) {
  block_map_.forEachBlock(visitor_fn);
}

template <typename CellDataT, int dim>
template <typename IndexedBlockVisitor>
void NdtreeBlockHash<CellDataT, dim>::forEachBlock(
    IndexedBlockVisitor visitor_fn) const {
  block_map_.forEachBlock(visitor_fn);
}

template <typename CellDataT, int dim>
template <typename IndexedLeafVisitorFunction>
void NdtreeBlockHash<CellDataT, dim>::forEachLeaf(
    IndexedLeafVisitorFunction visitor_fn) {
  struct StackElement {
    const OctreeIndex node_index;
    Node& node;
  };

  block_map_.forEachBlock([&visitor_fn, tree_height = max_height_](
                              const Index<dim>& block_index, Block& block) {
    std::stack<StackElement> stack;
    stack.emplace(StackElement{OctreeIndex{tree_height, block_index},
                               block.getRootNode()});
    while (!stack.empty()) {
      const OctreeIndex node_index = stack.top().node_index;
      Node& node = stack.top().node;
      stack.pop();

      for (NdtreeIndexRelativeChild child_idx = 0;
           child_idx < OctreeIndex::kNumChildren; ++child_idx) {
        const OctreeIndex child_node_index =
            node_index.computeChildIndex(child_idx);
        if (node.hasChild(child_idx)) {
          Node& child_node = *node.getChild(child_idx);
          stack.emplace(StackElement{child_node_index, child_node});
        } else {
          visitor_fn(child_node_index, node.data());
        }
      }
    }
  });
}

template <typename CellDataT, int dim>
template <typename IndexedLeafVisitorFunction>
void NdtreeBlockHash<CellDataT, dim>::forEachLeaf(
    IndexedLeafVisitorFunction visitor_fn) const {
  struct StackElement {
    const OctreeIndex node_index;
    const Node& node;
  };

  block_map_.forEachBlock([&visitor_fn, tree_height = max_height_](
                              const Index<dim>& block_index,
                              const Block& block) {
    std::stack<StackElement> stack;
    stack.emplace(StackElement{OctreeIndex{tree_height, block_index},
                               block.getRootNode()});
    while (!stack.empty()) {
      const OctreeIndex node_index = stack.top().node_index;
      const Node& node = stack.top().node;
      stack.pop();

      if (node.hasAtLeastOneChild()) {
        for (NdtreeIndexRelativeChild child_idx = 0;
             child_idx < OctreeIndex::kNumChildren; ++child_idx) {
          const OctreeIndex child_node_index =
              node_index.computeChildIndex(child_idx);
          if (const Node* child_node = node.getChild(child_idx); child_node) {
            stack.emplace(StackElement{child_node_index, *child_node});
          } else {
            visitor_fn(child_node_index, node.data());
          }
        }
      } else {
        visitor_fn(node_index, node.data());
      }
    }
  });
}

template <typename CellDataT, int dim>
inline Index<dim> NdtreeBlockHash<CellDataT, dim>::indexToBlockIndex(
    const NdtreeIndex<dim>& index) const {
  DCHECK_GE(index.height, 0);
  DCHECK_LE(index.height, max_height_);
  const IndexElement depth = max_height_ - index.height;
  return int_math::div_exp2_floor(index.position, depth);
}

template <typename CellDataT, int dim>
inline NdtreeIndex<dim> NdtreeBlockHash<CellDataT, dim>::indexToCellIndex(
    const NdtreeIndex<dim>& index) const {
  DCHECK_GE(index.height, 0);
  DCHECK_LE(index.height, max_height_);
  const IndexElement depth = max_height_ - index.height;
  return {index.height,
          int_math::div_exp2_floor_remainder(index.position, depth)};
}

template <typename CellDataT, int dim>
inline NdtreeIndex<dim>
NdtreeBlockHash<CellDataT, dim>::cellAndBlockIndexToIndex(
    const Index<dim>& block_index, const NdtreeIndex<dim>& cell_index) const {
  DCHECK_GE(cell_index.height, 0);
  DCHECK_LE(cell_index.height, max_height_);
  const IndexElement depth = max_height_ - cell_index.height;
  const IndexElement cells_per_side_at_height = int_math::exp2(depth);
  return {cell_index.height,
          cell_index.position + cells_per_side_at_height * block_index};
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_IMPL_NDTREE_BLOCK_HASH_INL_H_
