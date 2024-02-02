#ifndef WAVEMAP_UTILS_QUERY_IMPL_QUERY_ACCELERATOR_INL_H_
#define WAVEMAP_UTILS_QUERY_IMPL_QUERY_ACCELERATOR_INL_H_

#include <limits>
#include <utility>

namespace wavemap {
template <typename BlockDataT, int dim>
void QueryAccelerator<SpatialHash<BlockDataT, dim>>::reset() {
  last_block_index_ =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  last_block_ = nullptr;
}

template <typename BlockDataT, int dim>
BlockDataT* QueryAccelerator<SpatialHash<BlockDataT, dim>>::getBlock(
    const Index<dim>& block_index) {
  if (block_index != last_block_index_) {
    last_block_index_ = block_index;
    last_block_ = spatial_hash_.getBlock(block_index);
  }
  return last_block_;
}

template <typename BlockDataT, int dim>
template <typename... DefaultArgs>
BlockDataT& QueryAccelerator<SpatialHash<BlockDataT, dim>>::getOrAllocateBlock(
    const Index<dim>& block_index, DefaultArgs&&... args) {
  if (block_index != last_block_index_ || !last_block_) {
    last_block_index_ = block_index;
    last_block_ = &spatial_hash_.getOrAllocateBlock(
        block_index, std::forward<DefaultArgs>(args)...);
  }
  return *last_block_;
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
void QueryAccelerator<DenseBlockHash<CellDataT, dim, cells_per_side>>::reset() {
  block_index_ = Index<dim>::Constant(std::numeric_limits<IndexElement>::max());
  block_ = nullptr;
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
const typename QueryAccelerator<
    DenseBlockHash<CellDataT, dim, cells_per_side>>::BlockType*
QueryAccelerator<DenseBlockHash<CellDataT, dim, cells_per_side>>::getBlock(
    const Index<dim>& block_index) {
  if (block_index != block_index_) {
    block_index_ = block_index;
    block_ = dense_block_hash_.getBlock(block_index);
  }
  return block_;
}

template <typename CellDataT, int dim, unsigned int cells_per_side>
const CellDataT*
QueryAccelerator<DenseBlockHash<CellDataT, dim, cells_per_side>>::getValue(
    const Index<dim>& index) {
  const Index<dim> block_index = dense_block_hash_.indexToBlockIndex(index);
  if (const BlockType* block = getBlock(block_index); block) {
    const Index<dim> cell_index = dense_block_hash_.indexToCellIndex(index);
    return &block->at(cell_index);
  }
  return nullptr;
}

template <typename CellDataT, int dim>
void QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::reset() {
  block_index_ = Index<dim>::Constant(std::numeric_limits<IndexElement>::max());
  height = tree_height_;
  morton_code = std::numeric_limits<MortonIndex>::max();

  block_ = nullptr;
  node_stack = std::array<NodeType*, morton::kMaxTreeHeight<dim>>{};
}

template <typename CellDataT, int dim>
typename QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::BlockType*
QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::getBlock(
    const Index<dim>& block_index) {
  if (block_index != block_index_) {
    block_index_ = block_index;
    height = tree_height_;
    block_ = ndtree_block_hash_.getBlock(block_index);
    if (block_) {
      node_stack[tree_height_] = &block_->getRootNode();
    }
  }
  return block_;
}

template <typename CellDataT, int dim>
template <typename... DefaultArgs>
typename QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::BlockType&
QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::getOrAllocateBlock(
    const Index<dim>& block_index, DefaultArgs&&... args) {
  if (block_index != block_index_ || !block_) {
    block_index_ = block_index;
    height = tree_height_;
    block_ = &ndtree_block_hash_.getOrAllocateBlock(
        block_index, std::forward<DefaultArgs>(args)...);
    node_stack[tree_height_] = &block_->getRootNode();
  }
  return *block_;
}

template <typename CellDataT, int dim>
typename QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::NodeType*
QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::getNode(
    const OctreeIndex& index) {
  // Remember previous query indices and compute new ones
  const IndexElement previous_height = height;
  const MortonIndex previous_morton_code = morton_code;
  morton_code = convert::nodeIndexToMorton(index);

  // Fetch the block if needed and return null if it doesn't exist
  if (!getBlock(ndtree_block_hash_.indexToBlockIndex(index))) {
    return nullptr;
  }

  // Compute the last ancestor the current and previous query had in common
  if (height != tree_height_) {
    auto last_common_ancestor = OctreeIndex::computeLastCommonAncestorHeight(
        morton_code, index.height, previous_morton_code, previous_height);
    height = last_common_ancestor;
  }
  DCHECK_LE(height, tree_height_);

  if (height == index.height) {
    DCHECK_NOTNULL(node_stack[height]);
    return node_stack[height];
  }

  // Walk down the tree from height to index.height
  for (; index.height < height;) {
    DCHECK_NOTNULL(node_stack[height]);
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, height);
    // Check if the child is allocated
    NodeType* child = node_stack[height]->getChild(child_index);
    if (!child) {
      return node_stack[height];
    }
    node_stack[--height] = child;
  }

  return node_stack[height];
}

template <typename CellDataT, int dim>
template <typename... DefaultArgs>
typename QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::NodeType&
QueryAccelerator<NdtreeBlockHash<CellDataT, dim>>::getOrAllocateNode(
    const OctreeIndex& index, DefaultArgs&&... args) {
  // Remember previous query indices and compute new ones
  const IndexElement previous_height = height;
  const MortonIndex previous_morton_code = morton_code;
  morton_code = convert::nodeIndexToMorton(index);

  // Make sure the block is allocated
  getOrAllocateBlock(ndtree_block_hash_.indexToBlockIndex(index));

  // Compute the last ancestor the current and previous query had in common
  if (height != tree_height_) {
    auto last_common_ancestor = OctreeIndex::computeLastCommonAncestorHeight(
        morton_code, index.height, previous_morton_code, previous_height);
    height = last_common_ancestor;
  }
  DCHECK_LE(height, tree_height_);

  if (height == index.height) {
    DCHECK_NOTNULL(node_stack[height]);
    return *node_stack[height];
  }

  // Walk down the tree from height to index.height
  for (; index.height < height;) {
    DCHECK_NOTNULL(node_stack[height]);
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, height);
    // Get or allocate the child
    auto& child = node_stack[height]->getOrAllocateChild(
        child_index, std::forward<DefaultArgs>(args)...);
    node_stack[--height] = &child;
  }

  return *node_stack[height];
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_IMPL_QUERY_ACCELERATOR_INL_H_
