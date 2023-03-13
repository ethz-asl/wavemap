#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_WAVELET_OCTREE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_WAVELET_OCTREE_INL_H_

namespace wavemap {
inline size_t HashedWaveletOctree::size() const {
  size_t size = 0u;
  for (const auto& [block_index, block] : blocks_) {
    size += block.size();
  }
  return size;
}

inline FloatingPoint HashedWaveletOctree::getCellValue(
    const Index3D& index) const {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  if (hasBlock(block_index)) {
    const auto& block = getBlock(block_index);
    const CellIndex cell_index =
        computeCellIndexFromBlockIndexAndIndex(block_index, index);
    return block.getCellValue(cell_index);
  }
  return 0.f;
}

inline void HashedWaveletOctree::setCellValue(const Index3D& index,
                                              FloatingPoint new_value) {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, index);
  block.setCellValue(cell_index, new_value);
}

inline void HashedWaveletOctree::addToCellValue(const Index3D& index,
                                                FloatingPoint update) {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  auto& block = getOrAllocateBlock(block_index);
  const CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, index);
  block.addToCellValue(cell_index, update);
}

inline bool HashedWaveletOctree::hasBlock(const Index3D& block_index) const {
  return blocks_.count(block_index);
}

inline HashedWaveletOctree::Block& HashedWaveletOctree::getOrAllocateBlock(
    const Index3D& block_index) {
  if (!hasBlock(block_index)) {
    blocks_.try_emplace(block_index, this);
  }
  return blocks_.at(block_index);
}

inline HashedWaveletOctree::Block& HashedWaveletOctree::getBlock(
    const Index3D& block_index) {
  return blocks_.at(block_index);
}

inline const HashedWaveletOctree::Block& HashedWaveletOctree::getBlock(
    const Index3D& block_index) const {
  return blocks_.at(block_index);
}

inline void HashedWaveletOctree::forEachLeaf(
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn) const {
  for (const auto& [block_index, block] : blocks_) {
    block.forEachLeaf(block_index, visitor_fn);
  }
}

inline HashedWaveletOctree::CellIndex
HashedWaveletOctree::computeCellIndexFromBlockIndexAndIndex(
    const HashedWaveletOctree::BlockIndex& block_index, const Index3D& index) {
  const Index3D origin = kCellsPerBlockSide * block_index;
  const Index3D cell_index = index - origin;
  DCHECK((0 <= cell_index.array()).all());
  return convert::indexAndHeightToNodeIndex(cell_index, 0);
}

inline FloatingPoint HashedWaveletOctree::Block::getCellValue(
    const OctreeIndex& index) const {
  const MortonCode morton_code = index.computeMortonCode();
  const NodeType* node = &ndtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (int parent_height = kTreeHeight; index.height < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    value = Transform::backwardSingleChild({value, node->data()}, child_index);
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
  }
  return value;
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_WAVELET_OCTREE_INL_H_
