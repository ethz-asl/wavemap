#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_WAVELET_OCTREE_BLOCK_INL_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_WAVELET_OCTREE_BLOCK_INL_H_

namespace wavemap {
inline FloatingPoint HashedWaveletOctreeBlock::getTimeSinceLastUpdated() const {
  return (std::chrono::duration<FloatingPoint>(Clock::now() -
                                               last_updated_stamp_))
      .count();
}

inline FloatingPoint HashedWaveletOctreeBlock::getCellValue(
    const OctreeIndex& index) const {
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  const NodeType* node = &ndtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (int parent_height = tree_height_; index.height < parent_height;
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

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_IMPL_HASHED_WAVELET_OCTREE_BLOCK_INL_H_
