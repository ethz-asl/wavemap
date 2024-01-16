#ifndef WAVEMAP_UTILS_QUERY_IMPL_CLASSIFIED_MAP_INL_H_
#define WAVEMAP_UTILS_QUERY_IMPL_CLASSIFIED_MAP_INL_H_

#include <utility>

namespace wavemap {
inline bool ClassifiedMap::has(const OctreeIndex& index,
                               Occupancy::Id occupancy_type) const {
  return has(index, Occupancy::toMask(occupancy_type));
}

inline bool ClassifiedMap::isFully(const OctreeIndex& index,
                                   Occupancy::Id occupancy_type) const {
  return isFully(index, Occupancy::toMask(occupancy_type));
}

inline void ClassifiedMap::forEachLeafMatching(
    Occupancy::Id occupancy_type,
    std::function<void(const OctreeIndex&)> visitor_fn,
    IndexElement termination_height) const {
  forEachLeafMatching(Occupancy::toMask(occupancy_type), std::move(visitor_fn),
                      termination_height);
}

inline Occupancy::Mask ClassifiedMap::NodeData::occupancyMask() const {
  return Occupancy::toMask(has_free.any(), has_occupied.any(),
                           has_unobserved.any());
}

inline Occupancy::Mask ClassifiedMap::NodeData::childOccupancyMask(
    NdtreeIndexRelativeChild child_idx) const {
  return Occupancy::toMask(has_free[child_idx], has_occupied[child_idx],
                           has_unobserved[child_idx]);
}

inline bool ClassifiedMap::hasBlock(const Index3D& block_index) const {
  return block_map_.hasBlock(block_index);
}

inline ClassifiedMap::Block* ClassifiedMap::getBlock(
    const Index3D& block_index) {
  return block_map_.getBlock(block_index);
}

inline const ClassifiedMap::Block* ClassifiedMap::getBlock(
    const Index3D& block_index) const {
  return block_map_.getBlock(block_index);
}

inline ClassifiedMap::Block& ClassifiedMap::getOrAllocateBlock(
    const Index3D& block_index) {
  return block_map_.getOrAllocateBlock(block_index);
}

inline bool ClassifiedMap::hasValue(const OctreeIndex& index) const {
  return block_map_.hasValue(index.computeParentIndex());
}

inline std::optional<Occupancy::Mask> ClassifiedMap::getValue(
    const OctreeIndex& index) const {
  return getValueOrAncestor(index).first;
}

inline std::pair<std::optional<Occupancy::Mask>, ClassifiedMap::HeightType>
ClassifiedMap::getValueOrAncestor(const OctreeIndex& index) const {
  const auto [parent, parent_height] =
      block_map_.getValueOrAncestor(index.computeParentIndex());
  if (parent) {
    const MortonIndex morton = convert::nodeIndexToMorton(index);
    const NdtreeIndexRelativeChild relative_child_idx =
        OctreeIndex::computeRelativeChildIndex(morton, parent_height);
    return {parent->childOccupancyMask(relative_child_idx), parent_height - 1};
  }
  return {std::nullopt, getTreeHeight()};
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_IMPL_CLASSIFIED_MAP_INL_H_
