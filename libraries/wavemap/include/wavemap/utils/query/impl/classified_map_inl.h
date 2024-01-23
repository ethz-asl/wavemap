#ifndef WAVEMAP_UTILS_QUERY_IMPL_CLASSIFIED_MAP_INL_H_
#define WAVEMAP_UTILS_QUERY_IMPL_CLASSIFIED_MAP_INL_H_

#include <limits>
#include <utility>

namespace wavemap {
inline void ClassifiedMap::NodeData::setFree(NdtreeIndexRelativeChild child_idx,
                                             bool value) {
  has_free = bit_ops::set_bit(has_free, child_idx, value);
}

inline void ClassifiedMap::NodeData::setOccupied(
    NdtreeIndexRelativeChild child_idx, bool value) {
  has_occupied = bit_ops::set_bit(has_occupied, child_idx, value);
}

inline void ClassifiedMap::NodeData::setUnobserved(
    NdtreeIndexRelativeChild child_idx, bool value) {
  has_unobserved = bit_ops::set_bit(has_unobserved, child_idx, value);
}

inline bool ClassifiedMap::NodeData::isFree(
    NdtreeIndexRelativeChild child_idx) const {
  return bit_ops::is_bit_set(has_free, child_idx);
}

inline bool ClassifiedMap::NodeData::isOccupied(
    NdtreeIndexRelativeChild child_idx) const {
  return bit_ops::is_bit_set(has_occupied, child_idx);
}

inline bool ClassifiedMap::NodeData::isUnobserved(
    NdtreeIndexRelativeChild child_idx) const {
  return bit_ops::is_bit_set(has_unobserved, child_idx);
}

inline bool ClassifiedMap::NodeData::hasAnyFree() const {
  return has_free != static_cast<uint8_t>(0);
}

inline bool ClassifiedMap::NodeData::hasAnyOccupied() const {
  return has_occupied != static_cast<uint8_t>(0);
}

inline bool ClassifiedMap::NodeData::hasAnyUnobserved() const {
  return has_unobserved != static_cast<uint8_t>(0);
}

inline bool ClassifiedMap::has(const OctreeIndex& index,
                               Occupancy::Id occupancy_type) const {
  return has(index, Occupancy::toMask(occupancy_type));
}

inline bool ClassifiedMap::has(const OctreeIndex& index,
                               Occupancy::Mask occupancy_mask) const {
  return query_cache_.has(index, occupancy_mask, block_map_);
}

inline bool ClassifiedMap::isFully(const OctreeIndex& index,
                                   Occupancy::Id occupancy_type) const {
  return isFully(index, Occupancy::toMask(occupancy_type));
}

inline bool ClassifiedMap::isFully(const OctreeIndex& index,
                                   Occupancy::Mask occupancy_mask) const {
  return query_cache_.isFully(index, occupancy_mask, block_map_);
}

inline void ClassifiedMap::forEachLeafMatching(
    Occupancy::Id occupancy_type,
    std::function<void(const OctreeIndex&)> visitor_fn,
    IndexElement termination_height) const {
  forEachLeafMatching(Occupancy::toMask(occupancy_type), std::move(visitor_fn),
                      termination_height);
}

inline Occupancy::Mask ClassifiedMap::NodeData::occupancyMask() const {
  return Occupancy::toMask(hasAnyFree(), hasAnyOccupied(), hasAnyUnobserved());
}

inline Occupancy::Mask ClassifiedMap::NodeData::childOccupancyMask(
    NdtreeIndexRelativeChild child_idx) const {
  return Occupancy::toMask(isFree(child_idx), isOccupied(child_idx),
                           isUnobserved(child_idx));
}

inline bool ClassifiedMap::hasBlock(const Index3D& block_index) const {
  return getBlock(block_index);
}

inline const ClassifiedMap::Block* ClassifiedMap::getBlock(
    const Index3D& block_index) const {
  return query_cache_.getBlock(block_index, block_map_);
}

inline std::pair<const ClassifiedMap::Node*, ClassifiedMap::HeightType>
ClassifiedMap::getNodeOrAncestor(const OctreeIndex& index) const {
  return query_cache_.getNodeOrAncestor(index, block_map_);
}

inline const ClassifiedMap::Node* ClassifiedMap::getNode(
    const OctreeIndex& index) const {
  return getNodeOrAncestor(index).first;
}

inline bool ClassifiedMap::hasValue(const OctreeIndex& index) const {
  return getValue(index).has_value();
}

inline std::optional<Occupancy::Mask> ClassifiedMap::getValue(
    const OctreeIndex& index) const {
  return getValueOrAncestor(index).first;
}

inline const ClassifiedMap::Block* ClassifiedMap::QueryCache::getBlock(
    const Index3D& new_block_index,
    const ClassifiedMap::BlockHashMap& block_map) {
  if (new_block_index != block_index) {
    block_index = new_block_index;
    height = tree_height;
    block = block_map.getBlock(new_block_index);
    if (block) {
      node_stack[tree_height] = &block->getRootNode();
    }
  }
  return block;
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_IMPL_CLASSIFIED_MAP_INL_H_
