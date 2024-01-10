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

inline Occupancy::Mask ClassifiedMap::childOccupancyMask(
    const ClassifiedMap::Node& node, NdtreeIndexRelativeChild child_idx) {
  return Occupancy::toMask(node.data().has_free[child_idx],
                           node.data().has_occupied[child_idx],
                           node.data().has_unobserved[child_idx]);
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_IMPL_CLASSIFIED_MAP_INL_H_
