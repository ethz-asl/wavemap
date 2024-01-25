#ifndef WAVEMAP_UTILS_QUERY_COLLISION_UTILS_H_
#define WAVEMAP_UTILS_QUERY_COLLISION_UTILS_H_

#include "wavemap/common.h"
#include "wavemap/data_structure/aabb.h"
#include "wavemap/map/hashed_blocks.h"
#include "wavemap/map/map_base.h"
#include "wavemap/utils/random_number_generator.h"

namespace wavemap {
std::optional<Point3D> getCollisionFreePosition(
    const MapBase& occupancy_map, const HashedBlocks& esdf,
    FloatingPoint robot_radius,
    std::optional<AABB<Point3D>> aabb = std::nullopt);
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_COLLISION_UTILS_H_
