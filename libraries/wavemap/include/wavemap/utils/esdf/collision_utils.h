#ifndef WAVEMAP_UTILS_ESDF_COLLISION_UTILS_H_
#define WAVEMAP_UTILS_ESDF_COLLISION_UTILS_H_

#include "wavemap/common.h"
#include "wavemap/data_structure/volumetric/hashed_blocks.h"
#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/utils/random_number_generator.h"

namespace wavemap {
std::optional<Point3D> getCollisionFreePosition(
    const VolumetricDataStructureBase& occupancy_map, const HashedBlocks& esdf,
    FloatingPoint robot_radius);
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_ESDF_COLLISION_UTILS_H_
