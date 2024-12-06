#ifndef WAVEMAP_CORE_UTILS_RENDER_RAYCAST_H_
#define WAVEMAP_CORE_UTILS_RENDER_RAYCAST_H_

#include "wavemap/core/common.h"
#include "wavemap/core/utils/iterate/ray_iterator.h"

namespace wavemap::raycast {
// NOTE: The map can either be a raw map, or a QueryAccelerator instance. We
//       strongly recommend passing a QueryAccelerator to get good performance.
template <typename MapT>
std::optional<Index3D> first_collision_index(
    MapT& map, const Point3D& W_start_point, const Point3D& W_end_point,
    FloatingPoint log_odds_occupancy_threshold);

// NOTE: The map can either be a raw map, or a QueryAccelerator instance. We
//       strongly recommend passing a QueryAccelerator to get good performance.
template <typename MapT>
std::optional<FloatingPoint> first_collision_distance(
    MapT& map, const Point3D& W_start_point, const Point3D& W_end_point,
    FloatingPoint log_odds_occupancy_threshold);
}  // namespace wavemap::raycast

#include "wavemap/core/utils/render/impl/raycast_inl.h"

#endif  // WAVEMAP_CORE_UTILS_RENDER_RAYCAST_H_
