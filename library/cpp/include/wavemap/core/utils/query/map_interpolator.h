#ifndef WAVEMAP_CORE_UTILS_QUERY_MAP_INTERPOLATOR_H_
#define WAVEMAP_CORE_UTILS_QUERY_MAP_INTERPOLATOR_H_

#include "wavemap/core/common.h"
#include "wavemap/core/utils/query/query_accelerator.h"

namespace wavemap::interpolate {
template <typename MapT>
FloatingPoint nearestNeighbor(MapT& map, const wavemap::Point3D& position);

template <typename MapT>
FloatingPoint trilinear(MapT& map, const wavemap::Point3D& position);
}  // namespace wavemap::interpolate

#include "wavemap/core/utils/query/impl/map_interpolator_inl.h"

#endif  // WAVEMAP_CORE_UTILS_QUERY_MAP_INTERPOLATOR_H_
