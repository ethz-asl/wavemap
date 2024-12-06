#ifndef WAVEMAP_CORE_UTILS_RENDER_IMPL_RAYCAST_INL_H_
#define WAVEMAP_CORE_UTILS_RENDER_IMPL_RAYCAST_INL_H_

namespace wavemap::raycast {
template <typename MapT>
std::optional<Index3D> first_collision_index(
    MapT& map, const Point3D& W_start_point, const Point3D& W_end_point,
    FloatingPoint log_odds_occupancy_threshold) {
  const Ray ray(W_start_point, W_end_point, map.getMinCellWidth());
  for (const Index3D& ray_voxel_index : ray) {
    if (log_odds_occupancy_threshold < map.getCellValue(ray_voxel_index)) {
      return ray_voxel_index;
    }
  }
  return std::nullopt;
}

template <typename MapT>
std::optional<FloatingPoint> first_collision_distance(
    MapT& map, const Point3D& W_start_point, const Point3D& W_end_point,
    FloatingPoint log_odds_occupancy_threshold) {
  const auto colliding_index = first_collision_index<MapT>(
      map, W_start_point, W_end_point, log_odds_occupancy_threshold);
  if (colliding_index) {
    const FloatingPoint min_cell_width = map.getMinCellWidth();
    const Point3D voxel_center =
        convert::indexToCenterPoint(colliding_index.value(), min_cell_width);
    return (voxel_center - W_start_point).norm();
  }
  return std::nullopt;
}
}  // namespace wavemap::raycast

#endif  // WAVEMAP_CORE_UTILS_RENDER_IMPL_RAYCAST_INL_H_
