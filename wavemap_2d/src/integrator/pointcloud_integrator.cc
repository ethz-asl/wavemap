#include "wavemap_2d/integrator/pointcloud_integrator.h"

#include "wavemap_2d/integrator/grid_iterator.h"

namespace wavemap_2d {
void PointcloudIntegrator::integratePointcloud(
    const PosedPointcloud& pointcloud) {
  beam_model_.setStartPoint(pointcloud.getOrigin());
  for (const auto& point : pointcloud.getPointsGlobal()) {
    beam_model_.setEndPoint(point);
    if (beam_model_.exceedsMaxRange()) {
      continue;
    }

    aabb_min_ = aabb_min_.cwiseMin(point);
    aabb_max_ = aabb_max_.cwiseMax(point);

    const Index bottom_left_idx = beam_model_.getBottomLeftUpdateIndex();
    const Index top_right_idx = beam_model_.getTopRightUpdateIndex();
    const Grid grid(bottom_left_idx, top_right_idx);
    for (const auto& index : grid) {
      const FloatingPoint update = beam_model_.computeUpdateAt(index);
      occupancy_map_->updateCell(index, update);
    }
  }
}
}  // namespace wavemap_2d
